# Justfile — the developer command surface for SAT (sat-tracker).
#
# AGENTS.md §8 asks for every important command to live here. Run plain `just`
# to build, or `just --list` to see everything with its description.
#
# The recipes are grouped: setup, build/test, quality gates, invariants, and
# Docker. Gate recipes (`just gate-inv1`, `just gate-repro`) mirror what CI runs,
# so a failure can be reproduced locally with one command.

set shell := ["bash", "-euo", "pipefail", "-c"]

# Where CMake writes its build tree.
build_dir := "build"

# Prefer an existing VCPKG_ROOT; otherwise use a repo-local clone under ./vcpkg.
vcpkg_root := env_var_or_default("VCPKG_ROOT", justfile_directory() / "vcpkg")
toolchain  := vcpkg_root / "scripts/buildsystems/vcpkg.cmake"

# Pinned to the same commit as builtin-baseline in vcpkg.json.
vcpkg_baseline := "a1cae005c39be7b18ba319fced856b68d7276271"

# Parallelism for builds and sweeps.
jobs := num_cpus()

# Default recipe when you type plain `just`.
default: build

# Show every recipe with its doc comment.
help:
    @just --list --unsorted

# ---------------------------------------------------------------------------
# Setup
# ---------------------------------------------------------------------------

# Clone and bootstrap vcpkg at the pinned baseline if it is not already present.
#
# You do NOT need this to build: cmake/dependencies.cmake finds system packages
# first and fetches the small header-only ones. vcpkg is what CI uses to produce
# reproducible release artifacts with a pinned OpenCV and ONNX Runtime.
setup-vcpkg:
    #!/usr/bin/env bash
    set -euo pipefail
    if [[ -x "{{vcpkg_root}}/vcpkg" ]]; then
        echo "vcpkg already available at {{vcpkg_root}}"
        exit 0
    fi
    if [[ ! -d "{{vcpkg_root}}/.git" ]]; then
        echo "Cloning vcpkg (baseline {{vcpkg_baseline}}) into {{vcpkg_root}}…"
        git clone https://github.com/microsoft/vcpkg.git "{{vcpkg_root}}"
        git -C "{{vcpkg_root}}" checkout "{{vcpkg_baseline}}"
    fi
    echo "Bootstrapping vcpkg…"
    "{{vcpkg_root}}/bootstrap-vcpkg.sh" -disableMetrics

# Install the system packages this project can use if they are present.
# Debian/Ubuntu only; everything here is optional and the build works without it.
setup-system-deps:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "The following are OPTIONAL — the build works without them:"
    echo "  libeigen3-dev   Kalman/IMM linear algebra (otherwise fetched, 200 MB clone)"
    echo "  libopencv-dev   MP4 decode (design §8) and kernel test oracles (§16)"
    echo "  libglfw3-dev    GUI dashboard (design §12, currently deferred)"
    echo
    echo "sudo apt install libeigen3-dev libopencv-dev libglfw3-dev"

# ---------------------------------------------------------------------------
# Configure / build / test / run
# ---------------------------------------------------------------------------

# Configure the CMake build. Uses the vcpkg toolchain when it exists; otherwise
# cmake/dependencies.cmake resolves system packages and fetches the small ones.
configure:
    #!/usr/bin/env bash
    set -euo pipefail
    if [[ -f "{{toolchain}}" ]]; then
        echo "Configuring with vcpkg toolchain: {{toolchain}}"
        cmake -S . -B "{{build_dir}}" \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_TOOLCHAIN_FILE="{{toolchain}}" \
            -DVCPKG_MANIFEST_DIR="{{justfile_directory()}}" \
            -DBUILD_TESTING=ON
    else
        cmake -S . -B "{{build_dir}}" \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_TESTING=ON
    fi

# Configure a Debug build in build-debug/ (assertions, allocation trap, -O0).
configure-debug:
    cmake -S . -B "{{build_dir}}-debug" -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON

# Build all targets (configures first if needed).
build: configure
    cmake --build "{{build_dir}}" --parallel {{jobs}}

# Build the Debug tree.
build-debug: configure-debug
    cmake --build "{{build_dir}}-debug" --parallel {{jobs}}

# Run the full CTest suite.
test: build
    ctest --test-dir "{{build_dir}}" --output-on-failure --parallel {{jobs}}

# Run only the suites whose name matches a regex, e.g. `just test-one frames`.
test-one pattern: build
    ctest --test-dir "{{build_dir}}" --output-on-failure -R "{{pattern}}"

# Run the test suite in the Debug tree, where the INV-4 allocation trap is armed.
test-debug: build-debug
    ctest --test-dir "{{build_dir}}-debug" --output-on-failure --parallel {{jobs}}

# List every registered test without running anything.
test-list: build
    ctest --test-dir "{{build_dir}}" -N

# Run the sat-tracker binary. Extra arguments are passed through:
#   just run --version
#   just run --help
run *ARGS: build
    "{{build_dir}}/sat-tracker" {{ARGS}}

# ---------------------------------------------------------------------------
# Invariant gates — these mirror the CI jobs (design §2, §16)
# ---------------------------------------------------------------------------

# INV-1: perception/ai/tracking/search/control/plant must never see the world.
#
# The linker already enforces this via cmake/modules.cmake. This grep catches a
# header-only leak that would never reach the linker.
gate-inv1:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "INV-1: checking that no tracker-side module includes the world…"
    if grep -rn --include='*.hpp' --include='*.cpp' \
         -e '#include "world/' -e '#include "camera/' -e '#include "scenario/' \
         src/perception src/ai src/tracking src/search src/control src/plant 2>/dev/null; then
        echo "INV-1 VIOLATION: a tracker-side module includes simulation state." >&2
        exit 1
    fi
    echo "INV-1 ok."

# INV-3: no wall-clock reads anywhere in the simulation path.
#
# core/profile.hpp is the single sanctioned exception (timing never feeds back
# into the simulation and is excluded from the fingerprint). src/app may read a
# clock for wall-time reporting.
gate-no-chrono:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "INV-3: checking that <chrono> appears only where it is allowed…"
    hits=$(grep -rln --include='*.hpp' --include='*.cpp' '<chrono>' src \
           | grep -v '^src/core/profile' | grep -v '^src/app/' || true)
    if [[ -n "$hits" ]]; then
        echo "INV-3 VIOLATION: wall-clock access outside core/profile.hpp:" >&2
        echo "$hits" >&2
        exit 1
    fi
    echo "INV-3 (chrono) ok."

# INV-3: no rand() and no unseeded generators.
gate-no-rand:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "INV-3: checking that nothing uses rand() or an unseeded generator…"
    # The trailing filter drops comment lines: core/rng.hpp quotes the rule
    # itself in its header comment, and the gate must not trip on prose.
    hits=$(grep -rn --include='*.hpp' --include='*.cpp' \
             -e '\brand()' -e '\bsrand(' -e 'random_device' src 2>/dev/null \
           | grep -vE ':[0-9]+:\s*(//|\*|/\*)' || true)
    if [[ -n "$hits" ]]; then
        echo "INV-3 VIOLATION: unseeded randomness in the simulation path:" >&2
        echo "$hits" >&2
        exit 1
    fi
    echo "INV-3 (rand) ok."

# Run every static invariant gate.
gates: gate-inv1 gate-no-chrono gate-no-rand

# Everything CI runs, in the same order. Use this before pushing.
ci: gates test

# ---------------------------------------------------------------------------
# Housekeeping
# ---------------------------------------------------------------------------

# Wipe the build trees (including fetched dependencies under build/_deps).
clean:
    rm -rf "{{build_dir}}" "{{build_dir}}-debug" vcpkg_installed

# Wipe compiled output but keep fetched dependencies, so the next build is fast.
clean-build:
    #!/usr/bin/env bash
    set -euo pipefail
    if [[ -d "{{build_dir}}" ]]; then
        cmake --build "{{build_dir}}" --target clean
    fi

# Remove run outputs (logs, reports) without touching the build.
clean-logs:
    #!/usr/bin/env bash
    set -euo pipefail
    find logs -mindepth 1 ! -name '.gitkeep' -delete 2>/dev/null || true
    echo "logs/ cleared."

# Print the resolved dependency and feature summary.
info: configure
    @echo "build dir : {{build_dir}}"
    @echo "jobs      : {{jobs}}"
    @echo "vcpkg     : {{vcpkg_root}}"

# ---------------------------------------------------------------------------
# Docker (AGENTS.md §8 — reproducible dev environment)
# ---------------------------------------------------------------------------

# Reserved for the reproducible Docker environment. The image lands with CI.
docker-setup:
    @echo "docker-setup: Dockerfile not added yet (planned alongside the CI pipeline)."
    @echo "For now: just build && just test"
