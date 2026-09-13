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
# Dashboard (design §12 — graded under Functional Verification, 20%)
# ---------------------------------------------------------------------------

# Open the live dashboard. `just gui scenarios/fog_figure8.toml` picks a file.
gui file="": build
    #!/usr/bin/env bash
    set -euo pipefail
    if [[ -n "{{file}}" ]]; then
        "{{build_dir}}/sat-tracker" --gui --scenario "{{file}}"
    else
        "{{build_dir}}/sat-tracker" --gui
    fi

# Open the dashboard with the layout diagnostic on stderr.
gui-debug: build
    SAT_GUI_DEBUG=1 "{{build_dir}}/sat-tracker" --gui

# Confirm the project still builds with the dashboard compiled out.
#
# INV-7 generalised to optional dependencies: a machine with no GLFW must still
# build, still run every synthetic scenario, and still pass the whole suite.
build-headless:
    #!/usr/bin/env bash
    set -euo pipefail
    cmake -S . -B "{{build_dir}}-nogui" -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=ON -DSAT_WITH_GUI=OFF
    cmake --build "{{build_dir}}-nogui" --parallel {{jobs}}
    ctest --test-dir "{{build_dir}}-nogui" --output-on-failure --parallel {{jobs}}
    "{{build_dir}}-nogui/sat-tracker" --gui || true
    echo "headless build OK (--gui reports a clear message rather than failing to build)"

# ---------------------------------------------------------------------------
# Video (design §8 — Benchmark Performance-2, 30% of marks)
# ---------------------------------------------------------------------------

# Regenerate the test clips with ffmpeg. They are committed (72 KB total), so
# this is only needed when adding a case or after changing the generator.
make-test-videos:
    ./tools/make_test_videos.sh

# CP 0.7 ★ GATE — prove this build can decode MP4 at all.
#
# Design §14: "If this fails, STOP and solve it — 30% of marks depend on it."
# Run this first on any new machine or after changing the OpenCV configuration.
gate-video: build
    #!/usr/bin/env bash
    set -euo pipefail
    echo "CP 0.7 gate: probing every committed test clip…"
    failed=0
    for f in tests/video/clips/*.mp4; do
        name=$(basename "$f")
        if out=$("{{build_dir}}/sat-tracker" --probe-video "$f" 2>/dev/null); then
            printf "  %-28s %s\n" "$name" "$(echo "$out" | head -1)"
        else
            # truncated_noindex.mp4 is SUPPOSED to fail — it has no moov atom.
            if [[ "$name" == truncated_noindex.mp4 ]]; then
                printf "  %-28s refused cleanly (expected)\n" "$name"
            else
                printf "  %-28s FAILED\n" "$name"
                failed=1
            fi
        fi
    done
    exit $failed

# Probe a single video file: `just probe-video path/to/clip.mp4`
probe-video file: build
    "{{build_dir}}/sat-tracker" --probe-video "{{file}}"

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

# INV-1: prove the configure-time link guard actually rejects a violation.
#
# Design §2 CP 2.6 establishes the rule: after building a safety check,
# deliberately inject a violation and confirm it goes red. A guard nobody has
# watched fail is just a reassuring message.
gate-inv1-selftest:
    ./tools/verify_inv1_guard.sh

# INV-3 ★ GATE (CP 2.6) — run every built-in scenario twice and compare
# frame fingerprints. This is what makes every benchmark number evidence rather
# than an anecdote.
gate-repro: build
    "{{build_dir}}/sat-tracker" --verify-reproducibility --seeds 3 --duration 2.0

# INV-3 — prove the reproducibility check can actually fail, by injecting a
# wall-clock-derived timestep into the simulation path. Design §14 CP 2.6 asks
# for exactly this.
gate-repro-selftest:
    ./tools/verify_repro_guard.sh

# INV-3 — confirm -O0 and -O2 produce bit-identical results. This is the half
# that catches FP contraction or reassociation, which is why -ffp-contract=off
# is mandatory rather than a preference.
gate-repro-opt: build build-debug
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Comparing digests across optimisation levels…"
    "{{build_dir}}"/sat-tracker --verify-reproducibility --seeds 2 --duration 1.0 \
        | grep -E '^(static|linear|fast|multi|offset)' | awk '{print $1, $2, $5}' > /tmp/sat-O2.txt
    "{{build_dir}}"-debug/sat-tracker --verify-reproducibility --seeds 2 --duration 1.0 \
        | grep -E '^(static|linear|fast|multi|offset)' | awk '{print $1, $2, $5}' > /tmp/sat-O0.txt
    if diff -u /tmp/sat-O0.txt /tmp/sat-O2.txt; then
        echo "PASS: -O0 and -O2 produce bit-identical results."
    else
        echo "INV-3 VIOLATION: optimisation level changed the results." >&2
        exit 1
    fi

# Run every static invariant gate.
gates: gate-inv1 gate-no-chrono gate-no-rand gate-inv1-selftest

# Every gate including the slower reproducibility ones. This is what CI runs.
gates-full: gates gate-repro gate-repro-opt gate-repro-selftest

# Everything CI runs, in the same order. Use this before pushing.
ci: gates test

# The full pre-push check, including cross-optimisation reproducibility.
ci-full: gates-full test

# ---------------------------------------------------------------------------
# Housekeeping
# ---------------------------------------------------------------------------

# Wipe the build trees (including fetched dependencies under build/_deps).
clean:
    rm -rf "{{build_dir}}" "{{build_dir}}-debug" "{{build_dir}}-nogui" vcpkg_installed

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
