# Justfile — developer command surface for SAT (sat-tracker).
#
# Basic CP 0.1 recipes. Docker and heavier tooling grow in later checkpoints.
# Usage: `just`, `just build`, `just test`, `just run`, `just clean`, …

set shell := ["bash", "-euo", "pipefail", "-c"]

# Where CMake writes its build tree.
build_dir := "build"

# Prefer an existing VCPKG_ROOT; otherwise use a repo-local clone under ./vcpkg.
vcpkg_root := env_var_or_default("VCPKG_ROOT", justfile_directory() / "vcpkg")
toolchain  := vcpkg_root / "scripts/buildsystems/vcpkg.cmake"

# Pinned to the same commit as builtin-baseline in vcpkg.json.
vcpkg_baseline := "a1cae005c39be7b18ba319fced856b68d7276271"

# Default recipe when you type plain `just`.
default: build

# ---------------------------------------------------------------------------
# vcpkg
# ---------------------------------------------------------------------------

# Clone and bootstrap vcpkg at the pinned baseline if it is not already present.
setup-vcpkg:
    #!/usr/bin/env bash
    set -euo pipefail
    if [[ -x "{{vcpkg_root}}/vcpkg" ]]; then
        echo "vcpkg already available at {{vcpkg_root}}"
        exit 0
    fi
    if [[ ! -d "{{vcpkg_root}}/.git" ]]; then
        echo "Cloning vcpkg (baseline {{vcpkg_baseline}}) into {{vcpkg_root}}…"
        git clone --depth 1 https://github.com/microsoft/vcpkg.git "{{vcpkg_root}}"
        git -C "{{vcpkg_root}}" fetch --depth 1 origin "{{vcpkg_baseline}}"
        git -C "{{vcpkg_root}}" checkout "{{vcpkg_baseline}}"
    fi
    echo "Bootstrapping vcpkg…"
    "{{vcpkg_root}}/bootstrap-vcpkg.sh" -disableMetrics

# ---------------------------------------------------------------------------
# Configure / build / test / run
# ---------------------------------------------------------------------------

# Configure the CMake build. Uses vcpkg when the toolchain file exists;
# otherwise falls back to the no-vcpkg preset (fine while dependencies == []).
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
        echo "WARNING: vcpkg toolchain not found at {{toolchain}}"
        echo "         Run \`just setup-vcpkg\` (or set VCPKG_ROOT). Using no-vcpkg configure."
        cmake -S . -B "{{build_dir}}" \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_TESTING=ON
    fi

# Build all targets (configures first if needed).
build: configure
    cmake --build "{{build_dir}}" --parallel

# Run the full CTest suite.
test: build
    ctest --test-dir "{{build_dir}}" --output-on-failure

# Run the sat-tracker binary (prints version at CP 0.1).
run: build
    "{{build_dir}}/sat-tracker"

# Wipe the CMake build tree (and vcpkg_installed if present).
clean:
    rm -rf "{{build_dir}}" vcpkg_installed

# ---------------------------------------------------------------------------
# Docker (stub until a Dockerfile lands in a later checkpoint)
# ---------------------------------------------------------------------------

# Reserved for the reproducible Docker environment (AGENTS.md §8).
# CP 0.1 only scaffolds the recipe; the image itself is not built yet.
docker-setup:
    @echo "docker-setup: Dockerfile not added yet (planned with CI / Stage 0)."
    @echo "For now use: just setup-vcpkg && just build && just test"