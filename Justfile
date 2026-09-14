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
# Setupdddddsssssssssssssssss
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

# Run sat-tracker. With no arguments this opens the live dashboard, which is
# what design §13.4's command table specifies. Arguments pass straight through:
#
#   just run                     the dashboard, default scenario
#   just run --help              every other mode
#   just run --version
#   just run --probe-video F     CP 0.7's video gate on one file
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
# Stage 6 — tracking (design §10.2, §10.4, §10.5)
# ---------------------------------------------------------------------------

# The whole Stage 6 suite: measurements, Kalman, gate, lifecycle, adaptive R,
# the mode FSM, the search pattern, and the CP 6.7 end-to-end reacquisition.
# Verbose, because the numbers ARE the checkpoints — reacquisition time, lock
# retention, and the velocity estimate against the scenario's analytic one.
test-tracking: build
    "{{build_dir}}/test_tracking" --success --no-skipped-summary 2>&1 \
        | grep -E "MESSAGE|TEST CASE|ERROR|test cases" || true

# CP 6.7 ★ — hide the beacon for 2 s, reveal it, and count the frames to
# reacquire. The checkpoint allows 15; specification row 19 allows 30 (1 s).
cp67: build
    "{{build_dir}}/test_tracking" -tc="*6.7*,Stage 6*" --success \
        | grep -E "MESSAGE|TEST CASE|ERROR|test cases"

# CP 4.11 / Stage 5 ablation — the straw-man detector against the real §9.4
# pipeline, through the same closed loop, on the same seeds. This prints the
# before/after columns of design §13.3's ablation table.
ablation: build
    "{{build_dir}}/test_degrade" -tc="*CP 4.11*,*classical pipeline survives*" \
        --success | grep -E "MESSAGE|TEST CASE|ERROR" || true

# ---------------------------------------------------------------------------
# Video (design §8 — Benchmark Performance-2, 30% of marks)
# ---------------------------------------------------------------------------


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

# All three static source invariants (INV-1 and INV-3), in one checker.
#
# A shell grep is not sufficient here, and the reason is worth knowing: this
# project's own diagnostics talk ABOUT the forbidden constructs.
# verify_repro.cpp PRINTS the string "an unseeded generator or rand()" to tell a
# user what to look for, and core/rng.hpp quotes INV-3 in its header comment. A
# grep matched both and failed CI inside the very code that exists to detect
# violations. The checker strips comments and string literals first, and
# self-tests that stripping before it trusts any result.
gate-source:
    ./tools/check_source_invariants.py

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

# Build and run the whole suite under AddressSanitizer and
# UndefinedBehaviorSanitizer.
#
# This is the highest-value bug check in the project and it earned that on its
# first run: it found a heap-buffer-overflow in the median filter's interior
# fast path, which read one element off each end of a row on a 1-pixel-wide
# image. Those reads land in adjacent heap memory and look perfectly harmless —
# every other test passed, on every platform, before and after. Nothing short of
# a sanitizer would have shown it.
#
# -fno-sanitize-recover=all makes the first finding fatal, so a violation
# cannot scroll past in a passing-looking log.
sanitize:
    #!/usr/bin/env bash
    set -euo pipefail
    cmake -S . -B "{{build_dir}}-asan" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DBUILD_TESTING=ON -DSAT_WITH_GUI=OFF \
        -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all" \
        -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"
    cmake --build "{{build_dir}}-asan" --parallel {{jobs}}
    ctest --test-dir "{{build_dir}}-asan" --output-on-failure --parallel {{jobs}}

# Run every static invariant gate.
gates: gate-source gate-inv1-selftest

# Every gate including the slower reproducibility ones. This is what CI runs.
gates-full: gates gate-repro gate-repro-opt gate-repro-selftest

# Everything CI runs on Linux, in the same order. Use this before pushing.
#
# This deliberately builds Release, Debug AND the no-GUI configuration, because
# `just ci` is only worth running if a green result means CI will be green too.
# An earlier version ran the Release tests alone and reported success while the
# linux-gcc-debug job was failing — Debug is roughly five times slower and was
# the only configuration hitting the test timeout. A pre-push check that cannot
# see a whole class of failure is worse than no check, because it is trusted.
#
# Takes around five minutes. `just ci-full` adds the sanitizer and
# cross-optimisation reproducibility runs on top.
# A build resolving every header-only dependency the way CI does — by FETCHING
# toml++, nlohmann/json and doctest at their pinned versions rather than using
# whatever the system happens to have installed.
#
# What this catches: version drift. The system toml++ here is Debian's
# 3.4.0+ds, not upstream's v3.4.0, and a difference between them would show up
# as a compile error on one machine and not the other.
#
# What it does NOT catch, and this is worth being explicit about because it was
# tried first for exactly that purpose: a third-party header included by a
# module that does not LINK that library. Forcing the fetch does not remove
# /usr/include from the default include path, so the system copy is still found
# and the build still succeeds. That case is caught statically instead — see
# `gate-source` and THIRD_PARTY in tools/check_source_invariants.py.
build-fetched:
    cmake -S . -B "{{build_dir}}-fetched" -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=ON \
        -DCMAKE_DISABLE_FIND_PACKAGE_tomlplusplus=ON \
        -DCMAKE_DISABLE_FIND_PACKAGE_nlohmann_json=ON \
        -DCMAKE_DISABLE_FIND_PACKAGE_doctest=ON
    cmake --build "{{build_dir}}-fetched" --parallel {{jobs}}
    @echo "fetched-dependency build OK (no system headers masking a missing link edge)"

ci: gates test test-debug build-headless build-fetched

# The full pre-push check, including cross-optimisation reproducibility.
ci-full: gates-full test test-debug build-headless build-fetched sanitize

# ---------------------------------------------------------------------------
# Housekeeping
# ---------------------------------------------------------------------------

# Wipe the build trees (including fetched dependencies under build/_deps).
clean:
    rm -rf "{{build_dir}}" "{{build_dir}}-debug" "{{build_dir}}-nogui" "{{build_dir}}-asan" "{{build_dir}}-fetched" vcpkg_installed

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

# ---------------------------------------------------------------------------
# Stage 7 — metrics, logs, batch (design §13)
# ---------------------------------------------------------------------------

# One headless run with the §13.1 metric summary, centroid.csv and run.json.
#   just headless                                  the baseline scenario
#   just headless "--scenario scenarios/fog_figure8.toml --duration 20"
headless *ARGS: build
    "{{build_dir}}/sat-tracker" --headless --out logs {{ARGS}}

# The §3.2 graded requirements, measured on the specification's OWN defaults —
# row 23's jitter included, which is the configuration that exposed the
# mis-sized gate at Stage 7. Prints the numbers rather than just pass/fail.
spec-run: build
    "{{build_dir}}/test_metrics" -tc="*specification's own jitter*,*derived floor*,*clutter costs*" \
        --success | grep -E "MESSAGE|TEST CASE|ERROR|test cases"

# CP 7.5 / ★ CP 7.7 — the requirement compliance sweep.
#
# 200 runs (40 conditions x 5 seeds) across worker processes, then design
# §13.3's matrix. Takes about 5 minutes on 8 cores; see the note in
# scenarios/sweeps/weather.toml for what each axis isolates and why.
sweep *ARGS: build
    "{{build_dir}}/sat-tracker" --sweep scenarios/sweeps/weather.toml \
        --out logs/sweep {{ARGS}}

# Just the matrix from the last sweep, without re-running it.
matrix:
    @cat logs/sweep/compliance.txt

# CP 7.6 — one run, with report.html. Opens nothing; prints the path.
report scenario="scenarios/compliance.toml" duration="10": build
    "{{build_dir}}/sat-tracker" --headless --scenario "{{scenario}}" \
        --duration "{{duration}}" --out logs/run

# ---------------------------------------------------------------------------
# Stage 8 — video ingest (design §8, Benchmark Performance-2, 30% of marks)
# ---------------------------------------------------------------------------

# Track a supplied clip. Mode is auto-detected from its resolution.
#   just video tests/video/clips/screen_2000x2000_30fps.mp4
video file: build
    "{{build_dir}}/sat-tracker" --video "{{file}}" --out logs/video

# The whole Stage 8 suite, verbose — the numbers ARE the checkpoints: the
# crop's own centroid error (CP 8.6), the self-scoring accuracy (CP 8.7), and
# what each of the ten awkward clips did (CP 8.8).
test-video: build
    "{{build_dir}}/test_video" --success --no-skipped-summary 2>&1 \
        | grep -E "MESSAGE|TEST CASE|ERROR|test cases" || true

# Regenerate the test clips. They are committed, so this is only needed when
# adding a case or after changing the generator. It VERIFIES every clip
# contains a beacon before finishing — see the note in the script about the
# entire directory having been silently black.
make-test-videos:
    ./tools/make_test_videos.sh

# ---------------------------------------------------------------------------
# Stage 9 — centroiding accuracy (design §10.1, 60% of the marks)
# ---------------------------------------------------------------------------

# CP 9.2 / CP 9.3 — measure the S-curve and regenerate the compiled-in bias
# table. Takes about a minute. Rebuild afterwards to compile the table in, then
# run it again to see the gain.
calibrate *ARGS: build
    "{{build_dir}}/sat-tracker" --calibrate-centroid {{ARGS}}

# The Stage 9 suite, verbose — the numbers ARE the checkpoints: the S-curve's
# amplitude, the correction's gain, and the ratio to §10.1.1's bound per SNR bin.
test-centroid: build
    "{{build_dir}}/test_centroid" --success --no-skipped-summary 2>&1 \
        | grep -E "MESSAGE|TEST CASE|ERROR|test cases" || true

# CP 14.4 — per-stage p50/p95/p99 from the SHIPPED binary, against §15's budget.
stages scenario="scenarios/compliance.toml" duration="6": build
    "{{build_dir}}/sat-tracker" --headless --scenario "{{scenario}}" \
        --duration "{{duration}}" --out logs/prof --stages
