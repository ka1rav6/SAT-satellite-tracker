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

# How the ML recipes find their interpreter. A repo-local .venv wins (Windows
# puts it under Scripts/, POSIX under bin/), then $SAT_PYTHON, then python3.
# See the MotionNet section for why this is resolved rather than hardcoded.
py_resolve := '''
if   [[ -x .venv/bin/python ]];         then PY=.venv/bin/python
elif [[ -x .venv/Scripts/python.exe ]]; then PY=.venv/Scripts/python.exe
elif [[ -n "${SAT_PYTHON:-}" ]];        then PY="$SAT_PYTHON"
else                                         PY=python3
fi
'''

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
# Stage 10 — control refinement (design §10.3, §10.4)
# ---------------------------------------------------------------------------

# CP 10.1 ablation — velocity feedforward on and off, with the comparison plot.
#
# Writes logs/control/cp101.svg. The scenario is an INSTRUMENT, not a
# compliance claim: see the long note at the top of the file for why row 23's
# jitter and §9.1's clutter are removed, and why removing them is not making
# the numbers look better than they are.
cp101: build
    #!/usr/bin/env bash
    set -euo pipefail
    out="logs/control"
    mkdir -p "$out"
    for k in 0 1; do
        "{{build_dir}}/sat-tracker" --headless             --scenario scenarios/control/fast_linear.toml             --set "control.k_ff=$k" --trace --no-csv --no-report --quiet             --out "$out/ff$k"
        printf "  k_ff = %s   " "$k"
        python3 - "$out/ff$k/run.json" <<'PY'
    import json, sys
    m = json.load(open(sys.argv[1]))["metrics"]["tracking"]
    print("tracking RMS %7.2f px   p95 %7.2f px" % (m["rms_px"], m["p95_px"]))
    PY
    done
    python3 tools/plot_control.py -o "$out/cp101.svg" --column err_px         --title "CP 10.1 — velocity feedforward on a 200 px/s target"         --subtitle "scenarios/control/fast_linear.toml — kp=8, ki=0.5, kd=0.15"         --ylabel "pointing error, px"         "k_ff = 0 (feedback only)=$out/ff0/trace.csv"         "k_ff = 1 (feedforward)=$out/ff1/trace.csv"

# CP 10.2 ablation — a saturating 375 px slew, with and without anti-windup.
#
# Writes logs/control/cp102.svg. The criterion is "settles cleanly with no
# ringing": one overshoot lobe and then quiet. The counterfactual is the point —
# design §10.4 calls anti-windup "essential", and that claim is worth what the
# run without it says it is worth.
cp102: build
    #!/usr/bin/env bash
    set -euo pipefail
    out="logs/control"
    mkdir -p "$out"
    for aw in true false; do
        "{{build_dir}}/sat-tracker" --headless             --scenario scenarios/control/slew.toml             --set "control.anti_windup=$aw" --trace --no-csv --no-report --quiet             --out "$out/aw$aw"
    done
    python3 tools/plot_control.py -o "$out/cp102.svg" --column err_x_px         --title "CP 10.2 — a saturating 375 px slew, with and without anti-windup"         --subtitle "scenarios/control/slew.toml — kp=8, ki=2, kd=0.15; the loop asks for 3.75x the rate ceiling"         --ylabel "pointing error (azimuth), px"         "anti-windup on=$out/awtrue/trace.csv"         "anti-windup off=$out/awfalse/trace.csv"
    python3 tools/plot_control.py -o "$out/cp102_integrator.svg" --column integ_x         --title "CP 10.2 — what the integrator does during the slew"         --subtitle "conditional integration freezes it while the plant is against its stops"         --ylabel "integrator, urad*s"         "anti-windup on=$out/awtrue/trace.csv"         "anti-windup off=$out/awfalse/trace.csv"

# CP 10.5 ablation — the IMM against a single constant-velocity filter, on
# spec row 12's mandatory figure-8, with the mode probability panel.
#
# Writes logs/control/cp105.svg and cp105_modes.svg.
cp105: build
    #!/usr/bin/env bash
    set -euo pipefail
    out="logs/control"
    mkdir -p "$out"
    for m in false true; do
        "{{build_dir}}/sat-tracker" --headless             --scenario scenarios/control/figure8.toml             --set "tracking.imm=$m" --trace --no-csv --no-report --quiet             --out "$out/imm$m"
        printf "  imm = %-5s  " "$m"
        python3 - "$out/imm$m/run.json" <<'PY'
    import json, sys
    m = json.load(open(sys.argv[1]))["metrics"]["tracking"]
    print("tracking RMS %7.2f px   p95 %7.2f   max %7.2f" % (
        m["rms_px"], m["p95_px"], m["max_px"]))
    PY
    done
    python3 tools/plot_control.py -o "$out/cp105.svg" --column err_px         --title "CP 10.5 — the IMM on spec row 12's figure-8"         --subtitle "scenarios/control/figure8.toml — 8 s lap, peak 617 px/s^2"         --ylabel "pointing error, px"         "single CV filter=$out/immfalse/trace.csv"         "IMM (CV/CA/CT)=$out/immtrue/trace.csv"
    python3 tools/plot_control.py -o "$out/cp105_modes.svg" --column imm_cv         --title "CP 10.5 — mode probabilities around the lap"         --subtitle "the three models' posterior probability, which must sum to 1"         --ylabel "mode probability"         "CV=$out/immtrue/trace.csv#imm_cv"         "CA=$out/immtrue/trace.csv#imm_ca"         "CT=$out/immtrue/trace.csv#imm_ct"

# CP 10.6 — error and saturation against disturbance level.
#
# Design §1.3's central claim is that the disturbance can exceed the actuator's
# authority outright. This is the chart that states where: a 200 px/s target
# with spec row 25's platform motion scaled up until the mount runs out of
# rate.
#
# The arithmetic the chart is checked against: the mount's 5 deg/s ceiling is
# 87266 urad/s = 800 px/s per axis. The commanded boresight has to move at
# (target rate - drift rate) — see CP 10.3 for why it is a difference — so with
# the drift opposing the target the demand is 200 + d px/s and the ceiling is
# reached at d = 600 px/s.
#
# The variants are generated rather than committed: the axis is a velocity
# inside [[disturbance.platform]], an array of tables, and --set addresses
# dotted keys. Writing nine near-identical scenario files to vary one number is
# exactly the drift scenarios/control/fast_linear.toml's header warns about.
cp106: build
    @python3 tools/cp106_sweep.py --binary "{{build_dir}}/sat-tracker" \
        --scenario scenarios/control/fast_linear.toml --out logs/control

# CP 10.7 — handover success rate and time-to-handover, over 20 seeds per arm.
#
# The second arm adds spec row 23's maximum camera jitter, which a
# co-boresighted quadrant cell sees in full and the encoder does not see at
# all. See the tool's header for the arithmetic; the short version is that the
# specification's own jitter is five times the handover criterion, so the
# system correctly refuses to hand over rather than claiming an alignment a
# real fine sensor would immediately lose.
cp107: build
    @python3 tools/cp107_sweep.py --binary "{{build_dir}}/sat-tracker" \
        --scenario scenarios/control/fast_linear.toml --out logs/control

# ---------------------------------------------------------------------------
# Stage 15 — deliverables
# ---------------------------------------------------------------------------

# P3-4 — regenerate the committed baseline the documentation is checked against.
#
# `just gate-docs` verifies that every number a document MARKS is still the
# number the system produces. That check needs something to compare against,
# and comparing against a live run would make the gate re-run the simulator on
# every commit and turn a documentation check into a three-minute one.
#
# So one canonical run is committed, and this recipe is how it is refreshed.
# The command is fixed deliberately: the specification defaults, 30 s, default
# seed. A baseline whose command can drift is not a baseline.
#
# WHEN TO RUN THIS. Only when a change is MEANT to move the numbers, and then
# read the diff before committing it — `git diff docs/baseline/run.json` is the
# most direct statement available of what a change did to the graded metrics.
# Running it to make a red gate go green, without reading it, is the one way
# to make this whole mechanism worthless.
baseline: build
    #!/usr/bin/env bash
    set -euo pipefail
    {{build_dir}}/sat-tracker --headless \
        --scenario scenarios/spec_defaults.toml \
        --duration 30 --out docs/baseline --no-report --no-csv
    echo
    echo
    echo "  docs/baseline/run.json refreshed."
    echo "  Read 'git diff docs/baseline/run.json' before you commit it."
    echo
    echo "  Expect noise in that diff: wall_time_s, the CPU times and the peak"
    echo "  RSS differ on every run and on every machine. None of them is"
    echo "  anchored by a document, and none of them should be. What matters"
    echo "  in the diff is the accuracy and lock blocks."

# CP 15.5 — a release archive: the binary, the scenarios, the docs, the clips.
#
# "Download onto a clean machine, unzip, run." The archive carries everything a
# run needs and nothing that has to be rebuilt, and it records WHICH BUILD it
# came from — an artifact that cannot say which commit produced it is an
# artifact whose numbers cannot be reproduced.
package: build
    #!/usr/bin/env bash
    set -euo pipefail
    hash=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)
    dirty=$(git diff --quiet 2>/dev/null && echo "" || echo "-dirty")
    name="sat-tracker-${hash}${dirty}-$(uname -s | tr '[:upper:]' '[:lower:]')-$(uname -m)"
    stage="dist/$name"
    rm -rf "$stage"
    mkdir -p "$stage/bin" "$stage/docs" "$stage/clips"

    cp "{{build_dir}}/sat-tracker" "$stage/bin/"
    cp -r scenarios "$stage/"
    cp README.md QUICKSTART.md "$stage/"
    cp docs/ARCHITECTURE.md docs/RESULTS.md docs/MANUAL.md docs/METRICS.md        docs/DEMO.md "$stage/docs/"
    # The Technical Report is a submitted deliverable and the document that
    # explains what the numbers in the others MEAN. An archive that ships the
    # measurements without the argument for them is half an artifact.
    cp docs/report/TECHNICAL_REPORT.md "$stage/docs/"
    # One clip, not all fourteen: the archive is for running the system, and
    # the awkward cases are a test fixture that only means anything next to the
    # test that interprets them.
    cp tests/video/clips/screen_2000x2000_30fps.mp4 "$stage/clips/" 2>/dev/null || true

    # Provenance. Without this an archive is a binary of unknown origin, and
    # every number it produces is unattributable.
    {
        echo "sat-tracker"
        echo "commit   : $hash$dirty"
        echo "built    : $(uname -s) $(uname -m)"
        echo "packaged : $(date -u +%Y-%m-%dT%H:%M:%SZ)"
        echo
        echo "Start here: QUICKSTART.md"
        echo "  ./bin/sat-tracker --help"
        echo "  ./bin/sat-tracker --headless --scenario scenarios/spec_defaults.toml --out logs"
    } > "$stage/BUILD-INFO.txt"

    (cd dist && tar czf "$name.tar.gz" "$name")
    echo
    echo "  dist/$name.tar.gz  ($(du -h "dist/$name.tar.gz" | cut -f1))"
    echo "  Verify it on a clean machine with:  just verify-package dist/$name.tar.gz"

# CP 15.5's other half — unpack an archive somewhere else and prove it runs.
#
# A package nobody has unpacked is a package that does not work. This extracts
# into a temporary directory with NOTHING from the source tree on the path and
# runs a scenario out of the archive's own copy.
verify-package archive: 
    #!/usr/bin/env bash
    set -euo pipefail
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    tar xzf "{{archive}}" -C "$tmp"
    root=$(find "$tmp" -maxdepth 1 -mindepth 1 -type d | head -1)
    echo "unpacked to $root"
    cat "$root/BUILD-INFO.txt"
    echo
    ( cd "$root" && ./bin/sat-tracker --headless         --scenario scenarios/spec_defaults.toml --duration 3 --out logs --no-report )
    echo
    echo "  package verified: it runs from its own copy of everything."

# ---------------------------------------------------------------------------
# Stage 13 — acquisition strategy (design §10.5)
# ---------------------------------------------------------------------------

# CP 13.2 — all four search strategies benchmarked, with the camp-and-wait
# crossover plotted against target speed.
#
# Writes logs/search/cp132.svg. Takes about twenty minutes: 160 runs of 45 s
# each, because a cold acquisition metric needs many seeds to mean anything —
# P(beacon visible at t = 0) is 7.68%, so a single run mostly measures where
# the beacon happened to start.
cp132: build
    @python3 tools/cp132_sweep.py --binary "{{build_dir}}/sat-tracker" \
        --scenario scenarios/search/cold.toml --out logs/search

# CP 14.1 ★ — 5,000 random scenarios: no crash, no hang, no NaN.
#
# Every parameter across its legal range, with one draw in six taking an
# ENDPOINT rather than a uniform sample — uniform sampling almost never hits
# its own bounds, and the bounds are where the bugs are.
#
# Takes about ten minutes. `just fuzz-quick` is the 500-scenario version for a
# pre-commit check.
fuzz: build
    "{{build_dir}}/sat-tracker" --fuzz-scenarios 5000 --duration 0.3

# The same, short enough to run before a commit.
fuzz-quick: build
    "{{build_dir}}/sat-tracker" --fuzz-scenarios 500 --duration 0.3

# ---------------------------------------------------------------------------
# Stage 12 — the SAT supervisor (design §10.6)
# ---------------------------------------------------------------------------

# CP 12.2 ablation — the supervisor on and off through a weather change, with
# the strategy timeline.
#
# Writes logs/supervisor/cp122.svg (the CFAR threshold the supervisor chose,
# against time) and cp122_snr.svg (the smoothed SNR it chose it from).
cp122: build
    #!/usr/bin/env bash
    set -euo pipefail
    out="logs/supervisor"
    mkdir -p "$out"
    for e in false true; do
        "{{build_dir}}/sat-tracker" --headless             --scenario scenarios/supervisor/weather_change.toml             --set "supervisor.enabled=$e" --trace --no-csv --no-report --quiet             --out "$out/sup$e"
        printf "  supervisor %-5s  " "$e"
        python3 - "$out/sup$e/run.json" <<'PY'
    import json, sys
    m = json.load(open(sys.argv[1]))["metrics"]
    print("retention %6.2f %%   tracking RMS %7.2f px   centroid %8.3f px" % (
        100.0 * m["lock"]["retention_rate"], m["tracking"]["rms_px"],
        m["centroiding"]["rmse_image_px"]))
    PY
    done
    python3 tools/plot_control.py -o "$out/cp122.svg" --column sup_cfar_k         --title "CP 12.2 — the strategy timeline through a weather change"         --subtitle "fog at 10 s, clear at 25 s; at most one switch per second by construction"         --ylabel "CFAR threshold k, sigma"         "supervisor on=$out/suptrue/trace.csv#sup_cfar_k"         "supervisor off=$out/supfalse/trace.csv#sup_cfar_k"
    python3 tools/plot_control.py -o "$out/cp122_snr.svg" --column sup_snr         --title "CP 12.2 — the smoothed integrated SNR the switches are made on"         --subtitle "EMA over 15 frames; the thresholds are 8 and 15"         --ylabel "integrated SNR"         "smoothed SNR=$out/suptrue/trace.csv#sup_snr"

# CP 12.3 — per-condition Monte Carlo: which strategy wins where.
cp123: build
    @python3 tools/cp123_sweep.py --binary "{{build_dir}}/sat-tracker" \
        --scenario scenarios/supervisor/weather_change.toml --out logs/supervisor

# The Stage 12 checkpoints, verbose — the numbers ARE the checkpoints.
test-supervisor: build
    "{{build_dir}}/test_supervisor" --success --no-skipped-summary 2>&1         | grep -E "MESSAGE|TEST CASE|ERROR|test cases" || true

# The Stage 10 control checkpoints, verbose — the numbers ARE the checkpoints.
#
# Two binaries: test_control holds the integration cases (the whole engine,
# several simulated seconds each) and test_loop holds the controller's own unit
# tests, one of which is CP 10.1's closed-form steady-state lag.
test-control: build
    #!/usr/bin/env bash
    set -euo pipefail
    for t in test_control test_loop; do
        "{{build_dir}}/$t" -tc="*CP 10.*" --success --no-skipped-summary 2>&1 \
            | grep -E "MESSAGE|TEST CASE|ERROR|test cases" || true
    done

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
gates: gate-source gate-inv1-selftest gate-docs gate-model-cards

# Check that the documentation's own references resolve: every `just` recipe it
# names exists, every repository path it names exists, every link and
# cross-reference resolves, and no document numbers two sections the same.
#
# The docs are a deliverable, not a comment. Three real defects were found by
# running this by hand before it was a gate — a directory described as holding
# breaking cases that contained only a .gitkeep, two sections numbered 2.12 one
# of which was stale, and a recipe pointing at the wrong test binary after a
# suite was split. None was a typo; each was a statement that had stopped being
# true.
gate-docs:
    ./tools/check_docs.py

# Check that every model card's SAT-ML §6.6 gate table matches the eval.json
# its evaluation actually wrote.
#
# ML-8 makes the card a deliverable and its numbers are the claim the whole ML
# section rests on. They are transcribed by hand, and one had already drifted:
# the MotionNet v1 card quoted 45.7% / 53.9% from a training run that is not
# the one models/motionnet_v1.eval.json records (43.4% / 49.5%). Both pass the
# gate, so no conclusion changed — which is exactly why nobody noticed.
gate-model-cards:
    ./tools/check_model_cards.py

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
#   just video tests/video/clips/bp2_screen_2000x2000.mp4
#
# Scored against truth when a .csv of the same stem sits beside the clip, which
# is how the bp2_* fixtures are shipped. Without one the run still produces
# centroid.csv, and the summary says plainly that nothing was scored — an
# unscored run used to look like a successful one.
video file: build
    #!/usr/bin/env bash
    set -euo pipefail
    truth="${file%.*}.csv"
    if [ -f "$truth" ]; then
        "{{build_dir}}/sat-tracker" --video "{{file}}" --truth "$truth" --out logs/video
    else
        echo "note: no truth CSV at $truth — centroiding will not be scored."
        "{{build_dir}}/sat-tracker" --video "{{file}}" --out logs/video
    fi

# ---------------------------------------------------------------------------
# just bp2 — BENCHMARK PERFORMANCE-2, END TO END. 30% of the marks.
# ---------------------------------------------------------------------------
# The single most heavily graded capability in the problem statement is
# "Comparison of Centroiding error with predefined error values" on an
# evaluator-supplied MP4. Until this recipe existed the project had no
# end-to-end number for it: `just video` passed no --truth, no truth CSV was
# committed for any clip, and the CP 8.7 self-scoring test deliberately used a
# plain intensity-weighted centroid rather than the perception pipeline, ran
# the source alone rather than the closed loop, and only in direct mode.
#
# Three clips, each answering a different question:
#
#   clean direct   the CONTROL. A noiseless box has an exact centroid, so a
#                  non-zero result here is a bug in the crop or in the truth,
#                  not in the centroider. Knowing that first saves an
#                  afternoon.
#   noisy direct   the codec and row-22 noise, with the crop taken out: same
#                  resolution in and out.
#   screen 2000x2000
#                  THE REHEARSAL. The shape the PS describes — a complete
#                  screen with noise and a moving beacon — exercising
#                  acquisition, the crop, tracking and handover together.
#
# Truth is exact by construction: tools/make_test_videos.sh writes it from the
# same expression that draws the beacon, so the two cannot drift.
bp2: build
    #!/usr/bin/env bash
    set -euo pipefail
    clips="{{justfile_directory()}}/tests/video/clips"
    out="{{justfile_directory()}}/logs/bp2"
    mkdir -p "$out"
    if ! "{{build_dir}}/sat-tracker" --has-video >/dev/null; then
        echo "This build has no video support, so BP-2 cannot be measured."
        echo "Install OpenCV and reconfigure with -DSAT_WITH_OPENCV=ON."
        exit 1
    fi
    for name in bp2_clean_direct_640x480 bp2_noisy_direct_640x480 bp2_screen_2000x2000; do
        echo
        echo "════════════════════════════════════════════════════════════════"
        echo "  $name"
        echo "════════════════════════════════════════════════════════════════"
        "{{build_dir}}/sat-tracker" --video "$clips/$name.mp4" \
            --truth "$clips/$name.csv" --out "$out/$name"
    done
    echo
    echo "Artifacts in logs/bp2/. Each carries run.json, centroid.csv and report.html."

# The same three clips with per-stage timings, for the throughput half of the
# BP-2 question. Separate from `just bp2` because the timing table is long and
# the accuracy table is what a judge asks for first.
bp2-stages: build
    #!/usr/bin/env bash
    set -euo pipefail
    clips="{{justfile_directory()}}/tests/video/clips"
    for name in bp2_noisy_direct_640x480 bp2_screen_2000x2000; do
        echo "── $name ──"
        "{{build_dir}}/sat-tracker" --video "$clips/$name.mp4" \
            --truth "$clips/$name.csv" --stages --quiet \
            --out "{{justfile_directory()}}/logs/bp2/$name"
    done

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

# ---------------------------------------------------------------------------
# MotionNet (SAT-ML §6) — SAT simulator tracks only, no KITTI/MOT/TLE
# ---------------------------------------------------------------------------
#
# Every recipe below resolves its interpreter through `py_resolve` rather than
# naming one. The first version of these recipes hardcoded
# `.venv/Scripts/python.exe`, which is the Windows venv layout and exists on
# exactly one machine: CI is Linux, AGENTS.md §8 asks for a dev environment
# reproducible through Docker, and a training pipeline nobody else can run is
# a training pipeline whose results nobody else can check. The lookup order is
# deliberate — a repo-local .venv first (either layout), then SAT_PYTHON for a
# caller who knows better, then plain python3.

# Create ./.venv and install ml/requirements.txt into it.
#
# Optional: the recipes below fall back to python3 when the packages are
# already importable. It exists so that "how do I train this?" has a one-line
# answer that is the same on Linux, macOS and Windows.

# Create ./.venv and install the pinned ML dependencies into it.
ml-setup:
    #!/usr/bin/env bash
    set -euo pipefail
    if [[ ! -d .venv ]]; then
        echo "Creating .venv…"
        python3 -m venv .venv
    fi
    {{py_resolve}}
    "$PY" -m pip install --upgrade pip
    "$PY" -m pip install -r ml/requirements.txt
    echo
    echo "Ready. 'just ml-check' verifies the imports."

# Report which interpreter the ML recipes will use and whether it can import
# what they need.
#
# Run this first when an ML recipe fails. It separates "the environment is not
# set up" from "the code is broken", which otherwise arrive as the same
# traceback and cost an afternoon.

# Show which interpreter the ML recipes use, and whether its imports resolve.
ml-check:
    #!/usr/bin/env bash
    set -euo pipefail
    {{py_resolve}}
    echo "interpreter: $PY  ($("$PY" --version 2>&1))"
    "$PY" -c 'import importlib.util, sys; m=[x for x in ("numpy","torch","onnx","onnxruntime") if importlib.util.find_spec(x) is None]; print("missing: "+", ".join(m)+"\nrun: just ml-setup") or sys.exit(1) if m else print("all ML dependencies importable")'

# Dump four official row-12 regimes + weather, then window into shards.
# Default is a short factory (seeds 1-4, 8 s) so a laptop can finish. Raise
# --seed-end / --duration for the scored train.

# Generate the MotionNet track dataset from the simulator.
motion-data seed_end="30" duration="10":
    #!/usr/bin/env bash
    set -euo pipefail
    {{py_resolve}}
    "{{build_dir}}/sat-tracker" --version
    "$PY" -m ml.datagen --sweep ml/sweeps/motion_v1.toml \
        --bin "{{build_dir}}/sat-tracker" --out data/motion_v1 \
        --seed-end {{seed_end}} --duration {{duration}} --require-dropouts

# Train on data/motion_v1 train/val only (ML-7: test is evaluate-only).
train-motion:
    #!/usr/bin/env bash
    set -euo pipefail
    {{py_resolve}}
    "$PY" -m ml.train_motion --dataset data/motion_v1 --out models/motionnet_v1.pt

# SAT-ML §6.6 gate on the held-out test split.
eval-motion:
    #!/usr/bin/env bash
    set -euo pipefail
    {{py_resolve}}
    "$PY" -m ml.evaluate_motion --dataset data/motion_v1 --ckpt models/motionnet_v1.pt

# Export the checkpoint to fixed-shape ONNX (opset 17).
export-motion:
    #!/usr/bin/env bash
    set -euo pipefail
    {{py_resolve}}
    "$PY" -m ml.export models/motionnet_v1.pt models/motionnet_v1.onnx --task motion

# Official reacq / lock ablation: same seeds with and without the model.
motion-ablate:
    #!/usr/bin/env bash
    set -euo pipefail
    {{py_resolve}}
    "$PY" tools/motion_ablate.py --bin "{{build_dir}}/sat-tracker"

# The whole MotionNet pipeline in the order SAT-ML §6 specifies:
# data -> train -> §6.6 gate -> export.
#
# It stops at the first failure, which is the point: a model that misses the
# gate is never exported, so it can never reach a demo. eval-motion exits
# non-zero when the gate fails.

# The whole MotionNet pipeline: data, train, SAT-ML §6.6 gate, ONNX export.
motion-all: motion-data train-motion eval-motion export-motion
    @echo "MotionNet: data, train, §6.6 gate and ONNX export all complete."

# The Stage 9 suite, verbose — the numbers ARE the checkpoints: the S-curve's
# amplitude, the correction's gain, and the ratio to §10.1.1's bound per SNR bin.
test-centroid: build
    "{{build_dir}}/test_centroid" --success --no-skipped-summary 2>&1 \
        | grep -E "MESSAGE|TEST CASE|ERROR|test cases" || true

# CP 14.2 — time each kernel on its own, minimum of N runs.
#
# `just stages` measures ONE call per frame against a clock whose resolution is
# the frame itself, on a machine that is also running the rest of the frame;
# run-to-run spread on a laptop is 1.5x, which is larger than most of the
# changes worth making. This measures the kernel instead. Both numbers are
# needed and they answer different questions.
bench-kernels repeats="60": build
    "{{build_dir}}/sat-tracker" --bench-kernels --repeats "{{repeats}}"

# ---------------------------------------------------------------------------
# Documentation figures
# ---------------------------------------------------------------------------

# Regenerate every screenshot in docs/MANUAL.md.
#
# A screenshot pasted into a repository is a claim nobody can check: it rots
# silently the moment a panel moves. Every figure in the manual is the output of
# this recipe, so a stale one is a diff rather than a surprise. It is also the
# only automated check that the GUI still starts and draws — CP 15.0's
# acceptance criterion, which nothing in CI could verify before.
#
# The window is created HIDDEN, so this does not throw five windows across
# whatever you are doing, and at a fixed 2400x1350 so a window manager cannot
# crop the tables differently on every machine.
#
# ffmpeg quantises each shot to 64 colours afterwards. These are UI screenshots
# with a handful of distinct colours, so the palette is lossless in practice and
# takes 2.1 MB down to 0.4 MB — which is the difference between a repository
# that clones quickly and one that does not.
screenshots: build
    #!/usr/bin/env bash
    set -euo pipefail
    mkdir -p docs/img
    shot() {
        local out="$1"; shift
        "{{build_dir}}/sat-tracker" --gui --shot "docs/img/$out.png" "$@"
        if command -v ffmpeg >/dev/null 2>&1; then
            ffmpeg -y -loglevel error -i "docs/img/$out.png"                 -vf "palettegen=max_colors=64" "/tmp/sat-pal-$out.png"
            ffmpeg -y -loglevel error -i "docs/img/$out.png" -i "/tmp/sat-pal-$out.png"                 -lavfi "paletteuse=dither=none" "docs/img/$out.q.png"
            mv "docs/img/$out.q.png" "docs/img/$out.png"
            rm -f "/tmp/sat-pal-$out.png"
        fi
    }
    shot 01-overview  --scenario scenarios/spec_defaults.toml --shot-after 150
    shot 02-imm       --scenario scenarios/fog_figure8.toml --shot-after 400                       --shot-imm --shot-damage --shot-focus imm
    shot 03-priority  --scenario scenarios/compliance.toml --shot-after 200                       --shot-clutter 120 --shot-damage --shot-focus priority
    shot 04-strategy  --scenario scenarios/supervisor/weather_change.toml                       --shot-after 500 --shot-supervisor --shot-damage                       --shot-focus strategy
    shot 05-fsm       --scenario scenarios/hard/cold_start_in_clutter.toml --shot-after 60                       --shot-focus fsm --shot-random
    # CP 4.11's ablation. Identical to 03-priority's configuration except for
    # --shot-strawman, so the figure IS the comparison: same scenario, same
    # seed, same damage, same clutter, only the detector differs.
    shot 06-strawman  --scenario scenarios/compliance.toml --shot-after 150                       --shot-clutter 120 --shot-damage --shot-focus tracking                       --shot-strawman
    echo "docs/img/ regenerated"

# CP 14.4 — per-stage p50/p95/p99 from the SHIPPED binary, against §15's budget.
stages scenario="scenarios/compliance.toml" duration="6": build
    "{{build_dir}}/sat-tracker" --headless --scenario "{{scenario}}" \
        --duration "{{duration}}" --out logs/prof --stages
