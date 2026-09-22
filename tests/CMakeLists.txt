# tests/CMakeLists.txt
#
# AGENTS.md §6: "Create unit tests for everything. Test every feature separately
# so failures are isolated and diagnosable."
#
# That second sentence drives the layout here. Rather than one monolithic test
# binary, each subdirectory of tests/ becomes its own executable and its own
# CTest entry. When CI goes red the test NAME tells you which subsystem broke
# before you read a single line of output, and `ctest -R centroid` runs just the
# thing you are working on.
#
# doctest is the framework (design §4). Its main() is provided once in
# test_main.cpp and linked into every test binary, so the individual test files
# contain nothing but test cases.

# ---------------------------------------------------------------------------
# The shared doctest entry point. Compiled once, linked everywhere.
# ---------------------------------------------------------------------------
add_library(sat_test_main STATIC test_main.cpp)
target_link_libraries(sat_test_main PUBLIC doctest::doctest sat_core)
target_include_directories(sat_test_main PUBLIC
    "${CMAKE_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

# ---------------------------------------------------------------------------
# sat_add_test(<suite> SOURCES ... [DEPS ...])
#
# Declares one test executable and registers it with CTest.
#
# The timeout is deliberate: a test that hangs is a failure mode this
# project specifically cares about (CP 14.1 requires "no crash, no hang" over
# 5000 fuzzed scenarios), so an infinite loop must show up as a red test rather
# than a stuck CI job.
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# Test timeout, scaled by how slow the build configuration is.
#
# The timeout is not padding: CP 14.1 treats a hang as a first-class failure
# alongside a crash, so it must stay tight enough to catch one. But it has to be
# tight relative to the configuration actually being run, and the same code is
# an order of magnitude slower in some of them:
#
#   Release            the slowest suite is 46 s (control), with robustness at
#                      42 s and supervisor at 34 s behind it. Three suites are
#                      now within a factor of three of the cap.
#   Debug (-O0)        MEASURED at 8x, not the 5x first assumed here: the same
#                      suites run 56 -> 353 s, 45 -> 362 s, 31 -> 251 s. The
#                      earlier 5x was a guess, and a 600 s cap left the tracking
#                      suite using 60% of its budget on a developer laptop —
#                      which on a slower CI runner is the same red job this
#                      scaling was introduced to prevent.
#   Sanitizers         ~10x slower again, on top of Debug's own cost.
#
# The Debug and sanitizer figures are therefore derived from the Release cap
# times the measured ratio, with a margin, rather than typed as round numbers.
# Scaling by configuration keeps the check meaningful in each without making it
# vacuous in any.
# ---------------------------------------------------------------------------
# AMENDMENT. The base was 120 s, and by this file's own argument that was too
# tight. The Debug paragraph above rejects a cap that leaves a suite "using 60%
# of its budget on a developer laptop — which on a slower CI runner is the same
# red job this scaling was introduced to prevent". Release was in exactly that
# position: 46 s against 120 s is 38% of budget on an 8-core developer machine
# running the suite ALONE, and CI runs `ctest --parallel 4` on a shared 2-core
# runner, where the same work takes several times longer.
#
# 300 s applies the file's own rule instead of contradicting it: 6.5x the
# measured worst case, which survives CI contention while staying far tighter
# than a hang. CP 14.1 treats a hang as a first-class failure and a hang is
# unbounded, so the cap only has to be finite and prompt to catch one — 5
# minutes is both.
set(SAT_TIMEOUT_BASE 300)                 # optimised build, the reference
if(CMAKE_CXX_FLAGS MATCHES "sanitize")
    math(EXPR SAT_TEST_TIMEOUT "${SAT_TIMEOUT_BASE} * 16")   # 4800 s
    set(SAT_TIMEOUT_WHY "sanitizers, ~16x Release")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    math(EXPR SAT_TEST_TIMEOUT "${SAT_TIMEOUT_BASE} * 10")   # 3000 s
    set(SAT_TIMEOUT_WHY "Debug build, measured ~8x Release")
else()
    set(SAT_TEST_TIMEOUT ${SAT_TIMEOUT_BASE})
    set(SAT_TIMEOUT_WHY "optimised build")
endif()
message(STATUS "SAT: test timeout ${SAT_TEST_TIMEOUT}s (${SAT_TIMEOUT_WHY})")

function(sat_add_test suite)
    cmake_parse_arguments(ARG "" "" "SOURCES;DEPS" ${ARGN})

    set(target "test_${suite}")
    add_executable(${target} ${ARG_SOURCES})
    target_link_libraries(${target} PRIVATE sat_test_main ${ARG_DEPS})

    add_test(NAME ${suite} COMMAND ${target})
    set_tests_properties(${suite} PROPERTIES
        TIMEOUT ${SAT_TEST_TIMEOUT}
        LABELS "unit"
    )
endfunction()

# ===========================================================================
# Suites. One per subsystem, in roughly the order the checkpoints build them.
# ===========================================================================

# --- CP 0.2: units and coordinate frames -----------------------------------
sat_add_test(frames
    SOURCES frames/test_units.cpp
            frames/test_frames.cpp
    DEPS    sat_core
)

# --- CP 0.3: clock and RNG streams -----------------------------------------
sat_add_test(core_time_rng
    SOURCES core/test_time.cpp
            core/test_rng.cpp
    DEPS    sat_core
)

# --- CP 0.4: arena, ring buffer, triple buffer, hashing --------------------
sat_add_test(core_memory
    SOURCES core/test_arena.cpp
            core/test_ring.cpp
            core/test_triple_buffer.cpp
            core/test_hash.cpp
    DEPS    sat_core
)

# --- CP 1.3: exact-coverage splatting (the ground truth the 60% metric needs) -
sat_add_test(render
    SOURCES render/test_coverage.cpp
            render/test_splat.cpp
    DEPS    sat_core sat_camera sat_world
)

# --- CP 1.6-1.8: plant, controller, and the closed-loop gate ----------------
sat_add_test(loop
    SOURCES loop/test_gimbal.cpp
            loop/test_controller.cpp
            loop/test_closed_loop.cpp
    # sat_scenario: the A-3 regression loads a real scenario file, because the
    # platform motion stack it needs only exists on the scenario path.
    DEPS    sat_core sat_plant sat_control sat_engine sat_perception sat_scenario
)
# The A-3 regression loads scenarios/spec_defaults.toml from disk for the same
# reason the robustness suite does: the committed FILE is what has to work.
target_compile_definitions(test_loop PRIVATE
    SAT_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios"
)

# --- Stage 10: the control checkpoints -------------------------------------
#
# Its own suite, with its own timeout, because every case drives the whole
# engine for several simulated seconds: the feedforward's input is a Kalman
# estimate produced from noisy centroids through a gated association, and
# handing the controller an exact velocity would answer a question nobody
# asked. Folded into `loop` it made a suite of millisecond unit tests inherit a
# multi-minute runtime, and then time out.
sat_add_test(control
    SOURCES control/test_stage10.cpp
            control/test_imm_smith_benefit.cpp
    DEPS    sat_core sat_plant sat_control sat_engine sat_perception sat_scenario
)
# P2-11's ablations run real scenarios from the repository, because the whole
# point of the question is whether the benefit shows up on the SHIPPED
# configurations rather than on a fixture written to flatter it.
target_compile_definitions(test_control PRIVATE
    SAT_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios"
)
math(EXPR SAT_CONTROL_TIMEOUT "${SAT_TEST_TIMEOUT} * 3")
set_tests_properties(control PROPERTIES TIMEOUT ${SAT_CONTROL_TIMEOUT})

# --- Stage 12: the SAT supervisor ------------------------------------------
#
# Mostly pure-function tests of SatSupervisor, which is why they are fast: the
# question "does this chatter" is about a SEQUENCE OF DECISIONS, and answering
# it by running a simulation would answer the much weaker question of whether
# it chattered on one scenario's conditions. Two cases do need the engine and
# carry the suite's runtime.
sat_add_test(supervisor
    SOURCES supervisor/test_supervisor.cpp
    DEPS    sat_core sat_control sat_engine sat_scenario sat_perception sat_tracking
)
set_tests_properties(supervisor PROPERTIES TIMEOUT ${SAT_CONTROL_TIMEOUT})

# --- CP 14.3: INV-4's allocation trap --------------------------------------
#
# Vacuous in Release by construction — the trap is Debug-only — so the suite
# carries a self-test that FAILS if a Debug build ever stops defining
# SAT_ALLOC_TRAP. An invariant checker nobody has watched fail is one you are
# trusting rather than using.
sat_add_test(robustness
    SOURCES robustness/test_deadline.cpp
            robustness/test_alloc.cpp
    DEPS    sat_core sat_engine sat_scenario sat_app sat_metrics sat_search
)
# A longer budget than the control suite: this one runs 40 fuzzed scenarios and
# six adversarial ones on top of two full runs, and several of the fuzzed
# configurations are 1920x1080 with a thousand clutter sources.
math(EXPR SAT_ROBUST_TIMEOUT "${SAT_TEST_TIMEOUT} * 6")
set_tests_properties(robustness PROPERTIES TIMEOUT ${SAT_ROBUST_TIMEOUT})
# The adversarial cases are loaded from disk rather than built in code: their
# whole value is that the committed FILE is what gets exercised, so a file that
# stops parsing turns this red rather than passing against a copy in a test.
target_compile_definitions(test_robustness PRIVATE
    SAT_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios"
)

# --- CP 3.3-3.5: the motion algebra and its analytic derivatives ------------
sat_add_test(motion
    SOURCES motion/test_motion.cpp
            motion/test_edge_behaviour.cpp
    DEPS    sat_core sat_world sat_scenario
)

# --- CP 5.1-5.8: the perception kernels, against OpenCV oracles (§16) -------
sat_add_test(kernels
    SOURCES kernels/test_median.cpp
            kernels/test_morphology.cpp
            kernels/test_sat.cpp
            kernels/test_matched.cpp
            kernels/test_cfar.cpp
            kernels/test_grouping.cpp
    DEPS    sat_core sat_perception sat_scenario sat_camera sat_cv_oracle
)

# --- CP 3.1-3.2, 3.6: TOML loading and spec-row-citing validation -----------
sat_add_test(schema
    SOURCES schema/test_schema.cpp
    DEPS    sat_core sat_scenario
)
target_compile_definitions(test_schema PRIVATE
    SAT_TEST_BAD_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/bad_configs"
    SAT_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios"
)

# --- CP 4.4-4.9: the degradation chain and the disturbances ----------------
sat_add_test(degrade
    SOURCES degrade/test_noise.cpp
            degrade/test_fast_normal.cpp
            degrade/test_ablation.cpp
    DEPS    sat_core sat_degrade sat_scenario sat_world sat_engine
)
target_compile_definitions(test_degrade PRIVATE
    SAT_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios"
)

# --- CP 2.3-2.5: snapshots and the reproducibility fingerprint --------------
sat_add_test(repro
    SOURCES repro/test_fingerprint.cpp
    DEPS    sat_core sat_engine
)

# --- CP 5.7, 5.9: the whole detection pipeline against the real simulator ---
sat_add_test(perception
    SOURCES perception/test_pipeline.cpp
            perception/test_roi_and_snr_gate.cpp
            perception/test_refresh_band.cpp
    DEPS    sat_core sat_perception sat_camera sat_engine sat_scenario
)
target_compile_definitions(test_perception PRIVATE
    SAT_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios"
)

# --- CP 6.1-6.3: angular measurements, the Kalman filter and the gate ------
sat_add_test(tracking
    SOURCES tracking/test_measurement.cpp
            tracking/test_kalman.cpp
            tracking/test_track.cpp
            tracking/test_mode_fsm.cpp
            tracking/test_search.cpp
            tracking/test_priority.cpp
            tracking/test_reacquire.cpp
    DEPS    sat_core sat_tracking sat_control sat_search sat_perception sat_engine sat_scenario
)
target_compile_definitions(test_tracking PRIVATE
    SAT_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios"
)

# --- CP 9.1-9.6: centroiding accuracy, 60% of the marks --------------------
sat_add_test(centroid
    SOURCES centroid/test_centroid.cpp
    DEPS    sat_core sat_metrics sat_perception
)

# --- CP 7.1-7.2: the metric definitions and the graded centroid log --------
sat_add_test(metrics
    SOURCES metrics/test_series.cpp
            metrics/test_collector.cpp
            metrics/test_centroid_log.cpp
            metrics/test_spec_run.cpp
            metrics/test_compliance.cpp
            metrics/test_report.cpp
    DEPS    sat_core sat_metrics sat_app sat_engine sat_scenario
)
target_compile_definitions(test_metrics PRIVATE
    SAT_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios"
    # Artifacts land in the build tree, not /tmp, so a failing format test
    # leaves the CSV somewhere a person will actually go and look at it.
    SAT_TEST_TMP_DIR="${CMAKE_CURRENT_BINARY_DIR}"
)

# --- spec row 20: the frame budget, RUN SERIALLY ---------------------------
#
# Registered separately from the metrics suite because it is the one test in
# the project that asserts a WALL-CLOCK figure, and `ctest --parallel 8` would
# have it competing with fifteen other suites for the same cores. It passed
# serially and failed under contention — measuring the scheduler rather than
# the tracker — so CTest is told to run it alone.
#
# The test case itself is marked skip() so the ordinary metrics run does not
# include it; --no-skip here is what turns it on.
add_test(NAME frame_budget
         COMMAND test_metrics --no-skip -ts=perf -s)
# An empty run must FAIL, not pass. The first version of this entry filtered on
# a test-case name containing spaces, matched nothing, and CTest reported it
# green with zero assertions.
set_tests_properties(frame_budget PROPERTIES
    PASS_REGULAR_EXPRESSION "assertions: [1-9]")
set_tests_properties(frame_budget PROPERTIES
    RUN_SERIAL TRUE
    TIMEOUT ${SAT_TEST_TIMEOUT}
    LABELS "perf")

# --- spec row 20 on the VIDEO path, RUN SERIALLY ---------------------------
#
# The BP-2 twin of `frame_budget` above, and registered for the same reason:
# it is a wall-clock assertion and `ctest --parallel` would have it measuring
# the scheduler. Benchmark Performance-2 is 30 % of the marks and throughput is
# half of what it grades, so the figure has to be checked — on a quiet machine.
#
# Declared after the `video` suite below creates test_video; CMake resolves the
# target by name at generate time, so the order here does not matter.
add_test(NAME bp2_budget
         COMMAND test_video --no-skip -ts=perf -s)
set_tests_properties(bp2_budget PROPERTIES
    PASS_REGULAR_EXPRESSION "assertions: [1-9]")
set_tests_properties(bp2_budget PROPERTIES
    RUN_SERIAL TRUE
    TIMEOUT ${SAT_TEST_TIMEOUT}
    LABELS "perf")

# --- CP 0.7 ★ GATE: OpenCV can decode MP4 (30% of marks depend on it) ------
# Runs whether or not OpenCV is present: without it the gate test reports a loud
# SKIP rather than passing silently, so a build that cannot do video says so.
sat_add_test(video
    SOURCES video/test_probe.cpp
            video/test_video_source.cpp
    # sat_metrics: the P0-4 cases score a whole run against committed truth,
    # which is what Benchmark Performance-2 actually grades. They need the
    # §13.1 collector, not just the decoder.
    DEPS    sat_core sat_engine sat_scenario sat_metrics
)
target_compile_definitions(test_video PRIVATE
    SAT_TEST_CLIP_DIR="${CMAKE_CURRENT_SOURCE_DIR}/video/clips"
    SAT_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios"
)
# CP 8.8 probes each awkward clip in a SEPARATE PROCESS (see test_probe.cpp),
# so it needs the path to the shipped binary.
target_compile_definitions(test_video PRIVATE
    SAT_TRACKER_BINARY="$<TARGET_FILE:sat-tracker>"
)
add_dependencies(test_video sat-tracker)

# --- MotionNet tracks-only --gen-dataset (sat_app, INV-1) ------------------
sat_add_test(dataset
    SOURCES app/test_dataset.cpp
    DEPS    sat_core sat_app sat_engine sat_scenario
)
target_compile_definitions(test_dataset PRIVATE
    SAT_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios"
    SAT_TEST_TMP_DIR="${CMAKE_CURRENT_BINARY_DIR}"
)

# ===========================================================================
# End-to-end checks that run the real binary. These catch link and startup
# breakage that a unit test linking a static library cannot.
# ===========================================================================
add_test(NAME app_runs COMMAND sat-tracker --version)
set_tests_properties(app_runs PROPERTIES TIMEOUT 30 LABELS "smoke")
