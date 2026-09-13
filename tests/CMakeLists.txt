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
# The 120 s timeout is deliberate: a test that hangs is a failure mode this
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
#   Release            the perception suite runs the full pipeline over ~320
#                      frames in ~25 s.
#   Debug (-O0)        ~5x slower. A 120 s timeout fired here and nowhere else,
#                      which made it look like a Debug-specific bug when it was
#                      only Debug being Debug.
#   Sanitizers         ~10x slower again.
#
# Scaling by configuration keeps the check meaningful in each without making it
# vacuous in any.
# ---------------------------------------------------------------------------
if(CMAKE_CXX_FLAGS MATCHES "sanitize")
    set(SAT_TEST_TIMEOUT 900)
    set(SAT_TIMEOUT_WHY "sanitizers")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(SAT_TEST_TIMEOUT 600)
    set(SAT_TIMEOUT_WHY "Debug build, unoptimised")
else()
    set(SAT_TEST_TIMEOUT 120)
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
    DEPS    sat_core sat_plant sat_control sat_engine sat_perception
)

# --- CP 3.3-3.5: the motion algebra and its analytic derivatives ------------
sat_add_test(motion
    SOURCES motion/test_motion.cpp
    DEPS    sat_core sat_world
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
    DEPS    sat_core sat_perception sat_engine sat_scenario
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
            tracking/test_reacquire.cpp
    DEPS    sat_core sat_tracking sat_control sat_search sat_perception sat_engine sat_scenario
)
target_compile_definitions(test_tracking PRIVATE
    SAT_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios"
)

# --- CP 0.7 ★ GATE: OpenCV can decode MP4 (30% of marks depend on it) ------
# Runs whether or not OpenCV is present: without it the gate test reports a loud
# SKIP rather than passing silently, so a build that cannot do video says so.
sat_add_test(video
    SOURCES video/test_probe.cpp
    DEPS    sat_core sat_engine
)
target_compile_definitions(test_video PRIVATE
    SAT_TEST_CLIP_DIR="${CMAKE_CURRENT_SOURCE_DIR}/video/clips"
)
# CP 8.8 probes each awkward clip in a SEPARATE PROCESS (see test_probe.cpp),
# so it needs the path to the shipped binary.
target_compile_definitions(test_video PRIVATE
    SAT_TRACKER_BINARY="$<TARGET_FILE:sat-tracker>"
)
add_dependencies(test_video sat-tracker)

# ===========================================================================
# End-to-end checks that run the real binary. These catch link and startup
# breakage that a unit test linking a static library cannot.
# ===========================================================================
add_test(NAME app_runs COMMAND sat-tracker --version)
set_tests_properties(app_runs PROPERTIES TIMEOUT 30 LABELS "smoke")
