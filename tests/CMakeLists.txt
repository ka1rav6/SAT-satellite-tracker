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
function(sat_add_test suite)
    cmake_parse_arguments(ARG "" "" "SOURCES;DEPS" ${ARGN})

    set(target "test_${suite}")
    add_executable(${target} ${ARG_SOURCES})
    target_link_libraries(${target} PRIVATE sat_test_main ${ARG_DEPS})

    add_test(NAME ${suite} COMMAND ${target})
    set_tests_properties(${suite} PROPERTIES
        TIMEOUT 120
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

# --- CP 2.3-2.5: snapshots and the reproducibility fingerprint --------------
sat_add_test(repro
    SOURCES repro/test_fingerprint.cpp
    DEPS    sat_core sat_engine
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

# ===========================================================================
# End-to-end checks that run the real binary. These catch link and startup
# breakage that a unit test linking a static library cannot.
# ===========================================================================
add_test(NAME app_runs COMMAND sat-tracker --version)
set_tests_properties(app_runs PROPERTIES TIMEOUT 30 LABELS "smoke")
