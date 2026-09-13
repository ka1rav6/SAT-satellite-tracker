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

# Suites are registered here as each checkpoint lands.

# ===========================================================================
# End-to-end checks that run the real binary. These catch link and startup
# breakage that a unit test linking a static library cannot.
# ===========================================================================
add_test(NAME app_runs COMMAND sat-tracker --version)
set_tests_properties(app_runs PROPERTIES TIMEOUT 30 LABELS "smoke")
