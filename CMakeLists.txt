# SAT — Satellite Adaptive Tracker
#
# Root build file. See docs/SAT-DESIGN.md §5.1 for the module graph and §2 for
# the invariants this build system is responsible for enforcing.
#
# Requires CMake >= 3.20 and C++20. Dependencies are declared in vcpkg.json
# (manifest mode) and resolved by cmake/dependencies.cmake, which also accepts a
# system install so a fresh clone can build without waiting for vcpkg.

cmake_minimum_required(VERSION 3.20)

project(sat-tracker
    VERSION 0.1.0
    DESCRIPTION "Self-Adaptive beacon Tracking for mobile FSOC coarse alignment"
    LANGUAGES CXX
)

# --- Language / toolchain policy ------------------------------------------------
# C++20 is mandatory (design §4). Extensions off keeps MSVC and GCC/Clang closer.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile_commands.json for editors / clangd.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Put every binary in one place so the Justfile and CI do not have to guess.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

# --- INV-3: bit-exact reproducibility -------------------------------------------
#
# These flags are not a performance preference, they are a correctness
# requirement. Fast-math lets the compiler reassociate floating-point
# expressions, and FP contraction lets it fuse a multiply and an add into an
# FMA with a different rounding. Either one makes -O0 and -O2 disagree in the
# last bits, which the reproducibility job (CP 2.6) would correctly flag as a
# failure. Design §2 names the exact flags; they are reproduced here.
if(MSVC)
    add_compile_options(/W4 /permissive- /fp:precise)
    # MSVC's __cplusplus is stuck at 199711L without this, which breaks feature
    # detection in several dependencies.
    add_compile_options(/Zc:__cplusplus)
    # The standard library complains about perfectly fine C functions otherwise.
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -ffp-contract=off -fno-fast-math)
endif()

# Default to Release when the user did not pick a build type (single-config generators).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# --- Build provenance -----------------------------------------------------------
# Every run.json and centroid.csv header carries the commit it was produced by
# (design §13.2). Without it a benchmark number cannot be traced to code.
find_package(Git QUIET)
set(SAT_GIT_HASH "unknown")
if(GIT_FOUND)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" rev-parse --short=7 HEAD
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE SAT_GIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(NOT SAT_GIT_HASH)
        set(SAT_GIT_HASH "unknown")
    endif()
endif()
message(STATUS "SAT: build provenance ${PROJECT_VERSION}+${SAT_GIT_HASH}")

add_compile_definitions(
    SAT_VERSION="${PROJECT_VERSION}"
    SAT_GIT_HASH="${SAT_GIT_HASH}"
)

# --- Dependencies ---------------------------------------------------------------
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/dependencies.cmake")

# --- Module graph (enforces INV-1) ----------------------------------------------
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules.cmake")

# --- Tests ----------------------------------------------------------------------
include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# --- Install --------------------------------------------------------------------
install(TARGETS sat-tracker RUNTIME DESTINATION bin)
install(DIRECTORY scenarios/ DESTINATION share/sat-tracker/scenarios
        FILES_MATCHING PATTERN "*.toml" PATTERN "*.png")
