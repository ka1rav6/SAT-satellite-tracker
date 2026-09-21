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
#
# The per-config variables are not redundant. Multi-config generators (MSVC,
# Xcode) append the configuration name to CMAKE_RUNTIME_OUTPUT_DIRECTORY, so the
# binary lands in build/Release/sat-tracker.exe and every `./build/sat-tracker`
# in CI and in the Justfile misses it. Setting the per-config variables
# explicitly makes the path identical on every generator.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
foreach(cfg IN ITEMS DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${cfg} "${CMAKE_BINARY_DIR}")
endforeach()

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
#
# A-5: the commit hash is stamped at BUILD time, not at configure time.
#
# It used to be an `execute_process` right here, injected as a compile
# definition. CMake does not re-configure because HEAD moved, so the stamp
# froze at whatever was checked out when the build directory was created —
# observed two commits stale, with `--version` and every run.json confidently
# naming the wrong code. See cmake/stamp_git.cmake for the full argument and
# for why this is a generated header rather than CMAKE_CONFIGURE_DEPENDS.
#
# The version number stays a compile definition: it comes from project(), so it
# changes only when this file does, and this file is a configure dependency by
# definition.
set(SAT_BUILD_STAMP_DIR "${CMAKE_BINARY_DIR}/generated")
set(SAT_BUILD_STAMP "${SAT_BUILD_STAMP_DIR}/sat/build_stamp.hpp")

# Generate it once at configure time as well, so that the file exists before
# any compiler runs — a header that appears only after the first build would
# break a from-scratch parallel build and confuse every IDE.
execute_process(COMMAND "${CMAKE_COMMAND}"
    "-DSAT_SOURCE_DIR=${CMAKE_SOURCE_DIR}" "-DSAT_OUT=${SAT_BUILD_STAMP}"
    -P "${CMAKE_SOURCE_DIR}/cmake/stamp_git.cmake")

# And re-generate it on EVERY build. ALL puts it in the default target;
# everything else depends on it below.
add_custom_target(sat_build_stamp ALL
    COMMAND "${CMAKE_COMMAND}"
            "-DSAT_SOURCE_DIR=${CMAKE_SOURCE_DIR}" "-DSAT_OUT=${SAT_BUILD_STAMP}"
            -P "${CMAKE_SOURCE_DIR}/cmake/stamp_git.cmake"
    BYPRODUCTS "${SAT_BUILD_STAMP}"
    COMMENT "Stamping build provenance from git HEAD"
    VERBATIM)

add_compile_definitions(
    SAT_VERSION="${PROJECT_VERSION}"
)
# The generated header lives here; sat_add_module puts this on every module's
# include path so that <sat/version.hpp> can pull it in.
include_directories("${SAT_BUILD_STAMP_DIR}")

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
