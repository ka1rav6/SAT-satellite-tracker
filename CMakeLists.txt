# SAT — Satellite Adaptive Tracker
# Root build file. Checkpoint 0.1: produce a runnable binary that prints a version string.
#
# Requires CMake ≥ 3.20 and C++20. Dependencies are declared in vcpkg.json (manifest mode).

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

# INV-3 (bit-exact reproducibility): never enable fast-math or FP contraction.
# /fp:precise on MSVC; -ffp-contract=off -fno-fast-math on GCC/Clang.
if(MSVC)
    add_compile_options(/W4 /permissive- /fp:precise)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -ffp-contract=off -fno-fast-math)
endif()

# Default to Release when the user did not pick a build type (single-config generators).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Inject the project version into every translation unit that includes sat/version.hpp.
add_compile_definitions(SAT_VERSION="${PROJECT_VERSION}")

# --- Module graph (enforces INV-1) ----------------------------------------------
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules.cmake")

# --- Tests ----------------------------------------------------------------------
include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# --- Install (minimal for now) --------------------------------------------------
install(TARGETS sat-tracker RUNTIME DESTINATION bin)