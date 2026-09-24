# cmake/dependencies.cmake
#
# Dependency acquisition for SAT.
#
# The design doc (§4) names vcpkg manifest mode as the canonical way to get
# dependencies, and vcpkg.json remains the pinned, reproducible source of truth
# for CI and release artifacts. But a developer who has just cloned the repo
# should not have to wait an hour for vcpkg to build OpenCV from source before
# they can run `just test`. So every dependency is resolved through one helper
# that tries, in order:
#
#   1. find_package(... CONFIG)  — picks up vcpkg when the toolchain file is
#      active, and also picks up a system/apt install when it is not.
#   2. find_package(... MODULE)  — for packages that only ship FindXxx.cmake
#      (Eigen on Debian/Ubuntu is a common case).
#   3. FetchContent               — pinned by git tag, for the small header-only
#      libraries where building from source costs seconds, not hours.
#
# Anything heavy (OpenCV, ONNX Runtime) is NEVER fetched: it is found or the
# feature that needs it is switched off. That keeps INV-7 honest — the system
# must run without AI — and keeps the build usable on a machine that has not
# installed a 4 GB vision toolkit.
#
# Each dependency sets a SAT_HAVE_<NAME> variable in the parent scope so the
# module graph can gate optional features instead of failing to configure.

include(FetchContent)

# Fetched sources land in build/_deps. `just clean` removes them along with the
# build tree, which costs a re-download; `just clean-build` keeps them.
set(FETCHCONTENT_QUIET OFF)

# ---------------------------------------------------------------------------
# CMake 4 compatibility for the fetched dependencies.
#
# CMake 4.0 removed support for `cmake_minimum_required(VERSION <3.5)`, and
# several of the libraries below still declare one:
#
#     doctest v2.4.11        cmake_minimum_required(VERSION 3.0)
#     nlohmann/json v3.11.3  cmake_minimum_required(VERSION 3.1)
#
# So on any runner with CMake 4.x, configuring fails inside the DEPENDENCY, with
# an error that has nothing to do with this project. That is exactly what broke
# the windows-msvc-release job while every Linux job passed — GitHub's
# windows-latest image ships CMake 4.x and ubuntu-latest still ships 3.31.
#
# CMAKE_POLICY_VERSION_MINIMUM is CMake 4.0's documented escape hatch: it floors
# the effective minimum for projects that ask for less. It is set only around
# the fetches and restored afterwards, so it never relaxes policy for OUR code —
# this project targets 3.20 and should keep being held to it.
#
# Unknown to CMake < 4.0, where it is simply ignored, so this is safe on both.
set(SAT_SAVED_POLICY_MIN "${CMAKE_POLICY_VERSION_MINIMUM}")
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# ---------------------------------------------------------------------------
# Feature switches.
#
# AUTO means "use it if it is available". Setting one to OFF forces the
# classical / built-in fallback path, which is exactly what INV-7 demands we
# keep working. Setting one to ON makes a missing dependency a hard error,
# which is what CI wants.
# ---------------------------------------------------------------------------
set(SAT_WITH_OPENCV  "AUTO" CACHE STRING "OpenCV: video decode (§8) and test oracles (§16). AUTO/ON/OFF")
set(SAT_WITH_ONNX    "OFF"  CACHE STRING "ONNX Runtime: ML inference (§11). AUTO/ON/OFF")
set(SAT_WITH_GUI     "AUTO" CACHE STRING "GLFW + Dear ImGui + ImPlot dashboard (§12). AUTO/ON/OFF")
set(SAT_WITH_TRACY   "OFF"  CACHE STRING "Tracy profiler instrumentation (dev builds only). AUTO/ON/OFF")

# ---------------------------------------------------------------------------
# sat_require_package(<pkg> [VERSION v] [TARGETS t...] [MODULE])
#
# Find a package that we consider mandatory. Fails configure with a message
# that tells the developer exactly how to get it, rather than a wall of CMake
# internals.
# ---------------------------------------------------------------------------
function(sat_require_package pkg)
    cmake_parse_arguments(ARG "MODULE" "VERSION;HINT" "" ${ARGN})
    if(ARG_MODULE)
        find_package(${pkg} ${ARG_VERSION} REQUIRED)
    else()
        # CONFIG first (vcpkg / modern installs), then fall back to MODULE mode.
        find_package(${pkg} ${ARG_VERSION} CONFIG QUIET)
        if(NOT ${pkg}_FOUND)
            find_package(${pkg} ${ARG_VERSION} QUIET)
        endif()
    endif()
    if(NOT ${pkg}_FOUND)
        message(FATAL_ERROR
            "Required dependency '${pkg}' was not found.\n"
            "  ${ARG_HINT}\n"
            "  Or run `just setup-vcpkg` and re-configure with the vcpkg toolchain.")
    endif()
endfunction()

# ---------------------------------------------------------------------------
# sat_optional_package(<pkg> <switch> <out_var> ...)
#
# Resolve an optional dependency honouring AUTO/ON/OFF. Sets <out_var> in the
# caller's scope to TRUE/FALSE.
# ---------------------------------------------------------------------------
function(sat_optional_package pkg switch out_var)
    cmake_parse_arguments(ARG "" "VERSION" "COMPONENTS" ${ARGN})

    if(switch STREQUAL "" OR ${switch} STREQUAL "OFF")
        set(${out_var} FALSE PARENT_SCOPE)
        message(STATUS "SAT: ${pkg} disabled by switch (fallback path will be used)")
        return()
    endif()

    if(ARG_COMPONENTS)
        find_package(${pkg} ${ARG_VERSION} COMPONENTS ${ARG_COMPONENTS} CONFIG QUIET)
    else()
        find_package(${pkg} ${ARG_VERSION} CONFIG QUIET)
    endif()
    if(NOT ${pkg}_FOUND)
        find_package(${pkg} ${ARG_VERSION} QUIET)
    endif()

    if(${pkg}_FOUND)
        message(STATUS "SAT: ${pkg} found (${${pkg}_VERSION})")
        set(${out_var} TRUE PARENT_SCOPE)
    elseif(${switch} STREQUAL "ON")
        message(FATAL_ERROR "SAT: ${pkg} was requested with ${switch}=ON but not found.")
    else()
        message(STATUS "SAT: ${pkg} NOT found — the feature that uses it is disabled")
        set(${out_var} FALSE PARENT_SCOPE)
    endif()
endfunction()

# ===========================================================================
# Mandatory, tiny, header-only: fetched when absent.
# ===========================================================================

# --- doctest — the test framework (design §4) ------------------------------
# Chosen over Catch2/GoogleTest because it compiles roughly an order of
# magnitude faster, which matters when AGENTS.md §6 asks for a test per feature.
find_package(doctest CONFIG QUIET)
if(NOT doctest_FOUND)
    message(STATUS "SAT: fetching doctest v2.4.11")
    FetchContent_Declare(doctest
        GIT_REPOSITORY https://github.com/doctest/doctest.git
        GIT_TAG        v2.4.11
        GIT_SHALLOW    TRUE)
    set(DOCTEST_WITH_TESTS OFF CACHE BOOL "" FORCE)
    set(DOCTEST_NO_INSTALL ON  CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(doctest)
    # Make doctest_discover_tests() available exactly as the installed package does.
    list(APPEND CMAKE_MODULE_PATH "${doctest_SOURCE_DIR}/scripts/cmake")
    set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" PARENT_SCOPE)
endif()

# --- toml++ — the ONLY configuration language (design §4, §7) --------------
find_package(tomlplusplus CONFIG QUIET)
if(NOT tomlplusplus_FOUND)
    message(STATUS "SAT: fetching tomlplusplus v3.4.0")
    FetchContent_Declare(tomlplusplus
        GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
        GIT_TAG        v3.4.0
        GIT_SHALLOW    TRUE)
    FetchContent_MakeAvailable(tomlplusplus)
endif()

# --- nlohmann/json — run.json and sweep manifests (design §13.4) -----------
find_package(nlohmann_json 3.11 CONFIG QUIET)
if(NOT nlohmann_json_FOUND)
    message(STATUS "SAT: fetching nlohmann_json v3.11.3")
    FetchContent_Declare(nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG        v3.11.3
        GIT_SHALLOW    TRUE)
    set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(nlohmann_json)
endif()

# --- Eigen 3.4 — fixed-size linear algebra for Kalman / IMM (design §10.2) -
# Header-only, but a 200 MB clone, so we much prefer the system copy.
find_package(Eigen3 3.4 CONFIG QUIET)
if(NOT Eigen3_FOUND)
    find_package(Eigen3 3.4 MODULE QUIET)
endif()
if(NOT TARGET Eigen3::Eigen)
    message(STATUS "SAT: fetching Eigen 3.4.0 (install libeigen3-dev to skip this)")
    FetchContent_Declare(Eigen3
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG        3.4.0
        GIT_SHALLOW    TRUE)
    set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
    # Do not FORCE BUILD_TESTING into the cache. Eigen is an include-only
    # interface target here (no add_subdirectory), and a cache FORCE would
    # disable SAT's own CTest on every machine that fetches Eigen.
    FetchContent_Populate(Eigen3)
    # Eigen's own CMakeLists drags in tests and install rules we do not want;
    # an interface target over its include dir is all we need.
    add_library(Eigen3_headers INTERFACE)
    target_include_directories(Eigen3_headers SYSTEM INTERFACE "${eigen3_SOURCE_DIR}")
    add_library(Eigen3::Eigen ALIAS Eigen3_headers)
endif()

# Restore the policy floor: from here on, our own code is held to the minimum
# the root CMakeLists declares.
set(CMAKE_POLICY_VERSION_MINIMUM "${SAT_SAVED_POLICY_MIN}")
unset(SAT_SAVED_POLICY_MIN)

# ===========================================================================
# Optional, heavy: found or the feature is switched off.
# ===========================================================================

# --- OpenCV — MP4 decode (§8) and test oracles (§4.2, §16) -----------------
# Note the strict boundary in design §4.2: OpenCV is for decode, file I/O and
# as a *reference implementation to test against*. Nothing in the 30 Hz hot
# loop may call it.
sat_optional_package(OpenCV SAT_WITH_OPENCV SAT_HAVE_OPENCV VERSION 4.0)

# --- ONNX Runtime — model inference (§11) ----------------------------------
# Off by default: the ML stage is not started yet, and INV-7 requires the whole
# system to work without it.
sat_optional_package(onnxruntime SAT_WITH_ONNX SAT_HAVE_ONNX)

# --- GUI stack (§12) -------------------------------------------------------
#
# GLFW is found (system or vcpkg); Dear ImGui and ImPlot are fetched, because
# neither ships a CMake build and both are meant to be compiled into the
# application rather than linked as libraries. That is their documented usage,
# not a shortcut: ImGui in particular expects its config header to be
# substitutable per project.
#
# OpenGL is required only for a texture upload and a quad. Design §12: "OpenGL
# use is minimal: one GL_R8 texture upload per frame, one quad, ImGui for the
# rest." Everything needed for that is OpenGL 1.1, which every platform exports
# directly — so there is no glad, no GLEW, and no loader to go wrong. ImGui's
# own backend carries an embedded loader for the modern calls it makes.
if(NOT SAT_WITH_GUI STREQUAL "OFF")
    sat_optional_package(glfw3 SAT_WITH_GUI SAT_HAVE_GLFW)
    find_package(OpenGL QUIET)
    if(NOT OPENGL_FOUND)
        message(STATUS "SAT: OpenGL not found — the dashboard is disabled")
        set(SAT_HAVE_GLFW FALSE)
    endif()
else()
    set(SAT_HAVE_GLFW FALSE)
endif()

if(SAT_HAVE_GLFW)
    # Dear ImGui, docking branch — design §4 names "Dear ImGui (docking)", and
    # the docking build is what allows §12's many panels to be rearranged into a
    # layout that suits a demo rather than a fixed grid.
    FetchContent_Declare(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG        v1.91.8-docking
        GIT_SHALLOW    TRUE)
    FetchContent_MakeAvailable(imgui)

    FetchContent_Declare(implot
        GIT_REPOSITORY https://github.com/epezent/implot.git
        GIT_TAG        v0.16
        GIT_SHALLOW    TRUE)
    FetchContent_MakeAvailable(implot)

    # Neither ships a build, so compile them here as one static library.
    add_library(sat_imgui STATIC
        "${imgui_SOURCE_DIR}/imgui.cpp"
        "${imgui_SOURCE_DIR}/imgui_draw.cpp"
        "${imgui_SOURCE_DIR}/imgui_tables.cpp"
        "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
        "${imgui_SOURCE_DIR}/imgui_demo.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
        "${implot_SOURCE_DIR}/implot.cpp"
        "${implot_SOURCE_DIR}/implot_items.cpp"
    )
    target_include_directories(sat_imgui SYSTEM PUBLIC
        "${imgui_SOURCE_DIR}"
        "${imgui_SOURCE_DIR}/backends"
        "${implot_SOURCE_DIR}"
    )
    target_link_libraries(sat_imgui PUBLIC glfw OpenGL::GL)
    # Third-party code: our -Wall -Wextra -Wpedantic would drown the build in
    # warnings we are not going to fix upstream.
    if(NOT MSVC)
        target_compile_options(sat_imgui PRIVATE -w)
    endif()
    message(STATUS "SAT: dashboard enabled (Dear ImGui docking + ImPlot)")
endif()

# Publish results to the parent (root CMakeLists) scope.
set(SAT_HAVE_OPENCV "${SAT_HAVE_OPENCV}" CACHE INTERNAL "OpenCV available")
set(SAT_HAVE_ONNX   "${SAT_HAVE_ONNX}"   CACHE INTERNAL "ONNX Runtime available")
set(SAT_HAVE_GLFW   "${SAT_HAVE_GLFW}"   CACHE INTERNAL "GLFW available")

message(STATUS "SAT dependency summary:")
message(STATUS "    OpenCV        : ${SAT_HAVE_OPENCV}   (video ingest §8, test oracles §16)")
message(STATUS "    ONNX Runtime  : ${SAT_HAVE_ONNX}   (ML inference §11)")
message(STATUS "    GLFW/GUI      : ${SAT_HAVE_GLFW}   (dashboard §12)")
