# cmake/Findonnxruntime.cmake — locate ONNX Runtime from a plain install.
#
# ---------------------------------------------------------------------------
# WHY THIS EXISTS
# ---------------------------------------------------------------------------
# `sat_optional_package(onnxruntime ...)` tries CONFIG mode first, which works
# when ONNX Runtime came from vcpkg or from a distribution package that ships
# `onnxruntimeConfig.cmake`. The official release tarballs at
# github.com/microsoft/onnxruntime/releases ship NO CMake package at all — just
# `include/` and `lib/` — and that is how most people and most CI runners get
# it. Without this module, `-DSAT_WITH_ONNX=ON` is a hard FATAL_ERROR for them,
# which is why the ML path had never been built anywhere but one machine.
#
# The audit records the underlying problem as §22.2 blocker 2 ("ONNX Runtime in
# the C++ dependency graph — ABSENT"). This is half the fix; the vcpkg.json
# `ml` feature is the other half, for people who do use vcpkg.
#
# ---------------------------------------------------------------------------
# USAGE
# ---------------------------------------------------------------------------
#   cmake -S . -B build -DSAT_WITH_ONNX=ON \
#         -DONNXRUNTIME_ROOT=/path/to/onnxruntime-linux-x64-1.17.1
#
# `ONNXRUNTIME_ROOT` may also come from the environment. Without it the module
# still searches the normal system prefixes, so a distro package is found with
# no extra flag.
#
# Provides the imported target `onnxruntime::onnxruntime`, which is the same
# name the CONFIG package provides — so cmake/modules.cmake links one name and
# does not care which of the two found it.

include(FindPackageHandleStandardArgs)

# An explicit root wins over whatever else is on the system: someone who passes
# ONNXRUNTIME_ROOT is telling us which build to use, and silently preferring a
# different one found in /usr would be the worst possible outcome.
set(_ort_hints)
if(ONNXRUNTIME_ROOT)
    list(APPEND _ort_hints "${ONNXRUNTIME_ROOT}")
elseif(DEFINED ENV{ONNXRUNTIME_ROOT})
    list(APPEND _ort_hints "$ENV{ONNXRUNTIME_ROOT}")
endif()

find_path(onnxruntime_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    HINTS ${_ort_hints}
    PATH_SUFFIXES include include/onnxruntime include/onnxruntime/core/session
    DOC "Directory containing onnxruntime_cxx_api.h")

find_library(onnxruntime_LIBRARY
    NAMES onnxruntime
    HINTS ${_ort_hints}
    PATH_SUFFIXES lib lib64
    DOC "The ONNX Runtime shared library")

# Version, when the headers state one. Purely informational — nothing here
# requires a minimum — but `run.json` records the runtime version beside every
# ML result (SAT-ML §4.5 pins it as part of the reproducibility statement), so
# it is worth having CMake know it rather than leaving it "unknown".
set(onnxruntime_VERSION "")
if(onnxruntime_INCLUDE_DIR)
    foreach(_hdr "${onnxruntime_INCLUDE_DIR}/onnxruntime_c_api.h")
        if(EXISTS "${_hdr}")
            file(STRINGS "${_hdr}" _ort_ver_line
                 REGEX "^#define[ \t]+ORT_API_VERSION[ \t]+[0-9]+")
            if(_ort_ver_line)
                string(REGEX MATCH "[0-9]+" _ort_api_ver "${_ort_ver_line}")
                set(onnxruntime_VERSION "api-${_ort_api_ver}")
            endif()
        endif()
    endforeach()
endif()

find_package_handle_standard_args(onnxruntime
    REQUIRED_VARS onnxruntime_LIBRARY onnxruntime_INCLUDE_DIR
    VERSION_VAR   onnxruntime_VERSION)

if(onnxruntime_FOUND AND NOT TARGET onnxruntime::onnxruntime)
    # UNKNOWN rather than SHARED: on Windows the import library and the DLL are
    # different files, and declaring SHARED would require IMPORTED_IMPLIB to be
    # set separately. UNKNOWN lets CMake link whatever find_library returned.
    add_library(onnxruntime::onnxruntime UNKNOWN IMPORTED)
    set_target_properties(onnxruntime::onnxruntime PROPERTIES
        IMPORTED_LOCATION "${onnxruntime_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_INCLUDE_DIR}")
endif()

mark_as_advanced(onnxruntime_INCLUDE_DIR onnxruntime_LIBRARY)
