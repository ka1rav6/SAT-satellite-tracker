# cmake/modules.cmake
#
# Declares every SAT library target and the dependency graph from design §5.1.
#
# ---------------------------------------------------------------------------
# INV-1 IS ENFORCED HERE, BY THE LINKER
# ---------------------------------------------------------------------------
# "The CMake targets perception, ai, tracking, search, control and plant must
# not link world. Any attempt to read the simulator's true beacon position from
# those modules is a link error."
#
# That is the whole point of splitting the code into this many libraries. It is
# not modularity for its own sake: if the tracker can see the answer, every
# number the project produces is worthless. A convention would be forgotten; a
# link error cannot be. CI additionally greps those directories for
# `#include "world/` (see .github/workflows/invariants.yml) to catch a
# header-only leak that would not reach the linker.
#
# Adding sat_world to any target between the two BOUNDARY markers below is a
# change to the project's core claim, not a build tweak.

# ---------------------------------------------------------------------------
# sat_add_module(<name> [SOURCES ...] [PUBLIC_DEPS ...] [PRIVATE_DEPS ...]
#                [DEFINES ...])
#
# Creates a STATIC library when the module has compiled sources and an INTERFACE
# library when it is still header-only or not yet started. Either way the target
# exists and the dependency edges are declared, so the graph is complete and
# enforceable from the very first checkpoint.
# ---------------------------------------------------------------------------
function(sat_add_module name)
    cmake_parse_arguments(ARG "" "" "SOURCES;PUBLIC_DEPS;PRIVATE_DEPS;DEFINES" ${ARGN})

    if(ARG_SOURCES)
        add_library(${name} STATIC ${ARG_SOURCES})
        set(vis PUBLIC)
    else()
        add_library(${name} INTERFACE)
        set(vis INTERFACE)
    endif()

    # Every module includes its siblings as "module/file.hpp" from src/, and the
    # public headers under include/ as "sat/file.hpp".
    target_include_directories(${name} ${vis}
        "${CMAKE_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/src"
    )

    if(ARG_PUBLIC_DEPS)
        target_link_libraries(${name} ${vis} ${ARG_PUBLIC_DEPS})
    endif()
    if(ARG_PRIVATE_DEPS AND ARG_SOURCES)
        target_link_libraries(${name} PRIVATE ${ARG_PRIVATE_DEPS})
    endif()
    if(ARG_DEFINES)
        target_compile_definitions(${name} ${vis} ${ARG_DEFINES})
    endif()
endfunction()

# ===========================================================================
# core — no dependencies. Units, frames, clock, RNG, arenas, hashing, timing.
# Everything else in the program is allowed to depend on this and on nothing
# else by default.
# ===========================================================================
sat_add_module(sat_core
    SOURCES
        src/core/rng.cpp
        src/core/arena.cpp
        src/core/profile.cpp
)

# ===========================================================================
# Simulation side — these modules know the ground truth, and that is fine.
# ===========================================================================

# world — emitters, motion algebra, background, shape masks.
sat_add_module(sat_world
    PUBLIC_DEPS sat_core
)

# scenario — TOML parsing and schema validation (design §7).
sat_add_module(sat_scenario
    PUBLIC_DEPS sat_core
)

# camera — exact-coverage splatting and the viewport query (design §9.2).
sat_add_module(sat_camera
    PUBLIC_DEPS sat_core sat_world
)

# degrade — atmosphere, noise, fixed pattern, jitter, platform motion (§9.3).
# Note it does NOT depend on world: it operates on a rendered buffer and on the
# boresight, never on emitter positions.
sat_add_module(sat_degrade
    PUBLIC_DEPS sat_core
)

# ===========================================================================
# ───────────────────────── INV-1 BOUNDARY BEGINS ─────────────────────────
# Nothing below this line may link sat_world, directly or transitively.
# ===========================================================================

# perception — median, morphology, summed-area tables, matched filter, CFAR,
# grouping, centroiding. Sees an image and nothing else.
sat_add_module(sat_perception
    PUBLIC_DEPS sat_core
)

# ai — ONNX wrappers and the classical fallbacks that stand in for them.
sat_add_module(sat_ai
    PUBLIC_DEPS sat_core
)

# tracking — Kalman, IMM, association, lifecycle.
sat_add_module(sat_tracking
    PUBLIC_DEPS sat_core Eigen3::Eigen
)

# search — probability grid and acquisition strategies.
sat_add_module(sat_search
    PUBLIC_DEPS sat_core
)

# plant — the gimbal model. It is a *model of the hardware*, used by the
# controller for feedforward, and it is also what the simulator steps. It must
# not know where the target is.
sat_add_module(sat_plant
    PUBLIC_DEPS sat_core
)

# control — PID + feedforward, mode FSM, SAT supervisor.
sat_add_module(sat_control
    PUBLIC_DEPS sat_core sat_tracking sat_search sat_plant
)

# ===========================================================================
# ────────────────────────── INV-1 BOUNDARY ENDS ──────────────────────────
# ===========================================================================

# engine — the per-frame orchestrator and the frame sources. This is the only
# module that legitimately touches both sides: it owns the world AND drives the
# tracker, and it is responsible for keeping truth on the metrics side of the
# wall (design §8.1: "truth — metrics only").
sat_add_module(sat_engine
    PUBLIC_DEPS sat_core sat_world sat_camera sat_degrade sat_scenario
                sat_perception sat_tracking sat_control sat_plant sat_search sat_ai
)

# metrics — centroiding/tracking error, compliance matrix, logs, reports.
# Depends on world because computing an error requires the true position; this
# is exactly the boundary INV-1 draws, and metrics is on the permitted side.
sat_add_module(sat_metrics
    PUBLIC_DEPS sat_core sat_world
)

# gui — the dashboard (design §12). Deferred: the engine is built headless-first
# with the triple-buffered snapshot seam already in place, so the dashboard
# attaches later without touching the simulation. See the roadmap note in
# docs/SAT-DESIGN.md §14.
sat_add_module(sat_gui
    PUBLIC_DEPS sat_core
)

# ---------------------------------------------------------------------------
# Optional dependencies wired in where they belong.
# ---------------------------------------------------------------------------
if(SAT_HAVE_OPENCV)
    # OpenCV reaches the engine for cv::VideoCapture only (design §4.2). It is
    # deliberately NOT linked into sat_perception: nothing in the 30 Hz hot loop
    # may call it.
    target_compile_definitions(sat_engine INTERFACE SAT_HAVE_OPENCV=1)
endif()
if(SAT_HAVE_ONNX)
    target_compile_definitions(sat_ai INTERFACE SAT_HAVE_ONNX=1)
endif()

# ===========================================================================
# Application executable.
# ===========================================================================
add_executable(sat-tracker
    src/app/main.cpp
)

target_link_libraries(sat-tracker PRIVATE
    sat_core
    sat_engine
    sat_metrics
    sat_gui
    sat_scenario
)

target_include_directories(sat-tracker PRIVATE
    "${CMAKE_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/src"
)
