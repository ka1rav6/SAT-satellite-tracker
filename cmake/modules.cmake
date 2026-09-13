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
        src/core/image.cpp
        src/core/arena.cpp
        src/core/profile.cpp
)

# ===========================================================================
# Simulation side — these modules know the ground truth, and that is fine.
# ===========================================================================

# world — emitters, motion algebra, background, shape masks.
sat_add_module(sat_world
    SOURCES
        src/world/motion_component.cpp
        src/world/world_builder.cpp
        src/world/motion_factory.cpp
    PUBLIC_DEPS sat_core sat_scenario
)

# scenario — TOML parsing and schema validation (design §7).
sat_add_module(sat_scenario
    SOURCES
        src/scenario/schema.cpp
        src/scenario/toml_loader.cpp
    PUBLIC_DEPS sat_core sat_world
)
target_link_libraries(sat_scenario PRIVATE tomlplusplus::tomlplusplus)

# camera — exact-coverage splatting and the viewport query (design §9.2).
sat_add_module(sat_camera
    SOURCES
        src/camera/splat.cpp
    PUBLIC_DEPS sat_core sat_world
)

# degrade — atmosphere, noise, fixed pattern, jitter, platform motion (§9.3).
# Note it does NOT depend on world: it operates on a rendered buffer and on the
# boresight, never on emitter positions.
sat_add_module(sat_degrade
    SOURCES
        src/degrade/noise.cpp
        src/degrade/sensor.cpp
        src/degrade/disturbance.cpp
    PUBLIC_DEPS sat_core sat_scenario sat_world
)

# ===========================================================================
# ───────────────────────── INV-1 BOUNDARY BEGINS ─────────────────────────
# Nothing below this line may link sat_world, directly or transitively.
# ===========================================================================

# perception — median, morphology, summed-area tables, matched filter, CFAR,
# grouping, centroiding. Sees an image and nothing else.
sat_add_module(sat_perception
    SOURCES
        src/perception/simple_detector.cpp
        src/perception/median.cpp
        src/perception/morphology.cpp
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
    SOURCES
        src/engine/video_probe.cpp
        src/engine/synthetic_source.cpp
        src/engine/pipeline.cpp
        src/engine/snapshot.cpp
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
if(SAT_HAVE_GLFW)
    sat_add_module(sat_gui
        SOURCES
            src/gui/gl_texture.cpp
            src/gui/dashboard.cpp
        PUBLIC_DEPS sat_core sat_engine sat_scenario sat_imgui
    )
    target_compile_definitions(sat_gui
        PUBLIC  SAT_HAVE_GUI=1
        PRIVATE SAT_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios"
    )
else()
    # Declared but empty, so the application links the same way either
    # way and `--gui` reports a clear message instead of failing to build.
    sat_add_module(sat_gui
        PUBLIC_DEPS sat_core
    )
endif()

# ===========================================================================
# INV-1, checked by the build system itself.
#
# The boundary above is a convention about where lines are typed. This makes it
# a *configure error*: it walks each tracker-side target's transitive link
# closure and fails if sat_world is reachable through any path.
#
# Why bother, when the linker already enforces it? Because the linker only
# complains once someone actually calls a world symbol. Adding
# `target_link_libraries(sat_perception PUBLIC sat_world)` on its own builds
# perfectly happily, and the invariant is then silently gone — the next person
# to reach for ground truth finds it available. This catches the moment the
# edge is added, not the moment it is used, and it catches an edge added
# through an intermediate target, which reading modules.cmake by eye would not.
# ===========================================================================

# Collect a target's transitive link dependencies into <out_var>.
function(sat_collect_link_closure target out_var)
    set(seen "")
    set(pending "${target}")
    while(pending)
        list(POP_FRONT pending current)
        if(NOT TARGET ${current})
            continue()   # generator expression, system library, or a plain -lfoo
        endif()
        if(${current} IN_LIST seen)
            continue()
        endif()
        list(APPEND seen ${current})

        # INTERFACE libraries have no LINK_LIBRARIES, only the INTERFACE_ form;
        # static libraries have both. Read whichever exist.
        get_target_property(kind ${current} TYPE)
        set(edges "")
        if(NOT kind STREQUAL "INTERFACE_LIBRARY")
            get_target_property(direct ${current} LINK_LIBRARIES)
            if(direct)
                list(APPEND edges ${direct})
            endif()
        endif()
        get_target_property(iface ${current} INTERFACE_LINK_LIBRARIES)
        if(iface)
            list(APPEND edges ${iface})
        endif()

        foreach(dep IN LISTS edges)
            # Skip generator expressions; none of the SAT edges use them, and
            # evaluating them at configure time is not possible anyway.
            if(NOT dep MATCHES "^\\$<")
                list(APPEND pending ${dep})
            endif()
        endforeach()
    endwhile()
    set(${out_var} "${seen}" PARENT_SCOPE)
endfunction()

# Fail configure if `forbidden` is reachable from `target`.
function(sat_assert_no_link target forbidden)
    sat_collect_link_closure(${target} closure)
    if(${forbidden} IN_LIST closure)
        message(FATAL_ERROR
            "INV-1 VIOLATION: target '${target}' links '${forbidden}'.\n"
            "  The tracker must not be able to read the simulator's ground truth.\n"
            "  Transitive closure was: ${closure}\n"
            "  If a module genuinely needs truth, it belongs in sat_engine or\n"
            "  sat_metrics, which are on the permitted side of the boundary.\n"
            "  See docs/SAT-DESIGN.md §2 INV-1 and §5.1.")
    endif()
endfunction()

foreach(tracker_side IN ITEMS
        sat_perception sat_ai sat_tracking sat_search sat_control sat_plant)
    sat_assert_no_link(${tracker_side} sat_world)
    # sat_camera and sat_scenario also carry simulation state: the camera knows
    # emitter positions, and the scenario holds the true initial target location
    # (spec row 11). Neither belongs on the tracker side either.
    sat_assert_no_link(${tracker_side} sat_camera)
endforeach()
message(STATUS "SAT: INV-1 link boundary verified for 6 tracker-side targets")

# ---------------------------------------------------------------------------
# Optional dependencies wired in where they belong.
# ---------------------------------------------------------------------------
if(SAT_HAVE_OPENCV)
    # OpenCV reaches the engine for cv::VideoCapture only (design §4.2). It is
    # deliberately NOT linked into sat_perception: nothing in the 30 Hz hot loop
    # may call it, because cv::Mat allocates (violating INV-4) and OpenCV's
    # runtime SIMD dispatch can differ across machines (risking INV-3).
    #
    # Only the three modules we actually use are linked. Pulling in all of
    # OpenCV_LIBS would drag dnn, ml and photo into the binary for nothing, and
    # design §15 budgets ~60 MB for the whole Windows drop.
    target_compile_definitions(sat_engine PUBLIC SAT_HAVE_OPENCV=1)
    target_include_directories(sat_engine SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(sat_engine PUBLIC
        opencv_core        # cv::Mat, only at the decode boundary
        opencv_videoio     # cv::VideoCapture -- the reason OpenCV is here at all
        opencv_imgproc     # cvtColor to greyscale at decode (design §8.3 req 2)
    )

    # The test-oracle target. Design §16 requires our own kernels to be
    # bit-identical to cv::medianBlur, cv::morphologyEx and
    # cv::connectedComponents on 1000+ random inputs. That comparison lives in
    # the TEST binaries, which must therefore see OpenCV -- while the shipping
    # perception library still must not.
    add_library(sat_cv_oracle INTERFACE)
    target_compile_definitions(sat_cv_oracle INTERFACE SAT_HAVE_OPENCV=1)
    target_include_directories(sat_cv_oracle SYSTEM INTERFACE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(sat_cv_oracle INTERFACE opencv_core opencv_imgproc)
else()
    # A stand-in so test targets can link it unconditionally. Tests that need an
    # oracle skip themselves at runtime when SAT_HAVE_OPENCV is not defined.
    add_library(sat_cv_oracle INTERFACE)
endif()
if(SAT_HAVE_ONNX)
    target_compile_definitions(sat_ai INTERFACE SAT_HAVE_ONNX=1)
endif()

# ===========================================================================
# Application executable.
# ===========================================================================
add_executable(sat-tracker
    src/app/main.cpp
    src/app/verify_repro.cpp
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

# The default scenario --gui opens with. An installed build overrides this to
# the installed share/ path; for a developer build, pointing at the source tree
# is what makes `just gui` work with no arguments.
target_compile_definitions(sat-tracker PRIVATE
    SAT_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios"
)
