# cmake/modules.cmake
#
# Declares every SAT library target and the dependency graph from design §5.1.
# INV-1: perception / ai / tracking / search / control / plant must NEVER link
# sat_world. Anything that needs ground truth goes through sat_engine / sat_metrics.
#
# At checkpoint 0.1 most targets are INTERFACE stubs. Real sources land in later
# checkpoints; keeping the graph in place now means INV-1 is enforced from day one.

# ---------------------------------------------------------------------------
# Helper: create an INTERFACE library that will grow into a real library later.
# ---------------------------------------------------------------------------
function(sat_add_interface_lib name)
    add_library(${name} INTERFACE)
    target_include_directories(${name} INTERFACE
        "${CMAKE_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/src"
    )
endfunction()

# ---------------------------------------------------------------------------
# Core — no deps. Shared types, units, RNG, arenas, etc.
# ---------------------------------------------------------------------------
sat_add_interface_lib(sat_core)

# ---------------------------------------------------------------------------
# Simulation-side libraries (may know about the world)
# ---------------------------------------------------------------------------
sat_add_interface_lib(sat_world)
target_link_libraries(sat_world INTERFACE sat_core)

sat_add_interface_lib(sat_scenario)
target_link_libraries(sat_scenario INTERFACE sat_core)

sat_add_interface_lib(sat_camera)
target_link_libraries(sat_camera INTERFACE sat_core sat_world)

sat_add_interface_lib(sat_degrade)
target_link_libraries(sat_degrade INTERFACE sat_core)

# ---------------------------------------------------------------------------
# INV-1 BOUNDARY — nothing below may link sat_world
# ---------------------------------------------------------------------------
sat_add_interface_lib(sat_perception)
target_link_libraries(sat_perception INTERFACE sat_core)

sat_add_interface_lib(sat_ai)
target_link_libraries(sat_ai INTERFACE sat_core)

sat_add_interface_lib(sat_tracking)
target_link_libraries(sat_tracking INTERFACE sat_core)

sat_add_interface_lib(sat_search)
target_link_libraries(sat_search INTERFACE sat_core)

sat_add_interface_lib(sat_plant)
target_link_libraries(sat_plant INTERFACE sat_core)

sat_add_interface_lib(sat_control)
target_link_libraries(sat_control INTERFACE sat_core sat_tracking sat_search sat_plant)
# ---------------------------------------------------------------------------
# end INV-1 boundary
# ---------------------------------------------------------------------------

sat_add_interface_lib(sat_engine)
target_link_libraries(sat_engine INTERFACE
    sat_core sat_world sat_camera sat_degrade
    sat_perception sat_tracking sat_control sat_ai
)

sat_add_interface_lib(sat_metrics)
target_link_libraries(sat_metrics INTERFACE sat_core sat_world)

sat_add_interface_lib(sat_gui)
target_link_libraries(sat_gui INTERFACE sat_core)

# ---------------------------------------------------------------------------
# Application executable — the only thing CP 0.1 produces today.
# ---------------------------------------------------------------------------
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
)