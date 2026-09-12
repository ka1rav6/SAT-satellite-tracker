# Test suite root. Individual feature tests land in subdirectories as checkpoints
# land. CP 0.1 ships a smoke test so `ctest` / `just test` already work.

add_executable(sat_smoke_test
    smoke_test.cpp
)

target_include_directories(sat_smoke_test PRIVATE
    "${CMAKE_SOURCE_DIR}/include"
)

# Register with CTest. The test runs the smoke binary itself (not sat-tracker)
# so we can assert on the version string without shelling out.
add_test(NAME smoke_version COMMAND sat_smoke_test)

# Also run the real binary and require exit code 0 — catches link/runtime breaks.
add_test(NAME smoke_sat_tracker_runs COMMAND sat-tracker)