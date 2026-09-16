// core/mode.hpp — the system mode enum (design §10.4).
//
// ---------------------------------------------------------------------------
// WHY THIS IS IN core/ AND NOT WHERE IT STARTED
// ---------------------------------------------------------------------------
// TrackMode was originally declared in engine/snapshot.hpp, with a comment
// explaining that it lived there because "the snapshot and the centroid log
// both need to name it, and neither may depend on the controller". The premise
// was right and the conclusion was wrong: the set of modules that must name a
// mode is larger than the set that may depend on the engine.
//
// CP 6.6 made that concrete. control/mode_fsm.hpp is the code that PRODUCES the
// value, and sat_control cannot include engine/snapshot.hpp — sat_engine links
// sat_world, so that include would put ground truth inside the controller's
// dependency closure and cmake/modules.cmake fails the configure outright.
// INV-1 caught it before a line of it compiled, which is what it is for.
//
// A plain enum with no dependencies belongs at the bottom of the graph, where
// everything can see it and it can see nothing. engine/snapshot.hpp includes
// this header, so every existing use of the name keeps working unchanged.

#pragma once

#include <cstdint>

namespace sat {

/// Mode FSM state (design §10.4). Values are logged in centroid.csv and hashed
/// into the reproducibility fingerprint, so they are fixed.
enum class TrackMode : uint8_t {
  Idle = 0,
  Search,
  Detect,
  Acquire,
  Track,
  Reacquire,
  Handover,
  Safe
};

[[nodiscard]] const char *track_mode_name(TrackMode m) noexcept;

} // namespace sat
