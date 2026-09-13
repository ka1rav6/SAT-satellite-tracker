// app/timestamp.hpp — the wall clock, kept where it is allowed to be.
//
// ---------------------------------------------------------------------------
// WHY THIS IS IN app/ AND NOT IN metrics/, WHERE IT WAS FIRST WRITTEN
// ---------------------------------------------------------------------------
// INV-3 forbids the simulation from reading a wall clock, and
// tools/check_source_invariants.py enforces it by scanning every module except
// core/profile.hpp (the sanctioned timing exception), src/app and src/gui.
//
// utc_timestamp_now() was originally a free function in metrics/centroid_log,
// with a comment arguing that a provenance timestamp in a log header is not
// simulation state because it never feeds back into a computed value. That
// argument is true and it was still the wrong call: the checker went red, and
// the choice was then between widening the invariant and moving five lines.
//
// Widening it would have been the worse trade by a long way. sat_metrics is
// linked into the same graph as everything else, and once <chrono> is
// sanctioned there, the next person adding a metric has no mechanical reason
// not to reach for a real clock inside one — which is exactly the class of bug
// INV-3 exists to make impossible. A rule with a list of exceptions is only as
// strong as the shortest entry on that list.
//
// Stamping an artifact with the time it was produced is an APPLICATION
// concern, not a metrics one: the same header could be filled from a --utc
// flag, from CI's build timestamp, or left blank, and nothing about the
// measurement would change. So the caller supplies it, and the only code that
// reads a clock lives where reading a clock is already allowed.

#pragma once

#include <string>

namespace sat {

/// ISO-8601 UTC, for a log or report header. Reads the system clock.
[[nodiscard]] std::string utc_timestamp_now();

}  // namespace sat
