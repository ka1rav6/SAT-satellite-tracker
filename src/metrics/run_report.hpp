// metrics/run_report.hpp — CP 7.4. `run.json`.
//
// "Contains every metric plus the full config and build hash."
//
// ---------------------------------------------------------------------------
// WHY A RUN IS WORTHLESS WITHOUT ITS PROVENANCE
// ---------------------------------------------------------------------------
// The compliance matrix at CP 7.7 is aggregated from hundreds of these files,
// and the report at CP 7.6 is generated from them. Both are claims about what
// this software does. A claim you cannot reproduce is an anecdote, and INV-3
// exists precisely so that these numbers are not anecdotes — but bit-exact
// reproducibility is only useful if the file says WHAT to re-run.
//
// So every run.json carries, alongside the metrics:
//
//   the build hash        which code produced this
//   the seed              which random draw
//   the full scenario     every §7.1 key, echoed — not a path to a file that
//                         may since have changed
//   the AI state          INV-7's disclosure: was a model in the loop
//   the frame fingerprint the INV-3 hash, so a re-run can be checked rather
//                         than assumed
//
// The scenario is echoed in full rather than referenced. A path is not
// provenance: `scenarios/baseline.toml` today is not necessarily
// `scenarios/baseline.toml` next week, and a results directory that silently
// depends on the working tree is exactly the trap this is avoiding.

#pragma once

#include "metrics/collector.hpp"
#include "scenario/scenario.hpp"

#include <string>

namespace sat {

/// Serialise one run. `fingerprint_hash` is the combined INV-3 hash over every
/// frame, or 0 if snapshots were not published.
[[nodiscard]] std::string run_json(const RunMetrics& m,
                                   const Scenario& sc,
                                   const std::string& build_hash,
                                   uint64_t fingerprint_hash);

/// Write it. Returns false if the path cannot be opened, which the caller must
/// report rather than swallow.
[[nodiscard]] bool write_run_json(const std::string& path,
                                  const RunMetrics& m,
                                  const Scenario& sc,
                                  const std::string& build_hash,
                                  uint64_t fingerprint_hash);

}  // namespace sat
