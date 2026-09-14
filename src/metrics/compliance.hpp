// metrics/compliance.hpp — the ★ CP 7.7 gate.
//
// "compliance matrix generated from a sweep, broken out per condition. Accept
//  when: --sweep over 500 runs prints the matrix with a status against spec
//  rows 16-20 plus centroiding error. DO NOT PROCEED UNTIL THIS WORKS."
//
// ---------------------------------------------------------------------------
// WHAT MAKES THIS A GATE RATHER THAN A REPORT
// ---------------------------------------------------------------------------
// Every number before Stage 7 came from one run of one scenario. This is the
// first thing in the project that states a requirement, a measurement over a
// DISTRIBUTION, and a verdict — and having it means that from here on, a change
// either moves those numbers or it does not, and the question is settled by
// running it rather than by arguing.
//
// Two design rules, both from §13.3:
//
//   "An honest MARGINAL is stronger than a suspicious all-green." A status is
//   PASS, MARGINAL or FAIL, and MARGINAL exists because a mean inside budget
//   with a p95 outside it is a real and different situation from both of the
//   others. Collapsing it into PASS is how a system ships with a known
//   problem nobody wrote down.
//
//   "broken out per condition". A single aggregate row hides the case that
//   matters: a system that is excellent in clear air and broken in fog has a
//   fine average. Every sweep axis value gets its own row.
//
// And one from §10.5, which the matrix has to carry because the specification
// and the geometry disagree: acquisition is reported TWICE, cold and in-view,
// with the cold row marked as a derived bound rather than a failure.

#pragma once

#include "metrics/collector.hpp"

#include <string>
#include <vector>

namespace sat {

// ---------------------------------------------------------------------------
// Requirements — spec rows 16-20, as the matrix checks them.
// ---------------------------------------------------------------------------
struct Requirements {
    double acquisition_s     = 2.0;    ///< row 16
    double tracking_error_px = 10.0;   ///< row 17
    double target_loss_frac  = 0.05;   ///< row 18
    double reacquisition_s   = 1.0;    ///< row 19
    double min_fps           = 20.0;   ///< row 20

    /// Not a specification row — the graded metric of §3.1, 60% of BP-1/BP-2.
    /// A budget rather than a requirement, and labelled as such in the matrix.
    double centroid_rmse_px  = 1.0;

    // -----------------------------------------------------------------------
    // A DERIVED FLOOR on row 17, when the configuration makes one.
    //
    // Rows 17 and 23 are in tension: row 17 caps tracking error at 10 px, row
    // 23 specifies up to +/-20 px/frame of jitter applied to the TRUE boresight
    // and invisible to the encoder. Uniform[-A,A] has variance A^2/3, so over
    // two axes the magnitude RMS is sqrt(2 A^2/3) = 16.33 px at A = 20 — a
    // floor no controller can go below, since the disturbance displaces the
    // boresight after the command has been issued.
    //
    // When the floor exceeds the requirement, the row is reported as BOUND
    // DERIVED rather than FAIL, exactly as §10.5's cold acquisition is, and the
    // floor is printed beside it. Marking it FAIL would be misleading in the
    // other direction: it reads as a defect in the loop when it is a property
    // of the specification, and the loop's own contribution (0.8 px over the
    // floor, measured) would be invisible either way.
    //
    // 0 means "no floor applies" — a video-mode run, or a scenario with the
    // disturbance disabled. The caller computes it; see app/sweep.cpp.
    // -----------------------------------------------------------------------
    double tracking_floor_px = 0.0;
};

/// One row's worth of measurements: every run that shares a condition.
struct ConditionGroup {
    std::string             label;
    std::vector<RunMetrics> runs;
};

/// Render the matrix. `overall` is every run; `groups` are the per-condition
/// breakouts §13.3 asks for.
[[nodiscard]] std::string compliance_matrix(const std::vector<RunMetrics>& overall,
                                            const std::vector<ConditionGroup>& groups,
                                            const Requirements& req,
                                            const std::string& provenance);

/// PASS / MARGINAL / FAIL for one requirement. Exposed so the rule is testable
/// on its own rather than only through a rendered table.
///
/// `value` is the central figure, `tail` the p95 (or p5 where small is the
/// failure). Lower-is-better unless `higher_is_better`.
[[nodiscard]] const char* verdict(double value, double tail, double limit,
                                  bool higher_is_better) noexcept;

}  // namespace sat
