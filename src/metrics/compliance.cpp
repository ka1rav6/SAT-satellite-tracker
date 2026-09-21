// metrics/compliance.cpp

#include "metrics/compliance.hpp"
#include "metrics/series.hpp"

#include <algorithm>
#include <cstdio>

namespace sat {

namespace {

/// Collect one field across a group of runs, so it can be summarised.
template <typename F>
Series field(const std::vector<RunMetrics>& runs, F get) {
    Series s;
    s.reserve(runs.size());
    for (const RunMetrics& m : runs) s.push(get(m));
    return s;
}

std::string row(const char* spec_row, const char* name, const char* units,
                double value, double tail, const char* tail_name,
                const char* status) {
    char b[256];
    std::snprintf(b, sizeof b, "%-4s %-24s %8.3f %-6s %s %8.3f   %s\n",
                  spec_row, name, value, units, tail_name, tail, status);
    return b;
}

// ---------------------------------------------------------------------------
// no_data — a row with nothing behind it.
//
// A metric computed over zero samples returns 0.0, and 0.0 compared against
// "must be below 10" is a PASS. The first sweep printed exactly that: a
// condition in which the beacon was never detected reported "Centroiding 0.000
// px PASS" and "Target loss 0.000% PASS", which is the most flattering possible
// rendering of a total failure.
//
// So a row whose denominator is zero says so and carries NO verdict. An absent
// measurement is not a passing one, and the distinction has to survive into the
// rendered table because the table is what anyone actually reads.
// ---------------------------------------------------------------------------
std::string no_data(const char* spec_row, const char* name, const char* why) {
    char b[256];
    std::snprintf(b, sizeof b, "%-4s %-24s %8s %-6s %s\n",
                  spec_row, name, "n/a", "", why);
    return b;
}

}  // namespace

// ---------------------------------------------------------------------------
// verdict
//
// PASS      the tail is inside the limit — not just the central figure. A
//           requirement met on average and missed one run in twenty is not met.
// MARGINAL  the central figure is inside, the tail is not. §13.3: "An honest
//           MARGINAL is stronger than a suspicious all-green."
// FAIL      the central figure is outside.
// ---------------------------------------------------------------------------
const char* verdict(double value, double tail, double limit,
                    bool higher_is_better) noexcept {
    const bool value_ok = higher_is_better ? (value >= limit) : (value <= limit);
    const bool tail_ok  = higher_is_better ? (tail  >= limit) : (tail  <= limit);
    if (value_ok && tail_ok) return "PASS";
    if (value_ok)            return "MARGINAL";
    return "FAIL";
}

std::string compliance_matrix(const std::vector<RunMetrics>& overall,
                              const std::vector<ConditionGroup>& groups,
                              const Requirements& req,
                              const std::string& provenance) {
    std::string out;
    char b[512];

    std::snprintf(b, sizeof b, "REQUIREMENT COMPLIANCE MATRIX     %s\n\n",
                  provenance.c_str());
    out += b;
    out += "Row  Requirement                 Value        Tail       Status\n";
    out += "--------------------------------------------------------------------\n";

    auto emit = [&](const std::vector<RunMetrics>& runs, const std::string& prefix) {
        if (runs.empty()) return;

        // --- row 16, twice -------------------------------------------------
        // §10.5: report acquisition_in_fov_s and acquisition_cold_s separately
        // and label them. A single number here would be either a claim the
        // geometry forbids or a needlessly bad one.
        {
            std::vector<RunMetrics> acquired;
            for (const RunMetrics& m : runs) if (m.acquired_in_fov) acquired.push_back(m);
            if (!acquired.empty()) {
                const Series s = field(acquired, [](const RunMetrics& m) {
                    return m.acquisition_in_fov_s; });
                out += prefix;
                out += row("16", "Acquisition (in view)", "s", s.mean(), s.p95(), "p95",
                           verdict(s.mean(), s.p95(), req.acquisition_s, false));
            }
            const Series cold = field(runs, [](const RunMetrics& m) {
                return m.acquired ? m.acquisition_cold_s : 0.0; });
            out += prefix;
            std::snprintf(b, sizeof b,
                          "%-4s %-24s %8.3f %-6s %s %8.3f   %s\n",
                          "16", "Acquisition (cold)", cold.mean(), "s", "p95", cold.p95(),
                          "BOUND DERIVED - design 10.5");
            out += b;
        }

        // --- row 17 ---------------------------------------------------------
        {
            std::vector<RunMetrics> scored;
            for (const RunMetrics& m : runs) if (m.tracking_frames > 0) scored.push_back(m);
            out += prefix;
            if (scored.empty()) {
                out += no_data("17", "Tracking error", "the track was never Confirmed");
            } else {
                const Series s = field(scored, [](const RunMetrics& m) { return m.tracking_rms_px; });
                const Series t = field(scored, [](const RunMetrics& m) { return m.tracking_p95_px; });
                if (req.tracking_floor_px > req.tracking_error_px) {
                    // The requirement is unreachable under the specified
                    // disturbance. Say so, with the floor, rather than printing
                    // FAIL — see Requirements::tracking_floor_px.
                    char note[96];
                    std::snprintf(note, sizeof note,
                                  "BOUND DERIVED - jitter floor %.2f px", req.tracking_floor_px);
                    out += row("17", "Tracking error", "px", s.mean(), t.p95(), "p95", note);
                } else {
                    out += row("17", "Tracking error", "px", s.mean(), t.p95(), "p95",
                               verdict(s.mean(), t.p95(), req.tracking_error_px, false));
                }
            }
        }

        // --- row 18 ---------------------------------------------------------
        //
        // GRADED ON THE POST-ACQUISITION DEFINITION — P0-2.
        //
        // The in-FOV definition normalises over frames where the beacon was
        // visible, which is the right denominator for "did the detector hold
        // what it could see" and the wrong one for spec row 18. A run that
        // never finds the beacon has almost no in-FOV frames, so a handful of
        // lucky ones set the ratio: the old baseline scenario scored 7.59 %
        // and PASSED row 18 on a run where the mount pointed at empty sky for
        // 95.6 % of its length.
        //
        // The post-acquisition figure normalises over every frame from the
        // first lock onward. It cannot be flattered by failing to acquire,
        // because failing to acquire leaves it undefined rather than small.
        // Both are printed — the in-FOV one as an unGRADED companion row —
        // so the difference is visible instead of being a silent redefinition.
        {
            std::vector<RunMetrics> scored;
            for (const RunMetrics& m : runs) if (m.post_acq_valid) scored.push_back(m);
            out += prefix;
            if (scored.empty()) {
                out += no_data("18", "Target loss", "the track was never Confirmed");
            } else {
                const Series s = field(scored, [](const RunMetrics& m) {
                    return 100.0 * m.target_loss_post_acq; });
                out += row("18", "Target loss (post-acq)", "%", s.mean(), s.p95(), "p95",
                           verdict(s.mean(), s.p95(), 100.0 * req.target_loss_frac, false));
            }
            std::vector<RunMetrics> in_fov;
            for (const RunMetrics& m : runs) if (m.frames_in_fov > 0) in_fov.push_back(m);
            if (!in_fov.empty()) {
                const Series s = field(in_fov, [](const RunMetrics& m) {
                    return 100.0 * m.target_loss_frac; });
                out += prefix;
                out += row("18", "Target loss (in-FOV)", "%", s.mean(), s.p95(), "p95",
                           "reported — row 18 is graded on post-acq above");
            }
        }

        // --- FOV containment: the PS's own objective ------------------------
        //
        // Not a numbered spec row — the PS states it as prose rather than in
        // the table — but it is the sentence the whole problem statement is
        // built on: "first locate and MAINTAIN the remote terminal within its
        // camera Field-of-View". A compliance matrix that grades five numbered
        // rows and never reports the objective they exist to serve is
        // measuring the proxy instead of the thing.
        //
        // No threshold, so no verdict: the PS gives none, and inventing one
        // would be this project asserting its own pass mark. Reported as a
        // number, which is what makes the row-18 pair above interpretable.
        {
            std::vector<RunMetrics> scored;
            for (const RunMetrics& m : runs) if (m.post_acq_valid) scored.push_back(m);
            if (!scored.empty()) {
                const Series s = field(scored, [](const RunMetrics& m) {
                    return 100.0 * m.fov_containment_post_acq; });
                out += prefix;
                out += row("--", "FOV containment (post-acq)", "%", s.mean(), s.p95(), "p95",
                           "PS objective - reported, not graded");
            }
        }

        // --- row 19 ---------------------------------------------------------
        {
            std::vector<RunMetrics> with;
            for (const RunMetrics& m : runs) if (m.reacquisitions > 0) with.push_back(m);
            if (!with.empty()) {
                const Series s = field(with, [](const RunMetrics& m) {
                    return m.reacquisition_mean_s; });
                const Series t = field(with, [](const RunMetrics& m) {
                    return m.reacquisition_max_s; });
                out += prefix;
                out += row("19", "Re-acquisition", "s", s.mean(), t.p95(), "p95",
                           verdict(s.mean(), t.p95(), req.reacquisition_s, false));
            } else {
                out += prefix;
                std::snprintf(b, sizeof b, "%-4s %-24s %8s\n",
                              "19", "Re-acquisition", "no episodes");
                out += b;
            }
        }

        // --- row 20 ---------------------------------------------------------
        // Higher is better, so the tail that matters is the LOW one: fps
        // derived from the 95th-percentile frame, not from the median.
        {
            const Series s = field(runs, [](const RunMetrics& m) { return m.fps_mean; });
            const Series t = field(runs, [](const RunMetrics& m) { return m.fps_p5; });
            out += prefix;
            out += row("20", "Processing speed", "fps", s.mean(), t.p5(), "p5 ",
                       verdict(s.mean(), t.p5(), req.min_fps, true));
        }

        // --- the graded metric ----------------------------------------------
        // Not a specification row; §3.1's 60%. Both frames, because §13.1's
        // screen figure carries pointing error the detector cannot influence
        // (INV-6) and the image figure is the detector alone.
        {
            std::vector<RunMetrics> scored;
            for (const RunMetrics& m : runs) if (m.centroid_frames > 0) scored.push_back(m);
            if (scored.empty()) {
                out += prefix;
                out += no_data("--", "Centroiding", "no frame had a detection with the "
                                                    "beacon in view");
                return;
            }
            const Series im = field(scored, [](const RunMetrics& m) {
                return m.centroid_rmse_image_px; });
            const Series sc = field(scored, [](const RunMetrics& m) {
                return m.centroid_rmse_screen_px; });
            const Series bias = field(scored, [](const RunMetrics& m) {
                return std::hypot(m.centroid_bias_x_px, m.centroid_bias_y_px); });
            out += prefix;
            out += row("--", "Centroiding (image)", "px", im.mean(), im.p95(), "p95",
                       verdict(im.mean(), im.p95(), req.centroid_rmse_px, false));
            out += prefix;
            std::snprintf(b, sizeof b, "%-4s %-24s %8.3f %-6s %s %8.3f   %s\n",
                          "--", "Centroiding (screen)", sc.mean(), "px", "p95", sc.p95(),
                          "carries pointing error - INV-6");
            out += b;
            out += prefix;
            std::snprintf(b, sizeof b, "%-4s %-24s %8.4f %-6s\n",
                          "--", "Centroiding bias", bias.mean(), "px");
            out += b;
        }
    };

    emit(overall, "");

    // --- per condition, §13.3's "broken out per condition" ------------------
    // The reason this is not optional: a system excellent in clear air and
    // broken in fog has a perfectly acceptable average, and the average is the
    // only thing the block above shows.
    if (groups.size() > 1) {
        out += "\nBroken out per condition:\n";
        for (const ConditionGroup& g : groups) {
            std::snprintf(b, sizeof b, "\n  [%s]  %zu runs\n",
                          g.label.c_str(), g.runs.size());
            out += b;
            emit(g.runs, "  ");
        }
    }
    return out;
}

}  // namespace sat
