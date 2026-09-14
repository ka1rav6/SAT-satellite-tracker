// metrics/report.hpp — CP 7.6.
//
// "JSON -> Jinja -> self-contained report.html with inlined plots. Accept when:
//  finishing a run produces a showable report with zero manual steps."
//
// ---------------------------------------------------------------------------
// A DEVIATION FROM THE DESIGN, AND WHY
// ---------------------------------------------------------------------------
// §14 specifies Jinja, which is a Python templating library. Using it would
// mean the shipping C++ binary cannot produce its own report: `--headless`
// would write run.json and then a separate Python step would have to run,
// with its own interpreter, its own dependency, and its own chance to be
// missing on the evaluator's machine.
//
// The checkpoint's acceptance criterion is "finishing a run produces a showable
// report with ZERO MANUAL STEPS", and a Python post-process is a manual step
// unless something invokes it — at which point the C++ binary depends on a
// Python installation, which §4's stack does not include and CP 15.5's "unzip
// and double-click" explicitly rules out.
//
// So the report is generated in C++. The cost is that the template is a string
// in a source file rather than a file a designer can edit; the benefit is that
// the criterion is actually met on a clean machine. This is a deliberate
// deviation, recorded here rather than silently taken.
//
// ---------------------------------------------------------------------------
// SELF-CONTAINED MEANS SELF-CONTAINED
// ---------------------------------------------------------------------------
// No CDN, no external CSS, no JavaScript library, no image files beside it. The
// plots are inline SVG generated from the data. The report has to survive being
// emailed as a single attachment, opened on a machine with no network, and
// still show its charts — which is exactly the situation a demo is in.

#pragma once

#include "metrics/collector.hpp"
#include "metrics/compliance.hpp"

#include <string>
#include <vector>

namespace sat {

/// One series to plot: a label and the per-frame values.
struct ReportTrace {
    std::string label;
    std::string units;
    std::vector<double> x;     ///< time, seconds
    std::vector<double> y;
    double budget = 0.0;       ///< draw a budget line at this y, 0 for none
    std::string budget_label;
};

struct ReportInput {
    std::string title = "SAT — run report";
    std::string build;
    std::string utc;
    std::string scenario_name;
    std::string scenario_description;

    RunMetrics  metrics{};
    Requirements requirements{};

    /// Per-frame traces, for the plots. Empty is fine: the report then shows
    /// the tables alone rather than failing.
    std::vector<ReportTrace> traces;

    /// The compliance matrix, if this report covers a sweep.
    std::string compliance_text;
};

/// Render the whole document. Returns HTML.
[[nodiscard]] std::string render_report(const ReportInput& in);

[[nodiscard]] bool write_report(const std::string& path, const ReportInput& in);

/// An inline SVG line chart. Exposed because it is the part with arithmetic in
/// it, and arithmetic that only runs inside a 400-line string template is
/// arithmetic nobody tests.
[[nodiscard]] std::string svg_line_chart(const ReportTrace& t, int w, int h);

}  // namespace sat
