// metrics/report.cpp

#include "metrics/report.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace sat {

namespace {

/// Escape text that goes into HTML. Scenario names and descriptions come from
/// a TOML file the evaluators may supply, so they are untrusted input as far as
/// this document is concerned — an unescaped '<' would silently break the page.
std::string esc(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        switch (c) {
            case '&': out += "&amp;";  break;
            case '<': out += "&lt;";   break;
            case '>': out += "&gt;";   break;
            case '"': out += "&quot;"; break;
            default:  out += c;        break;
        }
    }
    return out;
}

std::string fmt(const char* f, double v) {
    char b[64];
    std::snprintf(b, sizeof b, f, v);
    return b;
}

/// The same, for the rows whose "target" column is a sentence about frame
/// counts rather than a single number ("583 / 600 frames after first lock").
/// Separate from fmt() rather than a variadic template so that the format
/// string stays a literal at the call site and -Wformat-security keeps
/// working; there are only two shapes needed and this is one of them.
std::string fmt2(const char* f, long long a, long long b_) {
    char b[96];
    std::snprintf(b, sizeof b, f, a, b_);
    return b;
}

/// A metric row with a status pill.
std::string metric_row(const std::string& name, const std::string& value,
                       const std::string& requirement, const char* status) {
    std::string cls = "na";
    if (std::string(status) == "PASS")     cls = "pass";
    else if (std::string(status) == "FAIL") cls = "fail";
    else if (std::string(status) == "MARGINAL") cls = "marginal";
    return "<tr><td>" + esc(name) + "</td><td class=\"num\">" + esc(value) +
           "</td><td class=\"req\">" + esc(requirement) +
           "</td><td><span class=\"pill " + cls + "\">" + esc(status) +
           "</span></td></tr>\n";
}

}  // namespace

// ---------------------------------------------------------------------------
// svg_line_chart
//
// Hand-rolled rather than pulled from a charting library, because §4's stack
// does not include one and because "self-contained" rules out a CDN. It is
// about sixty lines and the only interesting part is the axis rounding.
// ---------------------------------------------------------------------------
std::string svg_line_chart(const ReportTrace& t, int w, int h) {
    std::ostringstream s;
    const int pad_l = 56, pad_r = 14, pad_t = 12, pad_b = 28;
    const double plot_w = w - pad_l - pad_r;
    const double plot_h = h - pad_t - pad_b;

    s << "<svg viewBox=\"0 0 " << w << " " << h << "\" class=\"chart\" "
         "preserveAspectRatio=\"none\" role=\"img\" aria-label=\"" << esc(t.label) << "\">";

    if (t.x.size() < 2) {
        s << "<text x=\"" << w / 2 << "\" y=\"" << h / 2
          << "\" class=\"empty\" text-anchor=\"middle\">no data</text></svg>";
        return s.str();
    }

    const double x0 = t.x.front(), x1 = t.x.back();
    double y1 = *std::max_element(t.y.begin(), t.y.end());
    if (t.budget > y1) y1 = t.budget;
    // A flat trace at zero would otherwise divide by zero and draw nothing.
    if (!(y1 > 0.0)) y1 = 1.0;
    y1 *= 1.08;                          // headroom so the peak is not on the frame

    auto px = [&](double x) { return pad_l + plot_w * (x - x0) / std::max(1e-9, x1 - x0); };
    auto py = [&](double y) { return pad_t + plot_h * (1.0 - y / y1); };

    // --- grid and y labels ------------------------------------------------
    for (int i = 0; i <= 4; ++i) {
        const double v = y1 * i / 4.0;
        const double y = py(v);
        s << "<line class=\"grid\" x1=\"" << pad_l << "\" y1=\"" << y
          << "\" x2=\"" << (w - pad_r) << "\" y2=\"" << y << "\"/>";
        s << "<text class=\"tick\" x=\"" << (pad_l - 6) << "\" y=\"" << (y + 3)
          << "\" text-anchor=\"end\">" << fmt("%.3g", v) << "</text>";
    }
    s << "<text class=\"tick\" x=\"" << pad_l << "\" y=\"" << (h - 8) << "\">"
      << fmt("%.1f", x0) << " s</text>";
    s << "<text class=\"tick\" x=\"" << (w - pad_r) << "\" y=\"" << (h - 8)
      << "\" text-anchor=\"end\">" << fmt("%.1f", x1) << " s</text>";

    // --- the budget line, if there is one ---------------------------------
    // Drawn UNDER the trace, so a trace sitting on its budget is still visible.
    if (t.budget > 0.0 && t.budget < y1) {
        const double y = py(t.budget);
        s << "<line class=\"budget\" x1=\"" << pad_l << "\" y1=\"" << y
          << "\" x2=\"" << (w - pad_r) << "\" y2=\"" << y << "\"/>";
        s << "<text class=\"budget-label\" x=\"" << (w - pad_r - 4) << "\" y=\"" << (y - 4)
          << "\" text-anchor=\"end\">" << esc(t.budget_label) << "</text>";
    }

    // --- the trace ---------------------------------------------------------
    s << "<polyline class=\"trace\" points=\"";
    for (size_t i = 0; i < t.x.size() && i < t.y.size(); ++i) {
        if (i) s << ' ';
        s << fmt("%.1f", px(t.x[i])) << ',' << fmt("%.1f", py(t.y[i]));
    }
    s << "\"/></svg>";
    return s.str();
}

// ---------------------------------------------------------------------------
// render_report
// ---------------------------------------------------------------------------
std::string render_report(const ReportInput& in) {
    const RunMetrics& m = in.metrics;
    const Requirements& q = in.requirements;
    std::ostringstream s;

    s << "<!doctype html>\n<html lang=\"en\"><head><meta charset=\"utf-8\">\n"
         "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
         "<title>" << esc(in.title) << "</title>\n<style>\n"
         // Deliberately plain and print-friendly. This gets opened on an
         // unfamiliar machine, possibly projected, possibly printed.
         ":root{--fg:#1a1d21;--muted:#6b7280;--line:#e3e6ea;--bg:#fff;"
         "--pass:#12805c;--fail:#b4232c;--marg:#9a6700;--accent:#2b5fd9}\n"
         "@media (prefers-color-scheme:dark){:root{--fg:#e6e8ea;--muted:#9aa3ad;"
         "--line:#2b3138;--bg:#15181c;--pass:#3fb07f;--fail:#e8636d;--marg:#d5a138;"
         "--accent:#6f9bff}}\n"
         "*{box-sizing:border-box}body{margin:0;padding:32px;background:var(--bg);"
         "color:var(--fg);font:14px/1.55 ui-sans-serif,system-ui,-apple-system,"
         "'Segoe UI',Roboto,sans-serif;max-width:1000px;margin-inline:auto}\n"
         "h1{font-size:22px;margin:0 0 4px}h2{font-size:15px;margin:32px 0 10px;"
         "text-transform:uppercase;letter-spacing:.06em;color:var(--muted)}\n"
         ".sub{color:var(--muted);margin:0 0 6px}\n"
         "table{border-collapse:collapse;width:100%;margin:8px 0 4px}\n"
         "th,td{text-align:left;padding:6px 10px;border-bottom:1px solid var(--line)}\n"
         "th{font-weight:600;color:var(--muted);font-size:12px;text-transform:uppercase;"
         "letter-spacing:.04em}\n"
         "td.num{font-variant-numeric:tabular-nums;text-align:right;white-space:nowrap}\n"
         "td.req{color:var(--muted)}\n"
         ".pill{display:inline-block;padding:1px 8px;border-radius:999px;font-size:12px;"
         "font-weight:600}\n"
         ".pass{background:color-mix(in srgb,var(--pass) 15%,transparent);color:var(--pass)}\n"
         ".fail{background:color-mix(in srgb,var(--fail) 15%,transparent);color:var(--fail)}\n"
         ".marginal{background:color-mix(in srgb,var(--marg) 18%,transparent);color:var(--marg)}\n"
         ".na{background:color-mix(in srgb,var(--muted) 15%,transparent);color:var(--muted)}\n"
         ".chart{width:100%;height:180px;border:1px solid var(--line);border-radius:6px;"
         "background:var(--bg)}\n"
         ".grid{stroke:var(--line);stroke-width:1}\n"
         ".tick{fill:var(--muted);font-size:10px}\n"
         ".trace{fill:none;stroke:var(--accent);stroke-width:1.5;"
         "vector-effect:non-scaling-stroke}\n"
         ".budget{stroke:var(--fail);stroke-width:1;stroke-dasharray:4 3}\n"
         ".budget-label{fill:var(--fail);font-size:10px}\n"
         ".empty{fill:var(--muted);font-size:12px}\n"
         "pre{background:color-mix(in srgb,var(--muted) 8%,transparent);padding:14px;"
         "border-radius:6px;overflow-x:auto;font-size:12px;line-height:1.45}\n"
         "footer{margin-top:40px;color:var(--muted);font-size:12px;"
         "border-top:1px solid var(--line);padding-top:12px}\n"
         "</style></head><body>\n";

    s << "<h1>" << esc(in.title) << "</h1>\n";
    s << "<p class=\"sub\">" << esc(in.scenario_name);
    if (!in.scenario_description.empty()) s << " — " << esc(in.scenario_description);
    s << "</p>\n<p class=\"sub\">build " << esc(in.build)
      << " · seed " << m.seed
      << " · " << m.frames_total << " frames · " << fmt("%.2f", m.duration_s) << " s simulated"
      << " · AI " << (m.ai_enabled ? "enabled" : "disabled")
      << " · " << esc(in.utc) << "</p>\n";

    // --- graded metric first ----------------------------------------------
    // §3.1 weights centroiding at 60%. It goes at the top because that is what
    // the report is mostly about, not because it is the best-looking number.
    s << "<h2>Centroiding — graded, 60%</h2>\n<table>\n"
         "<tr><th>metric</th><th>value</th><th>note</th><th></th></tr>\n";
    if (m.centroid_frames > 0) {
        s << metric_row("RMSE, image frame", fmt("%.4f px", m.centroid_rmse_image_px),
                        "the detector alone",
                        verdict(m.centroid_rmse_image_px, m.centroid_p95_image_px,
                                q.centroid_rmse_px, false));
        s << metric_row("p95, image frame", fmt("%.4f px", m.centroid_p95_image_px), "", "");
        s << metric_row("RMSE, screen frame", fmt("%.4f px", m.centroid_rmse_screen_px),
                        "+ unmeasured pointing error (INV-6)", "");
        s << metric_row("bias (mean signed)",
                        fmt("%+.4f", m.centroid_bias_x_px) + ", " +
                        fmt("%+.4f px", m.centroid_bias_y_px),
                        "a removable offset, unlike RMSE", "");
        s << metric_row("frames scored", std::to_string(m.centroid_frames),
                        "detection present and beacon in view", "");
    } else {
        s << "<tr><td colspan=\"4\">No frame had both a detection and the beacon in "
             "view — the centroiding metric is undefined for this run.</td></tr>\n";
    }
    s << "</table>\n";

    // --- specification rows ------------------------------------------------
    s << "<h2>Specification requirements</h2>\n<table>\n"
         "<tr><th>row</th><th>value</th><th>requirement</th><th>status</th></tr>\n";

    if (m.acquired_in_fov) {
        s << metric_row("16 · acquisition (in view)", fmt("%.3f s", m.acquisition_in_fov_s),
                        fmt("≤ %.1f s", q.acquisition_s),
                        verdict(m.acquisition_in_fov_s, m.acquisition_in_fov_s,
                                q.acquisition_s, false));
    }
    s << metric_row("16 · acquisition (cold)",
                    m.acquired ? fmt("%.3f s", m.acquisition_cold_s) : std::string("never"),
                    "bound derived — design §10.5", "n/a");

    if (m.tracking_frames > 0) {
        const bool floored = q.tracking_floor_px > q.tracking_error_px;
        // Graded on the STEADY-STATE figure (P1-9). The whole-run RMS is
        // dominated by the post-lock slew — on a jitter-free run it is
        // 3.36 px against a p95 of 0.97 px, which measures the mount getting
        // there rather than the loop holding it. Both rows are shown.
        s << metric_row("17 · tracking error (steady RMS)",
                        fmt("%.3f px", m.tracking_rms_steady_px),
                        floored ? fmt("floor %.2f px — rows 17 vs 23", q.tracking_floor_px)
                                : fmt("≤ %.0f px", q.tracking_error_px),
                        floored ? "n/a"
                                : verdict(m.tracking_rms_steady_px, m.tracking_p95_steady_px,
                                          q.tracking_error_px, false));
        s << metric_row("17 · tracking error (steady p95)",
                        fmt("%.3f px", m.tracking_p95_steady_px), "", "");
        s << metric_row("17 · tracking error (mean)", fmt("%.3f px", m.tracking_mean_px),
                        "PS Performance Log asks for the average", "");
        if (m.tracking_frames_transient > 0) {
            s << metric_row("— · acquisition transient (RMS)",
                            fmt("%.3f px", m.tracking_rms_transient_px),
                            fmt2("first %lld frames after lock (of %lld scored)",
                                 static_cast<long long>(m.settle_frames),
                                 static_cast<long long>(m.tracking_frames_transient)), "");
        }
        s << metric_row("— · tracking error (whole run RMS)",
                        fmt("%.3f px", m.tracking_rms_px), "transient included", "");
    } else {
        s << metric_row("17 · tracking error", "n/a", "the track was never Confirmed", "n/a");
    }

    // P0-2: the PS's own objective, above row 18 because row 18 is
    // conditional on it.
    if (m.frames_with_truth > 0) {
        s << metric_row("★ · FOV containment (whole run)",
                        fmt("%.2f %%", 100.0 * m.fov_containment_frac),
                        "PS: \"maintain the terminal within its FOV\"", "");
        if (m.post_acq_valid) {
            s << metric_row("★ · FOV containment (post-acq)",
                            fmt("%.2f %%", 100.0 * m.fov_containment_post_acq),
                            fmt2("%lld / %lld frames after first lock",
                                 static_cast<long long>(m.frames_in_fov_post_acq),
                                 static_cast<long long>(m.frames_post_acq)), "");
        }
    }

    // Row 18, graded post-acquisition. See compliance.cpp for the argument.
    if (m.post_acq_valid) {
        s << metric_row("18 · target loss (post-acq)",
                        fmt("%.2f %%", 100.0 * m.target_loss_post_acq),
                        fmt("< %.0f %%", 100.0 * q.target_loss_frac),
                        verdict(100.0 * m.target_loss_post_acq,
                                100.0 * m.target_loss_post_acq,
                                100.0 * q.target_loss_frac, false));
    } else {
        s << metric_row("18 · target loss", "n/a", "the track was never Confirmed", "n/a");
    }
    if (m.frames_in_fov > 0) {
        s << metric_row("— · target loss (in-FOV)", fmt("%.2f %%", 100.0 * m.target_loss_frac),
                        "detector-centric denominator; not graded", "");
    }

    if (m.reacquisitions > 0) {
        s << metric_row("19 · re-acquisition (mean)", fmt("%.3f s", m.reacquisition_mean_s),
                        fmt("≤ %.1f s", q.reacquisition_s),
                        verdict(m.reacquisition_mean_s, m.reacquisition_max_s,
                                q.reacquisition_s, false));
    } else {
        s << metric_row("19 · re-acquisition", "no episodes",
                        "the lock was never lost and regained", "n/a");
    }

    s << metric_row("20 · processing speed", fmt("%.1f fps", m.fps_mean),
                    fmt("≥ %.0f fps", q.min_fps),
                    verdict(m.fps_mean, m.fps_p5, q.min_fps, true));
    s << metric_row("20 · frame time p50 / p95 / p99",
                    fmt("%.2f", m.frame_ms_p50) + " / " + fmt("%.2f", m.frame_ms_p95) +
                    " / " + fmt("%.2f ms", m.frame_ms_p99),
                    "never the mean — §13.1", "");
    s << "</table>\n";

    // --- plots -------------------------------------------------------------
    if (!in.traces.empty()) {
        s << "<h2>Per-frame traces</h2>\n";
        for (const ReportTrace& t : in.traces) {
            s << "<p class=\"sub\">" << esc(t.label);
            if (!t.units.empty()) s << " (" << esc(t.units) << ")";
            s << "</p>\n" << svg_line_chart(t, 940, 180) << "\n";
        }
    }

    if (!in.compliance_text.empty()) {
        s << "<h2>Compliance matrix</h2>\n<pre>" << esc(in.compliance_text) << "</pre>\n";
    }

    s << "<footer>Generated by SAT " << esc(in.build)
      << ". Metric definitions: docs/METRICS.md, reproduced from design §13.1. "
         "This file is self-contained — no network, no external assets.</footer>\n"
         "</body></html>\n";
    return s.str();
}

bool write_report(const std::string& path, const ReportInput& in) {
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out << render_report(in);
    return static_cast<bool>(out);
}

}  // namespace sat
