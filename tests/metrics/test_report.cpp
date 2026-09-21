// tests/metrics/test_report.cpp — CP 7.6.
//
// "JSON -> self-contained report.html with inlined plots. Accept when:
//  finishing a run produces a showable report with zero manual steps."
//
// "Self-contained" is the testable half and the half most likely to rot: a
// stylesheet link or a charting CDN added later would still look fine on the
// machine that added it and would be a blank page on the evaluator's, which is
// exactly the situation a demo is in.

#include <doctest/doctest.h>

#include "metrics/report.hpp"

#include <regex>
#include <string>

using namespace sat;

namespace {

ReportInput sample() {
    ReportInput in;
    in.build = "a3f21c9";
    in.utc   = "2026-09-14T10:22:31Z";
    in.scenario_name = "baseline";
    in.scenario_description = "specification defaults";

    RunMetrics& m = in.metrics;
    m.seed = 42;
    m.frames_total = 3600;
    m.duration_s = 120.0;
    m.centroid_frames = 3412;
    m.centroid_rmse_image_px = 0.062;
    m.centroid_p95_image_px  = 0.15;
    m.centroid_rmse_screen_px = 0.081;
    m.centroid_bias_x_px = 0.004;
    m.centroid_bias_y_px = -0.002;
    m.tracking_frames = 3300;
    m.tracking_rms_px = 3.2;
    m.tracking_p95_px = 7.1;
    m.frames_in_fov = 3400;
    m.frames_confirmed = 3340;
    m.target_loss_frac = 0.018;
    m.acquired = true;
    m.acquired_in_fov = true;
    m.acquisition_in_fov_s = 0.31;
    m.acquisition_cold_s = 4.7;
    m.reacquisitions = 3;
    m.reacquisition_mean_s = 0.21;
    m.reacquisition_max_s = 0.44;
    m.fps_mean = 412.0;
    m.fps_p5 = 380.0;
    m.frame_ms_p50 = 2.4;
    m.frame_ms_p95 = 2.6;
    m.frame_ms_p99 = 3.1;

    ReportTrace t;
    t.label = "Tracking error";
    t.units = "px";
    t.budget = 10.0;
    t.budget_label = "row 17";
    for (int i = 0; i < 200; ++i) {
        t.x.push_back(i / 30.0);
        t.y.push_back(3.0 + 2.0 * std::sin(i * 0.1));
    }
    in.traces.push_back(t);
    return in;
}

}  // namespace

TEST_CASE("CP 7.6: the report references nothing outside itself") {
    // The checkpoint's word is "self-contained". A CDN link, an external
    // stylesheet or an <img src> beside the file all break it, and all of them
    // look perfectly fine on the machine that added them.
    const std::string h = render_report(sample());
    MESSAGE(h.size() << " bytes");

    std::regex ref(R"((?:src|href)\s*=\s*["']((?!#)[^"']+)["'])");
    auto begin = std::sregex_iterator(h.begin(), h.end(), ref);
    for (auto it = begin; it != std::sregex_iterator(); ++it) {
        FAIL_CHECK("external reference: " << (*it)[1].str());
    }
    CHECK(h.find("http://")  == std::string::npos);
    CHECK(h.find("https://") == std::string::npos);
    CHECK(h.find("<script")  == std::string::npos);  // no JS at all

    // And it is a complete document, not a fragment.
    CHECK(h.find("<!doctype html>") == 0);
    CHECK(h.find("</html>") != std::string::npos);
}

TEST_CASE("CP 7.6: the plots are inline SVG with real geometry") {
    const std::string h = render_report(sample());
    CHECK(h.find("<svg") != std::string::npos);
    CHECK(h.find("<polyline") != std::string::npos);
    // The budget line §12 asks to be drawn beside the error trace.
    CHECK(h.find("class=\"budget\"") != std::string::npos);
    CHECK(h.find("row 17") != std::string::npos);
}

TEST_CASE("CP 7.6: the chart survives degenerate data instead of emitting garbage") {
    SUBCASE("no samples") {
        ReportTrace t; t.label = "empty";
        const std::string svg = svg_line_chart(t, 400, 120);
        CHECK(svg.find("no data") != std::string::npos);
        CHECK(svg.find("nan") == std::string::npos);
    }
    SUBCASE("one sample cannot define an axis") {
        ReportTrace t; t.label = "one"; t.x = {0.0}; t.y = {1.0};
        CHECK(svg_line_chart(t, 400, 120).find("no data") != std::string::npos);
    }
    SUBCASE("a flat trace at zero does not divide by zero") {
        // Every value zero makes the y range zero. Without a guard this
        // produces inf/nan coordinates, and an SVG with nan in a points list
        // renders as nothing at all — a blank chart that looks like no data
        // rather than like a bug.
        ReportTrace t; t.label = "flat";
        for (int i = 0; i < 10; ++i) { t.x.push_back(i); t.y.push_back(0.0); }
        const std::string svg = svg_line_chart(t, 400, 120);
        CHECK(svg.find("nan") == std::string::npos);
        CHECK(svg.find("inf") == std::string::npos);
        CHECK(svg.find("<polyline") != std::string::npos);
    }
}

TEST_CASE("CP 7.6: scenario text is escaped, because it comes from a supplied file") {
    // §3.1: the evaluators supply the scenarios. A name containing '<' would
    // otherwise break the page silently.
    ReportInput in = sample();
    in.scenario_name = "<script>alert(1)</script>";
    in.scenario_description = "a & b \"quoted\"";
    const std::string h = render_report(in);
    CHECK(h.find("<script>alert") == std::string::npos);
    CHECK(h.find("&lt;script&gt;") != std::string::npos);
    CHECK(h.find("a &amp; b") != std::string::npos);
}

TEST_CASE("CP 7.6: an undefined metric is shown as undefined, not as a pass") {
    // The same flattering-zero the compliance matrix had to be fixed for. A run
    // that detected nothing must not produce a report full of green pills.
    ReportInput in;
    in.build = "x";
    in.metrics.frames_total = 120;      // everything else zero
    const std::string h = render_report(in);
    MESSAGE(h.substr(h.find("<h1>"), 1200));

    CHECK(h.find("the centroiding metric is undefined") != std::string::npos);
    // Rows 17 and 18 both. Since P0-2 row 18 is graded over post-acquisition
    // frames rather than in-FOV ones, so a run with no Confirmed frame has no
    // denominator for either and both give the same reason.
    CHECK(h.find("the track was never Confirmed") != std::string::npos);
    // Speed is the only row that can legitimately be judged here, and with a
    // zero frame rate it must not be a pass either.
    CHECK(h.find(">PASS<") == std::string::npos);
}

TEST_CASE("CP 7.6: row 17 is reported against its derived floor when one applies") {
    // Rows 17 and 23 are in tension (docs/METRICS.md §2.11). When the floor
    // exceeds the requirement the row must not read FAIL, or a property of the
    // specification is presented as a defect in the loop.
    ReportInput in = sample();
    // Row 17 is graded on the STEADY-STATE figure since P1-9, so that is the
    // field the floor logic has to be exercised through. The whole-run RMS is
    // set to match so the fixture stays self-consistent.
    in.metrics.tracking_rms_px        = 17.1;
    in.metrics.tracking_rms_steady_px = 17.1;
    in.metrics.tracking_p95_steady_px = 17.1;
    in.requirements.tracking_error_px = 10.0;
    in.requirements.tracking_floor_px = 16.33;
    const std::string h = render_report(in);
    CHECK(h.find("floor 16.33 px") != std::string::npos);

    // Without a floor the same number IS a failure, and says so.
    in.requirements.tracking_floor_px = 0.0;
    CHECK(render_report(in).find(">FAIL<") != std::string::npos);
}
