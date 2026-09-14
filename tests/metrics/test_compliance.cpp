// tests/metrics/test_compliance.cpp — CP 7.5 and the ★ CP 7.7 gate.
//
// The sweep's own machinery, tested without spawning anything: the TOML
// overlay, the spec parser, the cartesian expansion, the run.json round trip,
// and the matrix's verdict rule.
//
// Spawning 500 processes is not something a unit test should do, and it is not
// where the interesting failures are. The interesting failures are a matrix
// that reports PASS for a metric with no samples behind it, an overlay that
// silently changes nothing, and a round trip that loses a field — all of which
// are cheap to test and all of which have happened here.

#include <doctest/doctest.h>

#include "app/sweep.hpp"
#include "metrics/compliance.hpp"
#include "metrics/run_report.hpp"
#include "scenario/overlay.hpp"
#include "scenario/schema.hpp"
#include "scenario/sweep_spec.hpp"

#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>

using namespace sat;

// ===========================================================================
// The overlay
// ===========================================================================

TEST_CASE("CP 7.5: an override changes one key and leaves the rest alone") {
    const std::string base =
        "[sim]\nseed = 42\nduration_s = 60.0\n\n[atmosphere]\nmode = \"clear\"\n";

    auto r = apply_overrides(base, {{"atmosphere.mode", "\"fog\""}});
    REQUIRE_MESSAGE(r.has_value(), r.error());
    MESSAGE("\n" << *r);

    CHECK(r->find("fog") != std::string::npos);
    CHECK(r->find("clear") == std::string::npos);
    CHECK(r->find("42") != std::string::npos);        // untouched
    CHECK(r->find("60") != std::string::npos);
}

TEST_CASE("CP 7.5: the value's TYPE comes from TOML, not from a guess") {
    // The overlay inserts the value as TOML source and lets the parser decide
    // what it is. That is what makes one code path work for a string, a float,
    // a bool and an array — and what makes a wrongly quoted value an error
    // rather than a silently stringified number.
    const std::string base = "[a]\nx = 1\n";

    auto num  = apply_overrides(base, {{"a.x", "2.5"}});
    auto str  = apply_overrides(base, {{"a.x", "\"two\""}});
    auto flag = apply_overrides(base, {{"a.x", "true"}});
    auto arr  = apply_overrides(base, {{"a.x", "[1, 2]"}});
    REQUIRE(num.has_value());
    REQUIRE(str.has_value());
    REQUIRE(flag.has_value());
    REQUIRE(arr.has_value());
    // Assert the VALUE, not the spelling: toml++ may emit a string as 'two'
    // (a TOML literal string) or "two", and both are correct. A test that
    // pinned one of them would be testing the serialiser's taste.
    CHECK(num->find("2.5") != std::string::npos);
    CHECK(flag->find("true") != std::string::npos);
    auto str_back = apply_overrides(*str, {});
    REQUIRE(str_back.has_value());
    CHECK(str_back->find("two") != std::string::npos);

    // And a value that is not valid TOML is rejected, naming the key.
    auto bad = apply_overrides(base, {{"a.x", "two"}});
    CHECK_FALSE(bad.has_value());
    CHECK(bad.error().find("a.x") != std::string::npos);
}

TEST_CASE("CP 7.5: an override can create a table the base file never mentions") {
    const std::string base = "[sim]\nseed = 1\n";
    auto r = apply_overrides(base, {{"clutter.static_sources", "120"}});
    REQUIRE_MESSAGE(r.has_value(), r.error());
    CHECK(r->find("clutter") != std::string::npos);
    CHECK(r->find("120") != std::string::npos);
}

TEST_CASE("CP 7.5: overriding through a non-table is an error, not a silent no-op") {
    const std::string base = "[a]\nx = 1\n";
    auto r = apply_overrides(base, {{"a.x.y", "2"}});
    CHECK_FALSE(r.has_value());
    CHECK(r.error().find("not a table") != std::string::npos);
}

TEST_CASE("CP 7.5: an overridden scenario still goes through the schema") {
    // The reason the overlay works on TOML rather than on struct fields: an
    // out-of-range override must produce §7.5's error, with the file, the
    // value and the specification row, exactly as a hand-written file would.
    auto base = load_scenario(std::string(SAT_SCENARIO_DIR) + "/baseline.toml");
    REQUIRE(base.has_value());
    std::string text;
    {
        std::ifstream in(std::string(SAT_SCENARIO_DIR) + "/baseline.toml", std::ios::binary);
        text.assign((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    }

    auto bad = apply_overrides(text, {{"gimbal.max_pan_dps", "14.0"}});
    REQUIRE(bad.has_value());
    auto parsed = parse_scenario(*bad, "swept.toml");
    REQUIRE_FALSE(parsed.has_value());
    MESSAGE(parsed.error());
    CHECK(parsed.error().find("max_pan_dps") != std::string::npos);
    CHECK(parsed.error().find("row 13") != std::string::npos);
}

// ===========================================================================
// The spec and the expansion
// ===========================================================================

TEST_CASE("CP 7.5: a spec expands to the full cartesian product, at every seed") {
    const std::string toml =
        "[sweep]\n"
        "base = \"scenarios/baseline.toml\"\n"
        "seed_count = 4\n"
        "[[axis]]\nkey = \"atmosphere.mode\"\nvalues = [\"clear\", \"fog\"]\n"
        "[[axis]]\nkey = \"noise.salt_pepper\"\nvalues = [0.0, 0.05, 0.10]\n";

    auto spec = parse_sweep_spec(toml);
    REQUIRE_MESSAGE(spec.has_value(), spec.error());
    CHECK(spec->seeds.size() == 4);
    CHECK(spec->seeds.front() == 1);
    CHECK(spec->seeds.back() == 4);
    CHECK(spec->axes.size() == 2);
    CHECK(spec->run_count() == 2 * 3 * 4);

    const std::vector<SweepRun> runs = expand(*spec, "/tmp/x");
    CHECK(runs.size() == 24);

    // Every combination appears at every seed, and each run has its own
    // directory — the sweep's isolation depends on that being true.
    std::set<std::string> dirs;
    std::set<std::string> labels;
    for (const SweepRun& r : runs) {
        CHECK(dirs.insert(r.dir).second);
        labels.insert(r.label);
        CHECK(r.overrides.size() == 2);
    }
    CHECK(labels.size() == 6);
}

TEST_CASE("CP 7.5: a float axis value is labelled readably") {
    // 0.1 round-trips through a double as 0.10000000000000001, and the label
    // is what appears as a row heading in the matrix — the one output in this
    // project most likely to be read by someone who did not write it.
    auto spec = parse_sweep_spec(
        "[sweep]\nbase = \"b.toml\"\n"
        "[[axis]]\nkey = \"noise.salt_pepper\"\nvalues = [0.1]\n");
    REQUIRE(spec.has_value());
    REQUIRE(spec->axes.size() == 1);
    CHECK(spec->axes[0].values[0] == "0.1");
}

TEST_CASE("CP 7.5: a string axis value stays quoted, so the overlay can insert it") {
    // Without quotes the overlay inserts `mode = fog`, which is not valid TOML,
    // and every string axis fails. Either quote style is fine — toml++ emits a
    // literal string — so the test checks what actually matters: that the value
    // survives a round trip through the overlay.
    auto spec = parse_sweep_spec(
        "[sweep]\nbase = \"b.toml\"\n"
        "[[axis]]\nkey = \"atmosphere.mode\"\nvalues = [\"fog\"]\n");
    REQUIRE(spec.has_value());
    const std::string v = spec->axes[0].values[0];
    CHECK((v.front() == '"' || v.front() == '\''));

    auto applied = apply_overrides("[atmosphere]\nmode = \"clear\"\n",
                                   {{"atmosphere.mode", v}});
    REQUIRE_MESSAGE(applied.has_value(), applied.error());
    CHECK(applied->find("fog") != std::string::npos);
    CHECK(applied->find("clear") == std::string::npos);
}

// ===========================================================================
// run.json round trip — the sweep's only channel between processes
// ===========================================================================

TEST_CASE("CP 7.4: run.json round-trips every metric the matrix reads") {
    RunMetrics m;
    m.centroid_rmse_screen_px = 0.081;
    m.centroid_p95_screen_px  = 0.19;
    m.centroid_rmse_image_px  = 0.062;
    m.centroid_p95_image_px   = 0.15;
    m.centroid_bias_x_px      = 0.004;
    m.centroid_bias_y_px      = -0.002;
    m.centroid_frames         = 3412;
    m.tracking_rms_px         = 3.2;
    m.tracking_p95_px         = 7.1;
    m.tracking_frames         = 3300;
    m.acquired                = true;
    m.acquisition_cold_s      = 4.7;
    m.acquired_in_fov         = true;
    m.acquisition_in_fov_s    = 0.31;
    m.reacquisitions          = 3;
    m.reacquisition_mean_s    = 0.21;
    m.reacquisition_max_s     = 0.44;
    m.lock_retention_rate     = 0.982;
    m.target_loss_frac        = 0.018;
    m.frames_in_fov           = 3400;
    m.frames_confirmed        = 3340;
    m.false_tracks            = 2;
    m.saturation_frac         = 0.07;
    m.frame_ms_p50            = 2.4;
    m.frame_ms_p95            = 2.6;
    m.fps_mean                = 412.0;
    m.fps_p5                  = 380.0;
    m.frames_total            = 3600;
    m.duration_s              = 120.0;

    Scenario sc;
    sc.name = "baseline";
    sc.seed = 42;
    const std::string json = run_json(m, sc, "a3f21c9", 0xdeadbeefULL);

    auto back = metrics_from_json(json);
    REQUIRE_MESSAGE(back.has_value(), back.error());

    CHECK(back->centroid_rmse_screen_px == doctest::Approx(m.centroid_rmse_screen_px));
    CHECK(back->centroid_rmse_image_px  == doctest::Approx(m.centroid_rmse_image_px));
    CHECK(back->centroid_bias_x_px      == doctest::Approx(m.centroid_bias_x_px));
    CHECK(back->centroid_frames         == m.centroid_frames);
    CHECK(back->tracking_rms_px         == doctest::Approx(m.tracking_rms_px));
    CHECK(back->tracking_p95_px         == doctest::Approx(m.tracking_p95_px));
    CHECK(back->tracking_frames         == m.tracking_frames);
    CHECK(back->acquired                == m.acquired);
    CHECK(back->acquisition_in_fov_s    == doctest::Approx(m.acquisition_in_fov_s));
    CHECK(back->reacquisitions          == m.reacquisitions);
    CHECK(back->lock_retention_rate     == doctest::Approx(m.lock_retention_rate));
    CHECK(back->frames_in_fov           == m.frames_in_fov);
    CHECK(back->fps_p5                  == doctest::Approx(m.fps_p5));
    CHECK(back->seed                    == 42);
    CHECK(back->scenario_name           == "baseline");

    // Provenance is present and is the FIRST thing in the file: anyone opening
    // it should see what produced these numbers before seeing the numbers.
    CHECK(json.find("provenance") < json.find("\"metrics\""));
    CHECK(json.find("a3f21c9") != std::string::npos);
    // And the full scenario is echoed, not referenced.
    CHECK(json.find("\"scenario\"") != std::string::npos);
    CHECK(json.find("gimbal") != std::string::npos);
}

TEST_CASE("CP 7.5: unreadable run.json is an error, not a row of zeros") {
    // A sweep that silently averaged in a default-constructed RunMetrics would
    // report a BETTER number the more of its runs crashed.
    CHECK_FALSE(metrics_from_json("").has_value());
    CHECK_FALSE(metrics_from_json("{}").has_value());
    CHECK_FALSE(metrics_from_json("not json at all").has_value());
    CHECK_FALSE(metrics_from_json("{\"metrics\": {}}").has_value());
}

// ===========================================================================
// CP 7.7 — the matrix's verdict rule
// ===========================================================================

TEST_CASE("CP 7.7: MARGINAL exists, and means what design 13.3 says") {
    // "An honest MARGINAL is stronger than a suspicious all-green."
    CHECK(std::string(verdict(3.2, 7.1, 10.0, false)) == "PASS");       // both inside
    CHECK(std::string(verdict(6.4, 11.8, 10.0, false)) == "MARGINAL");  // p95 outside
    CHECK(std::string(verdict(14.0, 20.0, 10.0, false)) == "FAIL");     // mean outside

    // Higher-is-better, for fps: the tail that matters is the LOW one.
    CHECK(std::string(verdict(412.0, 380.0, 20.0, true)) == "PASS");
    CHECK(std::string(verdict(412.0, 18.0, 20.0, true)) == "MARGINAL");
    CHECK(std::string(verdict(15.0, 12.0, 20.0, true)) == "FAIL");

    // Exactly on the limit passes: the requirement is "<= 10 px", not "< 10".
    CHECK(std::string(verdict(10.0, 10.0, 10.0, false)) == "PASS");
}

TEST_CASE("CP 7.7: a metric with no samples reports n/a and never PASS") {
    // THE BUG THIS TEST EXISTS FOR. A metric computed over zero samples returns
    // 0.0, and 0.0 against "must be below 10" is a PASS. The first real sweep
    // printed "Centroiding 0.000 px PASS" and "Target loss 0.000% PASS" for a
    // condition in which the beacon was never detected at all — the most
    // flattering possible rendering of a total failure.
    RunMetrics empty;               // every denominator zero
    empty.frames_total = 120;

    const std::vector<RunMetrics> runs{empty};
    const std::string out = compliance_matrix(runs, {}, Requirements{}, "test");
    MESSAGE("\n" << out);

    CHECK(out.find("n/a") != std::string::npos);
    CHECK(out.find("the track was never Confirmed") != std::string::npos);
    CHECK(out.find("the beacon was never in view") != std::string::npos);
    // Nothing in this matrix may claim a pass.
    CHECK(out.find("PASS") == std::string::npos);
}

TEST_CASE("CP 7.7: the matrix breaks out per condition") {
    // §13.3: "broken out per condition". A system excellent in clear air and
    // broken in fog has a perfectly acceptable average, and the average is all
    // the summary block shows.
    auto make = [](double tracking, int64_t frames) {
        RunMetrics m;
        m.tracking_rms_px = tracking;
        m.tracking_p95_px = tracking * 1.2;
        m.tracking_frames = frames;
        m.frames_in_fov = frames; m.frames_confirmed = frames;
        m.centroid_frames = frames;
        m.centroid_rmse_image_px = 0.1;
        m.fps_mean = 400; m.fps_p5 = 380;
        m.acquired = true; m.acquired_in_fov = true; m.acquisition_in_fov_s = 0.3;
        return m;
    };

    const std::vector<ConditionGroup> groups{
        {"clear", {make(3.0, 100), make(3.2, 100)}},
        {"fog",   {make(24.0, 100), make(26.0, 100)}},
    };
    std::vector<RunMetrics> all;
    for (const auto& g : groups) for (const auto& m : g.runs) all.push_back(m);

    const std::string out = compliance_matrix(all, groups, Requirements{}, "test");
    MESSAGE("\n" << out);

    CHECK(out.find("[clear]") != std::string::npos);
    CHECK(out.find("[fog]") != std::string::npos);
    // The aggregate hides it; the breakout does not.
    const size_t fog_at = out.find("[fog]");
    REQUIRE(fog_at != std::string::npos);
    CHECK(out.find("FAIL", fog_at) != std::string::npos);
}
