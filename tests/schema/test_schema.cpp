// tests/schema/test_schema.cpp — CP 3.1, 3.2 and 3.6.
//
// CP 3.2's criterion: "15 files in tests/bad_configs/ each produce the correct
//                      file:line: message (specification row N)".
//
// The point of citing the specification row is not tidiness. The evaluators
// supply the scenarios (design §3.1); when one is rejected, the useful response
// is a sentence that lets someone holding the specification see immediately
// that the file asks for something the specification does not permit. Anything
// less turns a correct rejection into an argument.

#include <doctest/doctest.h>

#include "scenario/schema.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

using namespace sat;

namespace {

std::filesystem::path bad_config(const char* name) {
    return std::filesystem::path(SAT_TEST_BAD_CONFIG_DIR) / name;
}
std::filesystem::path scenario_file(const char* name) {
    return std::filesystem::path(SAT_SCENARIO_DIR) / name;
}

bool contains(const std::string& hay, const std::string& needle) {
    return hay.find(needle) != std::string::npos;
}

}  // namespace

// ===========================================================================
// CP 3.1 — the loader
// ===========================================================================

TEST_CASE("CP 3.1: the default scenario loads and carries the spec defaults") {
    auto r = load_scenario(scenario_file("spec_defaults.toml"));
    REQUIRE_MESSAGE(r.has_value(), r.error());
    const Scenario& s = *r;

    CHECK(s.name == "spec_defaults");
    CHECK(s.canvas_px[0] == 2000);        // row 1
    CHECK(s.canvas_px[1] == 2000);
    CHECK(s.resolution[0] == 640);        // row 3
    CHECK(s.resolution[1] == 480);
    CHECK(s.fov_deg[0] == doctest::Approx(4.0));   // row 4
    CHECK(s.fov_deg[1] == doctest::Approx(3.0));
    CHECK(s.camera_hz == 30);             // row 5
    CHECK(s.control_hz == 30);            // row 15
    CHECK(s.targets.size() == 1);         // row 8
    CHECK(s.targets[0].size_px == 10);    // row 10
    CHECK(s.max_pan_dps == doctest::Approx(5.0));   // row 13
    CHECK(s.max_tilt_dps == doctest::Approx(5.0));  // row 14
    CHECK(s.gaussian_sigma == doctest::Approx(20.0)); // row 22
    CHECK(s.salt_pepper == doctest::Approx(0.10));    // row 21
    CHECK(s.jitter_px_per_frame == doctest::Approx(20.0)); // row 23
    CHECK(s.atmosphere == Atmosphere::Clear);         // row 24
    CHECK(s.platform.size() == 1);                     // row 25

    // And the derived geometry matches design §1.4.
    CHECK(s.camera_geometry().ifov_urad() == doctest::Approx(109.08).epsilon(1e-4));
}

TEST_CASE("CP 3.1: changing a key changes behaviour with no recompile") {
    // The checkpoint's stated criterion.
    const char* toml = R"(
[sim]
duration_s = 10
[camera]
fov_deg = [8.0, 6.0]
[target]
size_px = 10
)";
    auto r = parse_scenario(toml);
    REQUIRE_MESSAGE(r.has_value(), r.error());
    // Double the field of view, double the IFOV: each pixel now covers twice
    // the angle, which changes every derived quantity downstream.
    CHECK(r->camera_geometry().ifov_urad() == doctest::Approx(2.0 * 109.08).epsilon(1e-3));
}

TEST_CASE("every shipped scenario loads without error") {
    // If a scenario in scenarios/ does not load, the demo has no fallback and
    // CP 15.3 has nothing to preload.
    //
    // This WALKS THE DIRECTORY rather than listing names, and the difference
    // is not stylistic. The list version named five files and silently ignored
    // the other fourteen, including every scenario under adversarial/ and
    // every one added after the list was written. It could not notice a
    // scenario that was renamed, moved or added — which is exactly what the
    // scenarios/hard/ reorganisation did to five of them.
    //
    // Sweep specs are excluded: scenarios/sweeps/*.toml are [sweep] documents,
    // not scenarios, and are parsed by parse_sweep_spec instead.
    namespace fs = std::filesystem;
    int checked = 0;
    for (const fs::directory_entry& e :
             fs::recursive_directory_iterator(SAT_SCENARIO_DIR)) {
        if (!e.is_regular_file() || e.path().extension() != ".toml") continue;
        if (e.path().parent_path().filename() == "sweeps") continue;
        INFO("scenario = " << e.path().string());
        auto r = load_scenario(e.path().string());
        CHECK_MESSAGE(r.has_value(), r.error());
        ++checked;
    }
    MESSAGE("checked " << checked << " shipped scenarios");
    // A walk that finds nothing would pass vacuously, which is the failure
    // mode the hardcoded list had in a different form.
    CHECK(checked >= 10);
}

TEST_CASE("an omitted key keeps its default rather than becoming an error") {
    // Design §7.1 shows a full file, but almost every key is optional. A
    // near-empty scenario must still run with the specification's defaults.
    auto r = parse_scenario("[target]\nsize_px = 10\n");
    REQUIRE_MESSAGE(r.has_value(), r.error());
    CHECK(r->canvas_px[0] == 2000);
    CHECK(r->camera_hz == 30);
    CHECK(r->max_pan_dps == doctest::Approx(5.0));
    CHECK(r->atmosphere == Atmosphere::Clear);
}

TEST_CASE("spec row 6: the camera defaults to the centre of the screen") {
    auto r = parse_scenario("[target]\nsize_px = 10\n");
    REQUIRE(r.has_value());
    CHECK(r->initial_pos_px[0] == doctest::Approx(999.5));
    CHECK(r->initial_pos_px[1] == doctest::Approx(999.5));
    // Which is the angular origin.
    CHECK(r->initial_boresight().x == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(r->initial_boresight().y == doctest::Approx(0.0).epsilon(1e-12));

    // And it follows the canvas size rather than being hardcoded.
    auto big = parse_scenario("[world]\ncanvas_px = [4000, 4000]\n[target]\nsize_px = 10\n");
    REQUIRE(big.has_value());
    CHECK(big->initial_pos_px[0] == doctest::Approx(1999.5));
}

// ===========================================================================
// CP 3.2 — every bad config produces the right message
// ===========================================================================

TEST_CASE("CP 3.2: every bad config is rejected with a spec-row-citing message") {
    struct Case {
        const char* file;
        const char* must_mention;   ///< the offending key
        const char* spec_row;       ///< "" when the rule is ours, not the spec's
    };

    // Eighteen files, three more than the checkpoint asks for. The extras cover
    // the cross-field checks, which are the interesting ones: a scenario can
    // have every individual value in range and still be impossible.
    const Case cases[] = {
        {"01_pan_speed_too_high.toml",        "gimbal.max_pan_dps",              "row 13"},
        {"02_tilt_speed_too_low.toml",        "gimbal.max_tilt_dps",             "row 14"},
        {"03_target_too_small.toml",          "target.size_px",                  "row 10"},
        {"04_target_too_large.toml",          "target.size_px",                  "row 10"},
        {"05_camera_rate_too_low.toml",       "sim.camera_hz",                   "row 5"},
        {"06_control_rate_too_low.toml",      "sim.control_hz",                  "row 15"},
        {"07_canvas_too_small.toml",          "world.canvas_px",                 "row 1"},
        {"08_noise_sigma_too_high.toml",      "noise.gaussian_sigma",            "row 22"},
        {"09_jitter_too_high.toml",           "disturbance.jitter_px_per_frame", "row 23"},
        {"10_salt_pepper_out_of_range.toml",  "noise.salt_pepper",               "row 21"},
        {"11_clock_not_divisible.toml",       "sim.truth_hz",                    ""},
        {"12_no_target.toml",                 "target",                          "row 8"},
        {"13_unknown_atmosphere.toml",        "atmosphere.mode",                 "row 24"},
        {"14_unknown_motion_kind.toml",       "kind",                            "row 12"},
        {"15_camera_larger_than_screen.toml", "camera.resolution",               ""},
        {"16_periodic_motion_zero_period.toml", "period_s",                      "row 12"},
        {"17_video_mode_without_file.toml",   "input.video_file",                ""},
        {"18_syntax_error.toml",              "",                                ""},
    };

    for (const auto& c : cases) {
        INFO("bad config = " << c.file);
        auto r = load_scenario(bad_config(c.file));

        // It must be rejected at all.
        REQUIRE_MESSAGE(!r.has_value(),
                        "expected this scenario to be rejected");

        const std::string err = r.error();
        INFO("message:\n" << err);

        // Design §7.5's format: the file name must appear, so the reader knows
        // which of several scenarios is at fault.
        CHECK(contains(err, c.file));

        // The offending key must be named.
        if (c.must_mention[0] != '\0') {
            CHECK(contains(err, c.must_mention));
        }

        // And where the specification is the authority, the row must be cited.
        if (c.spec_row[0] != '\0') {
            INFO("expected the message to cite specification " << c.spec_row);
            CHECK(contains(err, std::string("specification ") + c.spec_row));
        }
    }
}

TEST_CASE("CP 3.2: the error names the LINE the bad value is on") {
    // Without a line number the best we could say is "gimbal.max_pan_dps is
    // wrong", and the user would have to find it. This is most of why toml++
    // was chosen — it tracks source positions for every node.
    auto r = load_scenario(bad_config("01_pan_speed_too_high.toml"));
    REQUIRE_FALSE(r.has_value());

    // Find the line the offending key is actually on, so the test does not
    // hardcode a number that a comment edit would break. Comment lines are
    // skipped: each bad config documents what it expects in a header comment
    // that mentions the key by name, and an earlier version of this test
    // matched that comment instead of the assignment.
    std::ifstream in(bad_config("01_pan_speed_too_high.toml"));
    std::string line;
    int lineno = 0, expected = 0;
    while (std::getline(in, line)) {
        ++lineno;
        const size_t first = line.find_first_not_of(" \t");
        if (first == std::string::npos || line[first] == '#') continue;
        if (line.find("max_pan_dps") != std::string::npos) { expected = lineno; break; }
    }
    REQUIRE(expected > 0);

    INFO("error: " << r.error() << "\nexpected line " << expected);
    CHECK(contains(r.error(), ":" + std::to_string(expected) + ":"));
}

TEST_CASE("the message quotes the offending value and the permitted range") {
    auto r = load_scenario(bad_config("01_pan_speed_too_high.toml"));
    REQUIRE_FALSE(r.has_value());
    const std::string e = r.error();
    INFO(e);
    CHECK(contains(e, "14"));                 // what was asked for
    CHECK(contains(e, "[5, 10]"));            // what is permitted
    CHECK(contains(e, "specification row 13"));
}

TEST_CASE("every problem is reported at once, not just the first") {
    // Someone handed a rejected scenario should be able to fix it in one pass.
    const char* toml = R"(
[sim]
camera_hz  = 10
control_hz = 5
duration_s = 10
[gimbal]
max_pan_dps  = 99.0
max_tilt_dps = 1.0
[target]
size_px = 99
[noise]
gaussian_sigma = 500.0
)";
    auto r = parse_scenario(toml, "multi.toml");
    REQUIRE_FALSE(r.has_value());
    const std::string e = r.error();
    INFO(e);
    CHECK(contains(e, "sim.camera_hz"));
    CHECK(contains(e, "sim.control_hz"));
    CHECK(contains(e, "gimbal.max_pan_dps"));
    CHECK(contains(e, "gimbal.max_tilt_dps"));
    CHECK(contains(e, "target.size_px"));
    CHECK(contains(e, "noise.gaussian_sigma"));
}

TEST_CASE("a syntax error is reported in the same shape as a semantic one") {
    auto r = load_scenario(bad_config("18_syntax_error.toml"));
    REQUIRE_FALSE(r.has_value());
    INFO(r.error());
    CHECK(contains(r.error(), "18_syntax_error.toml"));
    CHECK(contains(r.error(), ":"));       // file:line:
}

TEST_CASE("a missing file is an error, not a crash") {
    auto r = load_scenario("does/not/exist.toml");
    CHECK_FALSE(r.has_value());
    CHECK(contains(r.error(), "not found"));
}

// ===========================================================================
// CP 3.6 — every spec row is reachable
// ===========================================================================

TEST_CASE("CP 3.6: every specification row with a numeric range is in the schema") {
    // The checkpoint's criterion is "you can point at any table row and name the
    // TOML key". This asserts the converse for the rows that carry bounds: each
    // one is represented by a schema entry that cites it, so a rejection can
    // always name its authority.
    const char* rows_with_ranges[] = {
        "row 1",   // screen size
        "row 3",   // camera resolution
        "row 4",   // camera FOV
        "row 5",   // camera update rate
        "row 10",  // target size
        "row 13",  // max pan speed
        "row 14",  // max tilt speed
        "row 15",  // update interval
        "row 16",  // acquisition time
        "row 17",  // tracking error
        "row 18",  // target loss
        "row 19",  // re-acquisition
        "row 20",  // processing speed
        "row 21",  // image noise
        "row 22",  // max noise sigma
        "row 23",  // max camera jitter
    };
    for (const char* row : rows_with_ranges) {
        bool found = false;
        for (const auto& f : schema()) {
            if (std::string(f.spec_row) == row) { found = true; break; }
        }
        INFO("specification " << row);
        CHECK(found);
    }
}

TEST_CASE("every schema entry has a sane range and a note or a spec row") {
    for (const auto& f : schema()) {
        INFO("schema entry: " << std::string(f.path));
        CHECK(f.min <= f.max);
        CHECK(f.path[0] != '\0');
        // Either the specification is the authority (cite the row) or we are
        // (say why). A bound with neither is a number nobody can argue with,
        // which is the wrong kind of unarguable.
        CHECK((f.spec_row[0] != '\0' || f.note[0] != '\0'));
    }
}

TEST_CASE("design 9.3's atmosphere table is reproduced exactly") {
    // Row 24's five modes, with the alpha and beta from design §9.3.
    struct E { Atmosphere mode; double alpha, beta; };
    const E expected[] = {
        {Atmosphere::Clear,    1.00,   0.0},
        {Atmosphere::Haze,     0.75,  20.0},
        {Atmosphere::Rain,     0.60,  15.0},
        {Atmosphere::Fog,      0.35,  60.0},
        {Atmosphere::LowLight, 0.40, -40.0},
    };
    for (const auto& e : expected) {
        INFO("atmosphere = " << atmosphere_name(e.mode));
        const auto c = atmosphere_coeffs(e.mode);
        CHECK(c.alpha == doctest::Approx(e.alpha));
        CHECK(c.beta  == doctest::Approx(e.beta));
    }
    // Fog really is the hard one: it more than halves contrast AND lifts the
    // floor, which is what CFAR's adaptive threshold exists for (§9.4.5).
    CHECK(atmosphere_coeffs(Atmosphere::Fog).alpha < 0.5);
    CHECK(atmosphere_coeffs(Atmosphere::Fog).beta > 50.0);
}

TEST_CASE("INV-8: damage generation is disabled in video modes") {
    // "When input.mode != synthetic, the noise and atmosphere generators are
    // forcibly disabled. The degradation is already in the supplied footage;
    // adding more corrupts the benchmark."
    //
    // Note this does NOT reject a config that sets noise in a video mode. The
    // evaluators' files may well carry noise settings copied from a synthetic
    // scenario, and refusing to run would be unhelpful. The loader keeps the
    // values (so run.json echoes what was asked for) and the generators stay
    // off regardless.
    const char* toml = R"(
[input]
mode       = "video_direct"
video_file = "clip.mp4"
[sim]
duration_s = 10
[target]
size_px = 10
[noise]
gaussian_sigma = 20.0
salt_pepper    = 0.10
[atmosphere]
mode = "fog"
)";
    auto r = parse_scenario(toml);
    REQUIRE_MESSAGE(r.has_value(), r.error());
    CHECK_FALSE(r->damage_enabled());
    // The request is still recorded, so the report can say we declined it.
    CHECK(r->gaussian_sigma == doctest::Approx(20.0));
    CHECK(r->atmosphere == Atmosphere::Fog);

    // A synthetic scenario is unaffected.
    auto syn = parse_scenario("[target]\nsize_px = 10\n");
    REQUIRE(syn.has_value());
    CHECK(syn->damage_enabled());
}

TEST_CASE("spec row 12: every motion kind parses from TOML") {
    const char* toml = R"(
[sim]
duration_s = 60
[target]
size_px = 10
[[target.motion]]
kind      = "constant"
offset_px = [1000.0, 1000.0]
[[target.motion]]
kind          = "linear"
velocity_px_s = [22.0, -11.0]
[[target.motion]]
kind         = "sinusoid"
axis         = "y"
amplitude_px = [0.0, 150.0]
period_s     = 5.0
[[target.motion]]
kind      = "circular"
radius_px = 300.0
period_s  = 12.0
[[target.motion]]
kind         = "lissajous"
amplitude_px = [700.0, 350.0]
freq_ratio   = 2.0
period_s     = 24.0
[[target.motion]]
kind        = "spiral"
r0_px       = 50.0
growth_px_s = 12.0
period_s    = 9.0
[[target.motion]]
kind       = "ou_noise"
sigma_px_s = 70.0
tau_s      = 3.0
[[target.motion]]
kind   = "waypoints"
points = [[0.0, 100.0, 100.0], [5.0, 500.0, 300.0], [10.0, 200.0, 800.0]]
)";
    auto r = parse_scenario(toml);
    REQUIRE_MESSAGE(r.has_value(), r.error());
    REQUIRE(r->targets.size() == 1);
    const auto& m = r->targets[0].motion;
    REQUIRE(m.size() == 8);
    CHECK(m[0].kind == "constant");
    CHECK(m[1].kind == "linear");
    CHECK(m[1].velocity_px_s[0] == doctest::Approx(22.0));
    CHECK(m[2].kind == "sinusoid");
    CHECK(m[2].axis == 1);                    // "y" resolves to axis 1
    CHECK(m[3].kind == "circular");
    CHECK(m[4].kind == "lissajous");
    CHECK(m[4].freq_ratio == doctest::Approx(2.0));   // figure of eight
    CHECK(m[5].kind == "spiral");
    CHECK(m[6].kind == "ou_noise");
    CHECK(m[7].kind == "waypoints");
    CHECK(m[7].points.size() == 3);
}

TEST_CASE("spec row 8: multiple targets are accepted") {
    const char* toml = R"(
[sim]
duration_s = 10
[[target]]
size_px = 10
[[target]]
size_px = 12
)";
    auto r = parse_scenario(toml);
    REQUIRE_MESSAGE(r.has_value(), r.error());
    CHECK(r->targets.size() == 2);
    CHECK(r->targets[1].size_px == 12);
}

TEST_CASE("design 7.4: event actions are a validated enum") {
    // "New actions require a C++ change — correct, since actions are behaviour."
    auto good = parse_scenario(R"(
[sim]
duration_s = 60
[target]
size_px = 10
[[event]]
t_s = 8.0
action = "set_atmosphere"
mode = "fog"
ramp_s = 2.0
)");
    REQUIRE_MESSAGE(good.has_value(), good.error());
    CHECK(good->events.size() == 1);
    CHECK(good->events[0].action == "set_atmosphere");

    auto bad = parse_scenario(R"(
[sim]
duration_s = 60
[target]
size_px = 10
[[event]]
t_s = 8.0
action = "summon_kraken"
)", "events.toml");
    REQUIRE_FALSE(bad.has_value());
    CHECK(contains(bad.error(), "summon_kraken"));
    CHECK(contains(bad.error(), "set_atmosphere"));   // lists what IS allowed
}

TEST_CASE("an event scheduled after the run ends is rejected") {
    // It would never fire, which is almost certainly not what was meant.
    auto r = parse_scenario(R"(
[sim]
duration_s = 10
[target]
size_px = 10
[[event]]
t_s = 500.0
action = "spawn_decoy"
)", "late.toml");
    REQUIRE_FALSE(r.has_value());
    CHECK(contains(r.error(), "never fire"));
}
