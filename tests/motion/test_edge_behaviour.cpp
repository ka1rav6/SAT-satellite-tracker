// tests/motion/test_edge_behaviour.cpp — spec row 8.
//
// `world.edge_behaviour` was parsed, validated by the schema and echoed into
// run.json for three stages, and read by nothing. The consequence, on the
// specification's own default scenario and its own default seed, was that the
// beacon left the 2000 x 2000 canvas after three seconds and every metric the
// run produced was a measurement of an empty screen. See docs/SAT-DESIGN.md
// §14.0c.
//
// These cases pin the three behaviours as pure functions, and then check the
// property that matters downstream: a target under `bounce` stays inside the
// canvas for as long as the run lasts, whatever its motion stack does.

#include <doctest/doctest.h>

#include "world/world_builder.hpp"
#include "scenario/schema.hpp"

#include <cmath>

using namespace sat;

TEST_CASE("row 8: exit leaves the coordinate alone") {
    double v = 5.0;
    CHECK(fold_edge(-37.0,  v, 100.0, EdgeBehaviour::Exit) == -37.0);
    CHECK(fold_edge(1234.0, v, 100.0, EdgeBehaviour::Exit) == 1234.0);
    CHECK(v == 5.0);                       // and the velocity untouched
}

TEST_CASE("row 8: wrap is a sawtooth and does not change the velocity") {
    double v = 5.0;
    CHECK(fold_edge(  0.0, v, 100.0, EdgeBehaviour::Wrap) == doctest::Approx(0.0));
    CHECK(fold_edge( 99.0, v, 100.0, EdgeBehaviour::Wrap) == doctest::Approx(99.0));
    CHECK(fold_edge(101.0, v, 100.0, EdgeBehaviour::Wrap) == doctest::Approx(1.0));
    CHECK(fold_edge(-1.0,  v, 100.0, EdgeBehaviour::Wrap) == doctest::Approx(99.0));
    // Many periods out, which is what an hour-long run would produce.
    CHECK(fold_edge(100000.0 + 42.0, v, 100.0, EdgeBehaviour::Wrap)
          == doctest::Approx(42.0));
    CHECK(v == 5.0);
}

TEST_CASE("row 8: bounce is a triangle wave and flips the velocity on reflection") {
    // Inside: untouched, velocity kept.
    { double v = 5.0;
      CHECK(fold_edge(42.0, v, 100.0, EdgeBehaviour::Bounce) == doctest::Approx(42.0));
      CHECK(v == 5.0); }

    // Just past the far edge: reflected, velocity flipped.
    { double v = 5.0;
      CHECK(fold_edge(101.0, v, 100.0, EdgeBehaviour::Bounce) == doctest::Approx(99.0));
      CHECK(v == -5.0); }

    // Just past zero: reflected off the near edge. Two folds, so the velocity
    // comes back the way it went — which is correct: the coordinate -1 is one
    // unit the wrong side of an edge it has bounced off once.
    { double v = 5.0;
      CHECK(fold_edge(-1.0, v, 100.0, EdgeBehaviour::Bounce) == doctest::Approx(1.0));
      CHECK(v == -5.0); }

    // A full period returns to itself.
    { double v = 5.0;
      CHECK(fold_edge(200.0 + 30.0, v, 100.0, EdgeBehaviour::Bounce)
            == doctest::Approx(30.0)); }

    // The result is ALWAYS inside [0, extent], however far outside the input is.
    for (double x = -100000.0; x <= 100000.0; x += 337.0) {
        double v = 1.0;
        const double f = fold_edge(x, v, 100.0, EdgeBehaviour::Bounce);
        REQUIRE(f >= -1e-9);
        REQUIRE(f <= 100.0 + 1e-9);
    }
}

TEST_CASE("row 8: a bouncing target stays on the canvas for a long run") {
    // The property the defect broke. A linear target started near a corner and
    // run for two simulated minutes must never leave the screen.
    const char* toml = R"(
[meta]
name = "edge"
[sim]
truth_hz = 300
camera_hz = 30
control_hz = 30
duration_s = 120
seed = 42
[world]
canvas_px = [2000, 2000]
edge_behaviour = "bounce"
[camera]
resolution = [640, 480]
fov_deg = [4.0, 3.0]
[target]
intensity = 120.0
size_px = 10
initial_px = [1936.0, 1831.0]
[[target.motion]]
kind = "linear"
velocity_px_s = [22.0, -11.0]
[clutter]
static_sources = 0
decoy_beacons = 0
)";
    auto sc = parse_scenario(toml, "edge");
    REQUIRE_MESSAGE(sc.has_value(), sc.error());

    RngSet rng;
    rng.seed_all(sc.value().seed);
    World w = build_world(sc.value(), rng);
    REQUIRE(w.emitters.n >= 1);

    double worst_x = 0.0, worst_y = 0.0;
    for (int tick = 0; tick <= 120 * 300; ++tick) {
        w.advance(tick / 300.0, 1.0 / 300.0, rng);
        const Pixel2 p = w.emitters.position(0);
        worst_x = std::max(worst_x, std::max(-p.x, p.x - 2000.0));
        worst_y = std::max(worst_y, std::max(-p.y, p.y - 2000.0));
    }
    INFO("worst excursion outside the canvas: ", worst_x, ", ", worst_y, " px");
    CHECK(worst_x <= 1e-6);
    CHECK(worst_y <= 1e-6);
}

TEST_CASE("row 8: exit really does let it leave — the behaviour is a choice") {
    // Not every scenario wants a bounce. CP 8.8's "beacon exits" case needs a
    // target that genuinely goes away, so `exit` must not be quietly clamped.
    const char* toml = R"(
[meta]
name = "edge-exit"
[sim]
truth_hz = 300
camera_hz = 30
control_hz = 30
duration_s = 30
seed = 42
[world]
canvas_px = [2000, 2000]
edge_behaviour = "exit"
[camera]
resolution = [640, 480]
fov_deg = [4.0, 3.0]
[target]
intensity = 120.0
size_px = 10
initial_px = [1936.0, 1831.0]
[[target.motion]]
kind = "linear"
velocity_px_s = [22.0, -11.0]
[clutter]
static_sources = 0
decoy_beacons = 0
)";
    auto sc = parse_scenario(toml, "edge-exit");
    REQUIRE_MESSAGE(sc.has_value(), sc.error());
    RngSet rng;
    rng.seed_all(sc.value().seed);
    World w = build_world(sc.value(), rng);
    for (int tick = 0; tick <= 30 * 300; ++tick) w.advance(tick / 300.0, 1.0 / 300.0, rng);
    CHECK(w.emitters.position(0).x > 2000.0);
}
