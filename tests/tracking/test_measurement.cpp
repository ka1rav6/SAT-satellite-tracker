// tests/tracking/test_measurement.cpp — CP 6.1.
//
// "Pixels -> angles via unproject + commanded boresight. Accept when: with the
//  camera slewing, a stationary emitter's angular position stays constant."
//
// This is the checkpoint that proves the tracker's state lives in a frame that
// does not move. It is easy to write a version of this test that passes for the
// wrong reason — if the camera barely moves, a pixel measurement would look
// constant too — so the test below asserts BOTH halves: the image position must
// sweep a long way across the sensor, AND the angle must not move.

#include <doctest/doctest.h>

#include "core/arena.hpp"
#include "engine/synthetic_source.hpp"
#include "perception/pipeline.hpp"
#include "scenario/schema.hpp"
#include "tracking/measurement.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using namespace sat;

namespace {

Scenario load(const char* name) {
    auto r = load_scenario(std::string(SAT_SCENARIO_DIR) + "/" + name);
    REQUIRE_MESSAGE(r.has_value(), r.error());
    return *r;
}

}  // namespace

TEST_CASE("CP 6.1: a stationary emitter holds a constant angle while the camera slews") {
    Scenario sc = load("spec_defaults.toml");

    // A stationary beacon, pinned well off the boresight so that the slew below
    // drags it right across the frame.
    REQUIRE(!sc.targets.empty());
    TargetSpec& t = sc.targets[0];
    t.random_initial = false;
    t.initial_px[0]  = sc.initial_pos_px[0] + 120.0;
    t.initial_px[1]  = sc.initial_pos_px[1];
    for (MotionSpec& m : t.motion) {
        if (m.kind == "constant") {
            m.offset_px[0] = t.initial_px[0];
            m.offset_px[1] = t.initial_px[1];
        }
    }
    // Everything else about the frame is irrelevant to this checkpoint, and a
    // detection that fails for a noise reason would make the result ambiguous.
    sc.targets.resize(1);
    sc.static_sources = 0;
    sc.decoy_beacons  = 0;

    SyntheticSource source;
    source.build_from_scenario(sc);
    // INV-8 in spirit: no disturbance, so `commanded` IS the true boresight and
    // any residual angular drift can only come from the conversion itself.
    source.set_boresight_disturbance(Angle2{});

    const CameraGeometry cam = sc.camera_geometry();

    PerceptionParams pp;
    pp.target_size_px = sc.targets[0].size_px;
    Arena arena;
    arena.reserve(64u << 20);
    PerceptionWorkspace ws;
    REQUIRE(ws.allocate(arena, cam.width, cam.height,
                        structuring_element_size(pp.target_size_px)));
    ClassicalPerception perception;
    perception.configure(pp);
    std::vector<Detection> dets;

    // Slew the camera by hand, 8 px worth of angle per frame. At 30 Hz that is
    // 240 px/s — a quarter of the mount's authority, so it is a realistic slew
    // and not a contrived one — and over 40 frames it drags the beacon 320 px
    // across a 640 px sensor. That margin is the point: the two hypotheses
    // ("the state lives in pixels" and "the state lives in angles") are
    // separated by a factor of several hundred, so the test cannot pass by
    // accident.
    constexpr int kFrames = 40;
    const double  kSlewStep = 8.0 * cam.ifov_urad();   // urad per frame

    std::vector<double> angle_x, angle_y, image_x;
    Angle2 boresight = sc.initial_boresight();

    for (int i = 0; i < kFrames; ++i) {
        boresight.x += kSlewStep;
        SourceFrame frame;
        REQUIRE(source.next(boresight, frame));

        perception.process(frame.pixels, frame.width, frame.height, ws, dets);
        if (dets.empty()) continue;

        // The strongest candidate. With one beacon, no clutter and a clean
        // scenario it is the beacon; the check below would catch it if not.
        const Measurement m = to_measurement(dets[0], cam, boresight,
                                             /*pointing_sigma_urad=*/0.0);
        angle_x.push_back(m.angle.x);
        angle_y.push_back(m.angle.y);
        image_x.push_back(dets[0].centroid_image.x);
    }

    REQUIRE(angle_x.size() >= static_cast<size_t>(kFrames - 2));

    // --- half one: the image position really did move --------------------
    const auto [img_lo, img_hi] = std::minmax_element(image_x.begin(), image_x.end());
    const double image_sweep = *img_hi - *img_lo;
    MESSAGE("image-frame sweep: " << image_sweep << " px");
    CHECK(image_sweep > 250.0);

    // --- half two: the angle did not ------------------------------------
    const auto [ax_lo, ax_hi] = std::minmax_element(angle_x.begin(), angle_x.end());
    const auto [ay_lo, ay_hi] = std::minmax_element(angle_y.begin(), angle_y.end());
    const double spread_x = *ax_hi - *ax_lo;
    const double spread_y = *ay_hi - *ay_lo;
    MESSAGE("angular spread: " << spread_x << " x " << spread_y << " urad ("
            << spread_x / cam.ifov_urad() << " x " << spread_y / cam.ifov_urad() << " px)");

    // -------------------------------------------------------------------
    // What the residual actually is, and why it is not zero.
    //
    // About 0.7 px peak-to-peak. That is NOT a frame error — a frame error
    // would scale with the slew, and this does not. It is the S-CURVE BIAS of
    // an intensity-weighted centroid: the estimator's error is a periodic
    // function of where the beacon's true centre falls WITHIN a pixel, and the
    // slew walks it through every sub-pixel phase, so the full amplitude of
    // that curve shows up as spread. §10.1.3 calls correcting it "the
    // highest-leverage single item" in the project and CP 9.3 does it; until
    // then this is the expected magnitude and it is honest to say so.
    //
    // The assertion is therefore relative, which is what the checkpoint is
    // really claiming: whatever the detector's own error is, the angular
    // measurement does not inherit the camera's motion.
    // -------------------------------------------------------------------
    CHECK(spread_x < 0.02 * image_sweep * cam.ifov_urad());
    CHECK(spread_y < 0.02 * image_sweep * cam.ifov_urad());
    CHECK(spread_x < 1.5 * cam.ifov_urad());
    CHECK(spread_y < 1.5 * cam.ifov_urad());
}

TEST_CASE("CP 6.1: R is floored by the pointing uncertainty, not just the centroid") {
    // A very confident centroid (0.01 px) with a 4 urad pointing uncertainty
    // must not claim 4 urad of accuracy... it must claim slightly more than the
    // pointing term alone, because the two add in quadrature.
    const double ifov = 109.08;
    const double s = measurement_sigma_urad(0.01f, ifov, 4.0);
    CHECK(s > 4.0);
    CHECK(s == doctest::Approx(std::sqrt(1.0908 * 1.0908 + 16.0)).epsilon(1e-6));

    // With no pointing uncertainty at all (video modes, INV-8) it collapses to
    // the centroid term exactly.
    CHECK(measurement_sigma_urad(0.25f, ifov, 0.0) == doctest::Approx(0.25 * ifov));

    // And a hopeless centroid dominates a small pointing term, as it must.
    CHECK(measurement_sigma_urad(3.0f, ifov, 4.0) > 300.0);
}
