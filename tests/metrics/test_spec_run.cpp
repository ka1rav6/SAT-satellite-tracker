// tests/metrics/test_spec_run.cpp — the whole system, at the SPECIFICATION's
// own parameters, measured with the §13.1 definitions.
//
// ---------------------------------------------------------------------------
// WHY THIS FILE EXISTS
// ---------------------------------------------------------------------------
// Every Stage 6 test passed while the system was, on the specification's own
// default scenario, holding lock on 0.6% of frames. They passed because every
// one of them built its Scenario by hand and set jitter to zero — reasonably,
// since each was testing one mechanism in isolation. The result was a suite
// that could not see a whole class of failure, which is worse than no suite,
// because it gets trusted.
//
// So this file runs scenarios/baseline.toml — spec row 23's jitter, row 25's
// platform motion, row 21's noise, §9.1's clutter, all at their stated values —
// and asserts the GRADED requirements from §3.2 rather than any internal
// property. If the loop stops working at the specification's parameters, this
// is what says so.
//
// The one deviation from baseline.toml is the beacon's initial position: row 11
// permits "random", and §10.5 derives that a cold search of the whole screen
// cannot meet row 16 by geometry. A run that spends its length searching
// measures acquisition, which is a different question and has its own
// (honestly labelled) metric. Pinning the beacon in view measures the loop.

#include <doctest/doctest.h>

#include "engine/pipeline.hpp"
#include "metrics/collector.hpp"
#include "scenario/schema.hpp"

#include <cmath>
#include <string>

using namespace sat;

namespace {

Scenario spec_scenario(bool clutter) {
    auto r = load_scenario(std::string(SAT_SCENARIO_DIR) + "/baseline.toml");
    REQUIRE_MESSAGE(r.has_value(), r.error());
    Scenario sc = *r;
    sc.duration_s = 10.0;

    // Row 11: pin it in view. Everything else is left exactly as the
    // specification's defaults have it — in particular jitter stays at row 23's
    // 20 px/frame, which is the value that broke the tracker.
    REQUIRE(!sc.targets.empty());
    sc.targets[0].random_initial = false;
    sc.targets[0].initial_px[0]  = sc.initial_pos_px[0] + 40.0;
    sc.targets[0].initial_px[1]  = sc.initial_pos_px[1] + 20.0;

    if (!clutter) { sc.static_sources = 0; sc.decoy_beacons = 0; }
    return sc;
}

RunMetrics run(const Scenario& sc) {
    Pipeline p;
    p.build_from_scenario(sc);
    p.set_publish_snapshots(false);

    MetricCollector m;
    m.begin(sc.name, sc.seed, sc.camera_geometry().ifov_urad(),
            static_cast<size_t>(sc.duration_s * sc.camera_hz) + 2);
    while (p.step()) m.add(p.last());
    return m.finish(p.timers(), 1.0, p.gimbal().saturation_frac());
}

}  // namespace

TEST_CASE("the loop holds lock at the specification's own jitter (row 23)") {
    const Scenario sc = spec_scenario(/*clutter=*/false);
    REQUIRE(sc.jitter_px_per_frame == doctest::Approx(20.0));   // row 23, unmodified

    const RunMetrics m = run(sc);
    MESSAGE("in-FOV " << m.frames_in_fov << " frames, confirmed " << m.frames_confirmed
            << ", retention " << (100.0 * m.lock_retention_rate) << "%");
    MESSAGE("acquisition (in view) " << m.acquisition_in_fov_s << " s");
    MESSAGE("tracking " << m.tracking_rms_px << " px RMS, centroid (image) "
            << m.centroid_rmse_image_px << " px RMSE");

    // The beacon is in view for essentially the whole run: the mount follows it.
    // This is the assertion that fails outright when the gate is mis-sized —
    // the camera slews away and never comes back.
    CHECK(m.frames_in_fov > static_cast<int64_t>(0.95 * m.frames_total));

    // Spec row 18: target loss < 5%.
    CHECK(m.lock_retention_rate > 0.95);
    CHECK(m.target_loss_frac < sc.target_loss_frac);

    // Spec row 16, in the sense §10.5 says is the intended one.
    CHECK(m.acquired);
    CHECK(m.acquisition_in_fov_s < sc.acquisition_s);

    // The DETECTOR is unaffected by any of this — it sees one frame at a time
    // and jitter does not move the beacon within the frame. Sub-pixel, and the
    // number to watch as §10.1.3's bias correction lands at CP 9.3.
    CHECK(m.centroid_rmse_image_px < 1.0);
}

TEST_CASE("tracking error at spec-row-23 jitter sits on its derived floor") {
    // -------------------------------------------------------------------
    // A BOUND, DERIVED AND STATED — the same treatment §10.5 gives acquisition.
    //
    // §13.1 defines tracking_error as |camera boresight - true beacon angle|,
    // and spec row 17 caps it at 10 px. Spec row 23 independently specifies a
    // camera jitter of up to +/-20 px per frame, which degrade/disturbance.cpp
    // draws uniformly and applies to the TRUE boresight.
    //
    // Those two requirements are in tension, and the arithmetic is short. For a
    // uniform distribution on [-A, A] the variance is A^2/3, so with A = 20 px
    // each axis contributes 133.3 px^2 and the magnitude over two independent
    // axes has
    //
    //     RMS = sqrt(2 * 400/3) = 16.33 px
    //
    // That is a FLOOR on row 17's metric which no controller can go below,
    // because the disturbance displaces the boresight after the command has
    // been issued and the encoder never observes it. A system reporting under
    // 10 px here would either not be applying the specified jitter or not be
    // measuring against the true boresight.
    //
    // This is reported rather than engineered around, exactly as §10.5's
    // acquisition bound is. The controllable part — how well the loop tracks
    // the beacon apart from the jitter — is what the no-jitter arm below
    // measures, and it is comfortably inside budget.
    // -------------------------------------------------------------------
    const double kJitterFloorPx = std::sqrt(2.0 * 400.0 / 3.0);

    Scenario sc = spec_scenario(/*clutter=*/false);
    const RunMetrics with = run(sc);

    sc.jitter_px_per_frame = 0.0;
    const RunMetrics without = run(sc);

    MESSAGE("tracking RMS: " << with.tracking_rms_px << " px with row-23 jitter, "
            << without.tracking_rms_px << " px without; derived jitter floor "
            << kJitterFloorPx << " px");

    // The jittered figure sits on the floor, not far above it: the loop is not
    // adding error of its own on top of the disturbance.
    CHECK(with.tracking_rms_px > 0.8 * kJitterFloorPx);
    CHECK(with.tracking_rms_px < 1.5 * kJitterFloorPx);

    // And with the disturbance removed, spec row 17 is met with room to spare.
    // This is the number that measures the CONTROLLER.
    CHECK(without.tracking_rms_px < sc.tracking_error_px);
}

TEST_CASE("clutter costs tracking accuracy, and the cost is measured not hidden") {
    // §9.1 makes some of the 120 static sources BRIGHTER than the beacon on
    // purpose. Nothing in Stage 6 can tell a bright point source from another
    // bright point source by appearance, so the tracker sometimes locks onto
    // clutter — which is CP 4.11's finding restated for the classical pipeline,
    // and whose designed answers are §10.2's priority policy (Stage 12) and
    // CandidateNet (Stage 11).
    //
    // The point of this test is not to assert that it works. It is to pin the
    // CURRENT number so that Stage 11 and 12 have a baseline to beat, and so
    // that a regression between here and there is visible.
    const RunMetrics clean   = run(spec_scenario(/*clutter=*/false));
    const RunMetrics cluttered = run(spec_scenario(/*clutter=*/true));

    MESSAGE("tracking RMS: " << clean.tracking_rms_px << " px clean, "
            << cluttered.tracking_rms_px << " px with 120 clutter + 1 decoy");
    MESSAGE("centroid (image) RMSE: " << clean.centroid_rmse_image_px << " px clean, "
            << cluttered.centroid_rmse_image_px << " px cluttered");

    // The lock itself survives — the tracker holds SOMETHING, consistently.
    CHECK(cluttered.lock_retention_rate > 0.95);
    // And the honest part: it is measurably worse, and by how much is recorded.
    CHECK(cluttered.tracking_rms_px > clean.tracking_rms_px);
}
