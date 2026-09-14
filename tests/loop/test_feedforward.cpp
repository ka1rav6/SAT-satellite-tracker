// tests/loop/test_feedforward.cpp — Stage 10's control checkpoints.
//
// CP 10.1  Velocity feedforward.
//          "On a 200 px/s target, tracking error drops from ~11 px to under
//           4 px; on/off comparison plot captured."
//
// CP 10.2  Anti-windup (conditional integration).
//          "A full-field slew settles cleanly with no ringing."
//
// ---------------------------------------------------------------------------
// WHY THESE RUN THE WHOLE PIPELINE
// ---------------------------------------------------------------------------
// tests/loop/test_controller.cpp already checks the controller in isolation —
// that kp scales the output, that the integrator freezes when told to, that the
// derivative does not kick on a setpoint step. Those are necessary and they are
// not what these two checkpoints claim.
//
// The feedforward's input is the KALMAN FILTER's velocity estimate, and the
// interesting question is whether that estimate, produced from noisy centroids
// through a gated association, is good enough to cancel the lag in a real run.
// Handing the controller an exact velocity would answer a question nobody
// asked. So these drive the full engine on the same scenario the checkpoint's
// plot is drawn from, and differ from it only in duration.

#include <doctest/doctest.h>

#include "engine/pipeline.hpp"
#include "scenario/scenario.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace sat;

namespace {

// ---------------------------------------------------------------------------
// The CP 10.1 scenario, in code.
//
// It mirrors scenarios/control/fast_linear.toml — the file the checkpoint's
// plot comes from — and the file carries the full argument for each choice.
// The short version: a 200 px/s target, nothing in the frame but the beacon,
// and no row-23 jitter, because jitter puts a 16.33 px floor under every
// tracking figure and an 11 px lag under a 16 px floor is not a measurement of
// the lag.
//
// Noise is KEPT. A noiseless image would make the centroid exact and the
// velocity estimate with it, and the test would be measuring an ideal
// feedforward rather than the one that ships.
// ---------------------------------------------------------------------------
constexpr double kSpeedPxS = 200.0;

Scenario fast_target(double duration_s) {
    Scenario sc;
    sc.duration_s = duration_s;
    sc.seed       = 42;

    // The camera starts on the target: a 700 px acquisition slew would
    // dominate a five-second RMS with something that is not lag.
    sc.initial_pos_px[0] = 300.0;
    sc.initial_pos_px[1] = 300.0;

    sc.static_sources      = 0;
    sc.decoy_beacons       = 0;
    sc.jitter_px_per_frame = 0.0;

    TargetSpec t;
    t.size_px        = 10;
    t.intensity      = 120.0;
    t.random_initial = false;
    t.initial_px[0]  = 300.0;
    t.initial_px[1]  = 300.0;
    MotionSpec m;
    m.kind = "linear";
    // |v| = 200.000 px/s exactly, on the diagonal, so the run never reaches an
    // edge: 300 + 141.42 * duration stays inside 2000 for duration <= 12 s.
    m.velocity_px_s[0] = kSpeedPxS / std::sqrt(2.0);
    m.velocity_px_s[1] = kSpeedPxS / std::sqrt(2.0);
    t.motion.push_back(m);
    sc.targets.push_back(t);
    return sc;
}

struct RunResult {
    double rms_px    = 0.0;   ///< over the settled portion only
    double mean_signed_x = 0.0;
    double max_px    = 0.0;
    int    scored    = 0;
    std::vector<double> error_px;   ///< every frame, for the ringing check
};

// Run, scoring only frames at or after `settle_s`. The first second is the
// filter converging on a velocity it has never seen, which is a transient and
// not the steady-state lag either checkpoint is about.
RunResult run(const Scenario& sc, double settle_s) {
    Pipeline p;
    p.build_from_scenario(sc);

    RunResult r;
    double sum_sq = 0.0, sum_x = 0.0;
    while (p.step()) {
        const FrameRecord& rec = p.last();
        r.error_px.push_back(rec.truth_valid ? rec.tracking_error_px : 0.0);
        if (!rec.truth_valid || rec.time_s < settle_s) continue;
        if (rec.track_state != TrackState::Confirmed) continue;

        const Pixel2 bore = sc.screen_geometry().to_pixel(rec.boresight_true);
        sum_x  += bore.x - rec.truth_screen.x;
        sum_sq += rec.tracking_error_px * rec.tracking_error_px;
        r.max_px = std::max(r.max_px, rec.tracking_error_px);
        ++r.scored;
    }
    if (r.scored > 0) {
        r.rms_px        = std::sqrt(sum_sq / r.scored);
        r.mean_signed_x = sum_x / r.scored;
    }
    return r;
}

}  // namespace

// ===========================================================================
// CP 10.1
// ===========================================================================
TEST_CASE("CP 10.1: velocity feedforward removes the tracking lag") {
    Scenario off = fast_target(6.0);
    off.control.k_ff = 0.0;
    Scenario on = off;
    on.control.k_ff = 1.0;

    const RunResult a = run(off, 1.5);
    const RunResult b = run(on,  1.5);

    REQUIRE(a.scored > 90);
    REQUIRE(b.scored > 90);

    MESSAGE("k_ff = 0: " << a.rms_px << " px RMS (signed x " << a.mean_signed_x
            << " px);  k_ff = 1: " << b.rms_px << " px RMS (signed x "
            << b.mean_signed_x << " px)");

    // -----------------------------------------------------------------------
    // The checkpoint's number, and where ours differs from the design's.
    //
    // §10.4 quotes "~11 px" of lag for a 200 px/s target on a 3 Hz loop. Our
    // kp is 8 rad/s, which is 1.27 Hz, so the predicted lag is v/kp =
    // 200/8 = 25 px rather than 11 — a bigger number for a slower loop, and
    // the formula, not the tuning, is what the checkpoint is really asserting.
    // So the "before" bound is checked against v/kp with a margin rather than
    // against the design's literal 11 px, which would fail for the right
    // reason and read as a defect.
    // -----------------------------------------------------------------------
    const double predicted_lag_px = kSpeedPxS / 8.0;
    CHECK(a.rms_px > 0.6 * predicted_lag_px);

    // The sign matters and is the whole reason mean_signed_x is computed: a
    // feedback-only loop must sit BEHIND the target. If this ever comes out
    // positive, something upstream is adding a lead — which is exactly the
    // defect CP 10.1 found in the aim point, where a one-frame-ahead setpoint
    // was double-counting the velocity and putting the mount 6.7 px AHEAD.
    CHECK(a.mean_signed_x < 0.0);

    // And the checkpoint's acceptance criterion.
    CHECK(b.rms_px < 4.0);
    CHECK(b.rms_px < 0.35 * a.rms_px);
}

TEST_CASE("CP 10.1: the feedforward gain is not a tuned constant") {
    // A guard against the failure this checkpoint actually had. The optimum
    // sat at k_ff = 0.75 rather than 1.0, because the aim point was already
    // being predicted a frame ahead and the velocity was therefore fed twice.
    // Tuning k_ff down to 0.75 would have hidden a gain that silently depends
    // on kp and the frame rate.
    //
    // With the double lead removed, 1.0 is correct on its own terms and
    // over-feeding must make things WORSE. If a future change reintroduces a
    // lead anywhere in the setpoint path, this is what catches it.
    Scenario s = fast_target(6.0);
    s.control.k_ff = 1.0;
    const RunResult unity = run(s, 1.5);
    s.control.k_ff = 1.5;
    const RunResult over = run(s, 1.5);

    MESSAGE("k_ff = 1.0: " << unity.rms_px << " px;  k_ff = 1.5: "
            << over.rms_px << " px");
    CHECK(over.rms_px > unity.rms_px);

    // Over-feeding puts the mount AHEAD of the target: a positive signed error
    // where the feedback-only case had a negative one. That sign change is the
    // direct evidence that k_ff is doing what the name says.
    CHECK(over.mean_signed_x > 0.0);
}
