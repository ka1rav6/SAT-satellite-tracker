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
    std::vector<double> t_s;        ///< every frame
    std::vector<double> signed_x;   ///< every frame, for the settling analysis
};

// ---------------------------------------------------------------------------
// Run, scoring frames in [from_s, to_s).
//
// A WINDOW, not just a settle time, and the reason is the most interesting
// thing CP 10.1 turned up. With integral action the tracking lag is a TRANSIENT,
// not a steady state: a PI loop against a constant velocity eventually nulls
// the error with time constant kp/ki = 4 s. Measured on the 200 px/s target
// with k_ff = 0:
//
//   t 0.5-1.5 s   24.3 px RMS        t 3-5 s    11.2 px
//   t 1.5-3.0 s   17.5 px            t 8-11 s    2.8 px
//
// So a whole-run RMS of the feedback-only arm is mostly a statement about how
// long the run was. Scored over eleven seconds it looks like 13 px; scored over
// the two seconds after acquisition it is 24 px. Neither is wrong, but only one
// of them is about the loop's response to a moving target.
//
// The window used below is the one spec row 16 cares about — the first couple
// of seconds, when the system has just acquired and is supposed to be inside
// row 17's budget. "It will be within budget in another four seconds" is not
// what the requirement says.
// ---------------------------------------------------------------------------
RunResult run(const Scenario& sc, double from_s, double to_s = 1e9) {
    Pipeline p;
    p.build_from_scenario(sc);

    RunResult r;
    double sum_sq = 0.0, sum_x = 0.0;
    while (p.step()) {
        const FrameRecord& rec = p.last();
        if (!rec.truth_valid) continue;
        const Pixel2 bore = sc.screen_geometry().to_pixel(rec.boresight_true);
        r.t_s.push_back(rec.time_s);
        r.signed_x.push_back(bore.x - rec.truth_screen.x);
        if (rec.time_s < from_s || rec.time_s >= to_s) continue;
        if (rec.track_state != TrackState::Confirmed) continue;

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

    // The first two seconds after acquisition: see the note on run().
    const RunResult a = run(off, 0.5, 2.5);
    const RunResult b = run(on,  0.5, 2.5);

    REQUIRE(a.scored > 45);
    REQUIRE(b.scored > 45);

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
    CHECK(a.rms_px < 1.4 * predicted_lag_px);

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
    const RunResult unity = run(s, 0.5, 2.5);
    s.control.k_ff = 1.5;
    const RunResult over = run(s, 0.5, 2.5);

    MESSAGE("k_ff = 1.0: " << unity.rms_px << " px;  k_ff = 1.5: "
            << over.rms_px << " px");
    CHECK(over.rms_px > unity.rms_px);

    // Over-feeding puts the mount AHEAD of the target: a positive signed error
    // where the feedback-only case had a negative one. That sign change is the
    // direct evidence that k_ff is doing what the name says.
    CHECK(over.mean_signed_x > 0.0);
}


TEST_CASE("CP 10.1: on a manoeuvring target the integrator cannot substitute") {
    // The strongest form of the argument, and the one that makes feedforward a
    // structural fix rather than a faster one.
    //
    // A PI loop DOES eventually null a constant-velocity lag — the integrator
    // gets there in about kp/ki = 4 s. That makes the constant-velocity
    // comparison partly a statement about run length, and invites the
    // objection "so just wait". The objection dies on a target whose velocity
    // keeps changing: the integrator is always chasing the velocity the target
    // had a few seconds ago, and it never arrives. The feedforward is right
    // immediately, because it is fed the velocity rather than deducing it from
    // accumulated error.
    //
    // A sinusoid at 200 px/s peak with a 4 s period is chosen so the period is
    // comparable to the integrator's own time constant: fast enough that it
    // cannot converge, slow enough that the Kalman filter tracks the velocity
    // well and the comparison is about the CONTROLLER.
    Scenario sc;
    sc.duration_s = 10.0;
    sc.seed       = 42;
    sc.static_sources = 0;
    sc.decoy_beacons  = 0;
    sc.jitter_px_per_frame = 0.0;
    sc.initial_pos_px[0] = 999.5;
    sc.initial_pos_px[1] = 999.5;

    TargetSpec t;
    t.size_px        = 10;
    t.intensity      = 120.0;
    t.random_initial = false;
    t.initial_px[0]  = 999.5;
    t.initial_px[1]  = 999.5;
    MotionSpec m;
    m.kind          = "sinusoid";
    m.axis          = 0;
    m.period_s      = 4.0;
    // peak speed = 2*pi*A/T, so A = v*T/(2*pi) for a 200 px/s peak.
    m.amplitude_px[0] = kSpeedPxS * m.period_s / (2.0 * 3.14159265358979);
    t.motion.push_back(m);
    sc.targets.push_back(t);

    Scenario off = sc; off.control.k_ff = 0.0;
    Scenario on  = sc; on.control.k_ff  = 1.0;

    // Scored LATE, from 4 s on — after the integrator has had more than its
    // own time constant to converge. If it were going to substitute for the
    // feedforward, this is the window where it would have.
    const RunResult a = run(off, 4.0);
    const RunResult b = run(on,  4.0);
    REQUIRE(a.scored > 100);
    REQUIRE(b.scored > 100);

    MESSAGE("sinusoid, 200 px/s peak, scored after 4 s: k_ff = 0 -> "
            << a.rms_px << " px RMS, k_ff = 1 -> " << b.rms_px << " px RMS");

    // Measured 25.6 -> 16.6 px: a 35% cut, not the 8x the constant-velocity
    // case gives. That gap is the honest part of this test and is worth
    // stating rather than tuning around.
    //
    // What is left is ACCELERATION lag. Velocity feedforward cancels the term
    // proportional to v; a sinusoid also has a term proportional to a, and
    // nothing in the loop feeds that forward. At 200 px/s peak on a 4 s period
    // the peak acceleration is 2*pi/T * v = 314 px/s^2, and a loop with
    // bandwidth kp trails an accelerating target by roughly a/kp^2 = 4.9 px on
    // top of whatever else is happening — plus the Kalman filter's own lag,
    // since a constant-velocity model is BY CONSTRUCTION wrong about an
    // accelerating target and its velocity estimate trails too.
    //
    // Both of those are CP 10.5's problem: the constant-acceleration and
    // coordinated-turn models in the IMM exist precisely so the estimate fed
    // forward is not always a constant-velocity extrapolation. This check is
    // therefore deliberately set at what velocity feedforward alone can
    // deliver, and CP 10.5 is expected to move it.
    CHECK(b.rms_px < 0.75 * a.rms_px);
}

// ===========================================================================
// CP 10.2 — anti-windup
// ===========================================================================
namespace {

// ---------------------------------------------------------------------------
// The CP 10.2 scenario, mirroring scenarios/control/slew.toml.
//
// A STATIONARY beacon 375 px off the boresight. 375 px matters: the error that
// commands the mount's rate ceiling is 87266 urad/s / kp 8 / 109.08 urad per px
// = 100 px, so the loop asks for 3.75x what the motor can deliver and stays
// saturated for most of the slew. A slew that never saturates is not a test of
// anti-windup, it is a test of damping.
//
// Stationary, because CP 10.1 already measures the feedforward separately and a
// moving target would mix its contribution into the settling shape. Here the
// feedforward input is zero by construction.
// ---------------------------------------------------------------------------
Scenario slew_target(double duration_s) {
    Scenario sc;
    sc.duration_s = duration_s;
    sc.seed       = 42;
    sc.static_sources      = 0;
    sc.decoy_beacons       = 0;
    sc.jitter_px_per_frame = 0.0;   // ringing is a question about SHAPE

    TargetSpec t;
    t.size_px        = 10;
    t.intensity      = 120.0;
    t.random_initial = false;
    t.initial_px[0]  = 999.5 + 300.0;
    t.initial_px[1]  = 999.5 + 225.0;
    MotionSpec m;
    // Stationary expressed as `linear` at zero velocity, NOT as `constant`:
    // world_builder.cpp treats a stack containing a `constant` component as
    // expressing an ABSOLUTE position and bases it at the origin, so
    // `constant` discards initial_px entirely.
    m.kind = "linear";
    m.velocity_px_s[0] = 0.0;
    m.velocity_px_s[1] = 0.0;
    t.motion.push_back(m);
    sc.targets.push_back(t);
    return sc;
}

struct Settling {
    double first_crossing_s = -1.0;   ///< when the error first changes sign
    double overshoot_px     = 0.0;    ///< worst excursion after that
    int    lobes            = 0;      ///< alternating excursions above the floor
};

// ---------------------------------------------------------------------------
// Settling analysis, and why "count the zero crossings" is the wrong test.
//
// The obvious ringing metric is the number of sign changes after the loop first
// reaches the target. It is wrong here, and measurably so: at ki = 8 the loop
// holds the error to 0.12 px RMS, so sensor noise flips the sign constantly and
// a crossing counter reports violent ringing on the BEST-regulated run in the
// set. A crossing of a zero the loop is holding to a tenth of a pixel is the
// loop working.
//
// So a lobe only counts if its PEAK exceeds a floor. Ringing is an alternating
// sequence of excursions of real size; anything under the floor is noise.
// ---------------------------------------------------------------------------
Settling settle(const RunResult& r, double floor_px) {
    Settling out;
    if (r.signed_x.empty()) return out;
    const int sign0 = r.signed_x[0] > 0.0 ? 1 : -1;

    size_t start = r.signed_x.size();
    for (size_t i = 0; i < r.signed_x.size(); ++i) {
        if (r.signed_x[i] * sign0 < 0.0) { start = i; out.first_crossing_s = r.t_s[i]; break; }
    }

    int cur_sign = 0;
    double cur_peak = 0.0;
    for (size_t i = start; i < r.signed_x.size(); ++i) {
        const int s = r.signed_x[i] >= 0.0 ? 1 : -1;
        if (s != cur_sign) {
            if (cur_sign != 0 && cur_peak > floor_px) ++out.lobes;
            cur_sign = s;
            cur_peak = std::fabs(r.signed_x[i]);
        } else {
            cur_peak = std::max(cur_peak, std::fabs(r.signed_x[i]));
        }
        out.overshoot_px = std::max(out.overshoot_px, std::fabs(r.signed_x[i]));
    }
    if (cur_sign != 0 && cur_peak > floor_px) ++out.lobes;
    return out;
}

}  // namespace

TEST_CASE("CP 10.2: a full-field slew settles cleanly with no ringing") {
    Scenario sc = slew_target(8.0);
    const RunResult r = run(sc, 5.0);   // the last three seconds
    const Settling s  = settle(r, /*floor_px=*/2.0);

    REQUIRE(s.first_crossing_s > 0.0);
    MESSAGE("375 px slew: reached the target at " << s.first_crossing_s
            << " s, overshoot " << s.overshoot_px << " px, "
            << s.lobes << " lobe(s), settled residual "
            << r.rms_px << " px RMS");

    // The mount is rate-limited at 5 deg/s = 87266 urad/s, so 375 px of screen
    // (40906 urad) cannot be crossed faster than 0.47 s however it is driven.
    // Anything much slower means the acceleration limit or the lag is
    // dominating rather than the rate ceiling, and the run is not testing what
    // it claims to.
    CHECK(s.first_crossing_s > 0.40);
    CHECK(s.first_crossing_s < 1.20);

    // "Settles cleanly with no ringing": ONE overshoot lobe and then nothing
    // above the floor. Two or more alternating lobes of real amplitude is
    // ringing and would fail here.
    CHECK(s.lobes == 1);

    // And the overshoot stays inside spec row 17's budget — which is the
    // criterion ki = 2.0 was chosen by. See controller.hpp for the table.
    CHECK(s.overshoot_px < 10.0);

    // It really does settle: the last three seconds are quiet.
    CHECK(r.rms_px < 3.0);
}

TEST_CASE("CP 10.2: anti-windup is worth measuring, not just asserting") {
    // The counterfactual. Design §10.4 calls anti-windup "essential", and a
    // claim like that is worth exactly as much as the run without it.
    Scenario on = slew_target(8.0);
    Scenario off = on;
    off.control.anti_windup = false;

    const RunResult a = run(on,  5.0);
    const RunResult b = run(off, 5.0);
    const Settling sa = settle(a, 2.0);
    const Settling sb = settle(b, 2.0);

    MESSAGE("slew overshoot: " << sa.overshoot_px << " px with anti-windup, "
            << sb.overshoot_px << " px without");

    // Measured at roughly 2x across every integral gain from 0.5 to 16, so 1.4
    // is a floor with margin rather than a fit to one number.
    CHECK(sb.overshoot_px > 1.4 * sa.overshoot_px);

    // Without it the overshoot leaves row 17's budget, which is the concrete
    // statement of what the safeguard buys.
    CHECK(sb.overshoot_px > 10.0);
}
