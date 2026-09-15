// tests/control/test_stage10.cpp — Stage 10's control checkpoints.
//
// It is its own CTest suite rather than part of `loop` because every case here
// drives the WHOLE engine for several simulated seconds — see the note below
// on why that is the only honest way to test a feedforward whose input is a
// Kalman estimate. That costs minutes, and mixing it into a suite of
// millisecond unit tests just means the unit tests inherit its timeout.
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
    // 3 s: the window scored is 0.5-2.5 s, and simulating past it only costs
    // wall-clock time in CI.
    Scenario off = fast_target(3.0);
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
    Scenario s = fast_target(3.0);
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

// ===========================================================================
// CP 10.3 — platform drift
//
// The checkpoint asks for "platform drift estimation and cancellation", with
// the criterion "with linear platform motion, residual error drops measurably".
//
// IT DOES NOT, AND IT SHOULD NOT, AND THAT IS A RESULT RATHER THAN A GAP.
//
// The design's premise is that platform drift is a disturbance the loop cannot
// see, so it has to be estimated and subtracted. In this architecture it is
// already cancelled, structurally, and adding the subtraction makes things
// worse. The argument is four lines of algebra:
//
//   the platform displaces the TRUE boresight      B_true = B_cmd + D
//   the beacon lands on the sensor at              T - B_true
//   tracking/measurement.hpp reconstructs the
//   angle through the COMMANDED boresight, which
//   is all the system knows                        z = B_cmd + (T - B_true)
//                                                    = T - D
//
// So the tracker does not see the target at T. It sees it at T - D, moving at
// v - D'. Driving B_cmd to T - D puts B_true = B_cmd + D at T — exactly on the
// beacon. The drift is indistinguishable from target motion, the Kalman
// velocity state absorbs it, and CP 10.1's feedforward carries it.
//
// Subtracting a SECOND estimate of D' would remove it twice.
//
// Measured on the 200 px/s target, sweeping the drift from nothing to nearly
// the target's own speed:
//
//   platform drift      0        17       67      170 px/s
//   tracking RMS     3.03      2.99     2.96     3.78 px
//
// The residual is flat. Nothing is being left on the table for an estimator to
// pick up.
//
// WHERE THE DESIGN'S PREMISE WOULD HOLD. If the measurement were reconstructed
// through the true boresight — an IMU-stabilised mount reporting its real
// attitude, say — then z = T, the filter would see only the target's own
// motion, and D' would have to be supplied separately. The platform_rate_est
// argument is kept in the controller's signature for exactly that case, and
// these tests pin the fact that today it must be zero.
// ===========================================================================
namespace {

/// fast_target(), plus spec row 25's linear platform motion at a chosen rate.
Scenario fast_target_with_drift(double duration_s, double vx, double vy) {
    Scenario sc = fast_target(duration_s);
    MotionSpec m;
    m.kind = "linear";
    m.velocity_px_s[0] = vx;
    m.velocity_px_s[1] = vy;
    sc.platform.push_back(m);
    return sc;
}

}  // namespace

TEST_CASE("CP 10.3: platform drift is already cancelled by the measurement frame") {
    const RunResult none = run(fast_target(3.0), 0.5, 2.5);
    // Spec row 25's baseline drift is (15, -8) px/s. 60 px/s is a deliberate
    // exaggeration — four times the specification — so that "no effect" is a
    // claim about the mechanism and not about a disturbance too small to see.
    const RunResult spec = run(fast_target_with_drift(3.0,  15.0,  -8.0), 0.5, 2.5);
    const RunResult hard = run(fast_target_with_drift(3.0,  60.0, -30.0), 0.5, 2.5);

    REQUIRE(none.scored > 45);
    REQUIRE(spec.scored > 45);
    REQUIRE(hard.scored > 45);

    MESSAGE("tracking RMS with platform drift 0 / 17 / 67 px/s: "
            << none.rms_px << " / " << spec.rms_px << " / " << hard.rms_px << " px");

    // Within 40% of the drift-free run at four times the specified drift. This
    // is the whole CP 10.3 finding: there is no residual for an estimator to
    // remove.
    CHECK(hard.rms_px < 1.4 * none.rms_px);
    CHECK(spec.rms_px < 1.4 * none.rms_px);
}

TEST_CASE("CP 10.3: cancelling the drift a second time makes it worse") {
    // The direct test of the double-count, and the reason the checkpoint is
    // reported as a finding rather than implemented as written.
    //
    // AxisController subtracts platform_rate_est from its output. Feeding it
    // the platform's true rate — a PERFECT estimator, better than anything an
    // estimator could achieve — should, if the design's premise held, improve
    // the result. It does the opposite, by exactly the amount the premise is
    // wrong by.
    constexpr double kDriftX = 60.0, kDriftY = -30.0;
    const Scenario sc = fast_target_with_drift(3.0, kDriftX, kDriftY);

    const double ifov = sc.camera_geometry().ifov_urad();
    const Rate2 perfect{kDriftX * ifov, kDriftY * ifov};

    Pipeline zero;  zero.build_from_scenario(sc);
    Pipeline dbl;   dbl.build_from_scenario(sc);
    dbl.set_platform_rate_est(perfect);

    auto score = [&](Pipeline& p) {
        double sum_sq = 0.0; int n = 0;
        while (p.step()) {
            const FrameRecord& r = p.last();
            if (!r.truth_valid || r.time_s < 0.5 || r.time_s >= 2.5) continue;
            if (r.track_state != TrackState::Confirmed) continue;
            sum_sq += r.tracking_error_px * r.tracking_error_px;
            ++n;
        }
        REQUIRE(n > 45);
        return std::sqrt(sum_sq / n);
    };

    const double a = score(zero);
    const double b = score(dbl);
    MESSAGE("platform_rate_est = 0 -> " << a << " px RMS;  "
            "fed the TRUE drift -> " << b << " px RMS");

    CHECK(b > 2.0 * a);
}

// ===========================================================================
// CP 10.6 — saturation fraction, and where the loop breaks down
//
// `just cp106` draws the chart. This pins the two numbers on it, because a
// chart regenerated by hand is not a regression test.
// ===========================================================================
TEST_CASE("CP 10.6: the mount saturates where the geometry says it will") {
    // The prediction, from nothing but the specification:
    //
    //   mount ceiling   spec rows 13-14, 5 deg/s = 87266 urad/s = 800 px/s
    //   demand          the commanded boresight must move at (v_target -
    //                   v_platform); see CP 10.3. With the drift opposing the
    //                   target the magnitudes add, so the demand is 200 + d.
    //   crossing        d = 600 px/s
    //
    // Below it the plant must never be at its stops; above it, it must be.
    auto saturation_at = [](double drift_px_s) {
        Scenario sc = fast_target_with_drift(4.0,
                                             -drift_px_s / std::sqrt(2.0),
                                             -drift_px_s / std::sqrt(2.0));
        Pipeline p;
        p.build_from_scenario(sc);
        while (p.step()) {}
        return p.gimbal().saturation_frac();
    };

    const double below = saturation_at(400.0);   // demand 600 px/s
    const double above = saturation_at(800.0);   // demand 1000 px/s
    MESSAGE("gimbal saturation: " << 100.0 * below << " % at a 600 px/s demand, "
            << 100.0 * above << " % at 1000 px/s (ceiling is 800 px/s)");

    CHECK(below == doctest::Approx(0.0));
    CHECK(above > 0.02);
}

TEST_CASE("CP 10.6: row 17 is lost BEFORE the actuator runs out") {
    // The finding the chart exists to make, and it is not the obvious one:
    // the loop stops holding spec row 17's 10 px while the plant has never
    // once been at its stops. Saturation is not what breaks it.
    //
    // `just cp106`, scoring each run the way the compliance metric does — every
    // confirmed frame, acquisition included:
    //
    //   drift     0    100   200   300   400   500   600   700   800 px/s
    //   demand  200    300   400   500   600   700   800   900  1000 px/s
    //   RMS    3.55   5.47  7.45  9.47 11.48 13.57 16.10 20.69 29.43 px
    //   sat       0      0     0     0     0     0  0.37  2.90  8.34 %
    //
    // Row 17 goes at 400 px/s of drift. Saturation does not start until 600,
    // exactly where the geometry says it should. In between, the loop is
    // failing the requirement with rate authority still in hand.
    //
    // WHAT IS ACTUALLY BREAKING. The error is proportional to the DEMAND and
    // decays within each run. Per window, at demands of 200 and 800 px/s:
    //
    //   t 0.3-1 s    5.21 -> 22.29 px      t 2-4 s   1.48 -> 5.38 px
    //   t 1-2 s      2.21 ->  8.08 px      t 6-8 s   0.53 -> 1.86 px
    //
    // Both the 4x ratio between columns (matching the 4x demand) and the decay
    // down each column are the point. The lag is transport delay — the command
    // that arrives is the one issued 19 ms of target motion ago — and velocity
    // feedforward cannot cancel a pure delay, only a velocity. The integrator
    // grinds it away over its own kp/ki = 4 s, which is far too slow to save
    // the acquisition window that row 16 and row 17 are both scored in.
    //
    // That is the case for CP 10.4's Smith predictor, stated as a number.
    auto whole_run_rms = [](double drift_px_s) {
        const RunResult r = run(fast_target_with_drift(
            8.0, -drift_px_s / std::sqrt(2.0), -drift_px_s / std::sqrt(2.0)), 0.0);
        REQUIRE(r.scored > 150);
        return r.rms_px;
    };

    const double none = whole_run_rms(0.0);     // demand  200 px/s
    const double lost = whole_run_rms(400.0);   // demand  600 px/s

    MESSAGE("whole-run tracking RMS: " << none << " px at a 200 px/s demand, "
            << lost << " px at 600 px/s");

    CHECK(none < 10.0);   // comfortably inside row 17
    CHECK(lost > 10.0);   // outside it, with the plant still off its stops

    // Linear in demand, to within 25%: tripling the demand roughly triples the
    // error. A rate limit would produce a knee, not a line.
    CHECK(lost > 2.2 * none);
    CHECK(lost < 3.8 * none);
}

// ===========================================================================
// CP 10.4 — the Smith predictor
//
// "Bandwidth improves without losing stability margin. If it destabilises,
//  leave it off — optional."
//
// It is off. These tests say why, and they are written so that they state the
// THRESHOLD rather than the verdict: if a future plant has a delay large
// enough for a predictor to earn its place, the first case turns red and says
// so, instead of quietly enshrining today's answer.
// ===========================================================================
TEST_CASE("CP 10.4: the loop is not delay-limited, so the predictor cannot help") {
    // The argument in one number. A Smith predictor buys bandwidth when the
    // DELAY is what is binding. The horizon here is the transport delay plus
    // one control period:
    //
    //   43 ms at 30 Hz with a 10 ms latency
    //
    // and the crossover is kp = 8 rad/s, so the delay costs
    //
    //   8 * 0.043 = 0.34 rad = 20 degrees
    //
    // of phase at crossover, out of a margin that starts near 90. Twenty
    // degrees is not what is limiting this loop; the acceleration limit during
    // acquisition and the rate ceiling at high demand are, and no amount of
    // prediction removes either.
    //
    // 45 degrees is the conventional line below which delay compensation
    // starts to pay. If the plant ever crosses it this fails and the
    // conclusion below has to be revisited.
    const double horizon_s = 0.010 + 1.0 / 30.0;
    const double phase_deg = 8.0 * horizon_s * 180.0 / 3.14159265358979;
    MESSAGE("delay phase at crossover: " << phase_deg << " degrees over a "
            << 1000.0 * horizon_s << " ms horizon");
    CHECK(phase_deg < 45.0);
}

TEST_CASE("CP 10.4: measured on the plant, the predictor costs rather than buys") {
    // The run behind the decision. Scored at a 600 px/s demand — high enough
    // that the lag is the dominant error, which is the regime a delay
    // compensator is supposed to own.
    Scenario off = fast_target_with_drift(6.0, -400.0 / std::sqrt(2.0),
                                               -400.0 / std::sqrt(2.0));
    off.control.smith = false;
    Scenario on = off;
    on.control.smith = true;

    const RunResult a = run(off, 0.0);
    const RunResult b = run(on,  0.0);
    REQUIRE(a.scored > 100);
    REQUIRE(b.scored > 100);

    MESSAGE("600 px/s demand: smith off " << a.rms_px << " px RMS (max "
            << a.max_px << "), smith on " << b.rms_px << " px RMS (max "
            << b.max_px << ")");

    // Not better. Stated as "does not improve" rather than "is worse", because
    // the useful assertion is that turning it ON is not leaving something on
    // the table — that is the claim the default rests on.
    CHECK(b.rms_px > 0.95 * a.rms_px);
}

TEST_CASE("CP 10.4: the predictor's model is a copy, and a wrong one shows") {
    // The risk §10.4 names — "amplifies model error" — demonstrated rather
    // than asserted. The controller's PlantModel is a copy of the plant's
    // numbers precisely so it CAN be wrong; a predictor that reads the plant's
    // own state is not a predictor.
    //
    // Five times the true transport delay is the sort of error a datasheet
    // figure or a bus that got slower would produce.
    const Scenario sc = fast_target_with_drift(6.0, -400.0 / std::sqrt(2.0),
                                                    -400.0 / std::sqrt(2.0));
    const double dt = 1.0 / static_cast<double>(sc.control_hz);

    auto score = [&](bool smith, double model_latency_s) {
        Scenario s2 = sc;
        s2.control.smith = smith;
        Pipeline p;
        p.build_from_scenario(s2);
        if (smith) {
            p.controller_mut().set_plant_model(
                PlantModel{model_latency_s, sc.time_constant_s,
                           deg_to_urad(sc.max_pan_dps),
                           deg_to_urad(sc.max_accel_dps2)}, dt);
        }
        double sum_sq = 0.0; int n = 0;
        while (p.step()) {
            const FrameRecord& r = p.last();
            if (!r.truth_valid) continue;
            if (r.track_state != TrackState::Confirmed) continue;
            sum_sq += r.tracking_error_px * r.tracking_error_px;
            ++n;
        }
        REQUIRE(n > 100);
        return std::sqrt(sum_sq / n);
    };

    const double honest = score(true, sc.latency_s);
    const double wrong  = score(true, 5.0 * sc.latency_s);
    MESSAGE("model latency 10 ms -> " << honest << " px RMS;  50 ms -> "
            << wrong << " px RMS");

    // A model that over-states the delay predicts motion that never happens.
    // The degradation is the point: it is why the predictor is a liability on
    // a loop that was not delay-limited to begin with.
    CHECK(wrong > honest);
}
