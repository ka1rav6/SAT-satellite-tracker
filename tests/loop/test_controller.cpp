// tests/loop/test_controller.cpp — CP 1.7, plus the CP 10.1-10.2 properties
// that the controller is written to have from the start.

#include <doctest/doctest.h>

#include "control/controller.hpp"
#include "plant/gimbal.hpp"

#include <cmath>

using namespace sat;

namespace {
constexpr double kDt = 1.0 / 30.0;   // one control update at 30 Hz
}

TEST_CASE("CP 1.7: a P controller produces a non-zero command when off-centre") {
    AxisController c;
    c.reset(ControlGains::proportional(6.0));

    const double cmd = c.compute(/*error=*/1000.0, /*measurement=*/0.0,
                                 0.0, 0.0, kDt, false);
    CHECK(cmd == doctest::Approx(6000.0));

    // Zero error, zero command.
    c.reset(ControlGains::proportional(6.0));
    CHECK(c.compute(0.0, 0.0, 0.0, 0.0, kDt, false) == doctest::Approx(0.0));

    // And it is signed correctly: a negative error drives the other way.
    c.reset(ControlGains::proportional(6.0));
    CHECK(c.compute(-1000.0, 0.0, 0.0, 0.0, kDt, false) == doctest::Approx(-6000.0));
}

TEST_CASE("the integrator accumulates and is clamped") {
    ControlGains g;
    g.kp = 0.0; g.ki = 1.0; g.kd = 0.0; g.k_ff = 0.0;
    g.i_limit = 100.0;

    AxisController c;
    c.reset(g);

    // Each step adds error*dt to the integral.
    const double out1 = c.compute(30.0, 0.0, 0.0, 0.0, kDt, false);
    CHECK(out1 == doctest::Approx(1.0));      // 30 * (1/30)

    // Drive it into the clamp.
    for (int i = 0; i < 1000; ++i) (void)c.compute(30.0, 0.0, 0.0, 0.0, kDt, false);
    CHECK(c.integrator() == doctest::Approx(100.0));
}

TEST_CASE("conditional integration is the anti-windup (CP 10.2)") {
    // Saturation is frequent here, not exceptional — design §1.3 shows the
    // disturbance can outrun the actuator. Without this, a long saturated slew
    // accumulates a demand that takes just as long to unwind, and the loop
    // overshoots every time it catches up.
    ControlGains g;
    g.kp = 0.0; g.ki = 1.0; g.kd = 0.0; g.k_ff = 0.0;

    AxisController saturated;
    AxisController free;
    saturated.reset(g);
    free.reset(g);

    for (int i = 0; i < 100; ++i) {
        (void)saturated.compute(1000.0, 0.0, 0.0, 0.0, kDt, /*saturated=*/true);
        (void)free.compute(1000.0, 0.0, 0.0, 0.0, kDt, /*saturated=*/false);
    }

    CHECK(saturated.integrator() == doctest::Approx(0.0));   // frozen
    CHECK(free.integrator() > 1000.0);                       // wound up
}

TEST_CASE("the derivative acts on the measurement, not the error") {
    // A setpoint step must NOT produce a derivative kick. The tracker steps its
    // setpoint every time it reacquires or switches target, and a kick there
    // slams the mount — very visible on a live plot, and entirely avoidable.
    ControlGains g;
    g.kp = 0.0; g.ki = 0.0; g.kd = 1.0; g.k_ff = 0.0;

    AxisController c;
    c.reset(g);

    // Settle with a constant measurement.
    (void)c.compute(0.0, 500.0, 0.0, 0.0, kDt, false);
    (void)c.compute(0.0, 500.0, 0.0, 0.0, kDt, false);

    // Now step the SETPOINT: the error jumps but the measurement has not moved.
    const double out = c.compute(10000.0, 500.0, 0.0, 0.0, kDt, false);
    CHECK(out == doctest::Approx(0.0));   // no kick

    // Whereas a moving MEASUREMENT does produce damping.
    const double damped = c.compute(10000.0, 600.0, 0.0, 0.0, kDt, false);
    CHECK(damped < 0.0);                  // opposes the motion
}

TEST_CASE("velocity feedforward passes the target rate straight through") {
    ControlGains g = ControlGains::proportional(0.0);
    g.k_ff = 1.0;

    AxisController c;
    c.reset(g);
    // With no error at all, the command is purely the feedforward term: the
    // controller moves with the target rather than waiting to observe a lag.
    CHECK(c.compute(0.0, 0.0, 12345.0, 0.0, kDt, false) == doctest::Approx(12345.0));
}

TEST_CASE("CP 10.1: feedforward removes the lag on a constantly moving target") {
    // The claim design §10.4 makes: "Pure feedback lags by speed / bandwidth. A
    // beacon at 200 px/s with a 3 Hz loop leaves ~11 px of lag — already over
    // the 10 px budget before any noise." This is that comparison, run against
    // the real plant.
    const double ifov      = deg_to_urad(4.0) / 640.0;
    const double target_px_s = 200.0;
    const double target_rate = target_px_s * ifov;      // urad/s

    // ki is deliberately ZERO here. With integral action the steady-state error
    // eventually goes to zero with or without feedforward — the integrator just
    // takes kp/ki = 16 seconds to get there, which says nothing about
    // feedforward and everything about how long you were willing to wait. (An
    // earlier version of this test ran for 6 s with ki = 0.5 and measured a
    // half-converged integrator.)
    //
    // With ki = 0 the comparison is analytic: proportional control following a
    // ramp settles at exactly error = v / kp, and feedforward removes that term
    // structurally rather than integrating it away. kd is zero for the same
    // reason: derivative action on a constant-rate measurement contributes a
    // constant -kd*v, which shifts the steady state and muddies the arithmetic
    // without changing the conclusion.
    auto settle_error_px = [&](double k_ff) {
        GimbalParams p = GimbalParams::from_dps(5.0, 50.0);
        p.tau_s = 0.020; p.latency_s = 0.010; p.encoder_lsb_urad = 0.0;

        GimbalAxis ax;
        ax.reset(p, 0.0);

        ControlGains g;
        g.kp = 8.0; g.ki = 0.0; g.kd = 0.0; g.k_ff = k_ff;
        AxisController c;
        c.reset(g);

        double target = 0.0;
        double err_px = 0.0;
        // 6 seconds at 30 Hz: long enough to reach steady state.
        for (int i = 0; i < 180; ++i) {
            target += target_rate * kDt;

            // Record the error the CONTROLLER acts on, at the instant it acts.
            // Sampling after the plant has stepped would shift every reading by
            // one control period of target motion — v * dt = 6.7 px at 200 px/s
            // — which is a property of where the tape measure was held, not of
            // the loop.
            const double err_urad = target - ax.position();
            err_px = std::fabs(err_urad) / ifov;

            const double cmd = c.compute(err_urad, ax.position(),
                                         target_rate, 0.0, kDt,
                                         std::fabs(ax.rate()) >= p.max_rate_urad_s * 0.999);
            // Step the plant at 300 Hz between control updates.
            for (int s = 0; s < 10; ++s) ax.step(cmd, 1.0 / 300.0);
        }
        return err_px;
    };

    const double without = settle_error_px(0.0);
    const double with_ff = settle_error_px(1.0);

    MESSAGE("steady-state lag at 200 px/s: feedback only = " << without
            << " px, with feedforward = " << with_ff << " px");

    // Proportional control following a ramp settles at error = v / kp. In
    // pixels that is 200 / 8 = 25 px — two and a half times spec row 17's
    // entire 10 px budget, before a single photon of noise. This is the
    // quantitative form of design §10.4's claim that pure feedback cannot meet
    // the requirement.
    const double predicted = 200.0 / 8.0;
    CHECK(without == doctest::Approx(predicted).epsilon(0.1));
    CHECK(without > 10.0);          // fails the spec on its own

    // Feedforward removes that term structurally. What is left is the plant's
    // transport delay and first-order lag, plus one control period of sampling
    // delay — small, and comfortably inside spec row 17's 10 px budget where
    // feedback alone was two and a half times over it.
    // In this idealised case it is not merely small, it is structurally zero:
    // the feedforward term supplies exactly the rate the target is moving at,
    // so the proportional path has no error left to act on.
    CHECK(with_ff < 1e-6);
    CHECK(with_ff < without / 1000.0);
}

TEST_CASE("bumpless switching: changing gains does not jump the output") {
    // Design §10.6 mandate 2. The SAT supervisor switches gains at runtime, and
    // a discontinuity in the command kicks the loop every time it does.
    ControlGains g1;
    g1.kp = 8.0; g1.ki = 0.5; g1.kd = 0.0; g1.k_ff = 0.0;

    AxisController c;
    c.reset(g1);
    // Wind the integrator up to something substantial.
    for (int i = 0; i < 100; ++i) (void)c.compute(500.0, 0.0, 0.0, 0.0, kDt, false);

    const double before = c.compute(500.0, 0.0, 0.0, 0.0, kDt, false);

    ControlGains g2 = g1;
    g2.ki = 2.0;                    // a 4x change in the integral gain
    c.set_gains(g2);

    const double after = c.compute(500.0, 0.0, 0.0, 0.0, kDt, false);

    // The integral CONTRIBUTION (ki * integ) must be continuous, so the total
    // output changes only by the one step of fresh integration.
    INFO("before = " << before << ", after = " << after);
    CHECK(std::fabs(after - before) < 0.02 * std::fabs(before));
}

TEST_CASE("clear_state wipes history for a fresh acquisition") {
    AxisController c;
    ControlGains g; g.ki = 1.0;
    c.reset(g);
    for (int i = 0; i < 50; ++i) (void)c.compute(1000.0, 0.0, 0.0, 0.0, kDt, false);
    CHECK(c.integrator() > 0.0);

    c.clear_state();
    CHECK(c.integrator() == doctest::Approx(0.0));
    // And the derivative must not fire off the stale measurement.
    CHECK(c.compute(0.0, 9999.0, 0.0, 0.0, kDt, false) == doctest::Approx(0.0));
}

TEST_CASE("platform rate is subtracted, cancelling known drift") {
    ControlGains g = ControlGains::proportional(0.0);
    g.k_ff = 0.0;
    AxisController c;
    c.reset(g);
    CHECK(c.compute(0.0, 0.0, 0.0, 777.0, kDt, false) == doctest::Approx(-777.0));
}
