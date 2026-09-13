// control/controller.hpp — the pointing controller.
//
// Design §10.4. Three things in here earn their place, and the comments say why
// for each, because two of them look like optional polish and are not.
//
// ---------------------------------------------------------------------------
// 1. VELOCITY FEEDFORWARD — "the highest-value ten lines in the project"
// ---------------------------------------------------------------------------
// Pure feedback lags by roughly speed / bandwidth. A beacon crossing at
// 200 px/s with a 3 Hz loop leaves ~11 px of steady-state lag, which is already
// over spec row 17's 10 px budget before any noise is added. Feeding the
// estimated target velocity forward removes that lag structurally: the
// controller stops waiting to observe an error before moving.
//
// CP 15.2 turns this off live in the demo and watches the error trace blow up,
// which is the single most convincing thing in the presentation.
//
// ---------------------------------------------------------------------------
// 2. ANTI-WINDUP — essential here, not a nicety
// ---------------------------------------------------------------------------
// Saturation is FREQUENT in this system, not exceptional: §1.3 shows the
// disturbance can exceed the actuator's authority outright. An integrator that
// keeps accumulating while the motor is already at its ceiling will have wound
// up a large demand by the time the error reverses, and the loop then overshoots
// badly. Conditional integration (stop integrating while saturated) is the
// cheapest correct fix and costs one branch.
//
// ---------------------------------------------------------------------------
// 3. DERIVATIVE ON MEASUREMENT, NOT ON ERROR
// ---------------------------------------------------------------------------
// If the setpoint steps — which it does every time the tracker switches target
// or reacquires — the derivative of the ERROR contains an impulse, and the
// controller kicks the mount hard. Differentiating the measurement instead gives
// the same damping with no derivative kick. This is standard practice and is
// free; getting it wrong is very visible on a live plot.

#pragma once

#include "core/units.hpp"

#include <cmath>

namespace sat {

// ---------------------------------------------------------------------------
// ControlGains — one axis's tuning.
//
// Units: error is in microradians, output is in microradians per second, so kp
// has units of 1/s and is literally the loop bandwidth in radians per second.
// kp = 8.0 means a ~1.3 Hz closed-loop bandwidth, which is a sane starting point
// for a mount with a 20 ms lag and a 10 ms transport delay.
// ---------------------------------------------------------------------------
struct ControlGains {
    double kp      = 8.0;
    double ki      = 0.5;
    double kd      = 0.15;
    double k_ff    = 1.0;    ///< velocity feedforward; 1.0 = full, 0.0 = off
    double i_limit = 2.0e5;  ///< integrator clamp, urad*s

    /// A pure proportional controller — CP 1.7's "cmd = kp * (detection −
    /// boresight)". Kept as a named factory because the demo switches between
    /// P, PID and PID+FF live (CP 15.2) and the report's ablation table needs
    /// each configuration to be nameable.
    [[nodiscard]] static ControlGains proportional(double kp_) noexcept {
        return ControlGains{kp_, 0.0, 0.0, 0.0, 0.0};
    }
    [[nodiscard]] static ControlGains pid(double kp_, double ki_, double kd_) noexcept {
        ControlGains g; g.kp = kp_; g.ki = ki_; g.kd = kd_; g.k_ff = 0.0; return g;
    }
};

// ---------------------------------------------------------------------------
// AxisController — one axis of the loop.
// ---------------------------------------------------------------------------
class AxisController {
public:
    void reset(const ControlGains& g) noexcept {
        g_          = g;
        integ_      = 0.0;
        prev_meas_  = 0.0;
        have_prev_  = false;
    }

    /// Change gains without kicking the loop.
    ///
    /// Design §10.6 calls this "bumpless switching" and makes it one of three
    /// mandatory properties of the SAT supervisor: "Carry the integrator across
    /// a gain change, or the switch kicks the loop."
    ///
    /// Carrying the raw integrator is not enough when ki changes, because the
    /// integral TERM is ki * integ_ and that is what the plant sees. Rescaling
    /// keeps the contributed term continuous, so the output does not jump.
    void set_gains(const ControlGains& g) noexcept {
        if (g.ki != 0.0 && g_.ki != 0.0) {
            integ_ *= g_.ki / g.ki;
        } else if (g.ki == 0.0) {
            integ_ = 0.0;          // no integral path to carry into
        }
        g_ = g;
        integ_ = clamp_abs(integ_, g_.i_limit);
    }

    // -----------------------------------------------------------------------
    // compute — one control update.
    //
    //   error              setpoint − measurement, microradians
    //   measurement        the encoder reading, microradians (for the D term)
    //   target_rate_est    the tracker's estimate of how fast the target is
    //                      moving, urad/s — the feedforward input
    //   platform_rate_est  estimated platform drift to cancel, urad/s
    //   dt                 seconds
    //   saturated          whether the plant is currently at its rate limit
    //
    // Returns a commanded rate in urad/s. The caller clamps it to the axis
    // limit; the plant clamps it again regardless, because a controller must
    // never be trusted to respect a physical limit on its own.
    // -----------------------------------------------------------------------
    [[nodiscard]] double compute(double error, double measurement,
                                 double target_rate_est, double platform_rate_est,
                                 double dt, bool saturated) noexcept {
        // --- derivative, on the measurement ---------------------------------
        double d = 0.0;
        if (have_prev_ && dt > 0.0) {
            // Negated because d/dt(error) = −d/dt(measurement) for a fixed
            // setpoint, and we want the damping sign of an error derivative.
            d = -(measurement - prev_meas_) / dt;
        }
        prev_meas_ = measurement;
        have_prev_ = true;

        // --- integral, with conditional integration --------------------------
        // Freezing the integrator while saturated is the anti-windup. Without
        // it, a long saturated slew accumulates a demand that takes just as long
        // to unwind, and the loop overshoots every time it catches up.
        if (!saturated) {
            integ_ += error * dt;
            integ_ = clamp_abs(integ_, g_.i_limit);
        }

        return g_.kp * error
             + g_.ki * integ_
             + g_.kd * d
             + g_.k_ff * target_rate_est    // the ten lines that matter most
             - platform_rate_est;           // cancel known platform drift
    }

    [[nodiscard]] double integrator() const noexcept { return integ_; }
    [[nodiscard]] const ControlGains& gains() const noexcept { return g_; }

    /// Zero the integrator and derivative history. Used when the FSM re-enters
    /// Search, where the old error history describes a target that is gone.
    void clear_state() noexcept { integ_ = 0.0; have_prev_ = false; prev_meas_ = 0.0; }

private:
    ControlGains g_{};
    double       integ_     = 0.0;
    double       prev_meas_ = 0.0;
    bool         have_prev_ = false;
};

// ---------------------------------------------------------------------------
// Controller — both axes, which is what the engine drives.
// ---------------------------------------------------------------------------
class Controller {
public:
    void reset(const ControlGains& g) noexcept { az_.reset(g); el_.reset(g); }
    void set_gains(const ControlGains& g) noexcept { az_.set_gains(g); el_.set_gains(g); }
    void clear_state() noexcept { az_.clear_state(); el_.clear_state(); }

    /// `aim` is where we want to point, `measured` is the encoder reading,
    /// both world-frame angles in microradians.
    [[nodiscard]] Rate2 compute(Angle2 aim, Angle2 measured,
                                Rate2 target_rate_est, Rate2 platform_rate_est,
                                double dt, bool az_saturated, bool el_saturated) noexcept {
        return {
            az_.compute(aim.x - measured.x, measured.x,
                        target_rate_est.x, platform_rate_est.x, dt, az_saturated),
            el_.compute(aim.y - measured.y, measured.y,
                        target_rate_est.y, platform_rate_est.y, dt, el_saturated)
        };
    }

    [[nodiscard]] AxisController&       az()       noexcept { return az_; }
    [[nodiscard]] AxisController&       el()       noexcept { return el_; }
    [[nodiscard]] const AxisController& az() const noexcept { return az_; }
    [[nodiscard]] const AxisController& el() const noexcept { return el_; }

private:
    AxisController az_;
    AxisController el_;
};

}  // namespace sat
