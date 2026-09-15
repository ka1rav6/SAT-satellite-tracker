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

#include "control/smith.hpp"
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
//
// ---------------------------------------------------------------------------
// WHERE ki = 2.0 COMES FROM (CP 10.2)
// ---------------------------------------------------------------------------
// It was 0.5, which was a guess. Integral gain buys steady-state accuracy and
// pays for it in overshoot, and both sides are measurable, so the number is
// chosen by a criterion instead:
//
//   ki = the largest integral gain whose FULL-FIELD SLEW overshoot stays
//        inside spec row 17's 10 px budget.
//
// Measured on scenarios/control/slew.toml, a saturating 375 px acquisition
// slew, with the overshoot read off the trace:
//
//   ki     slew overshoot     200 px/s residual (p95)
//   0.5        2.07 px              4.73 px
//   1.0        3.93 px              4.48 px
//   2.0        6.88 px              4.04 px      <- shipped
//   3.0       10.02 px              3.57 px      <- at the budget
//   4.0       12.93 px              3.04 px
//   8.0       23.80 px              1.50 px
//
// The temptation is ki = 8: it drives the steady-state residual to 1.5 px and
// the slew still settles in one lobe. But a 23.8 px overshoot blows row 17's
// budget on EVERY acquisition, and the overshoot is scored — the metric starts
// counting at the first confirmed frame, which is before the settle.
//
// At specification-representative speeds ki barely matters at all: on
// compliance.toml the target's apparent rate is 7.6 px/s and the tracking RMS
// moves from 17.20 to 17.27 px across the whole range above, because row 23's
// jitter floor swamps it. The gain is only visible on a fast target, which is
// the regime CP 10.1's scenario exists to expose.
// ---------------------------------------------------------------------------
struct ControlGains {
    double kp      = 8.0;
    double ki      = 2.0;
    double kd      = 0.15;
    double k_ff    = 1.0;    ///< velocity feedforward; 1.0 = full, 0.0 = off
    double i_limit = 2.0e5;  ///< integrator clamp, urad*s

    /// CP 10.4's Smith predictor. OFF, because it was measured and it does not
    /// help on this plant — smith.hpp carries the numbers and the reason. The
    /// design anticipated this outcome: "if it destabilises, leave it off —
    /// optional". It stays switchable so the claim can be re-checked whenever
    /// the plant's numbers change.
    bool   smith = false;

    /// How much the Smith predictor's model trusts the differentiated encoder
    /// for its rate seed: 1 is fully, 0 uses the model's own integrated rate.
    /// See smith.hpp. This was a HYPOTHESIS about why the predictor hurt —
    /// the encoder quantises to 20 urad and differentiating it over a 33 ms
    /// step injects 600 urad/s into the prediction — and the measurement
    /// rejected it: 17.00 / 16.97 / 16.96 px at blends of 0 / 0.5 / 1. Kept
    /// because a tested-and-rejected explanation is worth more written down
    /// than deleted, and a test pins that it still makes no difference.
    double smith_rate_blend = 1.0;

    /// Conditional integration (CP 10.2). Defaults ON, and turning it off is
    /// not an option anybody should exercise in a real run — it exists so the
    /// checkpoint can MEASURE what anti-windup is worth instead of asserting
    /// it, and so §13.3's ablation table has a row for it. A claim that a
    /// safeguard matters is only worth as much as the run without it.
    bool   anti_windup = true;

    /// A pure proportional controller — CP 1.7's "cmd = kp * (detection −
    /// boresight)". Kept as a named factory because the demo switches between
    /// P, PID and PID+FF live (CP 15.2) and the report's ablation table needs
    /// each configuration to be nameable.
    [[nodiscard]] static ControlGains proportional(double kp_) noexcept {
        ControlGains g;
        g.kp = kp_; g.ki = 0.0; g.kd = 0.0; g.k_ff = 0.0; g.i_limit = 0.0;
        return g;
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
        smith_.reset(model_, dt_hint_);
        smith_.set_rate_blend(g_.smith_rate_blend);
    }

    /// The controller's BELIEF about the plant, for CP 10.4. A copy of the
    /// numbers, never a handle on the GimbalAxis — see smith.hpp for why a
    /// predictor that cannot disagree with the plant tests nothing.
    void set_plant_model(const PlantModel& m, double dt) noexcept {
        model_    = m;
        dt_hint_  = dt;
        smith_.reset(model_, dt_hint_);
        smith_.set_rate_blend(g_.smith_rate_blend);
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
    //   integral_enabled   whether an integral term is meaningful right now
    //
    // Returns a commanded rate in urad/s. The caller clamps it to the axis
    // limit; the plant clamps it again regardless, because a controller must
    // never be trusted to respect a physical limit on its own.
    // -----------------------------------------------------------------------
    [[nodiscard]] double compute(double error, double measurement,
                                 double target_rate_est, double platform_rate_est,
                                 double dt, bool saturated,
                                 bool integral_enabled = true) noexcept {
        // --- derivative, on the measurement ---------------------------------
        double d = 0.0;
        double measured_rate = 0.0;
        if (have_prev_ && dt > 0.0) {
            measured_rate = (measurement - prev_meas_) / dt;
            // Negated because d/dt(error) = −d/dt(measurement) for a fixed
            // setpoint, and we want the damping sign of an error derivative.
            d = -measured_rate;
        }
        prev_meas_ = measurement;
        have_prev_ = true;

        // --- CP 10.4: the Smith predictor -----------------------------------
        //
        // Everything below this line then sees a measurement that has been
        // advanced to where the mount will be when a command issued now can
        // first take effect. The proportional, integral and derivative paths
        // all use it, because they are all reacting to the same staleness.
        //
        // The error is corrected rather than the setpoint: advancing the AIM
        // instead would be the same double-lead mistake CP 10.1 found, since
        // the target's motion is already the feedforward's job. This lead is
        // about the MOUNT's motion, which nothing else accounts for.
        double effective_error = error;
        if (g_.smith) {
            const double lead = smith_.lead(measured_rate);
            effective_error = error - lead;
        }

        // --- integral, with conditional integration --------------------------
        // Freezing the integrator while saturated is the anti-windup. Without
        // it, a long saturated slew accumulates a demand that takes just as long
        // to unwind, and the loop overshoots every time it catches up.
        //
        // integral_enabled is the SECOND freeze condition, and CP 10.1 is where
        // it had to be added. Conditional integration on saturation assumes the
        // only way to accumulate a demand you cannot honour is to hit the rate
        // limit. That is not true here: while the mode FSM is in Search the
        // setpoint is a SCANNING PATTERN, which steps from tile to tile and
        // reverses. The error against it is large, persistent and sign-flipping
        // by design, the plant never saturates, and the integrator happily
        // winds up against a setpoint that has no steady state to converge to.
        //
        // The measured symptom was not a pointing error — it was a FALSE TRACK.
        // With ki at 0.5 and the target blanked, a 102-frame fruitless search
        // confirmed a track on 2 frames; with ki at 0 it confirmed none. The
        // wound-up integrator was swinging the mount hard enough between tiles
        // that the motion smear rendered a streak the detector was right to
        // call a candidate. An integral term that manufactures targets is a
        // clear sign it is being asked to do something it is not for.
        //
        // Integral action exists to remove a STEADY-STATE bias against a
        // steady setpoint. Search has neither, so it gets no integrator.
        //
        // g_.anti_windup gates only the SATURATION freeze, not integral_enabled:
        // the Search freeze above is not a safeguard that can be traded away,
        // it is a statement about what an integral term means, and a run with
        // it disabled would be measuring a different controller rather than
        // this one without a safeguard.
        if ((!saturated || !g_.anti_windup) && integral_enabled) {
            integ_ += effective_error * dt;
            integ_ = clamp_abs(integ_, g_.i_limit);
        }

        const double out = g_.kp * effective_error
                         + g_.ki * integ_
                         + g_.kd * d
                         + g_.k_ff * target_rate_est  // the ten lines that matter most
                         - platform_rate_est;         // cancel known platform drift

        // AFTER computing it, never before: the command being issued cannot be
        // used to predict its own effect.
        smith_.push(out);
        return out;
    }

    [[nodiscard]] double integrator() const noexcept { return integ_; }

    /// CP 10.4: how far ahead the error is evaluated, seconds. Zero when the
    /// Smith predictor is off. The CALLER must advance the setpoint by the same
    /// amount — smith.hpp explains why advancing one side of the error biases
    /// it by exactly this.
    [[nodiscard]] double horizon_s() const noexcept {
        return g_.smith ? smith_.horizon_s() : 0.0;
    }
    [[nodiscard]] const ControlGains& gains() const noexcept { return g_; }

    /// Zero the integrator and derivative history. Used when the FSM re-enters
    /// Search, where the old error history describes a target that is gone.
    void clear_state() noexcept {
        integ_ = 0.0; have_prev_ = false; prev_meas_ = 0.0;
        smith_.reset(model_, dt_hint_);
    }

private:
    ControlGains g_{};
    double       integ_     = 0.0;
    double       prev_meas_ = 0.0;
    bool         have_prev_ = false;

    PlantModel     model_{};
    double         dt_hint_ = 1.0 / 30.0;
    SmithPredictor smith_{};
};

// ---------------------------------------------------------------------------
// Controller — both axes, which is what the engine drives.
// ---------------------------------------------------------------------------
class Controller {
public:
    void reset(const ControlGains& g) noexcept { az_.reset(g); el_.reset(g); }
    void set_plant_model(const PlantModel& m, double dt) noexcept {
        az_.set_plant_model(m, dt);
        el_.set_plant_model(m, dt);
    }
    void set_gains(const ControlGains& g) noexcept { az_.set_gains(g); el_.set_gains(g); }
    void clear_state() noexcept { az_.clear_state(); el_.clear_state(); }

    /// `aim` is where we want to point, `measured` is the encoder reading,
    /// both world-frame angles in microradians.
    [[nodiscard]] Rate2 compute(Angle2 aim, Angle2 measured,
                                Rate2 target_rate_est, Rate2 platform_rate_est,
                                double dt, bool az_saturated, bool el_saturated,
                                bool integral_enabled = true) noexcept {
        return {
            az_.compute(aim.x - measured.x, measured.x,
                        target_rate_est.x, platform_rate_est.x, dt, az_saturated,
                        integral_enabled),
            el_.compute(aim.y - measured.y, measured.y,
                        target_rate_est.y, platform_rate_est.y, dt, el_saturated,
                        integral_enabled)
        };
    }

    /// Both axes share a control period and a plant model, so one number.
    [[nodiscard]] double horizon_s() const noexcept { return az_.horizon_s(); }

    [[nodiscard]] AxisController&       az()       noexcept { return az_; }
    [[nodiscard]] AxisController&       el()       noexcept { return el_; }
    [[nodiscard]] const AxisController& az() const noexcept { return az_; }
    [[nodiscard]] const AxisController& el() const noexcept { return el_; }

private:
    AxisController az_;
    AxisController el_;
};

}  // namespace sat
