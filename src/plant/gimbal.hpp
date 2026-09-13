// plant/gimbal.hpp — the pan-tilt mount, modelled as a second-order plant with
// saturation and transport delay.
//
// Design decision 8 explains why this is 120 lines we own rather than a physics
// engine: "Nothing collides. The gimbal is a rate/acceleration-limited
// second-order plant with a transport delay — 120 lines you must own for INV-3
// and for applying saturation inside the integration step."
//
// ---------------------------------------------------------------------------
// WHY EACH NON-IDEALITY IS HERE
// ---------------------------------------------------------------------------
// A perfect integrator would make the control problem trivial and the results
// meaningless. Every term below is something a real gimbal does:
//
//   rate limit        spec rows 13-14, 5-10 deg/s. This is THE constraint of the
//                     whole project: §1.3 shows the disturbance can exceed the
//                     actuator's authority, which is why reactive control cannot
//                     meet the error budget and the system must predict.
//   acceleration limit a motor cannot step its speed; torque is finite.
//   first-order lag   the current loop takes time to reach a commanded rate.
//   transport delay   command-to-motion latency: bus, driver, and compute. This
//                     is what the Smith predictor in §10.4 exists to fight.
//   encoder quantisation the controller reads a quantised angle, never the true
//                     one. Modelling this is what makes position() vs
//                     true_position() a meaningful distinction (§10.3).
//
// ---------------------------------------------------------------------------
// SATURATION GOES INSIDE THE INTEGRATION STEP
// ---------------------------------------------------------------------------
// Design §10.3 is emphatic: "Saturation must be applied INSIDE the step, not
// after." Clamping the position afterwards would let the internal rate state run
// away while the output sat at its limit, and the plant would then take an
// unphysically long time to reverse — the classic integrator-windup shape, but
// in the plant rather than the controller, where no anti-windup scheme can
// reach it.

#pragma once

#include "core/ring.hpp"
#include "core/units.hpp"

#include <cmath>
#include <cstdint>

namespace sat {

// ---------------------------------------------------------------------------
// GimbalParams — one axis's physical description, in microradians (INV-5).
// ---------------------------------------------------------------------------
struct GimbalParams {
    double max_rate_urad_s   = 0.0;    ///< spec rows 13-14; 5 deg/s = 87266 urad/s
    double max_accel_urad_s2 = 0.0;    ///< 0 disables the limit (CP 1.6 behaviour)
    double tau_s             = 0.020;  ///< first-order lag; 0 makes the rate loop ideal
    double latency_s         = 0.010;  ///< transport delay, command to motion
    double encoder_lsb_urad  = 20.0;   ///< 0 disables quantisation
    double resonance_hz      = 0.0;    ///< 0 = disabled (structural mode, CP 4.10)

    /// Build from the scenario's degrees-per-second values (spec rows 13-14).
    [[nodiscard]] static GimbalParams from_dps(double max_rate_dps,
                                               double max_accel_dps2) noexcept {
        GimbalParams p;
        p.max_rate_urad_s   = deg_to_urad(max_rate_dps);
        p.max_accel_urad_s2 = deg_to_urad(max_accel_dps2);
        return p;
    }
};

// ---------------------------------------------------------------------------
// GimbalAxis — one axis. Pan and tilt are independent motors with independent
// limits (spec rows 13 and 14 are separate rows), so they are separate objects
// rather than a vector type with a shared limit.
// ---------------------------------------------------------------------------
class GimbalAxis {
public:
    GimbalAxis() { reset(GimbalParams{}, 0.0); }

    /// Configure and place the axis at a starting angle. Called once at
    /// startup (design §6.1 step A8) and between sweep runs.
    void reset(const GimbalParams& p, double initial_pos_urad) noexcept {
        p_    = p;
        pos_  = initial_pos_urad;
        rate_ = 0.0;
        // Prime the delay line with zero rate: before any command arrives, the
        // motor is commanded to hold still, not to do whatever was in memory.
        delay_.fill(0.0);
        sat_rate_ticks_  = 0;
        sat_accel_ticks_ = 0;
        total_ticks_     = 0;
    }

    // -----------------------------------------------------------------------
    // step — advance by dt, given a commanded rate.
    //
    // Order matters and follows design §10.3 exactly:
    //   1. push the command into the delay line, read the delayed one
    //   2. clamp the delayed command to the rate limit
    //   3. drive the rate toward it through the first-order lag
    //   4. clamp the implied acceleration
    //   5. integrate with the midpoint rule
    //   6. clamp the resulting rate again (the limit is on the state, not just
    //      on the command)
    // -----------------------------------------------------------------------
    void step(double cmd_rate_urad_s, double dt) noexcept {
        delay_.push(cmd_rate_urad_s);

        // How many ticks of history the transport delay corresponds to. Rounded
        // rather than truncated so a latency of exactly half a tick does not
        // silently become zero.
        const int d = static_cast<int>(std::lround(p_.latency_s / dt));
        const double delayed = delay_.at_back(static_cast<size_t>(d));

        // The motor physically cannot be commanded past its rate ceiling.
        const double want = p_.max_rate_urad_s > 0.0
                          ? clamp_abs(delayed, p_.max_rate_urad_s)
                          : delayed;

        // First-order lag: the rate loop closes with time constant tau. A tau of
        // zero means an ideal rate loop, which is what CP 1.6 wants — the
        // commanded rate is reached within one step, limited only by acceleration.
        double accel = (p_.tau_s > 0.0) ? (want - rate_) / p_.tau_s
                                        : (want - rate_) / dt;

        if (p_.max_accel_urad_s2 > 0.0) {
            const double limited = clamp_abs(accel, p_.max_accel_urad_s2);
            if (limited != accel) ++sat_accel_ticks_;
            accel = limited;
        }

        // Midpoint integration: position advances at the average of the rate
        // across the step, not at its start or end value. For a ramp that is
        // exact rather than first-order accurate, which matters because the
        // plant is stepped 300 times a second for the whole run and a
        // systematic half-step-per-tick error would accumulate into a real
        // pointing offset.
        const double rate_mid = rate_ + 0.5 * accel * dt;
        double next_rate = rate_ + accel * dt;
        if (p_.max_rate_urad_s > 0.0) {
            next_rate = clamp_abs(next_rate, p_.max_rate_urad_s);
        }
        rate_ = next_rate;
        pos_ += rate_mid * dt;

        ++total_ticks_;
        // "At the limit" rather than "exactly at": floating point will not land
        // on the ceiling exactly, and a 0.1% band is far tighter than any real
        // measurement of saturation.
        if (p_.max_rate_urad_s > 0.0 &&
            std::fabs(rate_) >= p_.max_rate_urad_s * 0.999) {
            ++sat_rate_ticks_;
        }
    }

    // -----------------------------------------------------------------------
    // THE TWO POSITION ACCESSORS ARE NOT REDUNDANT
    //
    // position() is what the controller is allowed to read: a real encoder
    // reports a quantised angle. true_position() is what the metrics use. Mixing
    // them up would either give the controller information it cannot have, or
    // report a tracking error contaminated by quantisation that the system did
    // not actually make.
    // -----------------------------------------------------------------------

    /// What the encoder reports. FOR THE CONTROLLER.
    [[nodiscard]] double position() const noexcept {
        return quantise(pos_, p_.encoder_lsb_urad);
    }

    /// Where the axis actually is. FOR METRICS ONLY.
    [[nodiscard]] double true_position() const noexcept { return pos_; }

    /// Current angular rate. Used by the feedforward path and by metrics.
    [[nodiscard]] double rate() const noexcept { return rate_; }

    /// Fraction of ticks spent at the rate ceiling — design §13.1's
    /// saturation_frac, and one of the SAT supervisor's 12 features (§10.6).
    /// A high value means the disturbance is outrunning the motor, which §1.3
    /// says is the central difficulty of the whole problem.
    [[nodiscard]] double saturation_frac() const noexcept {
        return total_ticks_ ? static_cast<double>(sat_rate_ticks_)
                            / static_cast<double>(total_ticks_) : 0.0;
    }
    [[nodiscard]] double accel_saturation_frac() const noexcept {
        return total_ticks_ ? static_cast<double>(sat_accel_ticks_)
                            / static_cast<double>(total_ticks_) : 0.0;
    }

    [[nodiscard]] const GimbalParams& params() const noexcept { return p_; }

    /// Force the axis to an angle, discarding its rate. Only for test setup and
    /// for scenario events; never call this from the control loop.
    void force_position(double urad) noexcept { pos_ = urad; rate_ = 0.0; }

private:
    GimbalParams p_{};
    double       pos_  = 0.0;
    double       rate_ = 0.0;

    // 64 entries covers 0.010 s of latency at any truth rate up to 6.4 kHz.
    Ring<double, 64> delay_{};

    uint64_t sat_rate_ticks_  = 0;
    uint64_t sat_accel_ticks_ = 0;
    uint64_t total_ticks_     = 0;
};

// ---------------------------------------------------------------------------
// Gimbal — the two axes together, which is what the engine actually steps.
// ---------------------------------------------------------------------------
class Gimbal {
public:
    void reset(const GimbalParams& pan, const GimbalParams& tilt,
               Angle2 initial) noexcept {
        az_.reset(pan,  initial.x);
        el_.reset(tilt, initial.y);
    }

    void step(Rate2 cmd, double dt) noexcept {
        az_.step(cmd.x, dt);
        el_.step(cmd.y, dt);
    }

    /// What the controller sees: quantised by the encoder.
    [[nodiscard]] Angle2 position() const noexcept {
        return {az_.position(), el_.position()};
    }
    /// What the metrics see. INV-6 depends on this being the true value.
    [[nodiscard]] Angle2 true_position() const noexcept {
        return {az_.true_position(), el_.true_position()};
    }
    [[nodiscard]] Rate2 rate() const noexcept { return {az_.rate(), el_.rate()}; }

    /// Worst of the two axes — a single number for the compliance report.
    [[nodiscard]] double saturation_frac() const noexcept {
        return std::fmax(az_.saturation_frac(), el_.saturation_frac());
    }

    [[nodiscard]] GimbalAxis&       az()       noexcept { return az_; }
    [[nodiscard]] GimbalAxis&       el()       noexcept { return el_; }
    [[nodiscard]] const GimbalAxis& az() const noexcept { return az_; }
    [[nodiscard]] const GimbalAxis& el() const noexcept { return el_; }

private:
    GimbalAxis az_;
    GimbalAxis el_;
};

}  // namespace sat
