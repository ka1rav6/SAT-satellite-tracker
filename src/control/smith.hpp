// control/smith.hpp — the Smith predictor (design §10.4, CP 10.4).
//
// ---------------------------------------------------------------------------
// WHAT IT IS FOR, AND THE NUMBER THAT JUSTIFIES IT
// ---------------------------------------------------------------------------
// CP 10.6 measured where the loop stops meeting spec row 17, and the answer was
// not the one the design expected. On a fast target the tracking error grows in
// proportion to the RATE DEMAND, and it does so while the mount still has a
// quarter of its authority in hand:
//
//   demand   200   300   400   500   600   700   800 px/s
//   RMS     3.55  5.47  7.45  9.47 11.48 13.57 16.10 px
//   plant saturation: zero everywhere up to 800 px/s
//
// The slope is 0.019 s. That is the mount's 10 ms transport delay plus part of
// its 20 ms rate-loop lag, and it is a PURE DELAY: the command that takes
// effect now is the one the controller issued 19 ms of target motion ago.
//
// Velocity feedforward (CP 10.1) cannot touch it. Feedforward cancels the term
// proportional to velocity, and a delay is not that term — feeding v forward
// harder just moves the mount to where the target was 19 ms ago, faster.
// Integral action does remove it, over its own kp/ki = 4 s, which is far too
// slow to help the acquisition window rows 16 and 17 are both scored in.
//
// ---------------------------------------------------------------------------
// HOW IT WORKS
// ---------------------------------------------------------------------------
// The textbook Smith predictor controls against a delay-free MODEL of the plant
// and corrects the model with the difference between the real output and the
// model's delayed output:
//
//   e = setpoint - ( y_measured + y_model_nodelay - y_model_delayed )
//
// When the model is exact the last two terms cancel the measurement's staleness
// and the controller sees the plant as if it had no delay.
//
// Here the plant is a rate-commanded integrator, so the whole correction
// collapses into one quantity: HOW FAR THE MOUNT IS ABOUT TO MOVE before a
// command issued now can take effect. Call it the lead. Add it to the measured
// position and the controller is looking at where the mount will be when its
// next command can first matter, rather than where it was.
//
// The lead is computed by running the plant's own first-order rate model
// forward. The exponential form is used rather than the Euler step because
// dt/tau = 0.0333/0.020 = 1.67 here: an Euler update of a lag whose time
// constant is shorter than the step overshoots and rings, and a predictor that
// rings is worse than no predictor.
//
// ---------------------------------------------------------------------------
// BOTH SIDES OF THE ERROR MOVE, OR NEITHER
// ---------------------------------------------------------------------------
// The first version of this advanced only the MEASUREMENT, and it did nothing
// useful — for the same reason CP 10.1's one-frame-ahead aim did harm.
//
//   error = aim - (measured + lead)
//
// In steady state the mount and the target are both moving at v, so lead = v*h.
// Nulling that error settles the mount at T - v*h: a lag of exactly the horizon,
// where CP 10.1's setpoint-only lead produced a lead of exactly the horizon.
// Advancing one side of a difference biases it by the horizon, and the sign
// just depends on which side you picked.
//
// So the aim is advanced by the SAME horizon, using the tracker's own
// prediction — which is free, the velocity is already estimated. The error then
// means what it should: the pointing error that will exist at the moment this
// command lands. That is the quantity worth nulling, and it is what the Smith
// structure is for.
//
// The horizon is latency + one control period, because a command computed at
// frame k cannot move the mount until frame k+1's sub-ticks, and then only
// after the transport delay. At 30 Hz with a 10 ms latency that is 43 ms —
// which also explains why the in-flight window alone was useless: round(10 ms /
// 33 ms) is ZERO steps. The delay that matters here is not the mount's, it is
// the loop's own sampling period.
//
// ---------------------------------------------------------------------------
// THE RESULT: IT IS OFF, BECAUSE IT DOES NOT HELP HERE
// ---------------------------------------------------------------------------
// §10.4 says "worth 30-50% bandwidth but amplifies model error", and the
// checkpoint says "if it destabilises, leave it off — optional". Measured on
// the CP 10.6 sweep, with the model matched to the plant in every parameter:
//
//   demand         200 px/s        600 px/s        800 px/s
//   off       3.55 / p95 6.15   11.48 / 18.55   16.10 / 25.90 px
//   on        3.88 / p95 8.26   12.41 / 25.66   16.96 / 34.78 px
//
// Worse everywhere, and much worse at p95 than at RMS — the harm is in the
// transients, not the steady state.
//
// WHY, AND IT IS NOT A BUG IN THE PREDICTOR. This loop is not delay-limited.
// The horizon is 43 ms and the crossover is kp = 8 rad/s, so the delay costs
// 8 * 0.043 = 0.34 rad = 20 degrees of phase at crossover, out of a margin that
// starts near 90. A Smith predictor buys bandwidth when the DELAY is the
// binding constraint. Here the binding constraints are the acceleration limit
// during acquisition and the rate ceiling at high demand, and no amount of
// predicting removes either.
//
// The cost, meanwhile, is real: extrapolating 43 ms of a plant that is slewing
// under its own limits is confidently wrong exactly when the error is largest,
// which is why p95 degrades by 34% while RMS degrades by 5%.
//
// Two hypotheses were tested and rejected before concluding this, and both are
// kept in the code because a rejected explanation written down is worth more
// than a deleted one:
//
//   the model had no saturation limits    19.95 -> 16.96 px at 800 px/s when
//                                         added. A real improvement, and still
//                                         worse than off.
//   the rate seed was differentiated
//   from a 20 urad encoder                17.00 / 16.97 / 16.96 px at seed
//                                         blends of 0 / 0.5 / 1. No effect.
//
// It stays switchable so the conclusion can be re-checked whenever the plant's
// numbers change — a slower camera or a longer transport delay would move the
// balance, and the tests state the threshold rather than the verdict.

#pragma once

#include "core/ring.hpp"
#include "core/units.hpp"

#include <cmath>
#include <cstddef>

namespace sat {

// ---------------------------------------------------------------------------
// PlantModel — the controller's belief about the mount.
//
// Deliberately a COPY of the numbers rather than a reference to the GimbalAxis.
// A Smith predictor that reads the plant's own state is not a predictor, it is
// a cheat: its entire risk is that the model and the plant disagree, and a
// version that cannot disagree tests nothing. Keeping them separate is also
// what lets a test set the model wrong on purpose.
// ---------------------------------------------------------------------------
struct PlantModel {
    double latency_s = 0.010;   ///< command-to-motion transport delay
    double tau_s     = 0.020;   ///< first-order rate-loop lag

    // -----------------------------------------------------------------------
    // The SATURATION limits, and they are not optional.
    //
    // The first working version of this predictor left them out, on the
    // reasoning that a Smith predictor is about delay and saturation is
    // somebody else's problem. It made every case measurably WORSE — at a
    // 800 px/s demand, 16.10 -> 19.95 px RMS and p95 25.90 -> 45.30 px.
    //
    // The reason is the transient. The controller's output during acquisition
    // is not a physical rate: the trace shows 510817 urad/s at frame 3, six
    // times the mount's ceiling, because that is what kp times a 60 px error
    // comes to. A model with no ceiling believes the mount is about to travel
    // 510817 * 0.043 = 200 px in one horizon, when it can manage 34. The
    // predicted position then overshoots the target, the error changes sign,
    // and the loop stops driving exactly when it most needs to.
    //
    // This is §10.4's "amplifies model error" arriving on schedule, and the
    // answer is not to abandon the predictor but to stop lying to it. The
    // model now clamps the same way plant/gimbal.hpp does, in the same order.
    // -----------------------------------------------------------------------
    double max_rate_urad_s   = 0.0;   ///< 0 disables, as in GimbalParams
    double max_accel_urad_s2 = 0.0;   ///< 0 disables
};

class SmithPredictor {
public:
    /// `dt` is the control period.
    void reset(const PlantModel& m, double dt) noexcept {
        m_  = m;
        dt_ = dt;
        horizon_s_ = (dt > 0.0) ? (m.latency_s + dt) : 0.0;
        cmds_.fill(0.0);
        model_rate_ = 0.0;
    }

    /// How far ahead this predictor looks, seconds. The caller advances the
    /// SETPOINT by the same amount — see the note above on why advancing one
    /// side of the error biases it by exactly this.
    [[nodiscard]] double horizon_s() const noexcept { return horizon_s_; }

    /// Record the command just issued. Called once per control step, AFTER
    /// compute(), so that the command being computed is not used to predict
    /// its own effect. Also advances the model's own rate state one step, so
    /// that a seed which does not trust the encoder has something to use.
    void push(double cmd_rate) noexcept {
        cmds_.push(cmd_rate);
        double c = cmd_rate;
        if (m_.max_rate_urad_s > 0.0) c = clamp_abs(c, m_.max_rate_urad_s);
        double next = c + (model_rate_ - c) * std::exp(-dt_ / (m_.tau_s > 0.0
                                                              ? m_.tau_s : 1e-9));
        if (m_.max_accel_urad_s2 > 0.0) {
            const double da = m_.max_accel_urad_s2 * dt_;
            if (next - model_rate_ >  da) next = model_rate_ + da;
            if (next - model_rate_ < -da) next = model_rate_ - da;
        }
        if (m_.max_rate_urad_s > 0.0) next = clamp_abs(next, m_.max_rate_urad_s);
        model_rate_ = next;
    }

    /// How much the model's rate seed trusts the differentiated encoder.
    void set_rate_blend(double b) noexcept { blend_ = b; }

    // -----------------------------------------------------------------------
    // lead — how far the mount will travel before a command issued NOW can
    // first take effect, given the commands already in flight.
    //
    // `measured_rate` seeds the model's rate state from reality every step,
    // which is the correction half of the Smith structure: the model is never
    // allowed to drift away from the plant, it only ever extrapolates one
    // latency window from a measured state. That bounds the damage a wrong
    // model can do to one window rather than letting it accumulate.
    // -----------------------------------------------------------------------
    [[nodiscard]] double lead(double measured_rate) const noexcept {
        if (horizon_s_ <= 0.0 || dt_ <= 0.0) return 0.0;

        // Exact first-order response over one step: r <- c + (r - c) * e^(-dt/tau).
        // Euler would be r += (c - r) * dt/tau, and with dt/tau = 1.67 that
        // overshoots the command every step and oscillates.
        const double decay = (m_.tau_s > 0.0)
                           ? std::exp(-dt_ / m_.tau_s)
                           : 0.0;                 // tau = 0 is an ideal rate loop

        // How the model's rate state is seeded each step, and it matters more
        // than it looks.
        //
        // The obvious choice is the differentiated encoder: it is the
        // correction half of the Smith structure, and it stops the model
        // drifting away from the plant. The problem is the differentiation.
        // The encoder quantises to 20 urad (design §10.3), so over a 33 ms step
        // the implied rate carries 600 urad/s of quantisation noise, and the
        // horizon multiplies it straight into the position the controller is
        // regulating.
        //
        // blend_ = 0 uses the model's own integrated rate, 1 uses the encoder.
        // Both are measured in tests/control/test_stage10.cpp; neither rescues
        // the predictor on this plant, which is the CP 10.4 result.
        double r = blend_ * measured_rate + (1.0 - blend_) * model_rate_;
        double travel = 0.0;
        double remaining = horizon_s_;
        // The commands that will act over the horizon are the ones already
        // issued. There are not usually enough of them — the horizon is
        // latency + dt while the in-flight history covers only latency — so
        // once the history runs out the most recent command is held. That is
        // the right assumption: it is what the plant will actually be doing
        // until this step's command reaches it.
        size_t k = static_cast<size_t>(std::lround(m_.latency_s / dt_));
        if (k > kCapacity) k = kCapacity;
        while (remaining > 1e-12) {
            const double step = (remaining < dt_) ? remaining : dt_;

            // Same order as plant/gimbal.hpp: clamp the COMMAND to the rate
            // ceiling first, then let the lag act, then clamp the implied
            // acceleration, then clamp the resulting rate. Doing it in a
            // different order gives a different answer, and the whole value of
            // this model is that it does not disagree with the plant.
            double c = cmds_.at_back(k);
            if (m_.max_rate_urad_s > 0.0) c = clamp_abs(c, m_.max_rate_urad_s);

            double r_next = c + (r - c) * std::exp(-step / (m_.tau_s > 0.0
                                                            ? m_.tau_s : 1e-9));
            if (m_.max_accel_urad_s2 > 0.0) {
                const double da = m_.max_accel_urad_s2 * step;
                if (r_next - r >  da) r_next = r + da;
                if (r_next - r < -da) r_next = r - da;
            }
            if (m_.max_rate_urad_s > 0.0) r_next = clamp_abs(r_next, m_.max_rate_urad_s);

            // Midpoint rule, matching plant/gimbal.hpp's own integration, so
            // the model and the plant do not disagree merely about arithmetic.
            travel += 0.5 * (r + r_next) * step;
            r = r_next;
            remaining -= step;
            if (k > 0) --k;
        }
        (void)decay;
        return travel;
    }

private:
    static constexpr size_t kCapacity = 63;   ///< one less than the ring's size

    PlantModel       m_{};
    double           dt_        = 0.0;
    double           horizon_s_  = 0.0;
    double           model_rate_ = 0.0;
    double           blend_      = 1.0;
    Ring<double, 64> cmds_{};
};

}  // namespace sat
