// tracking/imm.hpp — CP 10.5. The interacting multiple model filter.
//
// Design §10.2: "IMM — three models: CV, CA, CT ... Four steps: mix →
// mode-matched filter → mode probability update from innovation likelihood →
// combine."
//
// ---------------------------------------------------------------------------
// WHY, IN ONE SENTENCE FROM THE DESIGN
// ---------------------------------------------------------------------------
// "At the figure-8 crossing, acceleration reverses sign and single-model
// filters overshoot every lap. Figure-8 is a MANDATORY spec motion (row 12), so
// this is a before/after demo on graded functionality."
//
// That is the whole case. A constant-velocity filter is not wrong about a
// figure-8 in general — it tracks the straight sections perfectly — it is wrong
// precisely where the motion changes character, and a figure-8 changes
// character twice a lap by construction. The failure is a lag proportional to
// the acceleration, and CP 10.1 already measured what that costs downstream:
// the feedforward can only be as good as the velocity it is fed.
//
// ---------------------------------------------------------------------------
// SIX STATES, NOT FOUR, AND WHY THAT IS NOT NEGOTIABLE
// ---------------------------------------------------------------------------
// tracking/kalman.hpp is a 4-state filter, [az, el, az_rate, el_rate], and it
// stays that way: it is the default path and rewriting it would churn every
// reproducibility fingerprint in the project for a feature that is opt-in.
//
// This filter is 6-state, [az, el, az_rate, el_rate, az_accel, el_accel],
// because a constant-ACCELERATION model needs somewhere to keep the
// acceleration. The tempting shortcut is a bank of constant-velocity models
// that differ only in process noise — "CV-quiet and CV-noisy" — and it is a
// real and widely used IMM, but it is not what §10.2 asks for and it is weaker
// where it matters: a high-q CV model does not PREDICT a turn, it merely stops
// being surprised by one. On a figure-8 the difference is exactly the overshoot
// the checkpoint is about.
//
// The IMM's mixing step requires every model to live in one state space, so all
// three are written in the 6-state space and each says what it believes about
// the acceleration:
//
//   CV   position advances by v*T; acceleration is driven to zero, because
//        that is what "constant velocity" MEANS as a statement about accel.
//   CA   position advances by v*T + a*T^2/2, velocity by a*T, acceleration
//        held. Process noise is a jerk PSD.
//   CT   coordinated turn: the velocity VECTOR rotates at a turn rate
//        omega estimated from the combined state, omega = (vx*ay - vy*ax) /
//        (vx^2 + vy^2), which is the signed curvature times speed. This is the
//        model that makes the predicted covariance ANISOTROPIC — §10.2 notes
//        that the 4-state filter's covariance is exactly isotropic and that
//        "anisotropy arrives with the coordinate-turn model at CP 10.5", which
//        is why that filter was written in full 4x4 form rather than as two
//        decoupled 2x2 filters.
//
// ---------------------------------------------------------------------------
// NUMERICS
// ---------------------------------------------------------------------------
// Mode likelihoods are computed as LOG likelihoods and shifted by their maximum
// before exponentiating. The straightforward form, exp(-d^2/2) / sqrt(|2*pi*S|),
// underflows to zero for every mode at once as soon as one measurement is a few
// sigma out — which happens routinely in fog — and the mode probabilities then
// come out 0/0. Shifting by the max makes the largest term exactly 1 and the
// ratio is what the update needs anyway.
//
// Mode probabilities are floored at a small epsilon and renormalised. A mode
// that reaches exactly zero can never come back: the update multiplies by the
// prior, so zero is absorbing. On a figure-8 that would mean the first straight
// section permanently kills the turn model.
//
// INV-4: every matrix is a fixed-size Eigen type, so the whole filter is a flat
// block of doubles inside the Track. Nothing here allocates.

#pragma once

#include "core/units.hpp"
#include "tracking/kalman.hpp"
#include "tracking/measurement.hpp"

#include <Eigen/Dense>

#include <array>
#include <cstdint>

namespace sat {

/// Which of §10.2's three models. The order is the order of the mode
/// probability vector and of the mode-probability panel, so it is fixed.
enum class ImmMode : uint8_t { CV = 0, CA = 1, CT = 2 };

[[nodiscard]] const char* imm_mode_name(ImmMode m) noexcept;

/// Official row 12 → MotionNet / IMM regime id. Integer values match Python.
enum class MotionRegime : uint8_t { Line = 0, Circular = 1, Figure8 = 2, Random = 3 };

// ---------------------------------------------------------------------------
// ImmParams
// ---------------------------------------------------------------------------
struct ImmParams {
    /// The base filter's parameters — initial covariances and variance floors
    /// are shared, so a track seeded into the IMM starts from the same beliefs
    /// as one seeded into the plain filter.
    KalmanParams kf{};

    // -----------------------------------------------------------------------
    // Markov transition probabilities, "strongly diagonal (0.95 self)" per
    // §10.2.
    //
    // The diagonal is a statement about how long a manoeuvre lasts. 0.95 per
    // frame at 30 Hz gives an expected dwell of 1/(1-0.95) = 20 frames = 0.67 s,
    // which is the right order for a figure-8 lap of a few seconds: long enough
    // that the filter commits to a model, short enough that it can change its
    // mind twice a lap without lagging the crossing.
    //
    // Too high and the IMM becomes three independent filters that never switch;
    // too low and it chatters and loses the benefit of committing at all.
    // -----------------------------------------------------------------------
    double p_stay = 0.95;

    /// Process noise for the CA model, as a JERK power spectral density
    /// (urad^2/s^5). Derived from the scenario's maximum target acceleration
    /// the same way KalmanParams::from_max_accel derives q, one derivative up.
    double jerk_psd = 0.0;

    /// Process noise for the CT model's turn rate, rad/s per sqrt(s). The turn
    /// rate is estimated rather than a state, so this is how much the model is
    /// allowed to be wrong about it between frames.
    double turn_rate_sigma = 1.0;

    /// Below this speed the turn rate is not identifiable — omega is
    /// (vx*ay - vy*ax)/(vx^2 + vy^2) and the denominator vanishes. Under it the
    /// CT model degenerates to CV rather than dividing by something tiny.
    double min_turn_speed_urad_s = 1000.0;

    /// Largest turn rate the CT model will entertain, rad/s. A curvature
    /// estimate built from a noisy acceleration can spike arbitrarily high, and
    /// an unclamped omega*T near pi makes the rotation matrix predict the
    /// target has reversed.
    double max_turn_rate_rad_s = 4.0;

    /// Floor on each mode probability, before renormalisation. See the note on
    /// numerics: zero is absorbing.
    double mode_prob_floor = 1e-4;

    /// Build from the same inputs KalmanParams::from_max_accel takes.
    [[nodiscard]] static ImmParams from_max_accel(double a_max_urad_s2,
                                                  double dt_s) noexcept;
};

// ---------------------------------------------------------------------------
// ImmFilter
//
// The public surface is deliberately the SUBSET of KalmanFilter that
// tracking/track.cpp actually uses, with the same names and the same meanings,
// so that selecting between them is a branch at each call site rather than a
// redesign of Track.
// ---------------------------------------------------------------------------
class ImmFilter {
public:
    static constexpr int M = 3;    ///< CV, CA, CT
    static constexpr int N = 6;    ///< [az, el, vaz, vel, aaz, ael]

    using Vec6 = Eigen::Matrix<double, N, 1>;
    using Mat6 = Eigen::Matrix<double, N, N>;
    using Mat2 = Eigen::Matrix2d;

    void init(const Measurement& m, const ImmParams& p) noexcept;
    void init(Angle2 pos, Rate2 rate, double pos_sigma_urad,
              double rate_sigma_urad_s, const ImmParams& p) noexcept;

    void predict(double dt) noexcept;
    void update(Angle2 z, double sigma_urad) noexcept;
    void update(const Measurement& m) noexcept { update(m.angle, m.sigma_urad); }

    [[nodiscard]] Mat2   innovation_cov(double sigma_urad) const noexcept;
    [[nodiscard]] double mahalanobis2(Angle2 z, double sigma_urad) const noexcept;

    [[nodiscard]] Angle2 position() const noexcept { return {x_(0), x_(1)}; }
    [[nodiscard]] Rate2  rate()     const noexcept { return {x_(2), x_(3)}; }
    [[nodiscard]] Rate2  accel()    const noexcept { return {x_(4), x_(5)}; }
    [[nodiscard]] const Vec6& state()      const noexcept { return x_; }
    [[nodiscard]] const Mat6& covariance() const noexcept { return P_; }
    [[nodiscard]] double position_sigma_urad() const noexcept;

    /// Where the combined estimate says the target will be in `dt` seconds.
    /// Uses the CA form, because the combined state carries an acceleration
    /// and ignoring it would throw away the thing this filter exists to
    /// estimate.
    [[nodiscard]] Angle2 predict_position(double dt) const noexcept {
        return {x_(0) + x_(2) * dt + 0.5 * x_(4) * dt * dt,
                x_(1) + x_(3) * dt + 0.5 * x_(5) * dt * dt};
    }

    [[nodiscard]] bool initialised() const noexcept { return initialised_; }
    [[nodiscard]] Angle2 last_innovation() const noexcept { return {nu_(0), nu_(1)}; }
    [[nodiscard]] double last_nis() const noexcept { return nis_; }

    // --- the mode-probability panel (CP 10.5, §12's stacked area plot) ------
    [[nodiscard]] double mode_prob(ImmMode m) const noexcept {
        return mu_[static_cast<size_t>(m)];
    }
    [[nodiscard]] ImmMode dominant_mode() const noexcept;
    /// The CT model's current turn-rate estimate, rad/s. Signed.
    [[nodiscard]] double turn_rate_rad_s() const noexcept { return omega_; }

    /// Rebuild the stay/go Markov matrix, then boost this frame's regime.
    /// Boosts do not stack: every call starts from the default p_stay matrix.
    void set_regime_prior(MotionRegime regime, float confidence) noexcept;

private:
    void  mix() noexcept;
    void  reset_pi() noexcept;
    Mat6  transition(ImmMode m, double dt) const noexcept;
    Mat6  process_noise(ImmMode m, double dt) const noexcept;
    void  combine() noexcept;
    void  apply_floor(Mat6& P) const noexcept;

    ImmParams p_{};

    std::array<Vec6, M> xm_{};     ///< per-mode state
    std::array<Mat6, M> Pm_{};     ///< per-mode covariance
    std::array<double, M> mu_{};   ///< mode probabilities
    std::array<std::array<double, M>, M> pi_{};  ///< Markov P(to j | from i)

    Vec6 x_ = Vec6::Zero();        ///< combined
    Mat6 P_ = Mat6::Identity();

    Eigen::Vector2d nu_ = Eigen::Vector2d::Zero();
    double nis_   = 0.0;
    double omega_ = 0.0;           ///< CT turn rate, rad/s
    bool   initialised_ = false;
};

}  // namespace sat
