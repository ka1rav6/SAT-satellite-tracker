// tracking/kalman.hpp — CP 6.2. The filter.
//
// Design §10.2 is unusually specific, and the first sentence is the important
// one: "The filter is a plain linear Kalman filter, NOT an EKF."
//
// That is worth defending, because reaching for an EKF is the reflex. An EKF is
// needed when the measurement is a nonlinear function of the state. Here it is
// not: measurement.hpp has already turned pixels into an absolute world angle,
// so the measurement observes the state directly and H is a constant matrix of
// ones and zeros. Linearising something that is already linear buys nothing and
// costs a Jacobian that can be wrong silently.
//
//     state x = [az, el, az_rate, el_rate]^T        microradians, microradians/s
//     F = [[1,0,T,0],[0,1,0,T],[0,0,1,0],[0,0,0,1]]
//     H = [[1,0,0,0],[0,1,0,0]]
//     R = diag(sigma^2, sigma^2)                    adaptive, see CP 6.5
//     Q = q * [[T^3/3,0,T^2/2,0],[0,T^3/3,0,T^2/2],
//              [T^2/2,0,T,0],[0,T^2/2,0,T]]         continuous white noise accel
//
// ---------------------------------------------------------------------------
// WHY EIGEN, AND WHY IT DOES NOT BREAK INV-4
// ---------------------------------------------------------------------------
// INV-4 forbids heap allocation in the steady state. Eigen's FIXED-SIZE types
// (Matrix4d, Matrix2d, Vector4d) are plain arrays inside the object — there is
// no allocation anywhere in this file, and the filter is a member of the
// pipeline, so it is not even constructed per frame. It is Eigen's DYNAMIC
// types (MatrixXd) that allocate, and none appear here.
//
// The alternative was to hand-roll it. Given R = sigma^2 * I and the Q above,
// azimuth and elevation are exactly decoupled, so this 4-state filter is
// algebraically two independent 2-state filters and could be written with 2x2
// closed forms and no library at all. It is deliberately NOT written that way:
// CP 10.5's IMM adds a coordinate-turn model whose F couples the two axes, and
// the decoupled form would have to be thrown away and rewritten at exactly the
// point where the filter is hardest to debug. The full form costs a few
// microseconds a frame and survives that change unmodified.
//
// ---------------------------------------------------------------------------
// JOSEPH FORM
// ---------------------------------------------------------------------------
// The covariance update is written as
//
//     P = (I - KH) P (I - KH)^T + K R K^T
//
// rather than the shorter P = (I - KH) P. The short form is only correct for
// the exactly-optimal K and, being a difference of two similar quantities, it
// loses symmetry and eventually positive-definiteness in floating point. That
// matters here specifically because of coasting: §10.2's lifecycle allows 15
// consecutive misses, which is 15 predicts with no update, and a P that has
// drifted asymmetric makes the Mahalanobis gate in CP 6.3 produce negative
// distances. The Joseph form is unconditionally symmetric and stays PSD for any
// K, which is also what lets CP 6.5 vary R per frame without care.

#pragma once

#include "core/units.hpp"
#include "tracking/measurement.hpp"

#include <Eigen/Dense>

namespace sat {

// ---------------------------------------------------------------------------
// KalmanParams
// ---------------------------------------------------------------------------
struct KalmanParams {
    // -----------------------------------------------------------------------
    // q — the acceleration power spectral density, (urad/s^2)^2 per Hz, i.e.
    // urad^2/s^3.
    //
    // This is the single tuning knob, and it encodes one belief: how wrong the
    // constant-velocity assumption is. Too small and the filter refuses to
    // believe a manoeuvre, lagging through every figure-8 crossing. Too large
    // and it believes the noise, which is the same as not filtering at all.
    //
    // The default comes from from_max_accel() below rather than from a number
    // typed here, because a number typed here would have no defensible origin.
    // -----------------------------------------------------------------------
    double accel_psd_urad2_s3 = 0.0;

    /// One-point initialisation. The position sigma is the measurement's own,
    /// so only the rate needs a prior — and with no velocity information at all
    /// the honest prior is "anything up to the fastest thing that can happen",
    /// which is the mount's own slew limit (spec rows 13-14, 5-10 deg/s).
    double initial_rate_sigma_urad_s = 175000.0;   // 10 deg/s

    /// Floor on the diagonal of P. Guards against a long run of very confident
    /// measurements collapsing the covariance to the point where the gate in
    /// CP 6.3 becomes so tight that one bad frame ejects a perfectly good
    /// track and never lets it back in.
    double min_pos_var  = 1.0;      // urad^2, i.e. 1 urad sigma
    double min_rate_var = 1.0;      // (urad/s)^2

    // -----------------------------------------------------------------------
    // from_max_accel — turn a physical bound into q.
    //
    // The standard heuristic (Bar-Shalom, "Estimation with Applications to
    // Tracking and Navigation", §6.2.2) is that for CWNA the process noise
    // intensity should be of the order of the largest acceleration increment
    // the target can produce over one sample period:
    //
    //     q ~ a_max^2 * T        with T the sampling interval
    //
    // It is a heuristic and is labelled as one. What makes it defensible here
    // rather than arbitrary is that a_max is not invented: the scenario states
    // the target's motion analytically (§7.2), so the maximum acceleration over
    // a figure-8 or a circle is a number that can be computed rather than
    // guessed, and CP 6.2's acceptance test checks the resulting estimate
    // against that same analytic velocity to 2%.
    // -----------------------------------------------------------------------
    [[nodiscard]] static KalmanParams from_max_accel(double a_max_urad_s2,
                                                     double dt_s) noexcept {
        KalmanParams p;
        p.accel_psd_urad2_s3 = a_max_urad_s2 * a_max_urad_s2 * dt_s;
        return p;
    }
};

// ---------------------------------------------------------------------------
// KalmanFilter
// ---------------------------------------------------------------------------
class KalmanFilter {
public:
    using Vec4 = Eigen::Matrix<double, 4, 1>;
    using Mat4 = Eigen::Matrix<double, 4, 4>;
    using Mat2 = Eigen::Matrix<double, 2, 2>;
    using Vec2d = Eigen::Matrix<double, 2, 1>;

    /// One-point initialisation from a single measurement: believe the
    /// position, admit total ignorance of the velocity.
    ///
    /// Two-point initialisation (differencing the first two measurements to
    /// seed the rate) converges faster but needs a second frame to exist, and
    /// the whole point of the Tentative state in CP 6.4 is that we do not yet
    /// know whether there will be one. One-point plus a wide prior reaches the
    /// same place within three or four frames and has no special case.
    void init(const Measurement& m, const KalmanParams& p) noexcept;

    /// Direct initialisation, for tests and for reacquisition from a predicted
    /// position where a velocity IS known.
    void init(Angle2 pos, Rate2 rate, double pos_sigma_urad,
              double rate_sigma_urad_s, const KalmanParams& p) noexcept;

    /// x <- F x,  P <- F P F^T + Q.  `dt` in seconds.
    void predict(double dt) noexcept;

    /// Standard update with R = sigma^2 * I, Joseph form.
    void update(Angle2 z, double sigma_urad) noexcept;
    void update(const Measurement& m) noexcept { update(m.angle, m.sigma_urad); }

    // --- gating (CP 6.3) ---------------------------------------------------

    /// S = H P H^T + R — the innovation covariance. The gate's metric.
    [[nodiscard]] Mat2 innovation_cov(double sigma_urad) const noexcept;

    /// d^2 = (z - Hx)^T S^-1 (z - Hx). Compared against chi^2(2, 0.99) = 9.21.
    [[nodiscard]] double mahalanobis2(Angle2 z, double sigma_urad) const noexcept;

    // --- accessors ---------------------------------------------------------
    [[nodiscard]] Angle2 position() const noexcept { return {x_(0), x_(1)}; }
    [[nodiscard]] Rate2  rate()     const noexcept { return {x_(2), x_(3)}; }
    [[nodiscard]] const Vec4& state() const noexcept { return x_; }
    [[nodiscard]] const Mat4& covariance() const noexcept { return P_; }

    /// RMS position uncertainty over the two axes, microradians. What the GUI
    /// draws as the uncertainty ellipse's nominal radius, and what CP 6.4's
    /// coasting logic watches grow.
    [[nodiscard]] double position_sigma_urad() const noexcept;

    /// Where the target will be `dt` seconds from now, on the current estimate.
    /// This is what §6.2 step B25 aims at during Track, and what CP 6.7 points
    /// the camera at when reacquiring.
    [[nodiscard]] Angle2 predict_position(double dt) const noexcept {
        return {x_(0) + x_(2) * dt, x_(1) + x_(3) * dt};
    }

    [[nodiscard]] bool initialised() const noexcept { return initialised_; }

    /// The last innovation, microradians. CP 6.5's adaptive-R diagnostic and
    /// the normalised innovation squared (NIS) consistency check both read it.
    [[nodiscard]] Angle2 last_innovation() const noexcept { return {nu_(0), nu_(1)}; }

    /// Normalised innovation squared from the last update. For a consistent
    /// filter this averages 2 (the measurement dimension) — a cheap, standard
    /// self-check that catches a mistuned q without any ground truth, which
    /// means it is available at runtime and not just in tests.
    [[nodiscard]] double last_nis() const noexcept { return nis_; }

private:
    void apply_variance_floor() noexcept;

    Vec4         x_ = Vec4::Zero();
    Mat4         P_ = Mat4::Identity();
    KalmanParams p_{};
    Vec2d        nu_ = Vec2d::Zero();
    double       nis_ = 0.0;
    bool         initialised_ = false;
};

/// chi^2 inverse CDF at p = 0.99 with 2 degrees of freedom. Design §10.2 states
/// the number; it is written out here with its closed form because for 2 d.o.f.
/// the chi-square CDF is 1 - exp(-d^2/2), so the threshold is exactly
/// -2*ln(0.01) = 9.2103..., and a reader should be able to check it rather than
/// trust a table.
inline constexpr double kGateChi2_2dof_99 = 9.210340371976184;

}  // namespace sat
