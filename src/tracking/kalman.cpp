// tracking/kalman.cpp

#include "tracking/kalman.hpp"

#include <algorithm>
#include <cmath>

namespace sat {

void KalmanFilter::init(Angle2 pos, Rate2 rate, double pos_sigma_urad,
                        double rate_sigma_urad_s, const KalmanParams& p) noexcept {
    p_ = p;
    x_ << pos.x, pos.y, rate.x, rate.y;
    P_.setZero();
    const double pv = pos_sigma_urad * pos_sigma_urad;
    const double rv = rate_sigma_urad_s * rate_sigma_urad_s;
    P_(0, 0) = P_(1, 1) = pv;
    P_(2, 2) = P_(3, 3) = rv;
    apply_variance_floor();
    nu_.setZero();
    nis_ = 0.0;
    initialised_ = true;
}

void KalmanFilter::init(const Measurement& m, const KalmanParams& p) noexcept {
    init(m.angle, Rate2{}, m.sigma_urad, p.initial_rate_sigma_urad_s, p);
}

void KalmanFilter::predict(double dt) noexcept {
    if (!initialised_ || dt <= 0.0) return;

    // --- x <- F x -----------------------------------------------------------
    // F is written out by hand rather than built as a matrix and multiplied.
    // With F this sparse (identity plus two entries) the explicit form is both
    // faster and — more importantly — impossible to get subtly wrong in a way
    // that still produces plausible numbers.
    x_(0) += x_(2) * dt;
    x_(1) += x_(3) * dt;

    // --- P <- F P F^T -------------------------------------------------------
    // Also expanded. For F = I + dt*N with N the two-entry shift matrix,
    // F P F^T = P + dt*(N P + P N^T) + dt^2 * N P N^T, and writing that out
    // costs six lines against a 4x4 multiply's 128 flops.
    //
    // Every right-hand side must read the OLD P, so it is copied first — 128
    // bytes on the stack, no allocation. Writing in place would be a classic
    // aliasing bug: the position-position entry depends on the rate-rate entry,
    // which the same sweep also rewrites.
    const Mat4 P = P_;
    for (int a = 0; a < 2; ++a) {
        const int pr = a;       // position row/col for this axis
        const int rr = a + 2;   // rate row/col
        for (int b = 0; b < 2; ++b) {
            const int pc = b, rc = b + 2;
            // position-position
            P_(pr, pc) = P(pr, pc) + dt * (P(rr, pc) + P(pr, rc)) + dt * dt * P(rr, rc);
            // position-rate and its transpose
            P_(pr, rc) = P(pr, rc) + dt * P(rr, rc);
            P_(rr, pc) = P(rr, pc) + dt * P(rr, rc);
            // rate-rate is unchanged by F
            P_(rr, rc) = P(rr, rc);
        }
    }

    // --- P <- P + Q ---------------------------------------------------------
    // Continuous white noise acceleration, integrated exactly over the interval
    // (design §10.2). The cross term T^2/2 is what makes this different from
    // just inflating the diagonal: position and velocity errors accumulated by
    // an unmodelled acceleration are CORRELATED, and dropping the correlation
    // makes the filter think two independent things went wrong instead of one.
    const double q  = p_.accel_psd_urad2_s3;
    const double t2 = dt * dt;
    const double t3 = t2 * dt;
    const double qpp = q * t3 / 3.0;
    const double qpr = q * t2 / 2.0;
    const double qrr = q * dt;
    for (int a = 0; a < 2; ++a) {
        P_(a, a)         += qpp;
        P_(a, a + 2)     += qpr;
        P_(a + 2, a)     += qpr;
        P_(a + 2, a + 2) += qrr;
    }

    apply_variance_floor();
}

KalmanFilter::Mat2 KalmanFilter::innovation_cov(double sigma_urad) const noexcept {
    // S = H P H^T + R. With H selecting the first two states, H P H^T is simply
    // the top-left 2x2 block of P — no multiplication needed.
    Mat2 S = P_.topLeftCorner<2, 2>();
    const double r = sigma_urad * sigma_urad;
    S(0, 0) += r;
    S(1, 1) += r;
    return S;
}

double KalmanFilter::mahalanobis2(Angle2 z, double sigma_urad) const noexcept {
    if (!initialised_) return 0.0;
    const Vec2d nu(z.x - x_(0), z.y - x_(1));
    const Mat2  S = innovation_cov(sigma_urad);
    // Eigen's 2x2 inverse is the closed-form adjugate/determinant, not a
    // decomposition, so this is branch-free and exact up to rounding. S is a
    // covariance plus a positive diagonal, so it cannot be singular unless P
    // has gone bad — which the variance floor prevents.
    return nu.dot(S.inverse() * nu);
}

void KalmanFilter::update(Angle2 z, double sigma_urad) noexcept {
    if (!initialised_) return;

    const Vec2d nu(z.x - x_(0), z.y - x_(1));
    const Mat2  S     = innovation_cov(sigma_urad);
    const Mat2  S_inv = S.inverse();

    // K = P H^T S^-1. P H^T is the first two COLUMNS of P (4x2), again because
    // H is a selector.
    const Eigen::Matrix<double, 4, 2> PHt = P_.leftCols<2>();
    const Eigen::Matrix<double, 4, 2> K   = PHt * S_inv;

    x_ += K * nu;

    // --- Joseph form; see the header for why -------------------------------
    // (I - K H) has K's two columns subtracted from the first two columns of I.
    Mat4 IKH = Mat4::Identity();
    IKH.leftCols<2>() -= K;

    Mat2 R = Mat2::Zero();
    R(0, 0) = R(1, 1) = sigma_urad * sigma_urad;

    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();

    // Symmetry is guaranteed algebraically but not bit-exactly: A*P*A^T + K*R*K^T
    // computed in floating point can differ in the last ulp across the diagonal.
    // Forcing it costs nothing and keeps the gate's S strictly symmetric, which
    // its closed-form inverse assumes.
    P_ = (0.5 * (P_ + P_.transpose())).eval();

    apply_variance_floor();

    nu_  = nu;
    nis_ = nu.dot(S_inv * nu);
}

double KalmanFilter::position_sigma_urad() const noexcept {
    return std::sqrt(0.5 * (P_(0, 0) + P_(1, 1)));
}

void KalmanFilter::apply_variance_floor() noexcept {
    P_(0, 0) = std::max(P_(0, 0), p_.min_pos_var);
    P_(1, 1) = std::max(P_(1, 1), p_.min_pos_var);
    P_(2, 2) = std::max(P_(2, 2), p_.min_rate_var);
    P_(3, 3) = std::max(P_(3, 3), p_.min_rate_var);
}

}  // namespace sat
