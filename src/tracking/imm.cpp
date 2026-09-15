// tracking/imm.cpp — CP 10.5.

#include "tracking/imm.hpp"

#include <algorithm>
#include <cmath>

namespace sat {

const char* imm_mode_name(ImmMode m) noexcept {
    switch (m) {
        case ImmMode::CV: return "CV";
        case ImmMode::CA: return "CA";
        case ImmMode::CT: return "CT";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// ImmParams::from_max_accel
//
// KalmanParams::from_max_accel sets q = a_max^2 * dt, the standard continuous
// white-noise-acceleration sizing: the model is "velocity random-walks with an
// acceleration bounded by a_max over one step". The CA model is that argument
// one derivative up — its process noise is a JERK, and the jerk a manoeuvring
// target can produce is bounded by how fast its acceleration can reverse.
//
// A figure-8 reverses its acceleration once per crossing, so over a lap of a
// few seconds the jerk scale is roughly a_max / (lap time / 4). Rather than
// taking a lap time this file has no business knowing, the sizing uses the
// control period, which makes it deliberately GENEROUS: the CA model is
// permitted to change its acceleration quickly, which is exactly what it is
// for, and the IMM's mode probabilities are what stop that generosity being
// used when the target is going straight.
// ---------------------------------------------------------------------------
ImmParams ImmParams::from_max_accel(double a_max_urad_s2, double dt_s) noexcept {
    ImmParams p;
    p.kf = KalmanParams::from_max_accel(a_max_urad_s2, dt_s);
    const double dt = dt_s > 0.0 ? dt_s : 1.0 / 30.0;
    const double j_max = a_max_urad_s2 / dt;      // accel can reverse in one step
    p.jerk_psd = j_max * j_max * dt;
    return p;
}

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
void ImmFilter::init(Angle2 pos, Rate2 rate, double pos_sigma_urad,
                     double rate_sigma_urad_s, const ImmParams& p) noexcept {
    p_ = p;

    Vec6 x = Vec6::Zero();
    x(0) = pos.x;  x(1) = pos.y;
    x(2) = rate.x; x(3) = rate.y;
    // Acceleration starts at zero with a wide prior. Seeding it from anything
    // would be inventing a measurement; the CA model's whole job is to find it.

    Mat6 P = Mat6::Zero();
    P(0, 0) = P(1, 1) = pos_sigma_urad * pos_sigma_urad;
    P(2, 2) = P(3, 3) = rate_sigma_urad_s * rate_sigma_urad_s;
    // The acceleration prior is the rate prior one derivative up, over one
    // control period — the same reasoning as the rate prior itself, which
    // kalman.hpp sizes from the mount's slew rate rather than from nothing.
    const double a_sigma = rate_sigma_urad_s * 30.0;
    P(4, 4) = P(5, 5) = a_sigma * a_sigma;

    for (int m = 0; m < M; ++m) {
        xm_[static_cast<size_t>(m)] = x;
        Pm_[static_cast<size_t>(m)] = P;
        // Uniform prior over the three models. Anything else would be a claim
        // about the target's behaviour before a single frame has been seen.
        mu_[static_cast<size_t>(m)] = 1.0 / static_cast<double>(M);
    }
    x_ = x;
    P_ = P;
    nu_.setZero();
    nis_   = 0.0;
    omega_ = 0.0;
    initialised_ = true;
}

void ImmFilter::init(const Measurement& m, const ImmParams& p) noexcept {
    init(m.angle, Rate2{}, m.sigma_urad, p.kf.initial_rate_sigma_urad_s, p);
}

// ---------------------------------------------------------------------------
// transition — F for one model.
// ---------------------------------------------------------------------------
ImmFilter::Mat6 ImmFilter::transition(ImmMode m, double T) const noexcept {
    Mat6 F = Mat6::Identity();

    switch (m) {
        case ImmMode::CV:
            // Position advances by velocity. Acceleration is driven to ZERO,
            // because that is what "constant velocity" asserts about it — a CV
            // model that carried the acceleration forward would be a CA model
            // with extra steps and the IMM would have two of those and no CV.
            F(0, 2) = T;  F(1, 3) = T;
            F(4, 4) = 0.0; F(5, 5) = 0.0;
            break;

        case ImmMode::CA:
            F(0, 2) = T;  F(1, 3) = T;
            F(0, 4) = 0.5 * T * T;  F(1, 5) = 0.5 * T * T;
            F(2, 4) = T;  F(3, 5) = T;
            break;

        case ImmMode::CT: {
            // Coordinated turn: the velocity VECTOR rotates at omega while its
            // magnitude is preserved. This is the only model whose F couples
            // azimuth and elevation, and therefore the only source of an
            // anisotropic predicted covariance — §10.2 notes that the 4-state
            // CV filter's covariance is exactly isotropic and that anisotropy
            // "arrives with the coordinate-turn model at CP 10.5".
            const double w = omega_;
            if (std::fabs(w) < 1e-6) {
                // The limit as omega -> 0 is exactly CV. Taking it explicitly
                // rather than letting sin(wT)/w be computed as 0/0.
                F(0, 2) = T;  F(1, 3) = T;
            } else {
                const double wt = w * T;
                const double s  = std::sin(wt) / w;
                const double c  = (1.0 - std::cos(wt)) / w;
                F(0, 2) =  s;  F(0, 3) = -c;
                F(1, 2) =  c;  F(1, 3) =  s;
                F(2, 2) =  std::cos(wt);  F(2, 3) = -std::sin(wt);
                F(3, 2) =  std::sin(wt);  F(3, 3) =  std::cos(wt);
            }
            F(4, 4) = 0.0; F(5, 5) = 0.0;
            break;
        }
    }
    return F;
}

// ---------------------------------------------------------------------------
// process_noise — Q for one model.
// ---------------------------------------------------------------------------
ImmFilter::Mat6 ImmFilter::process_noise(ImmMode m, double T) const noexcept {
    Mat6 Q = Mat6::Zero();
    const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;

    switch (m) {
        case ImmMode::CV:
        case ImmMode::CT: {
            // Continuous white noise acceleration, exactly as kalman.hpp's Q,
            // embedded in the 6-state space. The acceleration block gets the
            // prior's own variance rather than zero: these models assert the
            // acceleration is zero, and a zero-variance assertion is one the
            // filter can never revise, which would poison the mixing step.
            const double q = p_.kf.accel_psd_urad2_s3;
            Q(0, 0) = Q(1, 1) = q * T3 / 3.0;
            Q(0, 2) = Q(2, 0) = Q(1, 3) = Q(3, 1) = q * T2 / 2.0;
            Q(2, 2) = Q(3, 3) = q * T;
            Q(4, 4) = Q(5, 5) = q / T;    // "we do not model it", not "it is exactly 0"
            break;
        }
        case ImmMode::CA: {
            // Wiener-process acceleration: the standard jerk-driven Q.
            const double s = p_.jerk_psd;
            Q(0, 0) = Q(1, 1) = s * T5 / 20.0;
            Q(0, 2) = Q(2, 0) = Q(1, 3) = Q(3, 1) = s * T4 / 8.0;
            Q(0, 4) = Q(4, 0) = Q(1, 5) = Q(5, 1) = s * T3 / 6.0;
            Q(2, 2) = Q(3, 3) = s * T3 / 3.0;
            Q(2, 4) = Q(4, 2) = Q(3, 5) = Q(5, 3) = s * T2 / 2.0;
            Q(4, 4) = Q(5, 5) = s * T;
            break;
        }
    }
    return Q;
}

// ---------------------------------------------------------------------------
// mix — step 1 of §10.2's four.
//
//   c_j      = sum_i pi_ij mu_i                     the predicted mode prob
//   mu_{i|j} = pi_ij mu_i / c_j                     the mixing weights
//   x0_j     = sum_i mu_{i|j} x_i
//   P0_j     = sum_i mu_{i|j} [ P_i + (x_i - x0_j)(x_i - x0_j)^T ]
//
// The spread term in P0_j is the part that makes an IMM an IMM rather than a
// bank of filters: it charges each model for the DISAGREEMENT between models,
// so a filter that is about to take over inherits an honest uncertainty instead
// of the confident covariance of a model that was wrong.
// ---------------------------------------------------------------------------
void ImmFilter::mix() noexcept {
    // Markov matrix, strongly diagonal, off-diagonals sharing the remainder
    // equally. Asymmetric transitions (CV -> CT more likely than CT -> CV, say)
    // would be a claim about the target's behaviour that nothing here supports.
    const double stay = p_.p_stay;
    const double go   = (1.0 - stay) / static_cast<double>(M - 1);

    std::array<double, M> c{};
    std::array<std::array<double, M>, M> w{};   // w[j][i] = mu_{i|j}

    for (int j = 0; j < M; ++j) {
        double cj = 0.0;
        for (int i = 0; i < M; ++i) {
            const double pij = (i == j) ? stay : go;
            cj += pij * mu_[static_cast<size_t>(i)];
        }
        c[static_cast<size_t>(j)] = cj;
        for (int i = 0; i < M; ++i) {
            const double pij = (i == j) ? stay : go;
            w[static_cast<size_t>(j)][static_cast<size_t>(i)] =
                cj > 0.0 ? pij * mu_[static_cast<size_t>(i)] / cj : 1.0 / M;
        }
    }

    std::array<Vec6, M> x0{};
    std::array<Mat6, M> P0{};
    for (int j = 0; j < M; ++j) {
        const size_t sj = static_cast<size_t>(j);
        x0[sj] = Vec6::Zero();
        for (int i = 0; i < M; ++i) {
            x0[sj] += w[sj][static_cast<size_t>(i)] * xm_[static_cast<size_t>(i)];
        }
        P0[sj] = Mat6::Zero();
        for (int i = 0; i < M; ++i) {
            const size_t si = static_cast<size_t>(i);
            const Vec6 d = xm_[si] - x0[sj];
            P0[sj] += w[sj][si] * (Pm_[si] + d * d.transpose());
        }
    }
    xm_ = x0;
    Pm_ = P0;
    // The predicted mode probabilities become the prior for the update.
    mu_ = c;
}

// ---------------------------------------------------------------------------
// predict — mix, then each model forward.
// ---------------------------------------------------------------------------
void ImmFilter::predict(double dt) noexcept {
    if (!initialised_ || dt <= 0.0) return;

    // The CT model's turn rate, from the COMBINED state. omega is the signed
    // curvature times speed: the component of acceleration perpendicular to
    // velocity, divided by speed squared.
    //
    // Estimated rather than made a seventh state, which is the other standard
    // choice. A state would be more principled and would also make the filter
    // nonlinear in its own state — an EKF — and §10.2 opens by insisting the
    // filter is "a plain linear Kalman filter, NOT an EKF". Estimating omega
    // from the previous combined state keeps every model's F a constant matrix
    // within the step, which is what makes that sentence still true.
    const double vx = x_(2), vy = x_(3), ax = x_(4), ay = x_(5);
    const double v2 = vx * vx + vy * vy;
    if (v2 > p_.min_turn_speed_urad_s * p_.min_turn_speed_urad_s) {
        omega_ = (vx * ay - vy * ax) / v2;
        omega_ = clamp_abs(omega_, p_.max_turn_rate_rad_s);
    } else {
        // Below the speed floor the turn rate is not identifiable — the
        // denominator vanishes and the estimate is noise divided by noise. The
        // CT model degenerates to CV, which is the right answer for a target
        // that is barely moving.
        omega_ = 0.0;
    }

    mix();

    for (int m = 0; m < M; ++m) {
        const size_t s = static_cast<size_t>(m);
        const Mat6 F = transition(static_cast<ImmMode>(m), dt);
        const Mat6 Q = process_noise(static_cast<ImmMode>(m), dt);
        xm_[s] = F * xm_[s];
        Pm_[s] = F * Pm_[s] * F.transpose() + Q;
        Pm_[s] = 0.5 * (Pm_[s] + Pm_[s].transpose().eval());   // keep it symmetric
        apply_floor(Pm_[s]);
    }
    combine();
}

// ---------------------------------------------------------------------------
// update — mode-matched updates, then the mode probabilities, then combine.
// ---------------------------------------------------------------------------
void ImmFilter::update(Angle2 z, double sigma_urad) noexcept {
    if (!initialised_) return;

    const double r = std::max(sigma_urad * sigma_urad, p_.kf.min_pos_var);
    Eigen::Matrix<double, 2, N> H = Eigen::Matrix<double, 2, N>::Zero();
    H(0, 0) = 1.0; H(1, 1) = 1.0;
    const Mat2 R = Mat2::Identity() * r;
    const Eigen::Vector2d zv(z.x, z.y);

    std::array<double, M> log_lik{};
    for (int m = 0; m < M; ++m) {
        const size_t s = static_cast<size_t>(m);
        const Eigen::Vector2d nu = zv - H * xm_[s];
        const Mat2 S  = H * Pm_[s] * H.transpose() + R;
        const Mat2 Si = S.inverse();
        const Eigen::Matrix<double, N, 2> K = Pm_[s] * H.transpose() * Si;

        xm_[s] += K * nu;
        // Joseph form, for the reasons kalman.hpp sets out at length: it is
        // unconditionally symmetric and stays positive semi-definite for any K,
        // which is what makes 15 coasted frames and a per-frame R safe.
        const Mat6 IKH = Mat6::Identity() - K * H;
        Pm_[s] = IKH * Pm_[s] * IKH.transpose() + K * R * K.transpose();
        Pm_[s] = 0.5 * (Pm_[s] + Pm_[s].transpose().eval());
        apply_floor(Pm_[s]);

        // log N(nu; 0, S), dropping the constant -log(2*pi) that is common to
        // every mode and cancels in the normalisation.
        const double d2  = nu.transpose() * Si * nu;
        const double det = std::max(S.determinant(), 1e-300);
        log_lik[s] = -0.5 * (d2 + std::log(det));
    }

    // Mode probabilities: mu_j <- c_j * Lambda_j, normalised. Done in the log
    // domain and shifted by the maximum — see the numerics note in the header.
    const double hi = *std::max_element(log_lik.begin(), log_lik.end());
    double total = 0.0;
    for (int m = 0; m < M; ++m) {
        const size_t s = static_cast<size_t>(m);
        mu_[s] *= std::exp(log_lik[s] - hi);
        total += mu_[s];
    }
    if (total > 0.0) {
        for (double& v : mu_) v /= total;
    } else {
        for (double& v : mu_) v = 1.0 / static_cast<double>(M);
    }
    // Floor and renormalise: a mode at exactly zero can never recover, because
    // the update multiplies by the prior. On a figure-8 that would mean the
    // first straight section permanently kills the turn model.
    double after = 0.0;
    for (double& v : mu_) { v = std::max(v, p_.mode_prob_floor); after += v; }
    for (double& v : mu_) v /= after;

    combine();

    // Reported against the COMBINED estimate, because that is what the gate and
    // the consistency check in CP 6.2 are about — not against whichever mode
    // happened to win.
    const Eigen::Vector2d nu = zv - H * x_;
    const Mat2 S  = H * P_ * H.transpose() + R;
    nu_  = nu;
    nis_ = nu.transpose() * S.inverse() * nu;
}

// ---------------------------------------------------------------------------
// combine — step 4. The probability-weighted mean, plus the spread.
// ---------------------------------------------------------------------------
void ImmFilter::combine() noexcept {
    x_ = Vec6::Zero();
    for (int m = 0; m < M; ++m) {
        const size_t s = static_cast<size_t>(m);
        x_ += mu_[s] * xm_[s];
    }
    P_ = Mat6::Zero();
    for (int m = 0; m < M; ++m) {
        const size_t s = static_cast<size_t>(m);
        const Vec6 d = xm_[s] - x_;
        P_ += mu_[s] * (Pm_[s] + d * d.transpose());
    }
    P_ = 0.5 * (P_ + P_.transpose().eval());
}

void ImmFilter::apply_floor(Mat6& P) const noexcept {
    P(0, 0) = std::max(P(0, 0), p_.kf.min_pos_var);
    P(1, 1) = std::max(P(1, 1), p_.kf.min_pos_var);
    P(2, 2) = std::max(P(2, 2), p_.kf.min_rate_var);
    P(3, 3) = std::max(P(3, 3), p_.kf.min_rate_var);
    P(4, 4) = std::max(P(4, 4), p_.kf.min_rate_var);
    P(5, 5) = std::max(P(5, 5), p_.kf.min_rate_var);
}

ImmFilter::Mat2 ImmFilter::innovation_cov(double sigma_urad) const noexcept {
    const double r = std::max(sigma_urad * sigma_urad, p_.kf.min_pos_var);
    Mat2 S;
    S << P_(0, 0) + r, P_(0, 1),
         P_(1, 0),     P_(1, 1) + r;
    return S;
}

double ImmFilter::mahalanobis2(Angle2 z, double sigma_urad) const noexcept {
    const Eigen::Vector2d nu(z.x - x_(0), z.y - x_(1));
    const Mat2 S = innovation_cov(sigma_urad);
    return nu.transpose() * S.inverse() * nu;
}

double ImmFilter::position_sigma_urad() const noexcept {
    return std::sqrt(std::max(0.5 * (P_(0, 0) + P_(1, 1)), 0.0));
}

ImmMode ImmFilter::dominant_mode() const noexcept {
    int best = 0;
    for (int m = 1; m < M; ++m) {
        if (mu_[static_cast<size_t>(m)] > mu_[static_cast<size_t>(best)]) best = m;
    }
    return static_cast<ImmMode>(best);
}

}  // namespace sat
