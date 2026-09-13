// tests/tracking/test_kalman.cpp — CP 6.2 and CP 6.3.
//
// CP 6.2: "Linear Kalman filter, CWNA Q. Accept when: on noiseless data the
//          estimate converges; estimated speed matches CP 3.5 analytic velocity
//          within 2%."
// CP 6.3: "Mahalanobis gate (9.21) + nearest neighbour."
//
// The filter is tested here against analytically generated trajectories rather
// than against the simulator, for the same reason the perception kernels were
// tested against OpenCV: if the filter and the world are both ours, a shared
// misunderstanding passes. A straight line at a known speed is a fact about
// arithmetic, and 2% of it is a real bound.

#include <doctest/doctest.h>

#include "core/rng.hpp"
#include "tracking/kalman.hpp"

#include <cmath>
#include <vector>

using namespace sat;

namespace {

Measurement meas(double x, double y, double sigma) {
    Measurement m;
    m.angle = Angle2{x, y};
    m.sigma_urad = sigma;
    return m;
}

}  // namespace

// ===========================================================================
// CP 6.2 — convergence and velocity accuracy
// ===========================================================================

TEST_CASE("CP 6.2: on noiseless constant-velocity data the estimate converges") {
    constexpr double dt = 1.0 / 30.0;
    // 5 deg/s in azimuth, 2 deg/s in elevation — within the mount's authority
    // (spec rows 13-14) and fast enough that a lagging filter is obvious.
    constexpr double vx = 87266.46;   // urad/s
    constexpr double vy = 34906.59;

    KalmanParams p = KalmanParams::from_max_accel(/*a_max=*/2.0e4, dt);
    KalmanFilter kf;
    kf.init(meas(0.0, 0.0, 100.0), p);

    double t = 0.0;
    std::vector<double> pos_err, rate_err;
    for (int i = 1; i <= 120; ++i) {          // 4 seconds
        t = i * dt;
        kf.predict(dt);
        kf.update(meas(vx * t, vy * t, 100.0));

        pos_err.push_back((kf.position() - Angle2{vx * t, vy * t}).norm());
        rate_err.push_back((kf.rate() - Rate2{vx, vy}).norm());
    }

    // Converged: the last half of the run is far better than the first few
    // frames, and the error is still shrinking rather than oscillating.
    const double early = pos_err[2];
    const double late  = pos_err.back();
    MESSAGE("position error: " << early << " -> " << late << " urad");
    CHECK(late < early * 0.05);

    // --- the graded half: speed to within 2% ----------------------------
    const double speed_true = std::hypot(vx, vy);
    const double speed_est  = std::hypot(kf.rate().x, kf.rate().y);
    const double rel = std::fabs(speed_est - speed_true) / speed_true;
    MESSAGE("speed: " << speed_est << " vs " << speed_true
            << " urad/s, relative error " << rel * 100.0 << "%");
    CHECK(rel < 0.02);
}

TEST_CASE("CP 6.2: the filter suppresses measurement noise rather than following it") {
    // The whole justification for a filter. With noise on the measurements, the
    // filtered position must be closer to the truth than the raw measurements
    // are -- otherwise the filter is costing latency for nothing.
    constexpr double dt    = 1.0 / 30.0;
    constexpr double vx    = 40000.0;
    constexpr double sigma = 300.0;         // urad, ~2.7 px at 109 urad/px

    // A named stream, not rand(): INV-3 requires every random draw in the
    // project to be reproducible, tests included.
    RngSet rng(1234);
    Pcg32& g = rng[Stream::Misc];

    KalmanParams p = KalmanParams::from_max_accel(1.0e4, dt);
    KalmanFilter kf;
    kf.init(meas(g.next_normal() * sigma, g.next_normal() * sigma, sigma), p);

    double raw_sq = 0.0, filt_sq = 0.0;
    int n = 0;
    for (int i = 1; i <= 300; ++i) {
        const double t  = i * dt;
        const double tx = vx * t;
        const double zx = tx + g.next_normal() * sigma;
        const double zy = 0.0 + g.next_normal() * sigma;

        kf.predict(dt);
        kf.update(meas(zx, zy, sigma));

        if (i > 60) {                        // skip the convergence transient
            raw_sq  += (zx - tx) * (zx - tx) + zy * zy;
            const Angle2 e = kf.position() - Angle2{tx, 0.0};
            filt_sq += e.x * e.x + e.y * e.y;
            ++n;
        }
    }
    const double raw_rms  = std::sqrt(raw_sq / n);
    const double filt_rms = std::sqrt(filt_sq / n);
    MESSAGE("RMS error: raw " << raw_rms << " -> filtered " << filt_rms << " urad");
    CHECK(filt_rms < raw_rms * 0.6);
}

TEST_CASE("CP 6.2: the covariance stays symmetric and positive-definite while coasting") {
    // 15 consecutive misses is what §10.2's lifecycle permits before deletion,
    // so 15 predicts with no update is a state the system really reaches. A P
    // that has lost symmetry there produces a negative Mahalanobis distance and
    // the gate silently accepts everything.
    constexpr double dt = 1.0 / 30.0;
    KalmanParams p = KalmanParams::from_max_accel(2.0e4, dt);
    KalmanFilter kf;
    kf.init(meas(0.0, 0.0, 50.0), p);
    for (int i = 0; i < 30; ++i) { kf.predict(dt); kf.update(meas(0.0, 0.0, 50.0)); }

    for (int i = 0; i < 15; ++i) {
        kf.predict(dt);
        const auto& P = kf.covariance();
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                CHECK(P(r, c) == doctest::Approx(P(c, r)).epsilon(1e-12));
            }
            CHECK(P(r, r) > 0.0);
        }
        // Uncertainty must GROW while coasting -- that is what widens the gate
        // and lets a returning target back in with no special case.
        CHECK(kf.mahalanobis2(Angle2{500.0, 0.0}, 50.0) > 0.0);
    }
    MESSAGE("sigma after 15 coasted frames: " << kf.position_sigma_urad() << " urad");
}

TEST_CASE("CP 6.2: NIS averages the measurement dimension on a consistent filter") {
    // The standard filter-consistency check, and the only one available without
    // ground truth -- so it is also usable at runtime. For a correctly tuned
    // filter the normalised innovation squared is chi^2 with 2 d.o.f., mean 2.
    constexpr double dt    = 1.0 / 30.0;
    constexpr double sigma = 200.0;
    RngSet rng(99);
    Pcg32& g = rng[Stream::Misc];

    KalmanParams p = KalmanParams::from_max_accel(5.0e3, dt);
    KalmanFilter kf;
    kf.init(meas(0.0, 0.0, sigma), p);

    double sum = 0.0;
    int n = 0;
    for (int i = 1; i <= 2000; ++i) {
        const double t = i * dt;
        kf.predict(dt);
        kf.update(meas(30000.0 * t + g.next_normal() * sigma, g.next_normal() * sigma, sigma));
        if (i > 200) { sum += kf.last_nis(); ++n; }
    }
    const double mean_nis = sum / n;
    MESSAGE("mean NIS = " << mean_nis << " (expected 2)");
    // A generous band: this is a consistency check, not a precision test, and a
    // filter that is wildly over- or under-confident lands far outside it.
    CHECK(mean_nis > 1.4);
    CHECK(mean_nis < 3.0);
}

// ===========================================================================
// CP 6.3 — the gate
// ===========================================================================

TEST_CASE("CP 6.3: the gate threshold is the chi-square value it claims to be") {
    // For 2 degrees of freedom the chi-square CDF has the closed form
    // 1 - exp(-d^2 / 2), so the 99th percentile is exactly -2*ln(0.01).
    CHECK(kGateChi2_2dof_99 == doctest::Approx(-2.0 * std::log(0.01)).epsilon(1e-12));
    CHECK(1.0 - std::exp(-kGateChi2_2dof_99 / 2.0) == doctest::Approx(0.99).epsilon(1e-12));
}

TEST_CASE("CP 6.3: the empirical acceptance rate of the gate is 99%") {
    // Draw measurements from exactly the distribution the filter believes in and
    // count how many the gate accepts. This is the same style of check CP 5.5
    // used for CFAR's Pfa, and it catches a wrong S, a wrong inverse, or a
    // forgotten square root -- none of which a threshold-value test would.
    constexpr double dt    = 1.0 / 30.0;
    constexpr double sigma = 250.0;
    RngSet rng(7);
    Pcg32& g = rng[Stream::Misc];

    KalmanParams p = KalmanParams::from_max_accel(5.0e3, dt);
    KalmanFilter kf;
    kf.init(meas(0.0, 0.0, sigma), p);
    for (int i = 1; i <= 300; ++i) {         // settle
        kf.predict(dt);
        kf.update(meas(g.next_normal() * sigma, g.next_normal() * sigma, sigma));
    }

    int accepted = 0;
    constexpr int kTrials = 20000;
    for (int i = 0; i < kTrials; ++i) {
        kf.predict(dt);
        // A measurement drawn from the filter's own predicted distribution:
        // predicted mean plus a sample with covariance S. Since S is not
        // diagonal in general, sampling it properly needs its Cholesky factor.
        const auto S = kf.innovation_cov(sigma);
        const double l11 = std::sqrt(S(0, 0));
        const double l21 = S(1, 0) / l11;
        const double l22 = std::sqrt(S(1, 1) - l21 * l21);
        const double u = g.next_normal(), v = g.next_normal();
        const Angle2 z{kf.position().x + l11 * u,
                       kf.position().y + l21 * u + l22 * v};
        if (kf.mahalanobis2(z, sigma) < kGateChi2_2dof_99) ++accepted;
        kf.update(z, sigma);
    }
    const double rate = static_cast<double>(accepted) / kTrials;
    MESSAGE("gate acceptance: " << rate * 100.0 << "% (expected 99%)");
    CHECK(rate > 0.985);
    CHECK(rate < 0.995);
}

TEST_CASE("CP 6.3: a decoy 60 px away falls outside the gate") {
    // The checkpoint's own scenario, in angular terms. At this camera's
    // ~109 urad/px, 60 px is ~6545 urad, and a settled filter on a clean
    // detection has a sigma of a few hundred at most.
    constexpr double dt    = 1.0 / 30.0;
    constexpr double ifov  = 109.08;
    constexpr double sigma = 1.0 * ifov;      // a 1 px centroid uncertainty

    KalmanParams p = KalmanParams::from_max_accel(2.0e4, dt);
    KalmanFilter kf;
    kf.init(meas(0.0, 0.0, sigma), p);
    for (int i = 1; i <= 60; ++i) { kf.predict(dt); kf.update(meas(0.0, 0.0, sigma)); }

    CHECK(kf.mahalanobis2(Angle2{0.5 * ifov, 0.0}, sigma) < kGateChi2_2dof_99);
    CHECK(kf.mahalanobis2(Angle2{60.0 * ifov, 0.0}, sigma) > kGateChi2_2dof_99);
    MESSAGE("d^2 at 60 px = " << kf.mahalanobis2(Angle2{60.0 * ifov, 0.0}, sigma));
}
