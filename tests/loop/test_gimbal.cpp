// tests/loop/test_gimbal.cpp — CP 1.6 (and the CP 4.10 behaviours, since §10.3
// specifies the whole plant in one place).
//
// "Commanding a huge rate produces movement at exactly max_rate_urad_s."

#include <doctest/doctest.h>

#include "plant/gimbal.hpp"

#include <cmath>

using namespace sat;

namespace {
// Spec row 13's default: 5 deg/s.
constexpr double kMaxRate = 87266.46259971648;   // deg_to_urad(5.0)
constexpr double kDt      = 1.0 / 300.0;         // one truth tick

GimbalParams ideal_axis() {
    // No lag, no delay, no acceleration limit, no encoder quantisation: CP 1.6's
    // "hard rate clamp only". The other terms are exercised separately below.
    GimbalParams p;
    p.max_rate_urad_s   = kMaxRate;
    p.max_accel_urad_s2 = 0.0;
    p.tau_s             = 0.0;
    p.latency_s         = 0.0;
    p.encoder_lsb_urad  = 0.0;
    return p;
}
}  // namespace

TEST_CASE("CP 1.6: a huge command produces motion at exactly the rate limit") {
    GimbalAxis ax;
    ax.reset(ideal_axis(), 0.0);

    // Command a thousand times the limit.
    for (int i = 0; i < 300; ++i) ax.step(kMaxRate * 1000.0, kDt);

    CHECK(ax.rate() == doctest::Approx(kMaxRate).epsilon(1e-12));
    CHECK(ax.saturation_frac() == doctest::Approx(1.0));

    // One second at the ceiling moves the ceiling's worth of angle, LESS half a
    // tick — because the axis started from rest and spent its first step
    // accelerating. That deficit is 0.5 * rate * dt = 145 urad, and it is
    // correct rather than an error: an axis at rest cannot instantaneously be
    // somewhere a moving axis would have reached. Midpoint integration reports
    // it honestly; forward Euler would hide it by over-travelling instead.
    const double startup_deficit = 0.5 * kMaxRate * kDt;
    CHECK(ax.true_position()
          == doctest::Approx(kMaxRate - startup_deficit).epsilon(1e-9));
}

TEST_CASE("the rate limit is symmetric") {
    GimbalAxis ax;
    ax.reset(ideal_axis(), 0.0);
    for (int i = 0; i < 300; ++i) ax.step(-kMaxRate * 1000.0, kDt);
    CHECK(ax.rate() == doctest::Approx(-kMaxRate).epsilon(1e-12));
    const double startup_deficit = 0.5 * kMaxRate * kDt;
    CHECK(ax.true_position()
          == doctest::Approx(-(kMaxRate - startup_deficit)).epsilon(1e-9));
}

TEST_CASE("26.7 px/frame is the camera's authority at 5 deg/s and 30 Hz") {
    // Design §1.4's central number, checked against the plant rather than
    // against arithmetic. This is the figure that makes §1.3's argument: the
    // disturbance reaches 40 px/frame, so it can exceed what the motor can do.
    //
    // "Authority" means the SUSTAINED travel per frame, so the axis is brought
    // up to speed first. Measuring from rest would include the one-off startup
    // half-tick and understate the figure at 25.3 px.
    GimbalAxis ax;
    ax.reset(ideal_axis(), 0.0);
    for (int i = 0; i < 100; ++i) ax.step(kMaxRate * 100.0, kDt);   // reach the ceiling

    const double ifov   = deg_to_urad(4.0) / 640.0;
    const double before = ax.true_position();
    for (int i = 0; i < 10; ++i) ax.step(kMaxRate * 100.0, kDt);    // one 30 Hz frame
    const double px_moved = (ax.true_position() - before) / ifov;

    CHECK(px_moved == doctest::Approx(26.67).epsilon(0.001));

    // And the corollary that makes the project hard: spec row 23's 20 px/frame
    // of jitter alone is 75% of that, and jitter plus platform motion can reach
    // 40 px/frame, which exceeds it outright (design §1.3).
    CHECK(20.0 / px_moved == doctest::Approx(0.75).epsilon(0.01));
    CHECK(40.0 > px_moved);
}

TEST_CASE("a command below the limit is followed exactly") {
    GimbalAxis ax;
    ax.reset(ideal_axis(), 0.0);
    const double cmd = kMaxRate * 0.25;
    for (int i = 0; i < 300; ++i) ax.step(cmd, kDt);
    CHECK(ax.rate() == doctest::Approx(cmd).epsilon(1e-12));
    CHECK(ax.saturation_frac() == doctest::Approx(0.0));
    // Same startup half-tick as above.
    CHECK(ax.true_position() == doctest::Approx(cmd - 0.5 * cmd * kDt).epsilon(1e-9));
}

TEST_CASE("the startup half-tick is a one-off, not a per-step drift") {
    // The deficit above must not accumulate. If it did, a two-minute run at
    // 300 Hz would lose 36000 half-ticks and the mount would be pointing
    // somewhere else entirely.
    GimbalAxis ax;
    ax.reset(ideal_axis(), 0.0);
    const double cmd = kMaxRate * 0.5;

    for (int i = 0; i < 300; ++i) ax.step(cmd, kDt);
    const double after_1s = ax.true_position();
    for (int i = 0; i < 300; ++i) ax.step(cmd, kDt);
    const double after_2s = ax.true_position();

    // The second second travels the full distance: no residual deficit.
    CHECK(after_2s - after_1s == doctest::Approx(cmd).epsilon(1e-12));
    CHECK(after_1s == doctest::Approx(cmd - 0.5 * cmd * kDt).epsilon(1e-9));
}

TEST_CASE("the acceleration limit produces a ramp, not a step") {
    GimbalParams p = ideal_axis();
    p.max_accel_urad_s2 = deg_to_urad(50.0);   // the scenario default

    GimbalAxis ax;
    ax.reset(p, 0.0);

    // Time to reach the rate ceiling from rest: max_rate / max_accel.
    const double expected_s = p.max_rate_urad_s / p.max_accel_urad_s2;
    int ticks = 0;
    while (std::fabs(ax.rate()) < p.max_rate_urad_s * 0.999 && ticks < 10000) {
        ax.step(p.max_rate_urad_s * 10.0, kDt);
        ++ticks;
    }
    const double took_s = ticks * kDt;
    INFO("expected " << expected_s << " s, took " << took_s << " s");
    CHECK(took_s == doctest::Approx(expected_s).epsilon(0.02));
    CHECK(ax.accel_saturation_frac() > 0.9);
}

TEST_CASE("the transport delay defers motion by exactly latency_s") {
    GimbalParams p = ideal_axis();
    p.latency_s = 0.010;                      // the scenario default

    GimbalAxis ax;
    ax.reset(p, 0.0);

    const int delay_ticks = static_cast<int>(std::lround(p.latency_s / kDt));   // 3
    CHECK(delay_ticks == 3);

    // For the first `delay_ticks` steps the motor is still seeing the primed
    // zero, so nothing moves.
    for (int i = 0; i < delay_ticks; ++i) {
        ax.step(kMaxRate * 0.5, kDt);
        INFO("tick " << i);
        CHECK(ax.rate() == doctest::Approx(0.0));
    }
    // On the next step the command finally arrives.
    ax.step(kMaxRate * 0.5, kDt);
    CHECK(ax.rate() == doctest::Approx(kMaxRate * 0.5).epsilon(1e-9));
}

TEST_CASE("the first-order lag approaches the commanded rate exponentially") {
    GimbalParams p = ideal_axis();
    p.tau_s = 0.020;

    GimbalAxis ax;
    ax.reset(p, 0.0);

    const double cmd = kMaxRate * 0.5;
    // After one time constant a first-order system has covered 1 - 1/e = 63.2%.
    const int ticks = static_cast<int>(std::lround(p.tau_s / kDt));
    for (int i = 0; i < ticks; ++i) ax.step(cmd, kDt);
    CHECK(ax.rate() / cmd == doctest::Approx(0.632).epsilon(0.05));

    // And it converges.
    for (int i = 0; i < 1000; ++i) ax.step(cmd, kDt);
    CHECK(ax.rate() == doctest::Approx(cmd).epsilon(1e-6));
}

TEST_CASE("position() is quantised and true_position() is not") {
    // Design §10.3's deliberate split. The controller reads an encoder; the
    // metrics read the truth. Confusing them either hands the controller
    // information it cannot have, or reports an error the system did not make.
    GimbalParams p = ideal_axis();
    p.encoder_lsb_urad = 20.0;

    GimbalAxis ax;
    ax.reset(p, 0.0);
    ax.force_position(1234.567);

    CHECK(ax.true_position() == doctest::Approx(1234.567));
    CHECK(ax.position() == doctest::Approx(1240.0));       // nearest 20 urad
    CHECK(ax.position() != ax.true_position());

    // The quantisation error never exceeds half an LSB.
    for (double v : {0.0, 9.9, 10.1, -10.1, 1e5, -1e5, 33333.3}) {
        ax.force_position(v);
        INFO("v = " << v);
        CHECK(std::fabs(ax.position() - ax.true_position()) <= 10.0 + 1e-9);
    }
}

TEST_CASE("saturation is applied inside the integration step") {
    // Design §10.3: "Saturation must be applied INSIDE the step, not after."
    //
    // The failure mode this guards against: if the internal rate state were
    // allowed to run past the limit while only the OUTPUT was clamped, then
    // reversing the command would take a long time to have any effect, because
    // the state would have to unwind first. That is integrator windup living in
    // the plant, where no controller-side anti-windup can reach it.
    GimbalAxis ax;
    ax.reset(ideal_axis(), 0.0);

    // Drive hard in one direction for a long time.
    for (int i = 0; i < 3000; ++i) ax.step(kMaxRate * 1000.0, kDt);
    CHECK(ax.rate() == doctest::Approx(kMaxRate).epsilon(1e-12));

    // Reverse. With saturation inside the step, this takes effect immediately.
    ax.step(-kMaxRate, kDt);
    CHECK(ax.rate() == doctest::Approx(-kMaxRate).epsilon(1e-9));
}

TEST_CASE("midpoint integration is exact for a constant-acceleration ramp") {
    // Under constant acceleration, position advances by v0*t + 0.5*a*t^2.
    // Euler integration would systematically lag by 0.5*a*dt per step, and over
    // 36000 ticks of a two-minute run that becomes a real pointing offset.
    GimbalParams p = ideal_axis();
    p.max_accel_urad_s2 = 1000.0;
    p.max_rate_urad_s   = 1e9;         // keep the rate limit out of the way

    GimbalAxis ax;
    ax.reset(p, 0.0);

    const int n = 300;                 // one second
    for (int i = 0; i < n; ++i) ax.step(1e9, kDt);

    const double t = n * kDt;
    const double expected = 0.5 * p.max_accel_urad_s2 * t * t;
    CHECK(ax.true_position() == doctest::Approx(expected).epsilon(1e-9));
}

TEST_CASE("the two axes are independent") {
    // Spec rows 13 and 14 are separate rows because pan and tilt are separate
    // motors that may have different limits.
    GimbalParams pan  = ideal_axis();
    GimbalParams tilt = ideal_axis();
    tilt.max_rate_urad_s = kMaxRate * 2.0;   // a 10 deg/s tilt axis

    Gimbal g;
    g.reset(pan, tilt, Angle2{});
    for (int i = 0; i < 300; ++i) g.step(Rate2{1e9, 1e9}, kDt);

    CHECK(g.rate().x == doctest::Approx(kMaxRate).epsilon(1e-9));
    CHECK(g.rate().y == doctest::Approx(kMaxRate * 2.0).epsilon(1e-9));
}

TEST_CASE("reset restores a clean state") {
    GimbalAxis ax;
    ax.reset(ideal_axis(), 0.0);
    for (int i = 0; i < 500; ++i) ax.step(kMaxRate, kDt);
    CHECK(ax.true_position() > 0.0);

    ax.reset(ideal_axis(), 1000.0);
    CHECK(ax.true_position() == doctest::Approx(1000.0));
    CHECK(ax.rate() == doctest::Approx(0.0));
    CHECK(ax.saturation_frac() == doctest::Approx(0.0));
    // The delay line must be re-primed, not left holding old commands.
    ax.step(0.0, kDt);
    CHECK(ax.rate() == doctest::Approx(0.0));
}
