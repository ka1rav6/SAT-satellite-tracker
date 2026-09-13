// tests/motion/test_motion.cpp — CP 3.3, 3.4 and especially CP 3.5.
//
// CP 3.5: "each component's analytic velocity matches a central finite
//          difference to 1e-6 at 100 sample times."
//
// That is the load-bearing test in this file. Velocity feedforward (§10.4),
// FrameTruth's world_rate, and CP 6.2's "estimated speed matches analytic
// velocity within 2%" all take the analytic derivative as ground truth. If a
// component's derivative disagrees with its own position function, every one of
// those becomes a comparison against a wrong number — and the failure would be
// invisible, because the filter would faithfully track a velocity that was
// never right.

#include <doctest/doctest.h>

#include "world/motion_component.hpp"

#include <algorithm>
#include <string>
#include <cmath>
#include <initializer_list>
#include <memory>
#include <vector>

using namespace sat;

namespace {

// ---------------------------------------------------------------------------
// The CP 3.5 check itself, factored out so every component gets exactly the
// same scrutiny.
//
// Central difference, because it is second-order accurate: its error is
// O(h²·f''') rather than the O(h·f'') of a forward difference. With h = 1e-5
// and the smooth functions here that leaves ~1e-10, comfortably under the 1e-6
// the checkpoint asks for — so a failure means a genuinely wrong derivative,
// not numerical noise in the test.
// ---------------------------------------------------------------------------
void check_derivative(const IMotionComponent& c, const char* name,
                      double t0 = 0.1, double t1 = 20.0, int samples = 100) {
    constexpr double h = 1e-5;
    double worst = 0.0;
    for (int i = 0; i < samples; ++i) {
        const double t = t0 + (t1 - t0) * (static_cast<double>(i) / (samples - 1));
        const MotionState s = c.eval(t);
        const MotionState a = c.eval(t - h);
        const MotionState b = c.eval(t + h);

        const double fd_vx = (b.x - a.x) / (2.0 * h);
        const double fd_vy = (b.y - a.y) / (2.0 * h);

        INFO(std::string(name) << " at t=" << t
             << "  analytic (" << s.vx << ", " << s.vy << ")"
             << "  finite-diff (" << fd_vx << ", " << fd_vy << ")");
        REQUIRE(std::abs(s.vx - fd_vx) < 1e-6);
        REQUIRE(std::abs(s.vy - fd_vy) < 1e-6);
        worst = std::max({worst, std::abs(s.vx - fd_vx), std::abs(s.vy - fd_vy)});
    }
    MESSAGE(std::string(name) << ": worst |analytic - finite difference| = " << worst);
}

}  // namespace

// ===========================================================================
// CP 3.5 — every deterministic component's derivative, against a finite
// difference. `ou_noise` is excluded because it is an integrated process rather
// than a function of t; it gets its own statistical tests below.
// ===========================================================================

TEST_CASE("CP 3.5: constant") {
    check_derivative(ConstantMotion(120.0, -40.0), "constant");
}

TEST_CASE("CP 3.5: linear — spec row 12's mandatory straight line") {
    check_derivative(LinearMotion(15.0, -8.0), "linear");
}

TEST_CASE("CP 3.5: accel") {
    check_derivative(AccelMotion(3.5, -1.25), "accel");
}

TEST_CASE("CP 3.5: sinusoid, both axes") {
    check_derivative(SinusoidMotion(0, 200.0, 7.0, 30.0), "sinusoid-x");
    check_derivative(SinusoidMotion(1, 150.0, 4.5, -60.0), "sinusoid-y");
}

TEST_CASE("CP 3.5: circular — spec row 12's mandatory circle") {
    check_derivative(CircularMotion(350.0, 12.0, 45.0), "circular");
}

TEST_CASE("CP 3.5: lissajous — spec row 12's mandatory figure of eight") {
    check_derivative(LissajousMotion(700.0, 350.0, 2.0, 24.0, 0.0), "lissajous");
}

TEST_CASE("CP 3.5: spiral — the product rule is the easy one to get wrong") {
    // p = (r0 + kt)(cos wt, sin wt) needs the product rule, and design §7.2
    // says only "product rule" without writing it out. This is the test that
    // would catch dropping either term.
    check_derivative(SpiralMotion(50.0, 12.0, 9.0, 0.0), "spiral");
    check_derivative(SpiralMotion(0.0, 25.0, 6.0, 90.0), "spiral-from-zero");
}

TEST_CASE("CP 3.5: waypoints — the chain rule factor is the easy one to drop") {
    // Catmull-Rom's derivative is with respect to the LOCAL parameter u, and
    // converting to px/s needs du/dt = 1/(t2 - t1). Omitting that factor makes
    // the velocity wrong by the segment duration — a silent error that only a
    // finite-difference comparison catches.
    std::vector<WaypointMotion::Waypoint> pts{
        {0.0,   100.0,  100.0},
        {4.0,   400.0,  250.0},
        {9.0,   250.0,  700.0},
        {14.0,  800.0,  500.0},
        {20.0, 1100.0,  900.0},
    };
    WaypointMotion w(pts);
    // Stay strictly inside the spline's range: the ends clamp to zero velocity
    // by design, which is a discontinuity a central difference would straddle.
    check_derivative(w, "waypoints", 0.2, 19.8);
}

// ===========================================================================
// Component behaviour — that each one is the shape it claims to be.
// ===========================================================================

TEST_CASE("linear travels at exactly its configured velocity") {
    LinearMotion m(15.0, -8.0);
    const MotionState s = m.eval(10.0);
    CHECK(s.x == doctest::Approx(150.0));
    CHECK(s.y == doctest::Approx(-80.0));
    CHECK(s.vx == doctest::Approx(15.0));
    CHECK(s.vy == doctest::Approx(-8.0));
}

TEST_CASE("circular stays on its circle and moves tangentially") {
    const double r = 350.0, period = 12.0;
    CircularMotion m(r, period, 0.0);
    for (int i = 0; i <= 24; ++i) {
        const double t = period * i / 24.0;
        const MotionState s = m.eval(t);
        INFO("t = " << t);
        // On the circle.
        CHECK(std::sqrt(s.x * s.x + s.y * s.y) == doctest::Approx(r).epsilon(1e-12));
        // Speed is constant: r * omega.
        const double speed = std::sqrt(s.vx * s.vx + s.vy * s.vy);
        CHECK(speed == doctest::Approx(r * 2.0 * kPi / period).epsilon(1e-12));
        // Velocity is perpendicular to the radius — the defining property of
        // circular motion, and zero dot product is a strong check on the signs.
        CHECK(std::abs(s.x * s.vx + s.y * s.vy) < 1e-6 * r * speed);
    }
    // It closes: one period returns to the start.
    const MotionState a = m.eval(0.0);
    const MotionState b = m.eval(period);
    CHECK(b.x == doctest::Approx(a.x).epsilon(1e-9));
    CHECK(b.y == doctest::Approx(a.y).epsilon(1e-9));
}

TEST_CASE("lissajous at freq_ratio 2 really is a figure of eight") {
    // The defining feature: the path crosses itself exactly once per period, at
    // the centre. That crossing is where design §10.2 says a single-model
    // filter overshoots every lap, which is the whole argument for the IMM.
    const double period = 24.0;
    LissajousMotion m(700.0, 350.0, 2.0, period, 0.0);

    // At t = 0 and t = period/2 the curve is at the origin travelling in
    // opposite x directions — that is the crossing.
    const MotionState a = m.eval(0.0);
    const MotionState b = m.eval(period * 0.5);
    CHECK(std::abs(a.x) < 1e-9);
    CHECK(std::abs(a.y) < 1e-9);
    CHECK(std::abs(b.x) < 1e-9);
    CHECK(std::abs(b.y) < 1e-9);
    CHECK(a.vx * b.vx < 0.0);        // opposite directions through the crossing

    // y completes two oscillations for every one of x — that ratio is what
    // makes freq_ratio = 2 a figure of eight rather than an ellipse.
    //
    // Counted over TEN periods rather than one. Over a single period the
    // endpoints dominate: sin(wt) has zeros at 0, T/2 and T while sin(2wt) has
    // zeros at 0, T/4, T/2, 3T/4 and T, so a naive count gives 1 and 3 rather
    // than the asymptotic 2:1. Over many periods the endpoint effect washes out
    // and the ratio is the property actually being claimed.
    const int kPeriods = 10, kSamples = 20000;
    int x_zeros = 0, y_zeros = 0;
    double px = m.eval(0.0).x, py = m.eval(0.0).y;
    for (int i = 1; i <= kSamples; ++i) {
        const double t = period * kPeriods * i / kSamples;
        const MotionState s = m.eval(t);
        if ((px < 0.0) != (s.x < 0.0)) ++x_zeros;
        if ((py < 0.0) != (s.y < 0.0)) ++y_zeros;
        px = s.x; py = s.y;
    }
    // sin(wt) vanishes at t = k*T/2 and sin(2wt) at t = k*T/4. The loop samples
    // the half-open range (0, 10T], and the crossing exactly AT 10T is shared by
    // both and registers for neither — sin(20*pi) evaluates to a value whose
    // sign matches its neighbour. So the exact counts over the interior are
    // 2N-1 and 4N-1, not 2N and 4N.
    //
    // Asserting the exact numbers rather than an approximate ratio means a
    // regression in the phase or the frequency ratio cannot pass by landing
    // close.
    INFO("x zero-crossings = " << x_zeros << ", y = " << y_zeros);
    CHECK(x_zeros == 2 * kPeriods - 1);
    CHECK(y_zeros == 4 * kPeriods - 1);
    CHECK(y_zeros == 2 * x_zeros + 1);
}

TEST_CASE("spiral grows linearly in radius") {
    SpiralMotion m(50.0, 12.0, 9.0, 0.0);
    for (double t : {0.0, 5.0, 10.0, 20.0}) {
        const MotionState s = m.eval(t);
        INFO("t = " << t);
        CHECK(std::sqrt(s.x * s.x + s.y * s.y)
              == doctest::Approx(50.0 + 12.0 * t).epsilon(1e-12));
    }
}

TEST_CASE("waypoints pass exactly through their control points") {
    // The reason for choosing Catmull-Rom: it INTERPOLATES, so the target
    // actually reaches the coordinates someone typed. A Bezier would only
    // approach them.
    std::vector<WaypointMotion::Waypoint> pts{
        {0.0,  100.0, 100.0},
        {4.0,  400.0, 250.0},
        {9.0,  250.0, 700.0},
        {14.0, 800.0, 500.0},
    };
    WaypointMotion w(pts);
    for (const auto& p : pts) {
        const MotionState s = w.eval(p.t);
        INFO("waypoint at t = " << p.t);
        CHECK(s.x == doctest::Approx(p.x).epsilon(1e-9));
        CHECK(s.y == doctest::Approx(p.y).epsilon(1e-9));
    }
}

TEST_CASE("waypoints clamp outside their range rather than extrapolating") {
    // A cubic extrapolated past its range diverges fast, and a target that flew
    // off the canvas because the run outlasted its waypoint list would be a
    // baffling failure to debug.
    std::vector<WaypointMotion::Waypoint> pts{{1.0, 10.0, 20.0}, {5.0, 50.0, 60.0}};
    WaypointMotion w(pts);

    const MotionState before = w.eval(-100.0);
    CHECK(before.x == doctest::Approx(10.0));
    CHECK(before.vx == doctest::Approx(0.0));

    const MotionState after = w.eval(1000.0);
    CHECK(after.x == doctest::Approx(50.0));
    CHECK(after.vx == doctest::Approx(0.0));
}

TEST_CASE("degenerate waypoint lists do not fault") {
    // CP 14.1 fuzzes every parameter, and an empty or single-point list is the
    // first thing it will produce.
    CHECK(WaypointMotion({}).eval(5.0).x == doctest::Approx(0.0));
    CHECK(WaypointMotion({{0.0, 7.0, 8.0}}).eval(5.0).x == doctest::Approx(7.0));
    // Duplicate timestamps would divide by a zero span.
    WaypointMotion dup({{0.0, 1.0, 2.0}, {0.0, 3.0, 4.0}, {1.0, 5.0, 6.0}});
    CHECK(std::isfinite(dup.eval(0.5).x));
}

// ===========================================================================
// ou_noise — spec row 12's mandatory RANDOM motion.
// ===========================================================================

TEST_CASE("ou_noise is reproducible from the same stream") {
    Pcg32 a(1234), b(1234);
    OuNoiseMotion m1(50.0, 2.0), m2(50.0, 2.0);
    for (int i = 0; i < 1000; ++i) {
        m1.advance(1.0 / 300.0, a);
        m2.advance(1.0 / 300.0, b);
    }
    CHECK(m1.eval(0.0).x == m2.eval(0.0).x);
    CHECK(m1.eval(0.0).vx == m2.eval(0.0).vx);
}

TEST_CASE("ou_noise has the SAME statistics at any truth rate") {
    // This is what the exact discretisation buys, and design §7.2 calls for it
    // explicitly. The stationary velocity variance of an OU process is sigma^2,
    // and with the exact update
    //     v <- a*v + sigma*sqrt(1-a^2)*N,  a = exp(-dt/tau)
    // that holds for ANY dt. A Euler discretisation's variance depends on the
    // step size, so the same scenario at 300 Hz and 600 Hz would produce
    // visibly different motion — and a benchmark run at one rate would not
    // predict the other.
    const double sigma = 50.0, tau = 2.0;

    auto measure_velocity_sd = [&](double dt, int steps) {
        Pcg32 rng(777);
        OuNoiseMotion m(sigma, tau);
        // Burn in past the transient from v = 0.
        for (int i = 0; i < static_cast<int>(20.0 * tau / dt); ++i) m.advance(dt, rng);
        double sum = 0.0, sum_sq = 0.0;
        for (int i = 0; i < steps; ++i) {
            m.advance(dt, rng);
            const double v = m.eval(0.0).vx;
            sum += v; sum_sq += v * v;
        }
        const double mean = sum / steps;
        return std::sqrt(sum_sq / steps - mean * mean);
    };

    const double sd_300 = measure_velocity_sd(1.0 / 300.0, 400000);
    const double sd_600 = measure_velocity_sd(1.0 / 600.0, 400000);
    const double sd_50  = measure_velocity_sd(1.0 / 50.0,  400000);

    MESSAGE("OU velocity sd: 50 Hz " << sd_50 << ", 300 Hz " << sd_300
            << ", 600 Hz " << sd_600 << " (sigma = " << sigma << ")");

    CHECK(sd_300 == doctest::Approx(sigma).epsilon(0.05));
    CHECK(sd_600 == doctest::Approx(sigma).epsilon(0.05));
    CHECK(sd_50  == doctest::Approx(sigma).epsilon(0.05));
}

TEST_CASE("ou_noise reverts to the mean rather than wandering off") {
    // The reason for OU over a random walk: a walk has unbounded variance, so
    // the target eventually leaves the screen and the scenario stops testing
    // tracking.
    Pcg32 rng(42);
    OuNoiseMotion m(30.0, 1.0);
    double max_speed = 0.0;
    for (int i = 0; i < 200000; ++i) {
        m.advance(1.0 / 300.0, rng);
        const MotionState s = m.eval(0.0);
        max_speed = std::max(max_speed, std::sqrt(s.vx * s.vx + s.vy * s.vy));
    }
    // Over 200k samples a bounded process should not stray far past a few sigma.
    CHECK(max_speed < 30.0 * 8.0);
}

TEST_CASE("ou_noise reset returns it to the origin") {
    Pcg32 rng(5);
    OuNoiseMotion m(20.0, 1.0);
    for (int i = 0; i < 500; ++i) m.advance(0.01, rng);
    CHECK(m.eval(0.0).x != 0.0);
    m.reset();
    CHECK(m.eval(0.0).x == 0.0);
    CHECK(m.eval(0.0).vx == 0.0);
}

// ===========================================================================
// CompositeMotion — the sum that covers spec rows 12 and 25.
// ===========================================================================

TEST_CASE("CP 3.3: a composite sums positions and velocities") {
    CompositeMotion c;
    c.add(std::make_unique<ConstantMotion>(1000.0, 1000.0));
    c.add(std::make_unique<LinearMotion>(20.0, -5.0));

    const MotionState s = c.eval(10.0);
    CHECK(s.x == doctest::Approx(1200.0));
    CHECK(s.y == doctest::Approx(950.0));
    CHECK(s.vx == doctest::Approx(20.0));
    CHECK(s.vy == doctest::Approx(-5.0));
}

TEST_CASE("CP 3.4: linear + sinusoid stacked gives spec row 12's sinusoidal motion") {
    // Spec row 12's optional "sinusoidal" is not a component — it is a stack.
    // That is design decision 5's claim, tested.
    CompositeMotion c;
    c.add(std::make_unique<LinearMotion>(30.0, 0.0));
    c.add(std::make_unique<SinusoidMotion>(1, 200.0, 5.0, 0.0));

    // x advances steadily while y oscillates: a sine wave travelling right.
    CHECK(c.eval(10.0).x == doctest::Approx(300.0));
    CHECK(c.eval(0.0).y == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(c.eval(1.25).y == doctest::Approx(200.0).epsilon(1e-9));   // quarter period
    CHECK(c.eval(3.75).y == doctest::Approx(-200.0).epsilon(1e-9));  // three quarters

    // And the composite's velocity is still exact.
    constexpr double h = 1e-5;
    for (int i = 0; i < 50; ++i) {
        const double t = 0.1 + i * 0.2;
        const MotionState s = c.eval(t);
        const double fd = (c.eval(t + h).y - c.eval(t - h).y) / (2.0 * h);
        INFO("t = " << t);
        CHECK(std::abs(s.vy - fd) < 1e-6);
    }
}

TEST_CASE("CP 3.4: all four mandatory spec row 12 motions are expressible") {
    // The mapping design §7.2 tabulates, made executable.
    struct Case { const char* required; std::unique_ptr<IMotionComponent> comp; };
    std::vector<Case> cases;
    cases.push_back({"straight line", std::make_unique<LinearMotion>(20.0, 10.0)});
    cases.push_back({"circular",      std::make_unique<CircularMotion>(300.0, 10.0, 0.0)});
    cases.push_back({"figure of 8",   std::make_unique<LissajousMotion>(700.0, 350.0, 2.0, 24.0, 0.0)});
    cases.push_back({"random",        std::make_unique<OuNoiseMotion>(40.0, 2.0)});

    for (auto& c : cases) {
        INFO("spec row 12 requires: " << c.required);
        CHECK(c.comp != nullptr);
        // Each must produce a finite state; the random one needs advancing.
        Pcg32 rng(1);
        if (c.comp->is_stochastic()) {
            for (int i = 0; i < 100; ++i) c.comp->advance(1.0 / 300.0, rng);
        }
        const MotionState s = c.comp->eval(3.0);
        CHECK(std::isfinite(s.x));
        CHECK(std::isfinite(s.y));
        CHECK(std::isfinite(s.vx));
        CHECK(std::isfinite(s.vy));
    }
}

TEST_CASE("a composite advances only its stochastic parts") {
    CompositeMotion c;
    c.add(std::make_unique<LinearMotion>(10.0, 0.0));
    c.add(std::make_unique<OuNoiseMotion>(25.0, 1.0));
    CHECK(c.has_stochastic());

    Pcg32 rng(9);
    const double deterministic_before = c.eval(5.0).x;
    for (int i = 0; i < 300; ++i) c.advance(1.0 / 300.0, rng);
    // The linear part is a pure function of t and must be unchanged; the
    // difference in the composite comes entirely from the OU part.
    const double after = c.eval(5.0).x;
    CHECK(after != deterministic_before);

    CompositeMotion d;
    d.add(std::make_unique<LinearMotion>(10.0, 0.0));
    CHECK_FALSE(d.has_stochastic());
    const double before_d = d.eval(5.0).x;
    for (int i = 0; i < 300; ++i) d.advance(1.0 / 300.0, rng);
    CHECK(d.eval(5.0).x == before_d);    // untouched, exactly
}

TEST_CASE("an empty composite is a valid stationary motion") {
    CompositeMotion c;
    CHECK(c.empty());
    const MotionState s = c.eval(12.34);
    CHECK(s.x == 0.0);
    CHECK(s.vx == 0.0);
}

TEST_CASE("every component reports its TOML kind string") {
    // These appear in validation errors and in run.json's provenance block.
    CHECK(std::string(ConstantMotion(0,0).kind()) == "constant");
    CHECK(std::string(LinearMotion(0,0).kind()) == "linear");
    CHECK(std::string(AccelMotion(0,0).kind()) == "accel");
    CHECK(std::string(SinusoidMotion(0,1,1,0).kind()) == "sinusoid");
    CHECK(std::string(CircularMotion(1,1,0).kind()) == "circular");
    CHECK(std::string(LissajousMotion(1,1,2,1,0).kind()) == "lissajous");
    CHECK(std::string(SpiralMotion(1,1,1,0).kind()) == "spiral");
    CHECK(std::string(OuNoiseMotion(1,1).kind()) == "ou_noise");
    CHECK(std::string(WaypointMotion({}).kind()) == "waypoints");
}

TEST_CASE("zero or negative periods do not produce NaN") {
    // CP 14.1 again: the fuzzer will try period_s = 0.
    for (double period : {0.0, -5.0}) {
        INFO("period = " << period);
        CHECK(std::isfinite(CircularMotion(100.0, period, 0.0).eval(3.0).x));
        CHECK(std::isfinite(SinusoidMotion(0, 100.0, period, 0.0).eval(3.0).x));
        CHECK(std::isfinite(LissajousMotion(100.0, 50.0, 2.0, period, 0.0).eval(3.0).x));
        CHECK(std::isfinite(SpiralMotion(10.0, 1.0, period, 0.0).eval(3.0).x));
    }
    CHECK(std::isfinite(OuNoiseMotion(10.0, 0.0).eval(0.0).x));
}
