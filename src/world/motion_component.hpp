// world/motion_component.hpp — the motion algebra.
//
// Design §7.2 and decision 5. Spec row 12 requires at least four motions
// (line, circular, figure-8, random) and optionally spiral, sinusoidal and
// "user-defined"; spec row 25 requires the same menu for platform motion. The
// design doc's answer to all of it is one idea:
//
//     position(t) = Σ component_i(t)
//     velocity(t) = Σ component_i'(t)
//
// Motion is a SUM of components, each with a closed-form position AND velocity.
// That single decision covers the whole of rows 12 and 25:
//
//     straight line   linear
//     circular        circular
//     figure of 8     lissajous with freq_ratio = 2
//     random          ou_noise
//     spiral          spiral
//     sinusoidal      linear + sinusoid, stacked
//     user-defined    waypoints, or any stack at all
//
// and it is why design decision 5 rejects embedding a scripting language: the
// declarative form validates, reproduces, and — crucially — differentiates.
//
// ---------------------------------------------------------------------------
// WHY ANALYTIC VELOCITY IS NOT A LUXURY
// ---------------------------------------------------------------------------
// Every component must provide its exact derivative, not a finite difference.
// Three things depend on it:
//
//   1. Velocity feedforward (§10.4) is "the highest-value ten lines in the
//      project", and it is fed the TRUE target rate when evaluating an upper
//      bound on tracking performance. A differenced velocity would be noisy and
//      one step stale, which would understate what feedforward can do.
//   2. FrameTruth reports world_rate, and CP 6.2's acceptance test compares the
//      Kalman filter's estimated speed against it to within 2%. That comparison
//      is meaningless if the reference is itself an approximation.
//   3. Motion blur (§9.2) interpolates emitter positions across the exposure.
//
// CP 3.5 makes this a checkpoint of its own: "each component's analytic velocity
// matches a central finite difference to 1e-6 at 100 sample times."

#pragma once

#include "core/rng.hpp"
#include "core/units.hpp"

#include <cmath>
#include <memory>
#include <vector>

namespace sat {

/// Position and velocity together — they are always wanted together, and
/// computing them in one call lets a component share intermediate terms
/// (a single sin/cos pair serves both for the periodic components).
struct MotionState {
    double x  = 0.0;
    double y  = 0.0;
    double vx = 0.0;
    double vy = 0.0;

    MotionState& operator+=(const MotionState& o) noexcept {
        x += o.x; y += o.y; vx += o.vx; vy += o.vy;
        return *this;
    }
};

// ---------------------------------------------------------------------------
// IMotionComponent
//
// Virtual dispatch is acceptable here: components are evaluated once per
// emitter per truth tick, so a few hundred indirect calls per tick against a
// 0.02 ms budget (design §15). The alternative — a variant and a visitor —
// would save nanoseconds and cost the ability to stack arbitrary components
// from a config file, which is the entire point.
// ---------------------------------------------------------------------------
class IMotionComponent {
public:
    virtual ~IMotionComponent() = default;

    /// Position and velocity at absolute time t. Must be PURE for deterministic
    /// components: calling it twice with the same t must give the same answer,
    /// and calling it out of order must be safe. Motion blur relies on both,
    /// since it evaluates at times either side of the frame timestamp.
    [[nodiscard]] virtual MotionState eval(double t_s) const = 0;

    /// True for components carrying integrator state that must be advanced in
    /// order (currently only the Ornstein-Uhlenbeck process).
    [[nodiscard]] virtual bool is_stochastic() const { return false; }

    /// Advance internal state by dt. Called once per truth tick, in order, and
    /// only for stochastic components.
    virtual void advance(double dt, Pcg32& rng) { (void)dt; (void)rng; }

    /// Reset to the initial state, so a sweep can rerun in one process.
    virtual void reset() {}

    /// The TOML `kind` string. Used in error messages and in run.json.
    [[nodiscard]] virtual const char* kind() const = 0;
};

// ===========================================================================
// The nine components of design §7.2.
//
// Each is a handful of lines, and each carries its derivative next to its
// position so the two cannot drift apart during a later edit.
// ===========================================================================

/// `constant` — a fixed offset. Position p0, velocity zero.
/// Mostly used as the base of a stack: "start here, then add motion".
class ConstantMotion final : public IMotionComponent {
public:
    ConstantMotion(double x, double y) : x_(x), y_(y) {}
    [[nodiscard]] MotionState eval(double) const override { return {x_, y_, 0.0, 0.0}; }
    [[nodiscard]] const char* kind() const override { return "constant"; }
private:
    double x_, y_;
};

/// `linear` — spec row 12's mandatory straight line. p = v*t, p' = v.
class LinearMotion final : public IMotionComponent {
public:
    LinearMotion(double vx, double vy) : vx_(vx), vy_(vy) {}
    [[nodiscard]] MotionState eval(double t) const override {
        return {vx_ * t, vy_ * t, vx_, vy_};
    }
    [[nodiscard]] const char* kind() const override { return "linear"; }
private:
    double vx_, vy_;
};

/// `accel` — constant acceleration. p = ½a t², p' = a t.
/// Stacked with `linear` this gives the classic ballistic profile, and it is
/// what exercises the IMM's constant-acceleration model (§10.2).
class AccelMotion final : public IMotionComponent {
public:
    AccelMotion(double ax, double ay) : ax_(ax), ay_(ay) {}
    [[nodiscard]] MotionState eval(double t) const override {
        return {0.5 * ax_ * t * t, 0.5 * ay_ * t * t, ax_ * t, ay_ * t};
    }
    [[nodiscard]] const char* kind() const override { return "accel"; }
private:
    double ax_, ay_;
};

/// `sinusoid` — oscillation along one axis.
/// p = A sin(ωt + φ), p' = Aω cos(ωt + φ).
/// Spec row 12's optional "sinusoidal" is this stacked on a `linear`.
class SinusoidMotion final : public IMotionComponent {
public:
    /// `axis` 0 = x, 1 = y.
    SinusoidMotion(int axis, double amplitude_px, double period_s, double phase_deg)
        : axis_(axis), a_(amplitude_px),
          w_(period_s > 0.0 ? 2.0 * kPi / period_s : 0.0),
          phi_(phase_deg * kPi / 180.0) {}

    [[nodiscard]] MotionState eval(double t) const override {
        const double arg = w_ * t + phi_;
        const double p = a_ * std::sin(arg);
        const double v = a_ * w_ * std::cos(arg);
        return axis_ == 0 ? MotionState{p, 0.0, v, 0.0}
                          : MotionState{0.0, p, 0.0, v};
    }
    [[nodiscard]] const char* kind() const override { return "sinusoid"; }
private:
    int    axis_;
    double a_, w_, phi_;
};

/// `circular` — spec row 12's mandatory circle.
/// p = r(cos ωt, sin ωt), p' = rω(−sin ωt, cos ωt).
class CircularMotion final : public IMotionComponent {
public:
    CircularMotion(double radius_px, double period_s, double phase_deg)
        : r_(radius_px),
          w_(period_s > 0.0 ? 2.0 * kPi / period_s : 0.0),
          phi_(phase_deg * kPi / 180.0) {}

    [[nodiscard]] MotionState eval(double t) const override {
        const double a = w_ * t + phi_;
        const double c = std::cos(a), s = std::sin(a);
        return {r_ * c, r_ * s, -r_ * w_ * s, r_ * w_ * c};
    }
    [[nodiscard]] const char* kind() const override { return "circular"; }
private:
    double r_, w_, phi_;
};

/// `lissajous` — spec row 12's mandatory FIGURE OF EIGHT, at freq_ratio = 2.
///
/// p = (A sin(ωt + φ), B sin(n ωt)), p' = (Aω cos(ωt + φ), Bnω cos(n ωt)).
///
/// Design §10.2 singles this one out: at the crossing of the figure-8 the
/// acceleration reverses sign, and a single-model filter overshoots every lap.
/// Since it is a MANDATORY spec motion, that makes the IMM a before/after demo
/// on graded functionality rather than a refinement.
class LissajousMotion final : public IMotionComponent {
public:
    LissajousMotion(double ax_px, double ay_px, double freq_ratio,
                    double period_s, double phase_deg)
        : ax_(ax_px), ay_(ay_px), n_(freq_ratio),
          w_(period_s > 0.0 ? 2.0 * kPi / period_s : 0.0),
          phi_(phase_deg * kPi / 180.0) {}

    [[nodiscard]] MotionState eval(double t) const override {
        const double a1 = w_ * t + phi_;
        const double a2 = n_ * w_ * t;
        return {ax_ * std::sin(a1),
                ay_ * std::sin(a2),
                ax_ * w_ * std::cos(a1),
                ay_ * n_ * w_ * std::cos(a2)};
    }
    [[nodiscard]] const char* kind() const override { return "lissajous"; }
private:
    double ax_, ay_, n_, w_, phi_;
};

/// `spiral` — spec row 12's optional spiral, and row 25's optional platform
/// spiral.
///
/// p = (r₀ + kt)(cos ωt, sin ωt). The derivative needs the product rule, which
/// design §7.2 notes but does not write out:
///     p'ₓ = k cos ωt − (r₀ + kt) ω sin ωt
///     p'_y = k sin ωt + (r₀ + kt) ω cos ωt
class SpiralMotion final : public IMotionComponent {
public:
    SpiralMotion(double r0_px, double growth_px_s, double period_s, double phase_deg)
        : r0_(r0_px), k_(growth_px_s),
          w_(period_s > 0.0 ? 2.0 * kPi / period_s : 0.0),
          phi_(phase_deg * kPi / 180.0) {}

    [[nodiscard]] MotionState eval(double t) const override {
        const double a = w_ * t + phi_;
        const double c = std::cos(a), s = std::sin(a);
        const double r = r0_ + k_ * t;
        return {r * c,
                r * s,
                k_ * c - r * w_ * s,     // product rule
                k_ * s + r * w_ * c};
    }
    [[nodiscard]] const char* kind() const override { return "spiral"; }
private:
    double r0_, k_, w_, phi_;
};

/// `ou_noise` — spec row 12's mandatory RANDOM motion, as an
/// Ornstein-Uhlenbeck process.
///
/// Why OU rather than a random walk or white noise? A random walk has unbounded
/// variance, so the target eventually wanders off the screen and the scenario
/// stops testing tracking. White noise is not differentiable, so it has no
/// velocity to feed forward and no physical meaning for a mass. OU is the
/// mean-reverting middle: bounded, continuous, with a real velocity, and
/// parameterised by two things a person can reason about — how fast it wanders
/// (sigma) and how long it persists in a direction (tau).
///
/// THE DISCRETISATION MUST BE EXACT, NOT EULER. Design §7.2 is explicit: the
/// component must behave identically at any truth rate. The exact update is
///     v ← a·v + σ√(1−a²)·N(0,1),   a = exp(−dt/τ)
/// which preserves the stationary variance for ANY dt. Euler (v ← v(1−dt/τ) +
/// σ√dt·N) does not: its variance depends on the step size, so the same
/// scenario at 300 Hz and 600 Hz would produce visibly different motion.
class OuNoiseMotion final : public IMotionComponent {
public:
    OuNoiseMotion(double sigma_px_s, double tau_s)
        : sigma_(sigma_px_s), tau_(tau_s > 0.0 ? tau_s : 1e-6) {}

    [[nodiscard]] MotionState eval(double) const override {
        // Ignores t: this component's state is integrated, not evaluated. It is
        // safe to call at any time within a tick — which is what motion blur
        // does — because the state is only changed by advance().
        return {x_, y_, vx_, vy_};
    }
    [[nodiscard]] bool is_stochastic() const override { return true; }

    void advance(double dt, Pcg32& rng) override {
        const double a = std::exp(-dt / tau_);
        const double s = sigma_ * std::sqrt(1.0 - a * a);
        vx_ = a * vx_ + s * rng.next_normal();
        vy_ = a * vy_ + s * rng.next_normal();
        x_ += vx_ * dt;
        y_ += vy_ * dt;
    }

    void reset() override { x_ = y_ = vx_ = vy_ = 0.0; }
    [[nodiscard]] const char* kind() const override { return "ou_noise"; }

private:
    double sigma_, tau_;
    double x_ = 0.0, y_ = 0.0, vx_ = 0.0, vy_ = 0.0;
};

/// `waypoints` — spec row 12's "user-defined", as a Catmull-Rom spline through
/// given points.
///
/// Catmull-Rom rather than a plain cubic or a Bezier because it INTERPOLATES its
/// control points: the target actually passes through the coordinates the user
/// typed, which is what someone writing a waypoint list means. It is also C¹
/// continuous, so the velocity has no jumps — which matters here, since a
/// velocity discontinuity would be a step input to the feedforward path.
class WaypointMotion final : public IMotionComponent {
public:
    struct Waypoint { double t, x, y; };

    explicit WaypointMotion(std::vector<Waypoint> pts) : pts_(std::move(pts)) {}

    [[nodiscard]] MotionState eval(double t) const override;
    [[nodiscard]] const char* kind() const override { return "waypoints"; }
    [[nodiscard]] size_t size() const noexcept { return pts_.size(); }

private:
    std::vector<Waypoint> pts_;
};

// ---------------------------------------------------------------------------
// CompositeMotion — the sum.
// ---------------------------------------------------------------------------
class CompositeMotion {
public:
    void add(std::unique_ptr<IMotionComponent> c) { parts_.push_back(std::move(c)); }

    [[nodiscard]] MotionState eval(double t_s) const {
        MotionState s{};
        for (const auto& p : parts_) s += p->eval(t_s);
        return s;
    }

    /// Advance the stochastic components. Deterministic ones are untouched,
    /// because they are pure functions of t.
    void advance(double dt, Pcg32& rng) {
        for (auto& p : parts_) {
            if (p->is_stochastic()) p->advance(dt, rng);
        }
    }

    void reset() { for (auto& p : parts_) p->reset(); }

    [[nodiscard]] size_t size() const noexcept { return parts_.size(); }
    [[nodiscard]] bool   empty() const noexcept { return parts_.empty(); }
    [[nodiscard]] const IMotionComponent& at(size_t i) const { return *parts_[i]; }

    /// True if any component carries integrator state, so the caller knows
    /// whether advance() must be called in order.
    [[nodiscard]] bool has_stochastic() const noexcept {
        for (const auto& p : parts_) if (p->is_stochastic()) return true;
        return false;
    }

private:
    std::vector<std::unique_ptr<IMotionComponent>> parts_;
};

}  // namespace sat
