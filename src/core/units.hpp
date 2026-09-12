// core/units.hpp — the vocabulary types the whole program speaks.
//
// INV-5 (design §2): *all internal state is in microradians.* Pixels exist only
// at the sensor boundary (project/unproject, see core/frames.hpp) and in the
// exported logs. A tracker state stored in pixels is a bug, not a shortcut.
//
// Why microradians and not degrees or radians?
//   * The whole problem lives at the scale of the tracking budget: 10 px is
//     1.09 mrad, one pixel is 109.08 urad. Numbers land in the 1..1e5 range,
//     which is comfortable for a double and legible in a log.
//   * Integer-ish magnitudes make it obvious when a value is nonsense. A
//     boresight rate of "65448" reads as 3.75 deg/s to anyone who has the
//     conversion in their head; "0.065448" reads as nothing.
//
// Everything here is a trivially-copyable aggregate with constexpr arithmetic.
// There is no virtual dispatch, no allocation, and no hidden cost: these must
// be free to pass around inside a 0.85 ms frame budget (design §15).

#pragma once

#include <cmath>
#include <cstdint>

namespace sat {

// ---------------------------------------------------------------------------
// Scalar conversion constants.
//
// Written out longhand rather than pulled from <numbers> so that the values are
// visible at the point of use — these appear in the derived-constants table in
// design §1.4 and get logged at startup, so a reader can check them by eye.
// ---------------------------------------------------------------------------
inline constexpr double kPi          = 3.14159265358979323846;
inline constexpr double kSqrt2       = 1.41421356237309504880;
inline constexpr double kUradPerRad  = 1.0e6;
inline constexpr double kRadPerUrad  = 1.0e-6;
inline constexpr double kUradPerDeg  = kPi / 180.0 * kUradPerRad;   // 17453.2925...
inline constexpr double kDegPerUrad  = 1.0 / kUradPerDeg;

// Plain conversion helpers. constexpr so they fold away entirely in release.
[[nodiscard]] constexpr double deg_to_urad(double deg) noexcept { return deg * kUradPerDeg; }
[[nodiscard]] constexpr double urad_to_deg(double urad) noexcept { return urad * kDegPerUrad; }
[[nodiscard]] constexpr double rad_to_urad(double rad) noexcept { return rad * kUradPerRad; }
[[nodiscard]] constexpr double urad_to_rad(double urad) noexcept { return urad * kRadPerUrad; }

// ---------------------------------------------------------------------------
// Vec2 — the shared implementation behind every 2-D quantity below.
//
// Angle2, Pixel2 and Rate2 are *distinct types* rather than aliases for one
// vector type. That is deliberate: the single most likely bug in this project
// is adding a pixel offset to an angular state, and the compiler can catch it
// for free. The tag parameter makes them non-interchangeable while sharing all
// the arithmetic.
// ---------------------------------------------------------------------------
template <typename Tag>
struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    constexpr Vec2() noexcept = default;
    constexpr Vec2(double x_, double y_) noexcept : x(x_), y(y_) {}

    // --- component-wise arithmetic ---------------------------------------
    constexpr Vec2 operator+(const Vec2& o) const noexcept { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(const Vec2& o) const noexcept { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator-() const noexcept              { return {-x, -y}; }
    constexpr Vec2 operator*(double s) const noexcept      { return {x * s, y * s}; }
    constexpr Vec2 operator/(double s) const noexcept      { return {x / s, y / s}; }

    constexpr Vec2& operator+=(const Vec2& o) noexcept { x += o.x; y += o.y; return *this; }
    constexpr Vec2& operator-=(const Vec2& o) noexcept { x -= o.x; y -= o.y; return *this; }
    constexpr Vec2& operator*=(double s) noexcept      { x *= s;   y *= s;   return *this; }
    constexpr Vec2& operator/=(double s) noexcept      { x /= s;   y /= s;   return *this; }

    // Exact equality. Used by tests and by the reproducibility checks, where
    // "close enough" would defeat the point (INV-3). Ordinary code should
    // compare with an explicit tolerance instead.
    constexpr bool operator==(const Vec2& o) const noexcept { return x == o.x && y == o.y; }
    constexpr bool operator!=(const Vec2& o) const noexcept { return !(*this == o); }

    // --- magnitudes --------------------------------------------------------
    // norm_sq is the one to use in hot loops and in gating comparisons: it
    // avoids a sqrt, and comparing squared distances against a squared
    // threshold is exactly equivalent.
    [[nodiscard]] constexpr double norm_sq() const noexcept { return x * x + y * y; }
    [[nodiscard]] double norm() const noexcept { return std::sqrt(norm_sq()); }

    // Largest absolute component — the Chebyshev norm. Handy for "is this
    // inside an axis-aligned box" tests, which is most of the FOV logic.
    [[nodiscard]] double norm_inf() const noexcept { return std::fmax(std::fabs(x), std::fabs(y)); }
};

template <typename Tag>
constexpr Vec2<Tag> operator*(double s, const Vec2<Tag>& v) noexcept { return v * s; }

// Tag types. Empty structs; they exist only to make the templates distinct.
struct AngleTag {};   // microradians
struct PixelTag {};   // sensor or screen pixels
struct RateTag  {};   // microradians per second

/// An angular position, in microradians. The two components are azimuth (x)
/// and elevation (y) in the world-fixed pointing frame, with the origin at the
/// centre of the simulated screen.
using Angle2 = Vec2<AngleTag>;

/// A pixel position. Sub-pixel by construction — the graded metric is a
/// sub-pixel centroid (design §13.1), so integer pixel coordinates would throw
/// away the answer. Convention: (0,0) is the *centre* of the top-left pixel,
/// matching the splatting maths in design §9.2.
using Pixel2 = Vec2<PixelTag>;

/// An angular rate, in microradians per second. Gimbal commands, target
/// velocity estimates and platform-motion estimates are all Rate2.
using Rate2 = Vec2<RateTag>;

// ---------------------------------------------------------------------------
// Explicit crossings between the families.
//
// These are the *only* sanctioned ways to move between angular and pixel
// space without going through the full camera projection. They exist for
// scale conversions (an IFOV multiply), not for changing frame of reference:
// converting a position needs frames.hpp, which knows where the camera points.
// ---------------------------------------------------------------------------

/// Scale a pixel-space quantity into angle space by the instantaneous field of
/// view (microradians per pixel). Valid for *differences and sizes*, not for
/// absolute positions.
[[nodiscard]] constexpr Angle2 px_to_urad(Pixel2 p, double ifov_urad) noexcept {
    return {p.x * ifov_urad, p.y * ifov_urad};
}

/// The inverse of px_to_urad. Same caveat: differences and sizes only.
[[nodiscard]] constexpr Pixel2 urad_to_px(Angle2 a, double ifov_urad) noexcept {
    return {a.x / ifov_urad, a.y / ifov_urad};
}

/// Pixels-per-frame to microradians-per-second — the conversion behind the
/// startup line design §9.3 insists we log:
///     20 px/frame -> 20 * 109.08 * 30 = 65448 urad/s = 3.75 deg/s
[[nodiscard]] constexpr double px_per_frame_to_urad_s(double px_per_frame,
                                                      double ifov_urad,
                                                      double frame_hz) noexcept {
    return px_per_frame * ifov_urad * frame_hz;
}

// ---------------------------------------------------------------------------
// Small numeric helpers used across the codebase.
//
// std::clamp is fine but takes references and is not always inlined at -O0,
// where our debug-build test runs still have to be quick. These also document
// intent at the call site.
// ---------------------------------------------------------------------------
[[nodiscard]] constexpr double clamp(double v, double lo, double hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

/// Symmetric clamp to [-limit, +limit]. This is what every rate and
/// acceleration limit in the plant (design §10.3) actually wants.
[[nodiscard]] constexpr double clamp_abs(double v, double limit) noexcept {
    return clamp(v, -limit, limit);
}

/// Clamp both components of a rate independently. Note this is a *box* limit,
/// not a disc limit: pan and tilt have independent motors with independent
/// rate ceilings (spec rows 13 and 14), so limiting them separately is the
/// physically correct thing to do.
[[nodiscard]] constexpr Rate2 clamp_rate(Rate2 r, double max_x, double max_y) noexcept {
    return {clamp_abs(r.x, max_x), clamp_abs(r.y, max_y)};
}

/// Quantise to a step size, rounding half away from zero. Models the gimbal's
/// encoder LSB (design §10.3): the controller must only ever see a quantised
/// position, while the metrics see the true one.
[[nodiscard]] inline double quantise(double v, double step) noexcept {
    if (step <= 0.0) return v;          // zero/negative step means "no quantisation"
    return std::round(v / step) * step;
}

}  // namespace sat
