// degrade/fast_normal.cpp — see the header for why this is not next_normal().

#include "degrade/fast_normal.hpp"

#include <cmath>
#include <limits>

namespace sat {

// ---------------------------------------------------------------------------
// Acklam's rational approximation to the inverse normal CDF.
//
// Three regions. The central one is a rational in r = (p - 0.5)^2, which is
// where 95.15% of probability lives and where no transcendental is needed. The
// two tails substitute q = sqrt(-2 ln p) and use a different rational; they are
// mirror images, so only the sign of the result differs.
//
// The region boundary p_low = 0.02425 is Acklam's, chosen so the two rationals
// meet with the stated error bound on both sides.
// ---------------------------------------------------------------------------
double acklam_inverse_normal(double p) noexcept {
    static constexpr double a[6] = {
        -3.969683028665376e+01,  2.209460984245205e+02, -2.759285104469687e+02,
         1.383577518672690e+02, -3.066479806614716e+01,  2.506628277459239e+00};
    static constexpr double b[5] = {
        -5.447609879822406e+01,  1.615858368580409e+02, -1.556989798598866e+02,
         6.680131188771972e+01, -1.328068155288572e+01};
    static constexpr double c[6] = {
        -7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00,
        -2.549732539343734e+00,  4.374664141464968e+00,  2.938163982698783e+00};
    static constexpr double d[4] = {
         7.784695709041462e-03,  3.224671290700398e-01,  2.445134137142996e+00,
         3.754408661907416e+00};

    constexpr double p_low  = 0.02425;
    constexpr double p_high = 1.0 - p_low;

    // Defensive, not expected: callers pass p in (0, 1) by construction. A NaN
    // reaching the sensor chain would propagate into the frame and CP 14.1
    // treats a NaN in any logged value as a failure, so it is stopped here.
    if (!(p > 0.0)) return -std::numeric_limits<double>::infinity();
    if (!(p < 1.0)) return  std::numeric_limits<double>::infinity();

    if (p < p_low) {
        const double q = std::sqrt(-2.0 * std::log(p));
        return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5])
             / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
    }
    if (p > p_high) {
        const double q = std::sqrt(-2.0 * std::log(1.0 - p));
        return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5])
              / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
    }
    const double q = p - 0.5;
    const double r = q * q;
    return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q
         / (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0);
}

// ---------------------------------------------------------------------------
// The table.
//
// Knot i holds Phi^-1(i / kCells). i = 0 would be -infinity and i = kCells
// would be +infinity, but neither is ever read: the first and last kExactCells
// cells take the closed-form path, so the interpolating code only ever touches
// knots kExactCells .. kCells - kExactCells. They are filled with the adjacent
// finite value anyway, so a future change that widened the interpolating range
// would produce a wrong number rather than an infinity that silently poisons a
// whole frame.
// ---------------------------------------------------------------------------
FastNormal::FastNormal() noexcept {
    for (int i = 1; i < kCells; ++i) {
        const double p = static_cast<double>(i) / static_cast<double>(kCells);
        knots_[static_cast<size_t>(i)] = static_cast<float>(acklam_inverse_normal(p));
    }
    knots_[0]      = knots_[1];
    knots_[kCells] = knots_[kCells - 1];
}

// The single instance. Const, so it lands in .data.rel.ro and is read-only for
// the life of the process — the decode thread and the simulation thread can
// both use it freely with no synchronisation at all.
const FastNormal kFastNormal;

}  // namespace sat
