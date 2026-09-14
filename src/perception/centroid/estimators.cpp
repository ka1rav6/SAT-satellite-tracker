// perception/centroid/estimators.cpp

#include "perception/centroid/estimators.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace sat {

const char* centroid_kind_name(CentroidKind k) noexcept {
    switch (k) {
        case CentroidKind::CoM:         return "com";
        case CentroidKind::WindowedCoM: return "windowed";
        case CentroidKind::SurfaceFit:  return "surface_fit";
        case CentroidKind::MatchedPeak: return "matched_peak";
        case CentroidKind::Learned:     return "learned";
        case CentroidKind::kCount:      break;
    }
    return "unknown";
}

bool parse_centroid_kind(std::string_view s, CentroidKind& out) noexcept {
    for (uint8_t i = 0; i < static_cast<uint8_t>(CentroidKind::kCount); ++i) {
        const auto k = static_cast<CentroidKind>(i);
        if (s == centroid_kind_name(k)) { out = k; return true; }
    }
    return false;
}

Pixel2 centroid_com(const BlobAccum& b) noexcept {
    return b.centroid();
}

// ---------------------------------------------------------------------------
// centroid_windowed
// ---------------------------------------------------------------------------
Pixel2 centroid_windowed(std::span<const int16_t> tophat, int width, int height,
                         Pixel2 peak, int win) noexcept {
    const int px = static_cast<int>(std::lround(peak.x));
    const int py = static_cast<int>(std::lround(peak.y));
    win = std::clamp(win, 1, 64);

    const int x0 = std::max(0, px - win), x1 = std::min(width  - 1, px + win);
    const int y0 = std::max(0, py - win), y1 = std::min(height - 1, py + win);

    double sw = 0.0, sx = 0.0, sy = 0.0;
    for (int y = y0; y <= y1; ++y) {
        const int16_t* row = tophat.data() + static_cast<ptrdiff_t>(y) * width;
        for (int x = x0; x <= x1; ++x) {
            // Negative top-hat values are background undershoot, not signal.
            // Including them would let a dark ring on one side pull the
            // estimate toward the other, which is a bias that looks exactly
            // like the S-curve and is not one.
            const double w = row[x] > 0 ? static_cast<double>(row[x]) : 0.0;
            sw += w; sx += w * x; sy += w * y;
        }
    }
    if (!(sw > 0.0)) return peak;
    return Pixel2{sx / sw, sy / sw};
}

// ---------------------------------------------------------------------------
// centroid_surface_fit
//
// A Gaussian beacon has ln I = -(x-x0)^2/(2s^2) - (y-y0)^2/(2s^2) + c, which is
// a quadratic in x and y with no cross term. Fitting
//
//     ln I ~ a x^2 + b x + c y^2 + d y + e
//
// by least squares over a 5x5 and solving for the vertex gives the centre
// analytically: x0 = -b/(2a), y0 = -d/(2c).
//
// The separable form (no xy term) is deliberate. A full conic would fit an
// arbitrarily rotated ellipse and needs six parameters, but the extra freedom
// mostly buys the fit room to chase noise on a 25-sample window. The beacon is
// specified as a square or a circle (spec row 9), so a separable fit is the
// right model and the smaller one.
//
// Because the x and y fits are independent and share the same 5-point grid,
// each reduces to a 3x3 normal-equation system with a CONSTANT design matrix —
// so the solution is a fixed set of weights and there is no matrix code here.
// ---------------------------------------------------------------------------
namespace {

/// Fit y = a t^2 + b t + c over t = -2..2 with per-sample weights, and return
/// the vertex -b/(2a). Returns `fallback` when the fit is not a maximum.
double parabola_vertex(const double v[5], const double w[5], double fallback) noexcept {
    // Weighted least squares on a 5-point symmetric grid. Written as explicit
    // moment sums rather than a general solver: five points, three unknowns,
    // and the alternative is a 3x3 inverse that would be the only linear
    // algebra in the perception module.
    double s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0;
    double b0 = 0, b1 = 0, b2 = 0;
    for (int i = 0; i < 5; ++i) {
        const double t = i - 2.0;
        const double ww = w[i];
        if (!(ww > 0.0)) continue;
        const double t2 = t * t;
        s0 += ww; s1 += ww * t; s2 += ww * t2; s3 += ww * t2 * t; s4 += ww * t2 * t2;
        b0 += ww * v[i]; b1 += ww * v[i] * t; b2 += ww * v[i] * t2;
    }
    // Solve [[s4,s3,s2],[s3,s2,s1],[s2,s1,s0]] [a,b,c]^T = [b2,b1,b0]^T.
    const double m11 = s4, m12 = s3, m13 = s2;
    const double m21 = s3, m22 = s2, m23 = s1;
    const double m31 = s2, m32 = s1, m33 = s0;
    const double det = m11 * (m22 * m33 - m23 * m32)
                     - m12 * (m21 * m33 - m23 * m31)
                     + m13 * (m21 * m32 - m22 * m31);
    if (std::fabs(det) < 1e-12) return fallback;

    const double a = (b2 * (m22 * m33 - m23 * m32)
                    - m12 * (b1 * m33 - m23 * b0)
                    + m13 * (b1 * m32 - m22 * b0)) / det;
    const double b = (m11 * (b1 * m33 - m23 * b0)
                    - b2 * (m21 * m33 - m23 * m31)
                    + m13 * (m21 * b0 - b1 * m31)) / det;

    // a must be negative for the vertex to be a maximum. A non-negative a means
    // the window does not contain a peak — a saturated blob, or one the window
    // is off-centre on — and the vertex would be a minimum or infinitely far
    // away. Falling back is the honest answer.
    if (!(a < 0.0)) return fallback;
    const double vertex = -b / (2.0 * a);
    // A vertex outside the fitted window is an extrapolation the data does not
    // support, however well the algebra worked.
    if (!(std::fabs(vertex) <= 2.0)) return fallback;
    return vertex;
}

}  // namespace

Pixel2 centroid_surface_fit(std::span<const int16_t> tophat, int width, int height,
                            Pixel2 peak) noexcept {
    const int px = static_cast<int>(std::lround(peak.x));
    const int py = static_cast<int>(std::lround(peak.y));
    if (px < 2 || py < 2 || px >= width - 2 || py >= height - 2) {
        // Not enough room for a 5x5. Rather than fit a truncated window — which
        // would be biased in exactly the direction of the missing samples —
        // fall back to the windowed estimate.
        return centroid_windowed(tophat, width, height, peak, 2);
    }

    // Marginal profiles: sum along the other axis, so each 1-D fit uses all 25
    // samples rather than a single row. That is both less noisy and, for a
    // separable profile, exactly equivalent.
    double vx[5]{}, vy[5]{}, wx[5]{}, wy[5]{};
    for (int j = 0; j < 5; ++j) {
        const int16_t* row = tophat.data() + static_cast<ptrdiff_t>(py - 2 + j) * width;
        for (int i = 0; i < 5; ++i) {
            const double s = row[px - 2 + i] > 0 ? static_cast<double>(row[px - 2 + i]) : 0.0;
            vx[i] += s;
            vy[j] += s;
        }
    }
    for (int i = 0; i < 5; ++i) {
        // The fit is on LOG intensity, where a Gaussian is exactly quadratic.
        // A zero or negative sample has no log; it is excluded by weight rather
        // than clamped, because clamping would invent a data point.
        wx[i] = vx[i] > 0.0 ? 1.0 : 0.0;
        wy[i] = vy[i] > 0.0 ? 1.0 : 0.0;
        vx[i] = vx[i] > 0.0 ? std::log(vx[i]) : 0.0;
        vy[i] = vy[i] > 0.0 ? std::log(vy[i]) : 0.0;
    }

    const double dx = parabola_vertex(vx, wx, 0.0);
    const double dy = parabola_vertex(vy, wy, 0.0);
    return Pixel2{px + dx, py + dy};
}

// ---------------------------------------------------------------------------
// centroid_matched_peak
//
// Three-point parabolic interpolation on the matched-filter response, per axis:
//
//     d = 0.5 * (L - R) / (L - 2C + R)
//
// the standard sub-sample peak estimator. The response is already the beacon's
// profile correlated with its own scale, so the noise has been integrated away
// — which is why §10.1.2 lists this as the best of the four at low SNR.
// ---------------------------------------------------------------------------
Pixel2 centroid_matched_peak(std::span<const float> response, int width, int height,
                             Pixel2 peak) noexcept {
    const int px = static_cast<int>(std::lround(peak.x));
    const int py = static_cast<int>(std::lround(peak.y));
    if (px < 1 || py < 1 || px >= width - 1 || py >= height - 1) return peak;

    auto interp = [](double l, double c, double r) -> double {
        const double denom = l - 2.0 * c + r;
        // A flat or concave-up triple has no interior maximum; 0 means "the
        // integer peak is the best I can say", which is true rather than
        // convenient.
        if (!(denom < 0.0)) return 0.0;
        const double d = 0.5 * (l - r) / denom;
        return (std::fabs(d) <= 1.0) ? d : 0.0;
    };

    const float* row = response.data() + static_cast<ptrdiff_t>(py) * width;
    const double dx = interp(row[px - 1], row[px], row[px + 1]);
    const double dy = interp(response[static_cast<size_t>(py - 1) * width + px],
                             row[px],
                             response[static_cast<size_t>(py + 1) * width + px]);
    return Pixel2{px + dx, py + dy};
}

Pixel2 centroid_estimate(CentroidKind kind, const BlobAccum& b,
                         std::span<const int16_t> tophat,
                         std::span<const float> response,
                         int width, int height, Pixel2 peak, int win) noexcept {
    switch (kind) {
        case CentroidKind::CoM:         return centroid_com(b);
        case CentroidKind::SurfaceFit:  return centroid_surface_fit(tophat, width, height, peak);
        case CentroidKind::MatchedPeak:
            // With no response map to interpolate there is nothing to do; the
            // windowed estimate is the honest fallback rather than the peak.
            if (response.empty()) return centroid_windowed(tophat, width, height, peak, win);
            return centroid_matched_peak(response, width, height, peak);
        case CentroidKind::Learned:
            // INV-7: the system must run fully without AI. Stage 11 replaces
            // this branch; until then it is the default estimator, so a
            // scenario asking for `learned` runs rather than refusing.
        case CentroidKind::WindowedCoM:
        case CentroidKind::kCount:
            break;
    }
    return centroid_windowed(tophat, width, height, peak, win);
}

}  // namespace sat
