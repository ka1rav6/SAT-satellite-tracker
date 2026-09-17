// camera/splat.cpp — see splat.hpp for why this is float and why it is exact.

#include "camera/splat.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace sat {

namespace {

// ---------------------------------------------------------------------------
// SEPARABILITY — why this file looks the way it does
// ---------------------------------------------------------------------------
// The first version of splat_emitter walked the footprint pixel by pixel and
// called the shape's coverage function at each one:
//
//     for j: for i: cov = gaussian_coverage(i, j, cx, cy, size)
//
// That is the obvious loop and it is quadratically wasteful, because two of the
// three shapes are SEPARABLE:
//
//     square_coverage(i,j)   = overlap_x(i) * overlap_y(j)
//     gaussian_coverage(i,j) = gauss_x(i)  * gauss_y(j) * norm
//
// so a W x H footprint needs W + H one-dimensional evaluations, not W * H
// two-dimensional ones. For a Gaussian each 2-D evaluation is FOUR erf() calls,
// and clutter sources run up to 24 px across, whose 6-sigma reach is a 124 x 124
// box. Per emitter per exposure substep that is 61,504 erf calls the old way
// and 496 the new way — 124x fewer, for an identical result.
//
// Measured on scenarios/compliance.toml (120 clutter sources, 8 blur substeps),
// the splat stage was 10,090 us p50 against design §15's 40 us budget. It was
// the single largest line in the frame, larger than the entire detector.
//
// The circle is NOT separable — a disk's boundary couples the axes — but it
// decomposes a different way. circle_coverage is an inclusion-exclusion over the
// pixel's four corners:
//
//     cov(i,j) = Q(x1,y1) - Q(x0,y1) - Q(x1,y0) + Q(x0,y0)
//
// and ADJACENT PIXELS SHARE CORNERS. Evaluating Q once per grid corner instead
// of four times per pixel is exactly 4x fewer calls, and because only two rows
// of corners are ever live the scratch is O(W), not O(W*H).
//
// ---------------------------------------------------------------------------
// BIT-EXACTNESS
// ---------------------------------------------------------------------------
// Every expression below multiplies its factors in the SAME ORDER as the
// coverage function it replaces, so the result is not merely close to the old
// one, it is the same double. That matters for two reasons: INV-3 (a run must
// reproduce bit for bit) and CP 14.2's acceptance criterion, which asks a faster
// kernel to be bit-identical to the scalar one rather than merely equivalent.
// tests/camera/test_splat.cpp checks it directly against the coverage functions.
// ---------------------------------------------------------------------------

/// Largest footprint span this file will factorise, in pixels. Beyond it the
/// scratch rows would be a silly amount of stack, so the direct per-pixel path
/// is used instead — correct, just slower. The schema allows sensors up to
/// 16384 px wide; nothing in this project configures one, and a 2048 px
/// footprint already means an emitter covering a 4-megapixel area.
constexpr int kMaxSpan = 2048;

}  // namespace

void splat_emitter(std::span<float> dst, int width, int height,
                   Pixel2 centre, double size_px, ShapeKind shape,
                   float intensity, double weight, double reach_sigmas) noexcept {
    if (dst.empty() || size_px <= 0.0 || weight == 0.0) return;

    // How far the emitter reaches from its centre.
    //
    // A Gaussian is quoted as a FWHM but its tails run further, so it gets SIX
    // sigma. Three would be plenty on a brightness argument — the tail there is
    // under 1/370 of the peak, below the 8-bit quantisation floor — but
    // brightness is the wrong test. Truncating the profile is asymmetric about
    // a sub-pixel centre, so it SHIFTS the rendered centroid, and this position
    // is the reference the graded metric is measured against. Measured
    // worst-case bias: 3.5e-3 px at 3 sigma, 7.0e-5 at 4, 5.3e-7 at 5, and
    // 8.7e-9 at 6.
    double reach = 0.5 * size_px;
    if (shape == ShapeKind::Gaussian) {
        constexpr double kFwhmToSigma = 1.0 / 2.3548200450309493;
        // Six sigma for anything whose position is graded; see splat.hpp for
        // why, and for why clutter does not need it.
        reach = std::max(0.5, reach_sigmas) * size_px * kFwhmToSigma;
    }

    // Bounding box of the footprint, clipped to the sensor. The +1/-1 slack
    // covers the partially-covered pixels at the very edge of the footprint;
    // without it a beacon's outermost row would be silently dropped, which
    // truncates the profile asymmetrically and biases the centroid — exactly
    // the failure mode design §10.1.3 attributes the S-curve to.
    const int i0 = std::max(0,          static_cast<int>(std::floor(centre.x - reach)) - 1);
    const int i1 = std::min(width  - 1, static_cast<int>(std::ceil (centre.x + reach)) + 1);
    const int j0 = std::max(0,          static_cast<int>(std::floor(centre.y - reach)) - 1);
    const int j1 = std::min(height - 1, static_cast<int>(std::ceil (centre.y + reach)) + 1);
    if (i0 > i1 || j0 > j1) return;   // entirely off-sensor

    const double scale = static_cast<double>(intensity) * weight;
    const int    nx    = i1 - i0 + 1;
    const int    ny    = j1 - j0 + 1;

    // ----------------------------------------------------------------------
    // The separable shapes: one 1-D table per axis, then an outer product.
    // ----------------------------------------------------------------------
    if (shape != ShapeKind::Circle && nx <= kMaxSpan && ny <= kMaxSpan) {
        // Deliberately uninitialised: only [0, nx) and [0, ny) are ever read,
        // and zeroing 32 KB per emitter would cost more than the erf calls we
        // just removed.
        std::array<double, kMaxSpan> ox;   // NOLINT(*-member-init)
        std::array<double, kMaxSpan> oy;   // NOLINT(*-member-init)
        double norm = 1.0;

        if (shape == ShapeKind::Gaussian) {
            constexpr double kFwhmToSigma = 1.0 / 2.3548200450309493;
            const double sigma = std::fmax(size_px * kFwhmToSigma, 1e-6);
            // Exactly gaussian_coverage's normaliser, computed once instead of
            // once per pixel: the value a pixel sitting on the peak receives.
            const double peak_1d = gauss_1d(-0.5, 0.5, 0.0, sigma);
            norm = 1.0 / std::fmax(peak_1d * peak_1d, 1e-300);
            for (int k = 0; k < nx; ++k) {
                const int i = i0 + k;
                ox[static_cast<size_t>(k)] = gauss_1d(pixel_lo(i), pixel_hi(i), centre.x, sigma);
            }
            for (int k = 0; k < ny; ++k) {
                const int j = j0 + k;
                oy[static_cast<size_t>(k)] = gauss_1d(pixel_lo(j), pixel_hi(j), centre.y, sigma);
            }
        } else {
            // Square, and Mask until CP 4.2's resampler lands — falling back to
            // a square keeps a mis-configured scenario running with a visible
            // beacon rather than an invisible one, which is the friendlier
            // failure during a demo.
            const double h   = 0.5 * size_px;
            const double ex0 = centre.x - h, ex1 = centre.x + h;
            const double ey0 = centre.y - h, ey1 = centre.y + h;
            for (int k = 0; k < nx; ++k) {
                const int i = i0 + k;
                ox[static_cast<size_t>(k)] = overlap_1d(pixel_lo(i), pixel_hi(i), ex0, ex1);
            }
            for (int k = 0; k < ny; ++k) {
                const int j = j0 + k;
                oy[static_cast<size_t>(k)] = overlap_1d(pixel_lo(j), pixel_hi(j), ey0, ey1);
            }
        }

        // The x extent that actually contributes. A square's box has dead
        // margins on both sides and a Gaussian's 1-D table can underflow to
        // zero in its tails, and trimming the loop bounds once per emitter is
        // strictly better than testing every pixel inside it.
        int kx0 = 0, kx1 = nx - 1;
        while (kx0 <= kx1 && !(ox[static_cast<size_t>(kx0)] > 0.0)) ++kx0;
        while (kx1 >= kx0 && !(ox[static_cast<size_t>(kx1)] > 0.0)) --kx1;
        if (kx0 > kx1) return;

        for (int kj = 0; kj < ny; ++kj) {
            const double oyv = oy[static_cast<size_t>(kj)];
            // A whole row of zero coverage is common: the Gaussian's box has
            // long dead margins, and a square's box has two of them.
            if (!(oyv > 0.0)) continue;
            float* row = dst.data()
                       + static_cast<size_t>(j0 + kj) * static_cast<size_t>(width) + i0;
            // No per-pixel branch. Inside [kx0, kx1] the coverage is positive
            // except where a Gaussian's table has underflowed, and adding an
            // exact zero to a positive pedestal is a no-op — so the test bought
            // nothing and cost the loop its vectorisation.
            for (int ki = kx0; ki <= kx1; ++ki) {
                // Same multiplication order as square_coverage/gaussian_coverage
                // so the double is bit-identical, not merely equal to tolerance.
                const double cov = (ox[static_cast<size_t>(ki)] * oyv) * norm;
                row[ki] += static_cast<float>(cov * scale);
            }
        }
        return;
    }

    // ----------------------------------------------------------------------
    // The circle: inclusion-exclusion over shared grid corners, two rows live.
    // ----------------------------------------------------------------------
    if (shape == ShapeKind::Circle && nx < kMaxSpan) {
        const double r = 0.5 * size_px;
        if (r <= 0.0) return;

        std::array<double, kMaxSpan + 1> qa;   // NOLINT(*-member-init)
        std::array<double, kMaxSpan + 1> qb;   // NOLINT(*-member-init)
        // Two rows, swapped by POINTER each step. Copying the array instead
        // would move 16 KB per image row and hand back most of what the corner
        // sharing just saved.
        double* qlo = qa.data();
        double* qhi = qb.data();

        // Corner abscissae, already shifted into the disk's frame — the same
        // shift disk_rect_area performs internally.
        std::array<double, kMaxSpan + 1> ax;    // NOLINT(*-member-init)
        for (int k = 0; k <= nx; ++k) {
            ax[static_cast<size_t>(k)] = pixel_lo(i0 + k) - centre.x;
        }

        // Row j's lower corners are row j-1's upper corners, so each grid corner
        // is evaluated exactly once over the whole footprint.
        double ay = pixel_lo(j0) - centre.y;
        for (int k = 0; k <= nx; ++k) {
            qlo[static_cast<size_t>(k)] = disk_quadrant_area(ax[static_cast<size_t>(k)], ay, r);
        }

        for (int kj = 0; kj < ny; ++kj) {
            ay = pixel_hi(j0 + kj) - centre.y;
            for (int k = 0; k <= nx; ++k) {
                qhi[static_cast<size_t>(k)] = disk_quadrant_area(ax[static_cast<size_t>(k)], ay, r);
            }

            float* row = dst.data()
                       + static_cast<size_t>(j0 + kj) * static_cast<size_t>(width) + i0;
            for (int ki = 0; ki < nx; ++ki) {
                const size_t a = static_cast<size_t>(ki), b = a + 1;
                // disk_rect_area's exact expression and grouping:
                //   Q(x1,y1) - Q(x0,y1) - Q(x1,y0) + Q(x0,y0), then clamped at 0.
                const double area = qhi[b] - qhi[a] - qlo[b] + qlo[a];
                const double cov  = std::fmax(0.0, area);
                if (cov > 0.0) row[ki] += static_cast<float>(cov * scale);
            }
            std::swap(qlo, qhi);
        }
        return;
    }

    // ----------------------------------------------------------------------
    // Fallback: a footprint wider than kMaxSpan. Correct, just not factorised.
    // ----------------------------------------------------------------------
    for (int j = j0; j <= j1; ++j) {
        float* row = dst.data() + static_cast<size_t>(j) * static_cast<size_t>(width);
        for (int i = i0; i <= i1; ++i) {
            double cov = 0.0;
            switch (shape) {
                case ShapeKind::Square:
                case ShapeKind::Mask:
                    cov = square_coverage(i, j, centre.x, centre.y, size_px);
                    break;
                case ShapeKind::Circle:
                    cov = circle_coverage(i, j, centre.x, centre.y, size_px);
                    break;
                case ShapeKind::Gaussian:
                    cov = gaussian_coverage(i, j, centre.x, centre.y, size_px);
                    break;
            }
            if (cov > 0.0) {
                row[i] += static_cast<float>(cov * scale);
            }
        }
    }
}

void splat_emitters(std::span<float> dst,
                    const CameraGeometry& cam, const ScreenGeometry& scr,
                    const EmitterSoA& emitters, std::span<const uint32_t> visible,
                    Angle2 boresight, double weight) noexcept {
    for (const uint32_t idx : visible) {
        const size_t i = idx;
        // Screen position -> image position at this boresight. Continuous
        // throughout: nothing is rounded on the way in.
        const Pixel2 img = screen_to_image(cam, scr, emitters.position(i), boresight);
        splat_emitter(dst, cam.width, cam.height, img,
                      static_cast<double>(emitters.size_px[i]),
                      emitters.shape_of(i), emitters.intensity[i], weight);
    }
}

}  // namespace sat
