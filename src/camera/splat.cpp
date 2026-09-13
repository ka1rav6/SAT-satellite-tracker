// camera/splat.cpp — see splat.hpp for why this is float and why it is exact.

#include "camera/splat.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace sat {

void splat_emitter(std::span<float> dst, int width, int height,
                   Pixel2 centre, double size_px, ShapeKind shape,
                   float intensity, double weight) noexcept {
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
        reach = 6.0 * size_px * kFwhmToSigma;
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

    for (int j = j0; j <= j1; ++j) {
        float* row = dst.data() + static_cast<size_t>(j) * static_cast<size_t>(width);
        for (int i = i0; i <= i1; ++i) {
            double cov = 0.0;
            switch (shape) {
                case ShapeKind::Square:
                    cov = square_coverage(i, j, centre.x, centre.y, size_px);
                    break;
                case ShapeKind::Circle:
                    cov = circle_coverage(i, j, centre.x, centre.y, size_px);
                    break;
                case ShapeKind::Gaussian:
                    cov = gaussian_coverage(i, j, centre.x, centre.y, size_px);
                    break;
                case ShapeKind::Mask:
                    // Masks land with CP 4.2 (stb_image + resample at load).
                    // Falling back to a square keeps a mis-configured scenario
                    // running with a visible beacon rather than an invisible
                    // one, which is the friendlier failure during a demo.
                    cov = square_coverage(i, j, centre.x, centre.y, size_px);
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

Pixel2 intensity_centroid(std::span<const float> img, int width, int height) noexcept {
    // Accumulate in double. The sum over a 640x480 frame of float values can
    // reach 1e7, where float has ~1 ulp per unit — enough to move the answer in
    // the third decimal place, which is the precision CP 1.3 asks for.
    double sw = 0.0, swx = 0.0, swy = 0.0;
    for (int j = 0; j < height; ++j) {
        const float* row = img.data() + static_cast<size_t>(j) * static_cast<size_t>(width);
        for (int i = 0; i < width; ++i) {
            const double w = static_cast<double>(row[i]);
            if (w <= 0.0) continue;
            sw  += w;
            swx += w * static_cast<double>(i);
            swy += w * static_cast<double>(j);
        }
    }
    if (sw <= 0.0) return Pixel2{0.0, 0.0};
    return Pixel2{swx / sw, swy / sw};
}

double total_flux(std::span<const float> img) noexcept {
    double s = 0.0;
    for (const float v : img) s += static_cast<double>(v);
    return s;
}

void quantise_u8(std::span<const float> src, std::span<uint8_t> dst) noexcept {
    const size_t n = std::min(src.size(), dst.size());
    for (size_t i = 0; i < n; ++i) {
        const float v = src[i];
        // NaN would compare false against both bounds and fall through to the
        // cast, which is undefined. Clamp explicitly.
        if (!(v > 0.0f))   { dst[i] = 0;   continue; }
        if (v >= 254.5f)   { dst[i] = 255; continue; }
        dst[i] = static_cast<uint8_t>(v + 0.5f);
    }
}

bool write_pgm(const char* path, std::span<const uint8_t> img,
               int width, int height) noexcept {
    if (img.size() < static_cast<size_t>(width) * static_cast<size_t>(height)) return false;
    std::FILE* f = std::fopen(path, "wb");
    if (!f) return false;
    // P5 is binary greyscale: a three-line ASCII header then raw bytes.
    std::fprintf(f, "P5\n%d %d\n255\n", width, height);
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    const bool ok = std::fwrite(img.data(), 1, n, f) == n;
    std::fclose(f);
    return ok;
}

}  // namespace sat
