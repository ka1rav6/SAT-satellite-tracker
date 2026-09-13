// core/image.cpp

#include "core/image.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace sat {

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


Pixel2 intensity_centroid_u8(std::span<const uint8_t> img, int width, int height,
                             double floor_value) noexcept {
    double sw = 0.0, swx = 0.0, swy = 0.0;
    for (int j = 0; j < height; ++j) {
        const size_t row = static_cast<size_t>(j) * static_cast<size_t>(width);
        for (int i = 0; i < width; ++i) {
            // Clip at zero: a negative weight would let a dark pixel pull the
            // centroid the wrong way, which is not something a light source does.
            const double w = static_cast<double>(img[row + static_cast<size_t>(i)])
                           - floor_value;
            if (w <= 0.0) continue;
            sw  += w;
            swx += w * static_cast<double>(i);
            swy += w * static_cast<double>(j);
        }
    }
    if (sw <= 0.0) return Pixel2{0.0, 0.0};
    return Pixel2{swx / sw, swy / sw};
}

}  // namespace sat
