// perception/simple_detector.cpp — see the header for why this is intentionally
// the worst detector in the project.

#include "perception/simple_detector.hpp"

#include <algorithm>

namespace sat {

SimpleDetection detect_brightest(std::span<const uint8_t> img,
                                 int width, int height) noexcept {
    SimpleDetection d{};
    if (img.empty() || width <= 0 || height <= 0) return d;

    const size_t n = std::min(img.size(),
                              static_cast<size_t>(width) * static_cast<size_t>(height));
    uint8_t best     = 0;
    size_t  best_idx = 0;
    bool    any      = false;

    // Strict > keeps the FIRST maximum in raster order. Using >= would keep the
    // last, which is equally arbitrary but harder to reason about; either way
    // the point is that the rule is fixed (INV-3).
    for (size_t i = 0; i < n; ++i) {
        if (!any || img[i] > best) {
            best = img[i];
            best_idx = i;
            any = true;
        }
    }
    if (!any) return d;

    d.found      = true;
    d.peak       = static_cast<float>(best);
    d.integrated = d.peak;
    d.centre     = Pixel2{static_cast<double>(best_idx % static_cast<size_t>(width)),
                          static_cast<double>(best_idx / static_cast<size_t>(width))};
    return d;
}

SimpleDetection detect_brightest_subpixel(std::span<const uint8_t> img,
                                          int width, int height,
                                          int window, float floor_value) noexcept {
    SimpleDetection d = detect_brightest(img, width, height);
    if (!d.found) return d;

    const int px = static_cast<int>(d.centre.x);
    const int py = static_cast<int>(d.centre.y);

    const int i0 = std::max(0,          px - window);
    const int i1 = std::min(width  - 1, px + window);
    const int j0 = std::max(0,          py - window);
    const int j1 = std::min(height - 1, py + window);

    double sw = 0.0, swx = 0.0, swy = 0.0;
    for (int j = j0; j <= j1; ++j) {
        const size_t row = static_cast<size_t>(j) * static_cast<size_t>(width);
        for (int i = i0; i <= i1; ++i) {
            // Subtract the background floor and clip at zero. Negative weights
            // would let a dark pixel pull the centroid the wrong way, which is
            // not a physically meaningful thing for a light source to do.
            const double w = static_cast<double>(img[row + static_cast<size_t>(i)])
                           - static_cast<double>(floor_value);
            if (w <= 0.0) continue;
            sw  += w;
            swx += w * static_cast<double>(i);
            swy += w * static_cast<double>(j);
        }
    }

    if (sw > 0.0) {
        d.centre     = Pixel2{swx / sw, swy / sw};
        d.integrated = static_cast<float>(sw);
    }
    // If every sample was at or below the floor, the peak-pixel position from
    // detect_brightest stands. It is worse, but it is not a lie.
    return d;
}

}  // namespace sat
