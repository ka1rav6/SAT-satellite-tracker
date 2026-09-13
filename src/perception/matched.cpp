#include "perception/matched.hpp"

#include <algorithm>
#include <cmath>

namespace sat {

float matched_response(const SummedArea& sa, int x, int y, int k) noexcept {
    if (k <= 0) return 0.0f;
    const int h = k / 2;
    int count = 0;
    // Half-open, centred on (x, y): [x-h, x-h+k) x [y-h, y-h+k).
    const int64_t sum = sa.box_sum_clipped(x - h, y - h, x - h + k, y - h + k, count);
    if (count <= 0) return 0.0f;

    // Normalise by sqrt(count), not by count.
    //
    // Dividing by the count would give the local MEAN, whose noise falls as
    // 1/sqrt(n) — so larger boxes would look quieter and the max over scales
    // would always pick the smallest. Dividing by sqrt(count) gives the
    // matched-filter output: the noise standard deviation is then identical at
    // every scale, and the maximum is chosen by signal rather than by geometry.
    //
    // sqrt(count) rather than sqrt(k*k) = k so that a box clipped at the frame
    // edge is still normalised by the pixels it actually summed.
    return static_cast<float>(static_cast<double>(sum) / std::sqrt(static_cast<double>(count)));
}

MatchedPeak matched_best(const SummedArea& sa, int x, int y) noexcept {
    MatchedPeak best{};
    bool first = true;
    for (const int k : kMatchedScales) {
        const float r = matched_response(sa, x, y, k);
        if (first || r > best.response) {
            best.response = r;
            best.scale    = k;
            first = false;
        }
    }
    return best;
}

void matched_filter(const SummedArea& sa, int width, int height,
                    std::span<float> response, std::span<uint8_t> scale) noexcept {
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (width <= 0 || height <= 0 || response.size() < n || scale.size() < n) return;

    for (int y = 0; y < height; ++y) {
        const size_t row = static_cast<size_t>(y) * static_cast<size_t>(width);
        for (int x = 0; x < width; ++x) {
            const MatchedPeak p = matched_best(sa, x, y);
            response[row + static_cast<size_t>(x)] = p.response;
            scale[row + static_cast<size_t>(x)]    = static_cast<uint8_t>(p.scale);
        }
    }
}

}  // namespace sat
