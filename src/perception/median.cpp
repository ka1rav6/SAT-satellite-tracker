#include "perception/median.hpp"

namespace sat {

void median_3x3(std::span<const uint8_t> src, std::span<uint8_t> dst,
                int width, int height) noexcept {
    if (width <= 0 || height <= 0) return;
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (src.size() < n || dst.size() < n) return;

    // A 1x1 or Nx1 image has no 3x3 neighbourhood to speak of; replicate makes
    // every sample the same pixel, so the median is the pixel itself.
    if (width < 2 || height < 2) {
        for (size_t i = 0; i < n; ++i) dst[i] = src[i];
        return;
    }

    const uint8_t* s = src.data();
    uint8_t*       d = dst.data();

    // Clamped row/column lookup, which is the REPLICATE border rule.
    auto at = [&](int x, int y) -> uint8_t {
        const int cx = x < 0 ? 0 : (x >= width  ? width  - 1 : x);
        const int cy = y < 0 ? 0 : (y >= height ? height - 1 : y);
        return s[static_cast<size_t>(cy) * static_cast<size_t>(width) + static_cast<size_t>(cx)];
    };

    for (int y = 0; y < height; ++y) {
        const bool interior_row = (y > 0 && y < height - 1);
        uint8_t* drow = d + static_cast<size_t>(y) * static_cast<size_t>(width);

        // Interior rows take a fast path with direct row pointers: no clamping
        // per sample, which is most of the work in the naive version.
        if (interior_row) {
            const uint8_t* r0 = s + static_cast<size_t>(y - 1) * static_cast<size_t>(width);
            const uint8_t* r1 = s + static_cast<size_t>(y)     * static_cast<size_t>(width);
            const uint8_t* r2 = s + static_cast<size_t>(y + 1) * static_cast<size_t>(width);

            drow[0] = median9(at(-1, y - 1), r0[0], r0[1],
                              at(-1, y),     r1[0], r1[1],
                              at(-1, y + 1), r2[0], r2[1]);

            for (int x = 1; x < width - 1; ++x) {
                drow[x] = median9(r0[x - 1], r0[x], r0[x + 1],
                                  r1[x - 1], r1[x], r1[x + 1],
                                  r2[x - 1], r2[x], r2[x + 1]);
            }

            const int xe = width - 1;
            drow[xe] = median9(r0[xe - 1], r0[xe], at(width, y - 1),
                               r1[xe - 1], r1[xe], at(width, y),
                               r2[xe - 1], r2[xe], at(width, y + 1));
        } else {
            for (int x = 0; x < width; ++x) {
                drow[x] = median9(at(x - 1, y - 1), at(x, y - 1), at(x + 1, y - 1),
                                  at(x - 1, y),     at(x, y),     at(x + 1, y),
                                  at(x - 1, y + 1), at(x, y + 1), at(x + 1, y + 1));
            }
        }
    }
}

}  // namespace sat
