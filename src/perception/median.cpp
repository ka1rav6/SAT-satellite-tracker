#include "perception/median.hpp"

namespace sat {

void median_3x3(std::span<const uint8_t> src, std::span<uint8_t> dst,
                int width, int height) noexcept {
    if (width <= 0 || height <= 0) return;
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (src.size() < n || dst.size() < n) return;

    // NOTE: there is deliberately no early-out for 1-pixel-wide or
    // 1-pixel-tall images.
    //
    // An earlier version copied them through unchanged, on the reasoning that a
    // degenerate image has no neighbourhood. That is wrong, and cv::medianBlur
    // disagreed with us on exactly such a case (a 1x17 column, 10 of 17 pixels
    // differing). Under REPLICATE, a single-column image still has a real 3x3
    // window: the column is replicated left and right, so the median is taken
    // over three ROWS and the filter does genuine vertical smoothing.
    //
    // The general clamped path below produces that for free. The only case that
    // truly degenerates is 1x1, where all nine samples clamp to the same pixel
    // and the median is that pixel — which is also what the general path gives.

    const uint8_t* s = src.data();
    uint8_t*       d = dst.data();
    const size_t   w = static_cast<size_t>(width);

    // Clamped lookup — the REPLICATE border rule.
    auto at = [&](int x, int y) -> uint8_t {
        const int cx = x < 0 ? 0 : (x >= width  ? width  - 1 : x);
        const int cy = y < 0 ? 0 : (y >= height ? height - 1 : y);
        return s[static_cast<size_t>(cy) * w + static_cast<size_t>(cx)];
    };

    for (int y = 0; y < height; ++y) {
        uint8_t* drow = d + static_cast<size_t>(y) * w;

        // Interior rows get direct row pointers: no per-sample clamping, which
        // is most of the work in the naive form.
        //
        // `width >= 3` is required as well as the row condition, and it is not
        // belt-and-braces. The fast path indexes r0[x-1] and r0[x+1] directly;
        // at width 1 those are r0[-1] and r0[1], both off the end of the row.
        // AddressSanitizer caught exactly that as a heap-buffer-overflow once
        // the degenerate-size early-out was removed to match cv::medianBlur —
        // the reads land in adjacent heap memory and look harmless, so nothing
        // short of a sanitizer would have shown it.
        //
        // Narrow images fall through to the clamped path, which handles them
        // correctly and is not performance-relevant at that size anyway.
        if (y > 0 && y < height - 1 && width >= 3) {
            const uint8_t* r0 = s + static_cast<size_t>(y - 1) * w;
            const uint8_t* r1 = s + static_cast<size_t>(y)     * w;
            const uint8_t* r2 = s + static_cast<size_t>(y + 1) * w;

            drow[0] = median9(r0[0], r0[0], r0[1],
                              r1[0], r1[0], r1[1],
                              r2[0], r2[0], r2[1]);

            for (int x = 1; x < width - 1; ++x) {
                drow[x] = median9(r0[x - 1], r0[x], r0[x + 1],
                                  r1[x - 1], r1[x], r1[x + 1],
                                  r2[x - 1], r2[x], r2[x + 1]);
            }

            const int xe = width - 1;
            drow[xe] = median9(r0[xe - 1], r0[xe], r0[xe],
                               r1[xe - 1], r1[xe], r1[xe],
                               r2[xe - 1], r2[xe], r2[xe]);
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
