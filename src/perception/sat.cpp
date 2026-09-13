#include "perception/sat.hpp"

#include <algorithm>

namespace sat {
namespace {

/// The one-pass builder, shared by both pixel types.
///
/// Row-major with a running row total: each output is the cell to the left plus
/// the running sum of the row above, which is the standard recurrence
///     S(x, y) = I(x, y) + S(x-1, y) + S(x, y-1) - S(x-1, y-1)
/// rewritten to avoid the fourth term by carrying the row accumulator. That is
/// one add instead of three, and it keeps the whole thing sequential in memory.
template <typename T>
void build(const T* img, int width, int height, SummedArea& out) noexcept {
    const int W1 = width + 1;
    out.W = width;
    out.H = height;

    // The zero border. It is what lets box_sum read x0 = 0 or y0 = 0 without a
    // branch in the innermost loop of the matched filter and CFAR.
    for (int x = 0; x <= width; ++x) {
        out.s[static_cast<size_t>(x)]  = 0;
        out.s2[static_cast<size_t>(x)] = 0;
    }

    for (int y = 0; y < height; ++y) {
        const size_t row_out  = static_cast<size_t>(y + 1) * W1;
        const size_t row_prev = static_cast<size_t>(y) * W1;
        out.s[row_out]  = 0;
        out.s2[row_out] = 0;

        int64_t  run    = 0;
        uint64_t run_sq = 0;
        const T* src = img + static_cast<size_t>(y) * static_cast<size_t>(width);

        for (int x = 0; x < width; ++x) {
            const int64_t v = static_cast<int64_t>(src[x]);
            run    += v;
            // v*v is non-negative even for a negative int16, so the cast to
            // unsigned is safe and the square never overflows: 32767^2 is 1.07e9
            // and the running total is bounded by 4.3e15 for a full 2000x2000
            // frame, against uint64's 1.8e19.
            run_sq += static_cast<uint64_t>(v * v);
            out.s[row_out + static_cast<size_t>(x + 1)] =
                out.s[row_prev + static_cast<size_t>(x + 1)] + run;
            out.s2[row_out + static_cast<size_t>(x + 1)] =
                out.s2[row_prev + static_cast<size_t>(x + 1)] + run_sq;
        }
    }
}

}  // namespace

void build_sat(std::span<const int16_t> img, int width, int height,
               SummedArea& out) noexcept {
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    const size_t need = SummedArea::elements(width, height);
    if (width <= 0 || height <= 0 || img.size() < n ||
        out.s.size() < need || out.s2.size() < need) return;
    build(img.data(), width, height, out);
}

void build_sat(std::span<const uint8_t> img, int width, int height,
               SummedArea& out) noexcept {
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    const size_t need = SummedArea::elements(width, height);
    if (width <= 0 || height <= 0 || img.size() < n ||
        out.s.size() < need || out.s2.size() < need) return;
    build(img.data(), width, height, out);
}

int64_t SummedArea::box_sum_clipped(int x0, int y0, int x1, int y1,
                                    int& count) const noexcept {
    x0 = std::clamp(x0, 0, W);  x1 = std::clamp(x1, 0, W);
    y0 = std::clamp(y0, 0, H);  y1 = std::clamp(y1, 0, H);
    count = std::max(0, x1 - x0) * std::max(0, y1 - y0);
    if (count == 0) return 0;
    return box_sum(x0, y0, x1, y1);
}

uint64_t SummedArea::box_sum_sq_clipped(int x0, int y0, int x1, int y1,
                                        int& count) const noexcept {
    x0 = std::clamp(x0, 0, W);  x1 = std::clamp(x1, 0, W);
    y0 = std::clamp(y0, 0, H);  y1 = std::clamp(y1, 0, H);
    count = std::max(0, x1 - x0) * std::max(0, y1 - y0);
    if (count == 0) return 0;
    return box_sum_sq(x0, y0, x1, y1);
}

}  // namespace sat
