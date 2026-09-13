// perception/sat.hpp — summed-area tables (design §9.4.3, "the keystone").
//
// ---------------------------------------------------------------------------
// WHY THIS IS THE KEYSTONE
// ---------------------------------------------------------------------------
// A summed-area table turns "sum of any rectangle" into four lookups and three
// additions, independent of the rectangle's size. Two stages depend on that
// being O(1), and neither would be affordable otherwise:
//
//   matched filter (§9.4.4)  the optimal detector for a square beacon IS a box
//                            sum. Six scales cost 24 lookups per pixel instead
//                            of six convolutions.
//   CFAR (§9.4.5)            needs a local mean AND a local variance over a
//                            61x61 window with a 31x31 guard band — 8 lookups
//                            for the mean, 8 more for the variance, rather than
//                            3721 samples per pixel.
//
// Without this, CFAR alone would be ~1.1 billion samples per frame. With it,
// the whole detection stage fits in §15's budget.
//
// ---------------------------------------------------------------------------
// INTEGER ACCUMULATORS, NEVER FLOAT. THIS IS NOT NEGOTIABLE.
// ---------------------------------------------------------------------------
// Design §9.4.3 is explicit, and there are two independent reasons:
//
// 1. PRECISION. A summed-area table is evaluated by the four-corner difference
//    s[y1][x1] - s[y0][x1] - s[y1][x0] + s[y0][x0]. Those four numbers are the
//    running total of everything above-left, so for a small rectangle late in
//    the image they are large and nearly equal — and their difference is small.
//    That is catastrophic cancellation. With float32, a total near 1e9 has an
//    ulp of 64, so a box sum that should be 300 could come back as anything in
//    a ±128 band. The answer would be noise.
//
// 2. REPRODUCIBILITY (INV-3). Integer addition is associative and exact, so the
//    table is bit-identical on every machine and at every optimisation level.
//    Floating-point addition is neither, and the compiler is free to reassociate
//    a reduction — which is exactly what -ffp-contract=off is there to prevent
//    elsewhere, and which we simply avoid here by not using floats at all.
//
// int64_t is ample and the margin is worth stating: the worst case is a full
// 2000x2000 frame of int16 values at their extreme, 4e6 * 32767 = 1.3e11, which
// is six orders of magnitude below int64's 9.2e18. The sum-of-squares table is
// uint64_t for the same reason: 4e6 * 32767^2 = 4.3e15, still far inside range.

#pragma once

#include <cstdint>
#include <span>

namespace sat {

// ---------------------------------------------------------------------------
// SummedArea — a (W+1) x (H+1) table over a W x H image.
//
// The extra row and column are zeros, which removes every bounds check from
// box_sum: a rectangle touching the top or left edge reads the zero border
// instead of needing a branch. That is worth more than it sounds — the branch
// would be in the innermost loop of both the matched filter and CFAR.
// ---------------------------------------------------------------------------
struct SummedArea {
    std::span<int64_t>  s;    ///< running sum,        (W+1)*(H+1)
    std::span<uint64_t> s2;   ///< running sum of x^2, (W+1)*(H+1)
    int W = 0, H = 0;

    /// Elements needed in `s` and `s2` for a width x height image.
    [[nodiscard]] static constexpr size_t elements(int width, int height) noexcept {
        return static_cast<size_t>(width + 1) * static_cast<size_t>(height + 1);
    }

    /// Sum over the half-open rectangle [x0, x1) x [y0, y1).
    ///
    /// Half-open because it composes: adjacent rectangles that share an edge do
    /// not double-count it, and a rectangle's width is simply x1 - x0. The four
    /// corners are exact integers, so the subtraction is exact.
    [[nodiscard]] int64_t box_sum(int x0, int y0, int x1, int y1) const noexcept {
        const int W1 = W + 1;
        return s[static_cast<size_t>(y1) * W1 + x1]
             - s[static_cast<size_t>(y0) * W1 + x1]
             - s[static_cast<size_t>(y1) * W1 + x0]
             + s[static_cast<size_t>(y0) * W1 + x0];
    }

    /// Sum of squares over the same rectangle. Feeds CFAR's variance.
    [[nodiscard]] uint64_t box_sum_sq(int x0, int y0, int x1, int y1) const noexcept {
        const int W1 = W + 1;
        return s2[static_cast<size_t>(y1) * W1 + x1]
             - s2[static_cast<size_t>(y0) * W1 + x1]
             - s2[static_cast<size_t>(y1) * W1 + x0]
             + s2[static_cast<size_t>(y0) * W1 + x0];
    }

    /// Same, with the rectangle clipped to the image first.
    ///
    /// Design §9.4.5: "Edges: clamp boxes to the image and adjust n. Never skip
    /// a border." The count is returned so the caller can divide by the ACTUAL
    /// number of pixels rather than the nominal window area — using the nominal
    /// area near an edge would understate the local mean and manufacture
    /// detections exactly where §9.4.5 says a target is about to be lost.
    [[nodiscard]] int64_t box_sum_clipped(int x0, int y0, int x1, int y1,
                                          int& count) const noexcept;
    [[nodiscard]] uint64_t box_sum_sq_clipped(int x0, int y0, int x1, int y1,
                                              int& count) const noexcept;
};

/// Build both tables from an int16 image in a single pass.
///
/// One pass rather than two because the second table is free while the row is
/// already in cache: the sum-of-squares costs one multiply and one add per
/// pixel, against the cache miss that a separate pass would pay again.
void build_sat(std::span<const int16_t> img, int width, int height,
               SummedArea& out) noexcept;

/// Build from an 8-bit image, for stages that run before the top-hat.
void build_sat(std::span<const uint8_t> img, int width, int height,
               SummedArea& out) noexcept;

}  // namespace sat
