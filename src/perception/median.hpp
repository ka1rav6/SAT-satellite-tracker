// perception/median.hpp — 3x3 median filter (design §9.4.1).
//
// ---------------------------------------------------------------------------
// WHY A MEDIAN IS THE RIGHT FIRST STAGE
// ---------------------------------------------------------------------------
// Spec row 21 asks for ~10% salt-and-pepper noise. Design §1.4 works out what
// that means: 30,720 corrupted pixels in a 640x480 frame against a 100-pixel
// beacon — a ratio of 307:1 — and every one of them is a 0 or a 255.
//
// Those are the two facts that decide the whole stage. The impulses are
// ISOLATED SINGLE PIXELS, and they are extreme. A median over a 3x3
// neighbourhood is exactly the operator that is blind to both: a single outlier
// among nine samples cannot move the middle one, while a solid 10x10 blob has
// eight neighbours agreeing with it and passes through essentially untouched.
// A mean would do the opposite — it would spread every impulse over its
// neighbours and destroy nothing.
//
// The measured cost of not doing this is in the CP 4.11 ablation: the
// brightest-pixel detector goes from 0.70 px of centroiding error to 610 px,
// and from 0% false alarms to 92%, when row 21's noise is switched on.
//
// ---------------------------------------------------------------------------
// WHY A SORTING NETWORK
// ---------------------------------------------------------------------------
// Sorting nine values and taking the middle one costs ~25 comparisons and, more
// importantly, BRANCHES — which on a 307,200-pixel frame with data-dependent
// outcomes means a mispredict roughly every other pixel.
//
// A sorting network does a fixed sequence of min/max operations with no
// branches at all. The 19-operation network below is the known minimum for a
// median of nine. Branch-free also means it vectorises: the same 19 operations
// with _mm256_min_epu8 / _mm256_max_epu8 process 32 pixels at once, which is
// where §15's 0.35 ms -> 0.04 ms comes from. The scalar version here stays as
// the reference the SIMD version must match bit-for-bit (§16).

#pragma once

#include <algorithm>
#include <cstdint>
#include <span>

namespace sat {

// ---------------------------------------------------------------------------
// median9 — the 19-operation network.
//
// Each MN(a, b) leaves the smaller value in `a` and the larger in `b`. After
// the sequence, p4 holds the median. The network does NOT fully sort its
// inputs, which is why it can beat the ~25 comparisons a full sort needs: it
// only does the work that constrains the middle element.
//
// The exact sequence is load-bearing — it is not "a" network, it is a specific
// one that has been proved correct. tests/kernels/test_median.cpp proves it
// exhaustively rather than trusting this comment (see the 0-1 principle note
// there), because a single transposed pair would still produce plausible
// output on most inputs.
// ---------------------------------------------------------------------------
#define SAT_MN(a, b) { const uint8_t t_ = std::min((a), (b)); (b) = std::max((a), (b)); (a) = t_; }

[[nodiscard]] inline uint8_t median9(uint8_t p0, uint8_t p1, uint8_t p2,
                                     uint8_t p3, uint8_t p4, uint8_t p5,
                                     uint8_t p6, uint8_t p7, uint8_t p8) noexcept {
    SAT_MN(p1, p2) SAT_MN(p4, p5) SAT_MN(p7, p8)
    SAT_MN(p0, p1) SAT_MN(p3, p4) SAT_MN(p6, p7)
    SAT_MN(p1, p2) SAT_MN(p4, p5) SAT_MN(p7, p8)
    SAT_MN(p0, p3) SAT_MN(p5, p8) SAT_MN(p4, p7)
    SAT_MN(p3, p6) SAT_MN(p1, p4) SAT_MN(p2, p5)
    SAT_MN(p4, p7) SAT_MN(p4, p2) SAT_MN(p6, p4)
    SAT_MN(p4, p2)
    return p4;
}

#undef SAT_MN

/// Number of min/max operations in the network above. Asserted by the tests so
/// an "optimisation" that adds one is noticed.
inline constexpr int kMedian9Ops = 19;

// ---------------------------------------------------------------------------
// median_3x3 — the whole frame.
//
// Border handling is REPLICATE: out-of-range samples take the value of the
// nearest edge pixel. Two reasons, and the second matters more:
//
//   * It matches cv::medianBlur's default, which makes §16's bit-exact oracle
//     comparison possible at all. A different border rule would make every
//     edge pixel differ and the comparison useless.
//   * Design §9.4.5 says of CFAR: "Never skip a border — a beacon near the edge
//     is exactly when you are about to lose it." The same argument applies
//     here. Leaving a border unfiltered would leave a ring of raw impulse noise
//     around the frame, precisely where a target is about to leave the field of
//     view and the tracker most needs a clean measurement.
//
// `src` and `dst` must not alias: the filter reads a 3x3 neighbourhood, so
// writing in place would feed already-filtered values back in.
// ---------------------------------------------------------------------------
void median_3x3(std::span<const uint8_t> src, std::span<uint8_t> dst,
                int width, int height) noexcept;

}  // namespace sat
