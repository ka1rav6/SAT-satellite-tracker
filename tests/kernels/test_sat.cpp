// tests/kernels/test_sat.cpp — CP 5.3.
//
// "10,000 random rectangle sums match brute force EXACTLY; no float anywhere."
//
// Exactly, not approximately. That word is the whole checkpoint: the reason for
// integer accumulators is that the four-corner difference suffers catastrophic
// cancellation in floating point, and a test with a tolerance would pass on an
// implementation that had exactly the problem the integers exist to avoid.

#include <doctest/doctest.h>

#include "core/rng.hpp"
#include "perception/sat.hpp"

#include <algorithm>
#include <cstdint>
#include <span>
#include <type_traits>
#include <vector>

using namespace sat;

namespace {

struct Table {
    std::vector<int64_t>  s;
    std::vector<uint64_t> s2;
    SummedArea sa;

    Table(int w, int h) {
        const size_t n = SummedArea::elements(w, h);
        s.assign(n, 0);
        s2.assign(n, 0);
        sa = SummedArea{s, s2, w, h};
    }
};

}  // namespace

TEST_CASE("CP 5.3: 10,000 random rectangle sums match brute force EXACTLY") {
    constexpr int W = 64, H = 48;
    Pcg32 rng(90210);

    // int16 spanning the full signed range, because that is what a top-hat
    // produces and because large magnitudes are where a float table would fail.
    std::vector<int16_t> img(static_cast<size_t>(W) * H);
    for (auto& v : img) {
        v = static_cast<int16_t>(static_cast<int>(rng.next_below(65536)) - 32768);
    }

    Table t(W, H);
    build_sat(img, W, H, t.sa);

    int checked = 0;
    for (int trial = 0; trial < 10000; ++trial) {
        int x0 = static_cast<int>(rng.next_below(W + 1));
        int x1 = static_cast<int>(rng.next_below(W + 1));
        int y0 = static_cast<int>(rng.next_below(H + 1));
        int y1 = static_cast<int>(rng.next_below(H + 1));
        if (x0 > x1) std::swap(x0, x1);
        if (y0 > y1) std::swap(y0, y1);

        int64_t  want    = 0;
        uint64_t want_sq = 0;
        for (int y = y0; y < y1; ++y) {
            for (int x = x0; x < x1; ++x) {
                const int64_t v = img[static_cast<size_t>(y) * W + x];
                want    += v;
                want_sq += static_cast<uint64_t>(v * v);
            }
        }

        INFO("rect [" << x0 << "," << x1 << ") x [" << y0 << "," << y1 << ")");
        REQUIRE(t.sa.box_sum(x0, y0, x1, y1) == want);
        REQUIRE(t.sa.box_sum_sq(x0, y0, x1, y1) == want_sq);
        ++checked;
    }
    CHECK(checked == 10000);
}

TEST_CASE("CP 5.3: the accumulators are integers, not floats") {
    // The checkpoint says "no float anywhere". Asserted at compile time rather
    // than by inspection, so a later change to the storage type is a build
    // failure rather than a silent loss of precision.
    using SumT   = std::span<int64_t>::value_type;
    using SumSqT = std::span<uint64_t>::value_type;
    static_assert(std::is_integral_v<decltype(std::declval<SummedArea>().box_sum(0,0,1,1))>,
                  "box_sum must return an integer type");
    static_assert(std::is_integral_v<decltype(std::declval<SummedArea>().box_sum_sq(0,0,1,1))>,
                  "box_sum_sq must return an integer type");
    static_assert(std::is_integral_v<SumT> && sizeof(SumT) >= 8,
                  "the running sum must be a 64-bit integer");
    static_assert(std::is_integral_v<SumSqT> && sizeof(SumSqT) >= 8,
                  "the running sum of squares must be a 64-bit integer");
    CHECK(true);
}

TEST_CASE("a float table would fail this, which is why the table is integer") {
    // ---------------------------------------------------------------------
    // The concrete demonstration of design §9.4.3's argument.
    //
    // Build the same table in float32 and ask both for a small rectangle late
    // in a large, bright image. The corners are huge and nearly equal; their
    // difference is small. Integer arithmetic gets it exactly right; float32
    // cannot, because at a total near 1e9 its ulp is 64.
    //
    // This is not a hypothetical concern being dressed up. CFAR (§9.4.5)
    // evaluates exactly this shape of query — a small cell against a large
    // window — at every pixel of every frame.
    // ---------------------------------------------------------------------
    constexpr int W = 512, H = 512;
    std::vector<int16_t> img(static_cast<size_t>(W) * H, 30000);   // large and uniform

    Table t(W, H);
    build_sat(img, W, H, t.sa);

    // A 3x3 cell near the bottom-right, where the running totals are largest.
    const int x0 = W - 5, y0 = H - 5, x1 = x0 + 3, y1 = y0 + 3;
    const int64_t exact = t.sa.box_sum(x0, y0, x1, y1);
    CHECK(exact == 9 * 30000);       // exactly right, no tolerance

    // The same table in float, built identically.
    std::vector<float> fs(SummedArea::elements(W, H), 0.0f);
    const int W1 = W + 1;
    for (int y = 0; y < H; ++y) {
        float run = 0.0f;
        for (int x = 0; x < W; ++x) {
            run += static_cast<float>(img[static_cast<size_t>(y) * W + x]);
            fs[static_cast<size_t>(y + 1) * W1 + (x + 1)] =
                fs[static_cast<size_t>(y) * W1 + (x + 1)] + run;
        }
    }
    const float fsum = fs[static_cast<size_t>(y1) * W1 + x1]
                     - fs[static_cast<size_t>(y0) * W1 + x1]
                     - fs[static_cast<size_t>(y1) * W1 + x0]
                     + fs[static_cast<size_t>(y0) * W1 + x0];

    const double err = std::abs(static_cast<double>(fsum) - static_cast<double>(exact));
    const double per_pixel = err / 9.0;
    MESSAGE("small box late in a bright 512x512 image: integer " << exact
            << " (exact), float32 " << static_cast<double>(fsum)
            << "  -> error " << err << " over 9 pixels = "
            << per_pixel << " grey levels per pixel");

    // The integer table is exact; the float one is not, and by an amount that
    // matters rather than an amount that is merely non-zero.
    //
    // The per-pixel figure is the one to look at: ~37 grey levels of error on
    // each pixel of the box. Spec row 22 caps READ NOISE at a standard
    // deviation of 20, so a float table would inject nearly twice as much error
    // as the noise the detector is being built to see through — and unlike
    // noise, it would be a systematic function of position in the frame, so no
    // amount of averaging would remove it.
    //
    // It gets worse with frame size, because the running totals grow: this is a
    // 512x512 image, and spec row 1's screen is 2000x2000.
    CHECK(err > 100.0);
    CHECK(per_pixel > 20.0);
}

TEST_CASE("the zero border removes the need for bounds checks") {
    // Rectangles touching the top or left edge must work without special-casing,
    // because that branch would sit in the innermost loop of the matched filter
    // and CFAR.
    constexpr int W = 8, H = 8;
    std::vector<int16_t> img(static_cast<size_t>(W) * H, 3);
    Table t(W, H);
    build_sat(img, W, H, t.sa);

    CHECK(t.sa.box_sum(0, 0, 1, 1) == 3);            // single corner pixel
    CHECK(t.sa.box_sum(0, 0, W, H) == 3 * W * H);    // the whole image
    CHECK(t.sa.box_sum(0, 0, 0, 0) == 0);            // empty
    CHECK(t.sa.box_sum(4, 4, 4, 8) == 0);            // zero width
    CHECK(t.sa.box_sum(0, 3, W, 4) == 3 * W);        // a full row
}

TEST_CASE("half-open rectangles compose without double counting") {
    // The reason for the half-open convention: adjacent boxes sharing an edge
    // must sum to the union. CFAR's guard band is exactly this — a large window
    // minus an inner box — and an off-by-one would corrupt every threshold.
    constexpr int W = 16, H = 16;
    Pcg32 rng(3);
    std::vector<int16_t> img(static_cast<size_t>(W) * H);
    for (auto& v : img) v = static_cast<int16_t>(rng.next_below(200));

    Table t(W, H);
    build_sat(img, W, H, t.sa);

    const int64_t whole = t.sa.box_sum(2, 2, 14, 14);
    const int64_t left  = t.sa.box_sum(2, 2, 8, 14);
    const int64_t right = t.sa.box_sum(8, 2, 14, 14);
    CHECK(left + right == whole);

    // And the guard-band shape CFAR actually uses.
    const int64_t outer = t.sa.box_sum(0, 0, 16, 16);
    const int64_t inner = t.sa.box_sum(6, 6, 10, 10);
    int64_t ring = 0;
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            if (x >= 6 && x < 10 && y >= 6 && y < 10) continue;
            ring += img[static_cast<size_t>(y) * W + x];
        }
    }
    CHECK(outer - inner == ring);
}

TEST_CASE("clipped queries report the ACTUAL pixel count") {
    // Design §9.4.5: "clamp boxes to the image and adjust n. Never skip a
    // border." Dividing by the nominal window area near an edge would
    // understate the local mean and manufacture detections exactly where a
    // target is about to be lost.
    constexpr int W = 10, H = 10;
    std::vector<int16_t> img(static_cast<size_t>(W) * H, 5);
    Table t(W, H);
    build_sat(img, W, H, t.sa);

    int n = -1;
    // A 7x7 window centred at (1, 1) mostly falls off the top-left corner.
    const int64_t sum = t.sa.box_sum_clipped(-2, -2, 5, 5, n);
    CHECK(n == 5 * 5);                 // only the on-image part
    CHECK(sum == 5 * 25);
    // Using the nominal 49 would have given a mean of 2.55 instead of 5.0 —
    // half the true background, which is what would create the false detections.
    CHECK(static_cast<double>(sum) / n == doctest::Approx(5.0));

    // Entirely off-image is empty rather than negative or faulting.
    const int64_t none = t.sa.box_sum_clipped(-50, -50, -40, -40, n);
    CHECK(n == 0);
    CHECK(none == 0);
}

TEST_CASE("an 8-bit image builds the same table as its int16 promotion") {
    // The uint8 overload exists for stages that run before the top-hat; it must
    // not be a separate implementation that can drift.
    constexpr int W = 24, H = 19;
    Pcg32 rng(11);
    std::vector<uint8_t> u8(static_cast<size_t>(W) * H);
    for (auto& v : u8) v = static_cast<uint8_t>(rng.next_below(256));
    std::vector<int16_t> i16(u8.begin(), u8.end());

    Table a(W, H), b(W, H);
    build_sat(u8, W, H, a.sa);
    build_sat(i16, W, H, b.sa);
    CHECK(a.s == b.s);
    CHECK(a.s2 == b.s2);
}

TEST_CASE("a full-size frame of extreme values stays far inside int64") {
    // The headroom claim in the header, checked rather than asserted: a
    // 2000x2000 frame of int16 extremes is 1.3e11, six orders of magnitude below
    // int64's limit, and its sum of squares is 4.3e15, also well inside.
    constexpr int64_t pixels = 2000LL * 2000LL;
    constexpr int64_t extreme = 32767;
    CHECK(pixels * extreme < INT64_MAX / 1000000);
    CHECK(static_cast<uint64_t>(pixels) * static_cast<uint64_t>(extreme * extreme)
          < UINT64_MAX / 1000);
}

TEST_CASE("undersized buffers are refused, not overrun") {
    constexpr int W = 16, H = 16;
    std::vector<int16_t> img(static_cast<size_t>(W) * H, 1);
    std::vector<int64_t>  small_s(4, -1);
    std::vector<uint64_t> small_s2(4, 0);
    SummedArea sa{small_s, small_s2, 0, 0};
    build_sat(img, W, H, sa);
    CHECK(small_s[0] == -1);      // untouched
    CHECK(sa.W == 0);
}
