// tests/kernels/test_median.cpp — CP 5.1.
//
// "Exhaustive test over a small alphabet proves the network; matches
//  cv::medianBlur bit-for-bit on 1000 random images."
//
// Both halves matter and they check different things. The exhaustive test
// proves the NETWORK computes a median; the OpenCV comparison proves the
// FRAME-LEVEL code — borders, indexing, row handling — agrees with a reference
// implementation everyone already trusts.

#include <doctest/doctest.h>

#include "perception/median.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

#include "core/rng.hpp"

#if SAT_HAVE_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#endif

using namespace sat;

// ===========================================================================
// CP 5.1, part one: prove the network.
// ===========================================================================

TEST_CASE("CP 5.1: the 19-op network is EXHAUSTIVELY proved by the 0-1 principle") {
    // ---------------------------------------------------------------------
    // The 0-1 principle (Knuth, TAOCP vol. 3): a comparator network sorts all
    // inputs if and only if it sorts all inputs drawn from {0, 1}. The same
    // argument extends to a network that computes an order STATISTIC, because
    // "is the output the k-th smallest" is a monotone question.
    //
    // That turns an infinite claim into a finite one: 2^9 = 512 cases. Checking
    // all of them is not a sample, it is a PROOF that this network computes the
    // median of nine values — every value, every ordering.
    //
    // This is why a sorting network is testable in a way that an ad-hoc
    // comparison chain is not, and it is worth far more than a million random
    // trials: a single transposed comparator pair would still produce plausible
    // output on the overwhelming majority of random inputs, and would be caught
    // here instantly.
    // ---------------------------------------------------------------------
    int checked = 0;
    for (unsigned bits = 0; bits < 512u; ++bits) {
        std::array<uint8_t, 9> v{};
        int ones = 0;
        for (int i = 0; i < 9; ++i) {
            v[static_cast<size_t>(i)] = (bits >> i) & 1u ? 1 : 0;
            ones += (bits >> i) & 1u;
        }
        // Median of nine 0/1 values is 1 exactly when five or more are 1.
        const uint8_t expected = ones >= 5 ? 1 : 0;
        const uint8_t got = median9(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]);

        INFO("bit pattern " << bits << " (" << ones << " ones)");
        REQUIRE(got == expected);
        ++checked;
    }
    CHECK(checked == 512);
}

TEST_CASE("CP 5.1: the network agrees with a full sort on random 8-bit inputs") {
    // The 0-1 principle already settles correctness. This is a second,
    // independent check against the obvious reference, over the actual 8-bit
    // domain — cheap insurance against a mistake in the reasoning above rather
    // than in the network.
    Pcg32 rng(4242);
    for (int trial = 0; trial < 200000; ++trial) {
        std::array<uint8_t, 9> v{};
        for (auto& x : v) x = static_cast<uint8_t>(rng.next_below(256));

        std::array<uint8_t, 9> sorted = v;
        std::sort(sorted.begin(), sorted.end());

        const uint8_t got = median9(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]);
        REQUIRE(got == sorted[4]);
    }
}

TEST_CASE("CP 5.1: the network handles ties and saturated values") {
    // Heavily-tied inputs are the norm here, not an edge case: a quantised
    // frame under salt-and-pepper is full of 0s and 255s.
    CHECK(median9(0, 0, 0, 0, 0, 0, 0, 0, 0) == 0);
    CHECK(median9(255, 255, 255, 255, 255, 255, 255, 255, 255) == 255);
    // Four impulses each way cannot move the median off the true value.
    CHECK(median9(0, 0, 0, 0, 42, 255, 255, 255, 255) == 42);
    // A single impulse among eight agreeing neighbours is ignored entirely —
    // the property the whole stage rests on.
    CHECK(median9(100, 100, 100, 100, 255, 100, 100, 100, 100) == 100);
    CHECK(median9(100, 100, 100, 100, 0, 100, 100, 100, 100) == 100);
}

// ===========================================================================
// CP 5.1, part two: the frame-level filter.
// ===========================================================================

TEST_CASE("a single impulse on a flat field is removed completely") {
    constexpr int W = 16, H = 16;
    std::vector<uint8_t> src(W * H, 64), dst(W * H, 0);
    src[static_cast<size_t>(8) * W + 8] = 255;   // salt
    src[static_cast<size_t>(4) * W + 4] = 0;     // pepper

    median_3x3(src, dst, W, H);
    for (size_t i = 0; i < dst.size(); ++i) {
        INFO("pixel " << i);
        REQUIRE(dst[i] == 64);
    }
}

TEST_CASE("a solid blob survives the filter") {
    // The other half of the claim: the median must be blind to impulses AND
    // gentle on a real target. A 10x10 beacon is design §3.2 row 10's default.
    constexpr int W = 40, H = 40;
    std::vector<uint8_t> src(W * H, 20), dst(W * H, 0);
    for (int y = 15; y < 25; ++y) {
        for (int x = 15; x < 25; ++x) src[static_cast<size_t>(y) * W + x] = 200;
    }
    median_3x3(src, dst, W, H);

    // The interior is untouched. Only the one-pixel rim can move, because there
    // the 3x3 window straddles the edge — which is the correct behaviour for a
    // median and costs a fraction of a pixel of centroid, not a detection.
    for (int y = 16; y < 24; ++y) {
        for (int x = 16; x < 24; ++x) {
            INFO("interior (" << x << ", " << y << ")");
            REQUIRE(dst[static_cast<size_t>(y) * W + x] == 200);
        }
    }
    CHECK(dst[static_cast<size_t>(5) * W + 5] == 20);   // background still background
}

TEST_CASE("10% salt and pepper is almost entirely removed") {
    // The realistic case: spec row 21's noise level on a frame with a beacon.
    constexpr int W = 128, H = 128;
    std::vector<uint8_t> clean(W * H, 30), noisy, dst(W * H, 0);
    for (int y = 60; y < 70; ++y) {
        for (int x = 60; x < 70; ++x) clean[static_cast<size_t>(y) * W + x] = 150;
    }
    noisy = clean;

    Pcg32 rng(7);
    int corrupted = 0;
    for (auto& v : noisy) {
        if (rng.next_double() < 0.10) { v = rng.next_bool() ? 255 : 0; ++corrupted; }
    }
    CHECK(corrupted > 1200);

    median_3x3(noisy, dst, W, H);

    int surviving = 0;
    for (size_t i = 0; i < dst.size(); ++i) {
        if (dst[i] == 0 || dst[i] == 255) ++surviving;
    }
    // Survivors are the rare cases where five or more of a nine-pixel window
    // were corrupted the same way — at p = 0.10 that is about 1 in 7000.
    MESSAGE("impulses: " << corrupted << " in, " << surviving << " surviving the median");
    CHECK(surviving < corrupted / 50);

    // And the beacon is still there at full strength.
    CHECK(dst[static_cast<size_t>(65) * W + 65] == 150);
}

TEST_CASE("borders are filtered, not skipped") {
    // Design §9.4.5's argument, applied here: an unfiltered border leaves raw
    // impulse noise exactly where a target is about to leave the field of view.
    constexpr int W = 8, H = 8;
    std::vector<uint8_t> src(W * H, 50), dst(W * H, 0);
    src[0] = 255;                                    // top-left corner
    src[W - 1] = 0;                                  // top-right corner
    src[static_cast<size_t>(H - 1) * W] = 255;       // bottom-left
    src[static_cast<size_t>(3) * W] = 255;           // left edge, mid-height

    median_3x3(src, dst, W, H);
    for (size_t i = 0; i < dst.size(); ++i) {
        INFO("pixel " << i);
        REQUIRE(dst[i] == 50);
    }
}

TEST_CASE("degenerate sizes do not fault") {
    // CP 14.1 fuzzes every parameter; a 1xN frame is the first thing it finds.
    std::vector<uint8_t> a(5, 7), b(5, 0);
    median_3x3(a, b, 5, 1);
    for (uint8_t v : b) CHECK(v == 7);
    median_3x3(a, b, 1, 5);
    for (uint8_t v : b) CHECK(v == 7);

    std::vector<uint8_t> empty;
    median_3x3(empty, empty, 0, 0);          // must not fault
    // Undersized buffers are refused rather than overrun.
    std::vector<uint8_t> small(3, 1), out(3, 9);
    median_3x3(small, out, 100, 100);
    CHECK(out[0] == 9);                      // untouched
}

// ===========================================================================
// CP 5.1, part three: the OpenCV oracle (design §16).
// ===========================================================================

TEST_CASE("CP 5.1: matches cv::medianBlur bit-for-bit on 1000 random images") {
#if !SAT_HAVE_OPENCV
    MESSAGE("SKIPPED: no OpenCV, so the §16 oracle comparison did not run.");
#else
    // Design §4.2 puts OpenCV firmly outside the hot loop but explicitly names
    // it as the "test oracle for every own kernel". This is that: our kernel is
    // compared against an implementation that has been used by everyone for
    // twenty years, on 1000 random images, and must agree on EVERY PIXEL.
    //
    // Bit-for-bit rather than approximately, because a median has no rounding:
    // the output is always one of the input values, so any disagreement is a
    // real difference in which value was selected, not a tolerance question.
    Pcg32 rng(20260913);
    int images = 0;
    size_t worst_diff_count = 0;

    for (int trial = 0; trial < 1000; ++trial) {
        // Deliberately awkward sizes, including ones with no interior row or
        // column, so the border paths are exercised rather than just the fast
        // interior loop.
        const int W = 1 + static_cast<int>(rng.next_below(40));
        const int H = 1 + static_cast<int>(rng.next_below(40));

        std::vector<uint8_t> src(static_cast<size_t>(W) * H);
        // A mix of smooth and impulsive content: a uniformly random image has
        // no structure and would not exercise tie-breaking the way a
        // quantised frame under salt-and-pepper does.
        const bool impulsive = (trial % 2) == 0;
        for (auto& v : src) {
            if (impulsive) {
                const uint32_t r = rng.next_below(10);
                v = r == 0 ? 0 : (r == 1 ? 255 : static_cast<uint8_t>(rng.next_below(64) + 32));
            } else {
                v = static_cast<uint8_t>(rng.next_below(256));
            }
        }

        std::vector<uint8_t> ours(src.size(), 0);
        median_3x3(src, ours, W, H);

        const cv::Mat in(H, W, CV_8UC1, src.data());
        cv::Mat theirs;
        cv::medianBlur(in, theirs, 3);

        REQUIRE(theirs.isContinuous());
        size_t diffs = 0;
        for (size_t i = 0; i < src.size(); ++i) {
            if (ours[i] != theirs.data[i]) ++diffs;
        }
        if (diffs) {
            INFO("trial " << trial << "  " << W << "x" << H
                 << (impulsive ? " impulsive" : " uniform")
                 << "  differing pixels: " << diffs);
            worst_diff_count = std::max(worst_diff_count, diffs);
        }
        REQUIRE(diffs == 0);
        ++images;
    }
    CHECK(images == 1000);
    CHECK(worst_diff_count == 0);
#endif
}
