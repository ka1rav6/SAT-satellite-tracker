// tests/kernels/test_morphology.cpp — CP 5.2.
//
// "Matches cv::morphologyEx bit-for-bit; timing flat as SE varies 5 -> 51."
//
// The second half is the interesting one. van Herk's whole claim is O(1) per
// pixel regardless of structuring-element size, and the way to check a
// complexity claim is to vary the parameter and watch the cost NOT move.

#include <doctest/doctest.h>

#include "core/rng.hpp"
#include "perception/morphology.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <vector>

#if SAT_HAVE_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#endif

using namespace sat;

namespace {

/// Buffers for the kernels under test. In a real run these come from the frame
/// arena; here a vector is fine because the test is not the hot path.
struct Work {
    std::vector<uint8_t> a, b, c, scratch;
    MorphWorkspace ws;

    Work(int w, int h, int k) {
        const size_t n = static_cast<size_t>(w) * static_cast<size_t>(h);
        a.assign(n, 0); b.assign(n, 0); c.assign(n, 0);
        scratch.assign(MorphWorkspace::scratch_bytes(w, h, k), 0);
        ws = MorphWorkspace{a, b, c, scratch};
    }
};

/// Brute-force local minimum with REPLICATE borders — the reference the fast
/// algorithm must reproduce. Deliberately the stupidest possible implementation,
/// because its only job is to be obviously correct.
std::vector<uint8_t> brute_erode(const std::vector<uint8_t>& src,
                                 int w, int h, int k) {
    std::vector<uint8_t> out(src.size(), 0);
    const int half = k / 2;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            uint8_t m = 255;
            for (int dy = -half; dy <= half; ++dy) {
                for (int dx = -half; dx <= half; ++dx) {
                    const int cx = std::clamp(x + dx, 0, w - 1);
                    const int cy = std::clamp(y + dy, 0, h - 1);
                    m = std::min(m, src[static_cast<size_t>(cy) * w + cx]);
                }
            }
            out[static_cast<size_t>(y) * w + x] = m;
        }
    }
    return out;
}

std::vector<uint8_t> brute_dilate(const std::vector<uint8_t>& src,
                                  int w, int h, int k) {
    std::vector<uint8_t> out(src.size(), 0);
    const int half = k / 2;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            uint8_t m = 0;
            for (int dy = -half; dy <= half; ++dy) {
                for (int dx = -half; dx <= half; ++dx) {
                    const int cx = std::clamp(x + dx, 0, w - 1);
                    const int cy = std::clamp(y + dy, 0, h - 1);
                    m = std::max(m, src[static_cast<size_t>(cy) * w + cx]);
                }
            }
            out[static_cast<size_t>(y) * w + x] = m;
        }
    }
    return out;
}

std::vector<uint8_t> random_image(Pcg32& rng, int w, int h) {
    std::vector<uint8_t> v(static_cast<size_t>(w) * h);
    for (auto& x : v) x = static_cast<uint8_t>(rng.next_below(256));
    return v;
}

}  // namespace

// ===========================================================================
// Correctness against a brute-force reference.
// ===========================================================================

TEST_CASE("CP 5.2: van Herk erosion matches brute force at every SE size") {
    Pcg32 rng(31337);
    for (int k : {3, 5, 9, 15, 25, 33, 51}) {
        for (int trial = 0; trial < 6; ++trial) {
            const int w = 12 + static_cast<int>(rng.next_below(40));
            const int h = 12 + static_cast<int>(rng.next_below(40));
            const auto src = random_image(rng, w, h);

            Work work(w, h, k);
            std::vector<uint8_t> got(src.size(), 0);
            erode_rect(src, got, w, h, k, work.a, work.scratch);

            const auto want = brute_erode(src, w, h, k);
            INFO("k=" << k << " size=" << w << "x" << h);
            REQUIRE(got == want);
        }
    }
}

TEST_CASE("CP 5.2: van Herk dilation matches brute force at every SE size") {
    Pcg32 rng(1234);
    for (int k : {3, 5, 9, 15, 25, 33, 51}) {
        for (int trial = 0; trial < 6; ++trial) {
            const int w = 12 + static_cast<int>(rng.next_below(40));
            const int h = 12 + static_cast<int>(rng.next_below(40));
            const auto src = random_image(rng, w, h);

            Work work(w, h, k);
            std::vector<uint8_t> got(src.size(), 0);
            dilate_rect(src, got, w, h, k, work.a, work.scratch);

            const auto want = brute_dilate(src, w, h, k);
            INFO("k=" << k << " size=" << w << "x" << h);
            REQUIRE(got == want);
        }
    }
}

TEST_CASE("an SE larger than the image still behaves") {
    // With REPLICATE borders, an SE wider than the frame reduces to the global
    // extremum. CP 14.1 will produce this; it must not fault.
    Pcg32 rng(5);
    const int w = 9, h = 7, k = 51;
    const auto src = random_image(rng, w, h);
    Work work(w, h, k);
    std::vector<uint8_t> got(src.size(), 0);
    erode_rect(src, got, w, h, k, work.a, work.scratch);
    const uint8_t global_min = *std::min_element(src.begin(), src.end());
    for (uint8_t v : got) CHECK(v == global_min);
}

// ===========================================================================
// The top-hat, which is what this is all for.
// ===========================================================================

TEST_CASE("the top-hat keeps a small beacon and removes a large gradient") {
    // The whole argument for background removal, in one image: a slow
    // illumination ramp (which the opening keeps, and the subtraction removes)
    // with a 10 px beacon on top (which the opening removes, and the
    // subtraction therefore keeps).
    constexpr int W = 96, H = 96;
    const int k = structuring_element_size(10);

    std::vector<uint8_t> src(W * H);
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            // A gradient spanning most of the 8-bit range.
            src[static_cast<size_t>(y) * W + x] = static_cast<uint8_t>(20 + (x + y));
        }
    }
    // A beacon, 60 grey levels above its local background.
    for (int y = 40; y < 50; ++y) {
        for (int x = 40; x < 50; ++x) {
            src[static_cast<size_t>(y) * W + x] =
                static_cast<uint8_t>(std::min(255, 20 + (x + y) + 60));
        }
    }

    Work work(W, H, k);
    std::vector<int16_t> th(W * H, 0);
    top_hat(src, th, W, H, k, work.ws);

    // The beacon survives at close to its full contrast.
    const int centre = th[static_cast<size_t>(45) * W + 45];
    INFO("beacon top-hat value: " << centre);
    CHECK(centre >= 55);

    // The gradient is gone: in the INTERIOR the top-hat is ~0 everywhere, even
    // though the raw image varies by nearly 200 grey levels across the frame.
    //
    // "Interior" means more than half a structuring element from the edge. See
    // the next test for why that qualifier is necessary and what it costs.
    const int margin = k / 2 + 1;
    int worst_background = 0;
    for (int y = margin; y < H - margin; ++y) {
        for (int x = margin; x < W - margin; ++x) {
            if (x >= 34 && x < 56 && y >= 34 && y < 56) continue;   // near the beacon
            worst_background = std::max(worst_background,
                                        std::abs(static_cast<int>(th[static_cast<size_t>(y) * W + x])));
        }
    }
    INFO("worst interior background residual: " << worst_background);
    CHECK(worst_background <= 2);

    // And the top-hat is non-negative, as the mathematics says it must be.
    for (int16_t v : th) REQUIRE(v >= 0);
}

TEST_CASE("the top-hat has a known border artifact, and here is how big it is") {
    // ---------------------------------------------------------------------
    // A REAL PROPERTY, MEASURED RATHER THAN HIDDEN
    // ---------------------------------------------------------------------
    // With REPLICATE borders, the opening is not exact within half a
    // structuring element of the frame edge: the erosion's window clamps, so its
    // minimum is higher than it should be, and the dilation then overshoots.
    // On a strong gradient the residual reaches tens of grey levels.
    //
    // This matters, and it is the opposite of academic. Design §9.4.5 says of
    // CFAR: "Never skip a border — a beacon near the edge is exactly when you
    // are about to lose it." A top-hat that manufactures a bright rim would
    // produce false candidates in precisely that region, and the tracker would
    // chase the frame edge instead of the target it was about to lose.
    //
    // Two consequences, both for later stages:
    //   * CFAR (§9.4.5) measures its threshold from the local image, so a raised
    //     border raises the threshold with it — the artifact is largely
    //     self-cancelling there, which is a further argument for CFAR over a
    //     fixed threshold.
    //   * The candidate gate (§9.4.7) should treat the outer k/2 ring as
    //     lower-confidence rather than discarding it, since discarding is what
    //     §9.4.5 forbids.
    //
    // The number is pinned here so a change in the border rule shows up as a
    // test failure rather than as mysterious edge detections at Stage 9.
    constexpr int W = 96, H = 96;
    const int k = structuring_element_size(10);

    std::vector<uint8_t> src(W * H);
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            src[static_cast<size_t>(y) * W + x] = static_cast<uint8_t>(20 + (x + y));
        }
    }

    Work work(W, H, k);
    std::vector<int16_t> th(W * H, 0);
    top_hat(src, th, W, H, k, work.ws);

    const int margin = k / 2 + 1;
    int worst_border = 0, worst_interior = 0;
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const int v = std::abs(static_cast<int>(th[static_cast<size_t>(y) * W + x]));
            const bool border = (x < margin || x >= W - margin ||
                                 y < margin || y >= H - margin);
            (border ? worst_border : worst_interior) =
                std::max(border ? worst_border : worst_interior, v);
        }
    }

    MESSAGE("top-hat on a full-range gradient, SE " << k << ": "
            << "interior residual " << worst_interior
            << ", border residual " << worst_border
            << " (border ring is " << margin << " px wide)");

    // The interior is clean...
    CHECK(worst_interior <= 2);
    // ...and the border is not, by a wide and reproducible margin.
    CHECK(worst_border > 10);
    // But it is bounded well below a real beacon's contrast (60 levels in the
    // test above), so a gate keyed on amplitude is not swamped by it.
    CHECK(worst_border < 40);
}

TEST_CASE("structuring element size follows design 9.4.2 and is always odd") {
    // k = target*2 + 5, clamped to [15, 51].
    CHECK(structuring_element_size(5)  == 15);   // 15 -> clamps at the floor
    CHECK(structuring_element_size(10) == 25);
    CHECK(structuring_element_size(20) == 45);
    CHECK(structuring_element_size(1)  == 15);   // below the floor
    CHECK(structuring_element_size(99) == 51);   // above the ceiling

    // Odd at every input: an even SE has no centre pixel, so the opening would
    // shift the image half a pixel and move every centroid.
    for (int t = 0; t <= 60; ++t) {
        INFO("target size " << t);
        CHECK(structuring_element_size(t) % 2 == 1);
    }
}

// ===========================================================================
// CP 5.2's timing claim: O(1) per pixel, independent of SE size.
// ===========================================================================

TEST_CASE("CP 5.2: cost is flat as the SE grows from 5 to 51") {
    // van Herk's entire justification. A naive local minimum costs O(k^2) per
    // pixel in 2-D, so 51 would be a hundred times 5; this must be flat.
    //
    // Timing in a unit test is normally a bad idea, and this one is written to
    // be robust about it: it compares a RATIO rather than an absolute, takes the
    // best of several runs to reject scheduler noise, and allows a wide margin.
    // It is checking a complexity class, not a benchmark figure — an O(k) or
    // O(k^2) implementation would be off by 10x or 100x, not by the 2x this
    // permits.
    constexpr int W = 320, H = 240;
    Pcg32 rng(99);
    const auto src = random_image(rng, W, H);

    auto best_time_us = [&](int k) {
        Work work(W, H, k);
        std::vector<uint8_t> dst(src.size(), 0);
        double best = 1e30;
        for (int rep = 0; rep < 5; ++rep) {
            const auto t0 = std::chrono::steady_clock::now();
            erode_rect(src, dst, W, H, k, work.a, work.scratch);
            const auto t1 = std::chrono::steady_clock::now();
            best = std::min(best, std::chrono::duration<double, std::micro>(t1 - t0).count());
        }
        return best;
    };

    const double t5  = best_time_us(5);
    const double t51 = best_time_us(51);
    MESSAGE("erode 320x240: k=5 -> " << t5 << " us, k=51 -> " << t51
            << " us, ratio " << (t51 / t5));

    CHECK(t5 > 0.0);
    // A naive implementation would be ~100x here. Anything under 2.5x is flat
    // for practical purposes and cannot be confused with O(k) or O(k^2).
    CHECK(t51 / t5 < 2.5);
}

// ===========================================================================
// The §16 OpenCV oracle.
// ===========================================================================

TEST_CASE("CP 5.2: matches cv::erode, cv::dilate and cv::morphologyEx bit-for-bit") {
#if !SAT_HAVE_OPENCV
    MESSAGE("SKIPPED: no OpenCV, so the §16 oracle comparison did not run.");
#else
    Pcg32 rng(777);
    for (int k : {3, 5, 15, 25, 51}) {
        for (int trial = 0; trial < 12; ++trial) {
            const int w = 20 + static_cast<int>(rng.next_below(60));
            const int h = 20 + static_cast<int>(rng.next_below(60));
            const auto src = random_image(rng, w, h);

            Work work(w, h, k);
            const cv::Mat in(h, w, CV_8UC1, const_cast<uint8_t*>(src.data()));
            const cv::Mat se = cv::getStructuringElement(cv::MORPH_RECT, {k, k});

            SUBCASE("") {}   // keep doctest from re-running the loop per subcase

            std::vector<uint8_t> ours(src.size(), 0);
            cv::Mat theirs;

            erode_rect(src, ours, w, h, k, work.a, work.scratch);
            cv::erode(in, theirs, se, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
            INFO("erode k=" << k << " " << w << "x" << h);
            REQUIRE(std::equal(ours.begin(), ours.end(), theirs.data));

            dilate_rect(src, ours, w, h, k, work.a, work.scratch);
            cv::dilate(in, theirs, se, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
            INFO("dilate k=" << k << " " << w << "x" << h);
            REQUIRE(std::equal(ours.begin(), ours.end(), theirs.data));

            open_rect(src, ours, w, h, k, work.ws);
            cv::morphologyEx(in, theirs, cv::MORPH_OPEN, se, cv::Point(-1, -1), 1,
                             cv::BORDER_REPLICATE);
            INFO("open k=" << k << " " << w << "x" << h);
            REQUIRE(std::equal(ours.begin(), ours.end(), theirs.data));
        }
    }
#endif
}
