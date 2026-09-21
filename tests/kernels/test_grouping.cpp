// tests/kernels/test_grouping.cpp — CP 5.6 and CP 5.8.
//
// "Label counts match cv::connectedComponents on 1000 random masks."
// "Centre of mass over the top-hat: on a clean frame, recovers a known
//  sub-pixel position to within 0.05 px."

#include <doctest/doctest.h>

#include "camera/splat.hpp"
#include "core/rng.hpp"
#include "perception/grouping.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#if SAT_HAVE_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#endif

using namespace sat;

namespace {

struct Work {
    std::vector<Run>     runs;
    std::vector<int32_t> parent, rank;
    GroupingWorkspace ws;

    Work(int w, int h) {
        // Worst case: alternating set/clear pixels gives one run every two
        // columns on every row.
        const size_t max_runs = static_cast<size_t>(w / 2 + 1) * static_cast<size_t>(h);
        runs.resize(max_runs);
        parent.resize(max_runs);
        rank.resize(max_runs);
        ws = GroupingWorkspace{runs, parent, rank};
    }
};

}  // namespace

// ===========================================================================
// CP 5.6 — connectivity
// ===========================================================================

TEST_CASE("separate blobs stay separate and touching ones merge") {
    constexpr int W = 32, H = 32;
    std::vector<uint8_t> mask(W * H, 0);
    std::vector<int16_t> wt(W * H, 100);

    auto set = [&](int x, int y) { mask[static_cast<size_t>(y) * W + x] = 1; };

    // Two clearly separate squares.
    for (int y = 2; y < 6; ++y) for (int x = 2; x < 6; ++x) set(x, y);
    for (int y = 20; y < 24; ++y) for (int x = 20; x < 24; ++x) set(x, y);

    Work work(W, H);
    std::vector<BlobAccum> blobs;
    CHECK(group_components(mask, wt, W, H, work.ws, blobs) == 2);
    CHECK(blobs[0].n == 16);
    CHECK(blobs[1].n == 16);

    // Now connect them diagonally, one pixel at a time. 8-connectivity means a
    // corner touch merges.
    std::fill(mask.begin(), mask.end(), 0);
    set(5, 5);
    set(6, 6);
    blobs.clear();
    CHECK(group_components(mask, wt, W, H, work.ws, blobs) == 1);
    CHECK(blobs[0].n == 2);

    // A knight's-move gap does not.
    std::fill(mask.begin(), mask.end(), 0);
    set(5, 5);
    set(7, 6);
    blobs.clear();
    CHECK(group_components(mask, wt, W, H, work.ws, blobs) == 2);
}

TEST_CASE("a U shape is one component, which needs the merge to work") {
    // The classic case that separates a correct union-find from a naive
    // single-pass labeller: the two arms get different labels on the way down
    // and must be merged when the base connects them.
    constexpr int W = 16, H = 16;
    std::vector<uint8_t> mask(W * H, 0);
    std::vector<int16_t> wt(W * H, 50);
    auto set = [&](int x, int y) { mask[static_cast<size_t>(y) * W + x] = 1; };

    for (int y = 2; y < 10; ++y) { set(3, y); set(8, y); }   // two arms
    for (int x = 3; x <= 8; ++x) set(x, 10);                 // the base

    Work work(W, H);
    std::vector<BlobAccum> blobs;
    CHECK(group_components(mask, wt, W, H, work.ws, blobs) == 1);
    CHECK(blobs[0].n == 8 + 8 + 6);
    CHECK(blobs[0].x0 == 3);
    CHECK(blobs[0].x1 == 8);
    CHECK(blobs[0].y0 == 2);
    CHECK(blobs[0].y1 == 10);
}

TEST_CASE("CP 5.6: label counts match cv::connectedComponents on 1000 random masks") {
#if !SAT_HAVE_OPENCV
    MESSAGE("SKIPPED: no OpenCV, so the §16 oracle comparison did not run.");
#else
    // Design §16's oracle for this kernel. The comparison is on the COUNT of
    // components, because the label VALUES are an implementation detail — ours
    // are assigned in order of first appearance in raster order, OpenCV's by its
    // own scheme — while the count is the thing both must agree on.
    Pcg32 rng(12345);
    int images = 0;

    for (int trial = 0; trial < 1000; ++trial) {
        const int W = 8 + static_cast<int>(rng.next_below(56));
        const int H = 8 + static_cast<int>(rng.next_below(56));

        // Sparse masks, like the ones CFAR actually produces. A dense random
        // mask percolates into one giant component and tests nothing; sweeping
        // the density exercises isolated pixels through to near-percolation.
        const double density = 0.02 + 0.28 * (static_cast<double>(trial % 10) / 9.0);

        std::vector<uint8_t> mask(static_cast<size_t>(W) * H, 0);
        for (auto& m : mask) m = rng.next_double() < density ? 1 : 0;
        std::vector<int16_t> wt(mask.size(), 10);

        Work work(W, H);
        std::vector<BlobAccum> blobs;
        const size_t ours = group_components(mask, wt, W, H, work.ws, blobs);

        cv::Mat in(H, W, CV_8UC1, mask.data());
        cv::Mat labels;
        // 8-connectivity, matching ours. The returned count includes the
        // background as label 0, hence the -1.
        const int theirs = cv::connectedComponents(in, labels, 8, CV_32S) - 1;

        INFO("trial " << trial << "  " << W << "x" << H
             << "  density " << density << "  ours " << ours << "  cv " << theirs);
        REQUIRE(static_cast<int>(ours) == theirs);
        ++images;
    }
    CHECK(images == 1000);
#endif
}

TEST_CASE("labels are a deterministic function of the mask") {
    // §9.4.6: "label numbering depends on merge order, deterministic only with
    // a fixed scan order." INV-3 needs the whole run reproducible, and the frame
    // fingerprint is order-sensitive on purpose, so a reordering would show up
    // as a divergence with no visible change to any detected position — the
    // worst kind of failure.
    constexpr int W = 40, H = 40;
    Pcg32 rng(7);
    std::vector<uint8_t> mask(W * H, 0);
    for (auto& m : mask) m = rng.next_double() < 0.15 ? 1 : 0;
    std::vector<int16_t> wt(mask.size(), 33);

    Work w1(W, H), w2(W, H);
    std::vector<BlobAccum> a, b;
    group_components(mask, wt, W, H, w1.ws, a);
    group_components(mask, wt, W, H, w2.ws, b);

    REQUIRE(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        INFO("blob " << i);
        REQUIRE(a[i].n == b[i].n);
        REQUIRE(a[i].x0 == b[i].x0);
        REQUIRE(a[i].y0 == b[i].y0);
        REQUIRE(a[i].sw == b[i].sw);
        REQUIRE(a[i].swx == b[i].swx);
    }
}

TEST_CASE("shape statistics distinguish a blob from a streak from a chain") {
    // These three feed §9.4.7's gate, so each needs to mean what the gate
    // assumes it means.
    constexpr int W = 32, H = 32;
    std::vector<int16_t> wt(W * H, 100);
    Work work(W, H);
    std::vector<BlobAccum> blobs;

    // A solid square: fill 1.0, aspect 1.0 — what a beacon looks like.
    {
        std::vector<uint8_t> mask(W * H, 0);
        for (int y = 10; y < 20; ++y) for (int x = 10; x < 20; ++x)
            mask[static_cast<size_t>(y) * W + x] = 1;
        blobs.clear();
        REQUIRE(group_components(mask, wt, W, H, work.ws, blobs) == 1);
        CHECK(blobs[0].fill_ratio() == doctest::Approx(1.0f));
        CHECK(blobs[0].aspect() == doctest::Approx(1.0f));
        CHECK(blobs[0].n == 100);
    }

    // A horizontal streak: fill 1.0 but aspect 20 — a star trail or a scratch.
    {
        std::vector<uint8_t> mask(W * H, 0);
        for (int x = 5; x < 25; ++x) mask[static_cast<size_t>(15) * W + x] = 1;
        blobs.clear();
        REQUIRE(group_components(mask, wt, W, H, work.ws, blobs) == 1);
        CHECK(blobs[0].aspect() == doctest::Approx(20.0f));
        CHECK(blobs[0].fill_ratio() == doctest::Approx(1.0f));
    }

    // A diagonal chain of impulse survivors: aspect 1 but fill ~1/n — the case
    // §9.4.7 calls out specifically, and the one that aspect alone would miss.
    {
        std::vector<uint8_t> mask(W * H, 0);
        for (int i = 0; i < 12; ++i) mask[static_cast<size_t>(5 + i) * W + (5 + i)] = 1;
        blobs.clear();
        REQUIRE(group_components(mask, wt, W, H, work.ws, blobs) == 1);
        CHECK(blobs[0].aspect() == doctest::Approx(1.0f));        // looks square
        CHECK(blobs[0].fill_ratio() < 0.1f);                      // but is not solid
    }
}

TEST_CASE("negative top-hat weights do not pull the centroid") {
    // A pixel below its local background carries no evidence about where a
    // bright source is. Letting it contribute a negative weight would drag the
    // centre of mass away from the light — and at low SNR half the pixels in a
    // blob's surround are below background.
    constexpr int W = 16, H = 16;
    std::vector<uint8_t> mask(W * H, 0);
    std::vector<int16_t> wt(W * H, 0);
    for (int y = 6; y < 10; ++y) {
        for (int x = 6; x < 10; ++x) {
            mask[static_cast<size_t>(y) * W + x] = 1;
            wt[static_cast<size_t>(y) * W + x] = 100;
        }
    }
    // One strongly negative pixel inside the blob.
    wt[static_cast<size_t>(6) * W + 6] = -1000;

    Work work(W, H);
    std::vector<BlobAccum> blobs;
    REQUIRE(group_components(mask, wt, W, H, work.ws, blobs) == 1);

    // With the clip, that pixel simply contributes nothing; without it, the
    // centroid would be dragged far outside the blob.
    const Pixel2 c = blobs[0].centroid();
    INFO("centroid (" << c.x << ", " << c.y << ")");
    CHECK(c.x >= 6.0);
    CHECK(c.x <= 9.0);
    CHECK(c.y >= 6.0);
    CHECK(c.y <= 9.0);
}

// ===========================================================================
// CP 5.8 — centre of mass over the top-hat
// ===========================================================================

TEST_CASE("CP 5.8: recovers a known sub-pixel position to within 0.05 px") {
    // The checkpoint's criterion. A beacon is rendered at a known continuous
    // position with the exact-coverage splatter (CP 1.3, accurate to 3e-9 px),
    // thresholded, grouped, and the centre of mass compared against the truth.
    constexpr int W = 64, H = 64;
    Work work(W, H);

    double worst = 0.0;
    int cases = 0;

    for (int sy = 0; sy < 8; ++sy) {
        for (int sx = 0; sx < 8; ++sx) {
            const Pixel2 truth{30.0 + sx / 8.0, 30.0 + sy / 8.0};

            // Render, then treat the render as its own top-hat: a clean frame
            // has no background to remove, which is what "on a clean frame"
            // means in the checkpoint.
            std::vector<float> img(static_cast<size_t>(W) * H, 0.0f);
            splat_emitter(img, W, H, truth, 10.0, ShapeKind::Square, 200.0f);

            std::vector<int16_t> wt(img.size());
            std::vector<uint8_t> mask(img.size(), 0);
            for (size_t i = 0; i < img.size(); ++i) {
                wt[i] = static_cast<int16_t>(std::lround(img[i]));
                // A low threshold, so the blob includes the partially-covered
                // rim pixels. Those rim pixels ARE the sub-pixel information:
                // thresholding them away is what makes a centroid quantise to
                // whole pixels.
                mask[i] = wt[i] > 2 ? 1 : 0;
            }

            std::vector<BlobAccum> blobs;
            REQUIRE(group_components(mask, wt, W, H, work.ws, blobs) == 1);

            const Pixel2 got = blobs[0].centroid();
            const double err = (got - truth).norm();
            INFO("truth (" << truth.x << ", " << truth.y << ")  got ("
                 << got.x << ", " << got.y << ")  error " << err);
            REQUIRE(err < 0.05);
            worst = std::max(worst, err);
            ++cases;
        }
    }
    MESSAGE("CP 5.8: worst centroid error over " << cases
            << " sub-pixel positions: " << worst << " px");
    CHECK(cases == 64);
}

TEST_CASE("the threshold is what limits sub-pixel accuracy, not the estimator") {
    // Worth knowing before Stage 9 tries to improve the estimator: raising the
    // mask threshold discards the partially-covered rim pixels, and those are
    // exactly where the sub-pixel information lives. No estimator can recover
    // what the threshold threw away.
    constexpr int W = 64, H = 64;
    Work work(W, H);
    const Pixel2 truth{30.37, 30.63};

    std::vector<float> img(static_cast<size_t>(W) * H, 0.0f);
    splat_emitter(img, W, H, truth, 10.0, ShapeKind::Square, 200.0f);
    std::vector<int16_t> wt(img.size());
    for (size_t i = 0; i < img.size(); ++i) wt[i] = static_cast<int16_t>(std::lround(img[i]));

    for (int16_t thresh : {2, 20, 100, 180}) {
        std::vector<uint8_t> mask(img.size(), 0);
        for (size_t i = 0; i < img.size(); ++i) mask[i] = wt[i] > thresh ? 1 : 0;

        std::vector<BlobAccum> blobs;
        if (group_components(mask, wt, W, H, work.ws, blobs) != 1) continue;
        const double err = (blobs[0].centroid() - truth).norm();
        MESSAGE("mask threshold " << thresh << " -> centroid error " << err
                << " px  (" << blobs[0].n << " px in the blob)");
    }
    CHECK(true);
}

TEST_CASE("degenerate inputs do not fault") {
    constexpr int W = 16, H = 16;
    std::vector<uint8_t> mask(W * H, 0);
    std::vector<int16_t> wt(W * H, 5);
    Work work(W, H);
    std::vector<BlobAccum> blobs;

    // An empty mask yields nothing.
    CHECK(group_components(mask, wt, W, H, work.ws, blobs) == 0);

    // A completely full mask is one component.
    std::fill(mask.begin(), mask.end(), 1);
    CHECK(group_components(mask, wt, W, H, work.ws, blobs) == 1);
    CHECK(blobs[0].n == W * H);

    // Undersized inputs are refused rather than overrun.
    std::vector<uint8_t> tiny(4, 1);
    std::vector<int16_t> tiny_w(4, 1);
    CHECK(group_components(tiny, tiny_w, W, H, work.ws, blobs) == 0);

    // A zero-weight blob still yields a position rather than a NaN.
    std::fill(wt.begin(), wt.end(), 0);
    blobs.clear();
    REQUIRE(group_components(mask, wt, W, H, work.ws, blobs) == 1);
    CHECK(std::isfinite(blobs[0].centroid().x));
    CHECK(std::isfinite(blobs[0].centroid().y));
}

// ===========================================================================
// INV-4 — the blob table is bounded, and the bound is honest
//
// CP 14.1's fuzzer drew a legal scenario (1920x534, heavy damage, a CFAR
// threshold its draw put low) whose mask grouped into 14,420 components. The
// blob vector had been reserved to 4,096, so it grew — 524,288 bytes,
// allocated inside the frame window, which is precisely what INV-4 forbids and
// what the Debug allocation trap aborted on.
//
// The fix bounds the table. These tests pin the three properties that make a
// bound acceptable rather than a fudge: it is ENFORCED, the truncation is
// DETERMINISTIC (INV-3), and the overflow is REPORTED so a frame whose answer
// is incomplete cannot pass for a clean one.
// ===========================================================================

TEST_CASE("the blob table stops at its bound and says how much it dropped") {
    // 64 separate single-pixel blobs on an 8x8 lattice, spaced so nothing
    // touches. Deliberately more components than the caps below.
    constexpr int W = 64, H = 64;
    std::vector<uint8_t> mask(W * H, 0);
    std::vector<int16_t> wt(W * H, 100);
    for (int gy = 0; gy < 8; ++gy) {
        for (int gx = 0; gx < 8; ++gx) {
            mask[static_cast<size_t>(gy * 8 + 1) * W + (gx * 8 + 1)] = 1;
        }
    }

    Work work(W, H);

    // Unbounded: all 64 are found. This is the reference the bounded runs are
    // compared against, so it has to be established first.
    std::vector<BlobAccum> all;
    REQUIRE(group_components(mask, wt, W, H, work.ws, all) == 64);

    SUBCASE("a bound below the component count truncates to exactly the bound") {
        std::vector<BlobAccum> blobs;
        blobs.reserve(10);
        size_t dropped = 0;
        const size_t n = group_components(mask, wt, W, H, work.ws, blobs,
                                          /*max_blobs=*/10, &dropped);
        CHECK(n == 10);
        CHECK(blobs.size() == 10);
        // Every component that did not fit is counted. 64 single-pixel blobs,
        // 10 kept, 54 with nowhere to go.
        CHECK(dropped == 54);
        // And the vector did not grow past what was reserved for it, which is
        // the whole point — a grown vector is a heap allocation in a frame.
        CHECK(blobs.capacity() == 10);
    }

    SUBCASE("truncation keeps the FIRST components in raster order") {
        // INV-3: which components survive must be a function of the mask
        // alone. Labels are assigned in order of first appearance in raster
        // order, so the survivors must be a prefix of the unbounded run —
        // byte-identical, not merely the same count.
        std::vector<BlobAccum> blobs;
        blobs.reserve(20);
        REQUIRE(group_components(mask, wt, W, H, work.ws, blobs, 20) == 20);
        for (size_t i = 0; i < blobs.size(); ++i) {
            CAPTURE(i);
            CHECK(blobs[i].centroid().x == all[i].centroid().x);
            CHECK(blobs[i].centroid().y == all[i].centroid().y);
            CHECK(blobs[i].n == all[i].n);
        }
    }

    SUBCASE("a bound above the component count changes nothing") {
        std::vector<BlobAccum> blobs;
        size_t dropped = 7;                       // must be cleared, not left
        CHECK(group_components(mask, wt, W, H, work.ws, blobs, 4096, &dropped) == 64);
        CHECK(dropped == 0);
    }

    SUBCASE("a bound of zero yields nothing and drops everything") {
        // The degenerate case exists because `out.capacity()` on a
        // default-constructed vector IS zero, and an earlier draft of this
        // bound read the capacity implicitly. That draft silently found no
        // components at all in every caller that had not reserved. The bound
        // is an explicit parameter now, and this pins the degenerate end of it.
        std::vector<BlobAccum> blobs;
        size_t dropped = 0;
        CHECK(group_components(mask, wt, W, H, work.ws, blobs, 0, &dropped) == 0);
        CHECK(dropped == 64);
    }
}
