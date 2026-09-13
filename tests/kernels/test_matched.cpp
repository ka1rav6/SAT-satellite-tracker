// tests/kernels/test_matched.cpp — CP 5.4.
//
// "A 10-px square scores highest at scale 11; a 5-px square at scale 5."
//
// That criterion is really two claims: the filter finds the target, and the
// WINNING SCALE is a usable size estimate. The second is what §9.4.4 calls "a
// free size estimate that feeds the centroid window", and §10.1.4 turns it into
// the uncertainty that feeds the Kalman gain — so a scale that tracked size
// badly would propagate into the filter.

#include <doctest/doctest.h>

#include "core/rng.hpp"
#include "perception/matched.hpp"
#include "perception/sat.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace sat;

namespace {

struct Scene {
    int W, H;
    std::vector<int16_t>  img;
    std::vector<int64_t>  s;
    std::vector<uint64_t> s2;
    SummedArea sa;

    Scene(int w, int h, int16_t background = 0) : W(w), H(h) {
        img.assign(static_cast<size_t>(w) * h, background);
        const size_t n = SummedArea::elements(w, h);
        s.assign(n, 0);
        s2.assign(n, 0);
        sa = SummedArea{s, s2, w, h};
    }

    /// A square beacon of the given size, centred at (cx, cy).
    void add_square(int cx, int cy, int size, int16_t amplitude) {
        const int h = size / 2;
        for (int y = cy - h; y < cy - h + size; ++y) {
            for (int x = cx - h; x < cx - h + size; ++x) {
                if (x < 0 || x >= W || y < 0 || y >= H) continue;
                img[static_cast<size_t>(y) * W + x] =
                    static_cast<int16_t>(img[static_cast<size_t>(y) * W + x] + amplitude);
            }
        }
    }

    void build() { build_sat(img, W, H, sa); }
};

}  // namespace

TEST_CASE("CP 5.4: the winning scale tracks the target size") {
    // The checkpoint's criterion, generalised across the whole of spec row 10's
    // 5-20 px range rather than just the two sizes it names.
    struct Case { int size; int expect; };
    const Case cases[] = {
        { 5,  5},   // the checkpoint's second example
        { 8,  8},
        {10, 11},   // the checkpoint's first example
        {14, 14},
        {17, 17},
        {20, 20},
    };

    for (const auto& c : cases) {
        Scene sc(96, 96, 0);
        sc.add_square(48, 48, c.size, 100);
        sc.build();

        const MatchedPeak p = matched_best(sc.sa, 48, 48);
        INFO("target size " << c.size << " px -> winning scale " << p.scale);
        CHECK(p.scale == c.expect);
    }
}

TEST_CASE("CP 5.4: the response peaks at the target, not beside it") {
    Scene sc(96, 96, 0);
    sc.add_square(48, 48, 10, 100);
    sc.build();

    std::vector<float>   resp(static_cast<size_t>(96) * 96, 0.0f);
    std::vector<uint8_t> scale(resp.size(), 0);
    matched_filter(sc.sa, 96, 96, resp, scale);

    // The global maximum must be at the beacon.
    const auto it = std::max_element(resp.begin(), resp.end());
    const size_t idx = static_cast<size_t>(it - resp.begin());
    const int px = static_cast<int>(idx % 96), py = static_cast<int>(idx / 96);
    INFO("peak at (" << px << ", " << py << ")");
    CHECK(std::abs(px - 48) <= 1);
    CHECK(std::abs(py - 48) <= 1);

    // And it falls away: far from the beacon the response is zero, because the
    // background here is zero.
    CHECK(resp[static_cast<size_t>(10) * 96 + 10] == doctest::Approx(0.0));
}

TEST_CASE("the sqrt normalisation makes scales comparable on pure noise") {
    // ---------------------------------------------------------------------
    // The reason for dividing by sqrt(n) rather than by n or by nothing.
    //
    // Over independent noise, a k x k box SUM has standard deviation sigma*k,
    // so an unnormalised max-over-scales would always pick the largest box
    // regardless of signal. Dividing by the pixel count gives a MEAN, whose
    // noise falls as 1/k, so that would always pick the smallest.
    //
    // Dividing by sqrt(n) — the matched-filter normalisation — makes the noise
    // standard deviation IDENTICAL at every scale, so the winner is decided by
    // the signal. This measures that directly.
    // ---------------------------------------------------------------------
    constexpr int W = 200, H = 200;
    Scene sc(W, H, 0);
    Pcg32 rng(4242);
    for (auto& v : sc.img) {
        v = static_cast<int16_t>(std::lround(30.0 * rng.next_normal()));
    }
    sc.build();

    for (const int k : kMatchedScales) {
        double sum = 0.0, sum_sq = 0.0;
        int n = 0;
        // Sample well inside, so clipping does not affect the statistics.
        for (int y = 30; y < H - 30; y += 3) {
            for (int x = 30; x < W - 30; x += 3) {
                const double r = matched_response(sc.sa, x, y, k);
                sum += r; sum_sq += r * r; ++n;
            }
        }
        const double mean = sum / n;
        const double sd   = std::sqrt(sum_sq / n - mean * mean);
        INFO("scale " << k << ": mean " << mean << ", sd " << sd);
        // Per-pixel sigma is 30, so the normalised response should have sd ~30
        // at EVERY scale. Without the normalisation this would be 30*k, i.e.
        // 150 at k=5 and 600 at k=20.
        CHECK(sd == doctest::Approx(30.0).epsilon(0.15));
        CHECK(std::abs(mean) < 6.0);
    }
}

TEST_CASE("without normalisation, the noise floor grows with scale") {
    // ---------------------------------------------------------------------
    // The counterfactual, measured directly rather than through a win rate.
    //
    // Over independent noise of per-pixel sigma, a k x k box SUM has standard
    // deviation sigma*k. So the raw noise floor at scale 20 is four times the
    // floor at scale 5, and a max-over-scales on raw sums is comparing numbers
    // that are not on the same footing — the winner is decided by geometry
    // rather than by signal, and the "free size estimate" §9.4.4 promises would
    // be worthless.
    //
    // Comparing the noise FLOORS is the sharp form of this. A win-rate
    // comparison is much weaker: the box sums at different scales are nested
    // and therefore highly correlated, so even a badly biased estimator picks
    // the largest scale only about a third of the time, which is hard to
    // distinguish from the 1/6 that chance would give. (An earlier version of
    // this test asserted a win rate above 50% and failed at the measured 37%.)
    // ---------------------------------------------------------------------
    constexpr int W = 200, H = 200;
    constexpr double kSigma = 30.0;
    Scene sc(W, H, 0);
    Pcg32 rng(9);
    for (auto& v : sc.img) v = static_cast<int16_t>(std::lround(kSigma * rng.next_normal()));
    sc.build();

    auto floors = [&](int k) {
        double raw_sq = 0.0, norm_sq = 0.0;
        int n = 0;
        for (int y = 30; y < H - 30; y += 3) {
            for (int x = 30; x < W - 30; x += 3) {
                const int h = k / 2;
                int cnt = 0;
                const double raw = static_cast<double>(
                    sc.sa.box_sum_clipped(x - h, y - h, x - h + k, y - h + k, cnt));
                const double nrm = matched_response(sc.sa, x, y, k);
                raw_sq += raw * raw;
                norm_sq += nrm * nrm;
                ++n;
            }
        }
        // Zero-mean noise, so the RMS is the standard deviation.
        return std::pair{std::sqrt(raw_sq / n), std::sqrt(norm_sq / n)};
    };

    const auto [raw5,  norm5]  = floors(5);
    const auto [raw20, norm20] = floors(20);

    MESSAGE("noise floor, sigma = " << kSigma << " per pixel:");
    MESSAGE("  raw box sum:  k=5 -> " << raw5  << ",  k=20 -> " << raw20
            << "   (ratio " << (raw20 / raw5) << ")");
    MESSAGE("  normalised:   k=5 -> " << norm5 << ",  k=20 -> " << norm20
            << "   (ratio " << (norm20 / norm5) << ")");

    // Raw: the floor scales with k, so 20/5 = 4x. That is the bias.
    CHECK(raw20 / raw5 == doctest::Approx(4.0).epsilon(0.15));

    // Normalised: the floor is sigma at EVERY scale, so the ratio is 1 and the
    // maximum over scales is decided by signal alone.
    CHECK(norm5  == doctest::Approx(kSigma).epsilon(0.15));
    CHECK(norm20 == doctest::Approx(kSigma).epsilon(0.15));
    CHECK(norm20 / norm5 == doctest::Approx(1.0).epsilon(0.15));
}

TEST_CASE("a matched filter beats a mismatched one, which is why six scales exist") {
    // The SNR argument from §9.4.4: a filter matched to the wrong size loses
    // signal-to-noise both ways — too small integrates only part of the target,
    // too large integrates background that is pure noise.
    constexpr int W = 128, H = 128;
    Scene sc(W, H, 0);
    Pcg32 rng(17);
    for (auto& v : sc.img) v = static_cast<int16_t>(std::lround(20.0 * rng.next_normal()));
    sc.add_square(64, 64, 11, 60);
    sc.build();

    // Measure the background sd at each scale, then the response at the beacon,
    // and compare the resulting SNR.
    auto snr_at_scale = [&](int k) {
        double sum = 0.0, sum_sq = 0.0; int n = 0;
        for (int y = 25; y < 45; ++y) {
            for (int x = 25; x < 45; ++x) {
                const double r = matched_response(sc.sa, x, y, k);
                sum += r; sum_sq += r * r; ++n;
            }
        }
        const double mean = sum / n;
        const double sd = std::sqrt(std::max(1e-9, sum_sq / n - mean * mean));
        return (matched_response(sc.sa, 64, 64, k) - mean) / sd;
    };

    const double snr_matched = snr_at_scale(11);
    const double snr_small   = snr_at_scale(5);
    const double snr_large   = snr_at_scale(20);
    MESSAGE("11 px beacon: SNR at scale 5 = " << snr_small
            << ", at scale 11 = " << snr_matched
            << ", at scale 20 = " << snr_large);

    CHECK(snr_matched > snr_small);
    CHECK(snr_matched > snr_large);
}

TEST_CASE("edges are handled by clipping, not by skipping") {
    // Design §9.4.5's rule, applied here: "Never skip a border — a beacon near
    // the edge is exactly when you are about to lose it."
    Scene sc(64, 64, 0);
    sc.add_square(2, 2, 5, 100);     // mostly off the top-left corner
    sc.build();

    const MatchedPeak corner = matched_best(sc.sa, 2, 2);
    INFO("response at a corner beacon: " << corner.response);
    CHECK(corner.response > 0.0f);

    // And the response is finite everywhere, including outside the image, which
    // CP 14.1's fuzzing will certainly try.
    for (int x : {-10, -1, 0, 63, 64, 200}) {
        for (int y : {-10, -1, 0, 63, 64, 200}) {
            const MatchedPeak p = matched_best(sc.sa, x, y);
            INFO("query at (" << x << ", " << y << ")");
            REQUIRE(std::isfinite(p.response));
        }
    }
}

TEST_CASE("the six scales span specification row 10's permitted range") {
    // Row 10 permits 5 to 20 px. The scale set must cover it, or a legal
    // scenario would have no well-matched filter.
    CHECK(kMatchedScales.front() == 5);
    CHECK(kMatchedScales.back() == 20);
    CHECK(kMatchedScales.size() == 6);
    // Roughly even spacing, so no legal size is far from a scale.
    for (size_t i = 1; i < kMatchedScales.size(); ++i) {
        CHECK(kMatchedScales[i] > kMatchedScales[i - 1]);
        CHECK(kMatchedScales[i] - kMatchedScales[i - 1] <= 3);
    }
}

TEST_CASE("degenerate inputs do not fault") {
    Scene sc(16, 16, 0);
    sc.build();
    CHECK(matched_response(sc.sa, 8, 8, 0) == 0.0f);
    CHECK(matched_response(sc.sa, 8, 8, -5) == 0.0f);

    std::vector<float> resp(4, -1.0f);
    std::vector<uint8_t> scale(4, 0);
    matched_filter(sc.sa, 16, 16, resp, scale);   // buffers too small
    CHECK(resp[0] == -1.0f);                      // refused, not overrun
}
