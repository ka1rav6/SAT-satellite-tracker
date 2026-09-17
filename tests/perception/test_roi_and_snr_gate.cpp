// tests/perception/test_roi_and_snr_gate.cpp
//
// Two changes to §9.4's detector, both with a measurement behind them:
//
//   the SNR gate (PerceptionParams::min_snr_factor) — CFAR's k is a statement
//   about ONE PIXEL, and grouping 29 false pixels on a smoothed matched-filter
//   response produces blobs that are exactly the shape of a beacon;
//
//   the detection window (DetectRoi, design §14.0b) — a confirmed track already
//   knows where the target is, so the detector is not asked to search the rest
//   of the frame for it.
//
// The window's correctness criterion is the strong one: a detection found
// inside a window must be IDENTICAL to the one the full-frame pass finds, in
// full-frame coordinates, to the last bit. Anything less and the graded
// centroiding metric would depend on how fast the frame happened to run.

#include <doctest/doctest.h>

#include "camera/splat.hpp"
#include "core/arena.hpp"
#include "core/image.hpp"
#include "core/rng.hpp"
#include "perception/pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace sat;

namespace {

constexpr int W = 640, H = 480;

struct Fixture {
    Arena               arena{64u << 20};
    PerceptionWorkspace ws;
    ClassicalPerception det;
    std::vector<float>   radiance;
    std::vector<uint8_t> pixels;

    Fixture() {
        PerceptionParams p;
        det.configure(p);
        REQUIRE(ws.allocate(arena, W, H, structuring_element_size(p.target_size_px)));
        radiance.assign(static_cast<size_t>(W) * H, 8.0f);
        pixels.assign(static_cast<size_t>(W) * H, 0);
    }

    void quantise() { quantise_u8(radiance, pixels); }
};

/// Gaussian read noise on the pedestal, and nothing else: no target, no
/// clutter, no impulses. Whatever the detector returns here is a false alarm.
void add_read_noise(std::vector<float>& img, double sigma, uint64_t seed) {
    Pcg32 g{seed};
    for (float& v : img) v += static_cast<float>(sigma * g.next_normal());
}

}  // namespace

TEST_CASE("the SNR gate removes the false alarms the shape gate cannot") {
    Fixture f;
    add_read_noise(f.radiance, 4.0, 20260917u);
    f.quantise();

    std::vector<Detection> with_gate, without_gate;

    // Without: the parameters as they were before the gate existed.
    f.det.params().min_snr_factor = 0.0f;
    f.det.params().min_snr_abs    = 0.0f;
    f.det.process(f.pixels, W, H, f.ws, without_gate);

    f.det.params().min_snr_factor = 1.5f;
    f.det.process(f.pixels, W, H, f.ws, with_gate);

    float worst = 0.0f;
    for (const Detection& d : without_gate) worst = std::max(worst, d.snr);
    INFO("ungated candidates on pure noise: ", without_gate.size(),
         ", highest SNR ", worst, ", gate at ", min_candidate_snr(f.det.params()));

    // The measurement that motivated the gate: a frame with nothing in it
    // produces candidates, every one of which passes the shape gate.
    CHECK(without_gate.size() >= 1);
    // And every one of them sits just over k, which is what a threshold
    // crossing looks like and what a beacon does not look like.
    CHECK(worst < 2.0f * f.det.params().cfar.k);
    // With the gate, none survive.
    CHECK(with_gate.empty());
}

TEST_CASE("the SNR gate keeps a beacon by a wide margin") {
    Fixture f;
    splat_emitter(f.radiance, W, H, Pixel2{320.4, 240.7}, 10.0,
                  ShapeKind::Square, 120.0f);
    add_read_noise(f.radiance, 4.0, 4242u);
    f.quantise();

    std::vector<Detection> out;
    f.det.process(f.pixels, W, H, f.ws, out);
    REQUIRE_FALSE(out.empty());
    INFO("beacon SNR ", out[0].snr, " against a gate of ",
         min_candidate_snr(f.det.params()));
    CHECK(out[0].snr > 10.0f * min_candidate_snr(f.det.params()));
    CHECK(out[0].centroid_image.x == doctest::Approx(320.4).epsilon(0.02));
    CHECK(out[0].centroid_image.y == doctest::Approx(240.7).epsilon(0.02));
}

TEST_CASE("§14.0b: a windowed detection is identical to the full-frame one") {
    Fixture f;
    // A beacon, plus clutter both inside and outside the window, so the
    // comparison is not trivially about an empty frame.
    splat_emitter(f.radiance, W, H, Pixel2{320.37, 240.62}, 10.0,
                  ShapeKind::Square, 120.0f);
    Pcg32 g{777u};
    for (int i = 0; i < 40; ++i) {
        splat_emitter(f.radiance, W, H,
                      Pixel2{g.next_range(0.0, W), g.next_range(0.0, H)},
                      static_cast<double>(3 + g.next_below(22)),
                      (g.next_below(4) == 0) ? ShapeKind::Square : ShapeKind::Gaussian,
                      static_cast<float>(g.next_range(0.35, 1.6) * 120.0));
    }
    add_read_noise(f.radiance, 4.0, 99u);
    f.quantise();

    std::vector<Detection> full, windowed;
    f.det.process(f.pixels, W, H, f.ws, full);

    // A window centred on the beacon, comfortably wider than the CFAR training
    // annulus so the background statistics near the target are the same.
    const DetectRoi roi{320 - 96, 240 - 96, 193, 193};
    f.det.process(f.pixels, W, H, roi, f.ws, windowed);

    REQUIRE_FALSE(full.empty());
    REQUIRE_FALSE(windowed.empty());

    // Find the beacon in each list. It is the one nearest where it was drawn.
    auto nearest = [](const std::vector<Detection>& v) {
        size_t best = 0;
        double bd = 1e18;
        for (size_t i = 0; i < v.size(); ++i) {
            const double d = std::hypot(v[i].centroid_image.x - 320.37,
                                        v[i].centroid_image.y - 240.62);
            if (d < bd) { bd = d; best = i; }
        }
        return v[best];
    };
    const Detection a = nearest(full);
    const Detection b = nearest(windowed);

    INFO("full (", a.centroid_image.x, ", ", a.centroid_image.y, ") snr ", a.snr,
         " / windowed (", b.centroid_image.x, ", ", b.centroid_image.y, ") snr ", b.snr);
    // EXACT. The window changes which pixels are looked at, not what is
    // computed from the ones that are.
    CHECK(b.centroid_image.x == a.centroid_image.x);
    CHECK(b.centroid_image.y == a.centroid_image.y);
    CHECK(b.snr        == a.snr);
    CHECK(b.area_px    == a.area_px);
    CHECK(b.size_est_px == a.size_est_px);

    // And the window really did exclude the rest of the frame: every windowed
    // detection lies inside it.
    for (const Detection& d : windowed) {
        CHECK(d.centroid_image.x >= roi.x0 - 1);
        CHECK(d.centroid_image.x <= roi.x0 + roi.width);
        CHECK(d.centroid_image.y >= roi.y0 - 1);
        CHECK(d.centroid_image.y <= roi.y0 + roi.height);
    }
    // ...which is the point: there was clutter outside it that the full-frame
    // pass found and this one did not.
    CHECK(windowed.size() < full.size());
}

TEST_CASE("§14.0b: an absurd window falls back to the whole frame") {
    // A predicted position off the sensor, or a degenerate rectangle, must not
    // make the detector look at nothing. Failing safe here is the difference
    // between a slow frame and a lost target.
    Fixture f;
    splat_emitter(f.radiance, W, H, Pixel2{100.0, 100.0}, 10.0,
                  ShapeKind::Square, 120.0f);
    f.quantise();

    std::vector<Detection> ref, degenerate;
    f.det.process(f.pixels, W, H, f.ws, ref);
    f.det.process(f.pixels, W, H, DetectRoi{-5000, -5000, 0, 0}, f.ws, degenerate);

    REQUIRE_FALSE(ref.empty());
    REQUIRE_FALSE(degenerate.empty());
    CHECK(degenerate[0].centroid_image.x == ref[0].centroid_image.x);
    CHECK(degenerate[0].centroid_image.y == ref[0].centroid_image.y);
}
