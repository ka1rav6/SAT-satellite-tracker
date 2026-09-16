// tests/render/test_splat.cpp — CP 1.3 acceptance.
//
// "Test: place a 10x10 square at x=100.37; intensity-weighted centre of rendered
//  pixels = 100.37 ± 0.001"
//
// This is the test that certifies the simulator's ground truth is exact, and
// everything the project claims about sub-pixel accuracy — 60% of the marks —
// is downstream of it. If the renderer quantised position, the "true" centre we
// measure error against would itself be wrong, and no centroid estimator could
// be shown to beat that error.

#include <doctest/doctest.h>

#include "camera/splat.hpp"
#include "camera/coverage.hpp"
#include "core/frames.hpp"
#include "core/rng.hpp"

#include <cmath>
#include <initializer_list>
#include <algorithm>
#include <vector>

using namespace sat;

namespace {

// A small render target. Big enough that a 20 px emitter is comfortably
// interior, small enough that a brute-force centroid is instant.
struct Canvas {
    static constexpr int W = 128;
    static constexpr int H = 128;
    std::vector<float> buf;

    Canvas() : buf(static_cast<size_t>(W) * H, 0.0f) {}
    void clear() { std::fill(buf.begin(), buf.end(), 0.0f); }
    [[nodiscard]] Pixel2 centroid() const { return intensity_centroid(buf, W, H); }
    [[nodiscard]] double flux() const { return total_flux(buf); }
};

}  // namespace

TEST_CASE("CP 1.3: a 10x10 square at x=100.37 has its centre recovered to 1e-3") {
    // The checkpoint's criterion, verbatim, on a canvas large enough to hold it.
    Canvas c;
    const Pixel2 truth{100.37, 64.0};
    splat_emitter(c.buf, Canvas::W, Canvas::H, truth, 10.0, ShapeKind::Square, 100.0f);

    const Pixel2 got = c.centroid();
    INFO("truth = (" << truth.x << ", " << truth.y << ")  got = ("
         << got.x << ", " << got.y << ")");
    CHECK(std::abs(got.x - truth.x) < 1e-3);
    CHECK(std::abs(got.y - truth.y) < 1e-3);
}

TEST_CASE("the recovered centre is exact at every sub-pixel offset, every size") {
    // The stronger version. A renderer that quantised or point-sampled would
    // pass at offset 0.0 and 0.5 and fail in between — which is precisely the
    // S-curve shape design §10.1.3 describes. Sweeping the full sub-pixel range
    // is what makes that failure impossible to miss.
    Canvas c;
    double worst = 0.0;
    for (uint16_t size : {5, 8, 10, 13, 17, 20}) {   // spec row 10's full range
        for (int k = 0; k < 40; ++k) {
            const double frac = static_cast<double>(k) / 40.0;
            const Pixel2 truth{60.0 + frac, 64.0 + frac * 0.5};
            c.clear();
            splat_emitter(c.buf, Canvas::W, Canvas::H, truth,
                          static_cast<double>(size), ShapeKind::Square, 200.0f);
            const Pixel2 got = c.centroid();
            worst = std::max(worst, std::abs(got.x - truth.x));
            worst = std::max(worst, std::abs(got.y - truth.y));
            INFO("size=" << size << " frac=" << frac);
            REQUIRE(std::abs(got.x - truth.x) < 1e-6);
            REQUIRE(std::abs(got.y - truth.y) < 1e-6);
        }
    }
    // Report the actual worst case: it should be at the level of float
    // accumulation noise, not merely under the threshold.
    MESSAGE("worst sub-pixel error across 240 cases: " << worst << " px");
}

TEST_CASE("intensity is peak brightness, independent of size") {
    // `intensity` means "grey levels above background", the same for any size.
    // Spec row 10 lets the beacon be 5-20 px, and a user changing that is
    // describing a different-sized beacon, not a 16x brighter one.
    Canvas c;
    for (uint16_t size : {5, 10, 20}) {
        c.clear();
        // Centre on a pixel so there is a fully-covered pixel to peak at.
        splat_emitter(c.buf, Canvas::W, Canvas::H, Pixel2{64.0, 64.0},
                      static_cast<double>(size), ShapeKind::Square, 200.0f);
        const float peak = *std::max_element(c.buf.begin(), c.buf.end());
        INFO("size=" << size);
        CHECK(peak == doctest::Approx(200.0f).epsilon(1e-5));
        // Total flux therefore scales with area, which is the physically
        // sensible reading: a bigger beacon of the same brightness emits more.
        CHECK(c.flux() == doctest::Approx(200.0 * size * size).epsilon(1e-4));
    }
}

TEST_CASE("weights accumulate linearly, which is what makes motion blur correct") {
    // Design §9.2 splits the exposure into N substeps each weighted 1/N. That
    // is only the right answer if splatting is additive and linear in weight.
    Canvas a, b;
    splat_emitter(a.buf, Canvas::W, Canvas::H, Pixel2{64.0, 64.0}, 10.0,
                  ShapeKind::Square, 100.0f, 1.0);
    (void)0;

    constexpr int kN = 8;
    for (int i = 0; i < kN; ++i) {
        splat_emitter(b.buf, Canvas::W, Canvas::H, Pixel2{64.0, 64.0}, 10.0,
                      ShapeKind::Square, 100.0f, 1.0 / kN);
    }
    CHECK(b.flux() == doctest::Approx(a.flux()).epsilon(1e-5));

    // And a genuinely moving emitter must land its centroid at the midpoint of
    // its travel, which is what a real exposure integrates to.
    Canvas m;
    const double x0 = 50.0, x1 = 70.0;
    for (int i = 0; i < kN; ++i) {
        const double t = (static_cast<double>(i) + 0.5) / kN;
        splat_emitter(m.buf, Canvas::W, Canvas::H,
                      Pixel2{x0 + (x1 - x0) * t, 64.0}, 10.0,
                      ShapeKind::Square, 100.0f, 1.0 / kN);
    }
    CHECK(m.centroid().x == doctest::Approx(0.5 * (x0 + x1)).epsilon(1e-6));
}

TEST_CASE("square and Gaussian recover their centre to floating-point noise") {
    Canvas c;
    for (auto shape : {ShapeKind::Square, ShapeKind::Gaussian}) {
        c.clear();
        const Pixel2 truth{64.37, 63.81};
        splat_emitter(c.buf, Canvas::W, Canvas::H, truth, 11.0, shape, 500.0f);
        const Pixel2 got = c.centroid();
        INFO("shape = " << static_cast<int>(shape));
        CHECK(std::abs(got.x - truth.x) < 1e-6);
        CHECK(std::abs(got.y - truth.y) < 1e-6);
    }
}

TEST_CASE("the circle's centre-of-mass bias is the S-curve, measured not tolerated") {
    // A rendered circle's centre of mass is NOT its true centre, and that is a
    // real effect rather than a defect in the renderer: coverage.hpp computes
    // the areas exactly (they sum to 1.0 to 1e-12 and are symmetric to 1e-15),
    // but a CoM estimator places each pixel's flux at the pixel CENTRE, while
    // the flux in a partially-covered boundary pixel actually sits to one side.
    // For a square that error cancels exactly; for a curved boundary it does
    // not, and the residue is a periodic function of sub-pixel offset.
    //
    // This is the S-curve of design §10.1.3 in its purest form — no noise, no
    // background residual, nothing else in the way — and Stage 9 gets to
    // validate its bias correction against it.
    //
    // The test pins down the SHAPE of the curve rather than just bounding it,
    // so that a genuine regression in the coverage maths (which would change
    // the shape) cannot hide behind a loose tolerance.
    Canvas c;
    struct Sample { double frac, err; };
    std::vector<Sample> curve;

    const double size = 11.0;
    for (int k = 0; k <= 20; ++k) {
        const double frac = static_cast<double>(k) / 20.0;
        const Pixel2 truth{64.0 + frac, 64.0};
        c.clear();
        splat_emitter(c.buf, Canvas::W, Canvas::H, truth, size, ShapeKind::Circle, 500.0f);
        curve.push_back({frac, c.centroid().x - truth.x});
    }

    // (a) It vanishes at the symmetric offsets: on a pixel centre, and exactly
    //     between two. Symmetry forces this, so a non-zero value there would
    //     mean the coverage itself had become asymmetric.
    CHECK(std::abs(curve.front().err) < 1e-9);          // frac = 0.0
    CHECK(std::abs(curve[10].err)     < 1e-9);          // frac = 0.5
    CHECK(std::abs(curve.back().err)  < 1e-9);          // frac = 1.0

    // (b) It is odd about the half-pixel point: err(0.5 - d) == -err(0.5 + d).
    for (int d = 1; d <= 9; ++d) {
        INFO("d = " << d);
        CHECK(curve[static_cast<size_t>(10 - d)].err
              == doctest::Approx(-curve[static_cast<size_t>(10 + d)].err).epsilon(1e-6));
    }

    // (c) Its amplitude is small, and known. Measured at 1.7e-3 px for an 11 px
    //     circle; the bound leaves room for float noise but would catch a
    //     regression to the 2.7e-2 px that 4x4 supersampling used to give.
    double peak = 0.0;
    for (const auto& s : curve) peak = std::max(peak, std::abs(s.err));
    MESSAGE("circle CoM bias amplitude at size " << size << ": " << peak << " px");
    CHECK(peak > 1e-4);        // it is genuinely there — do not "fix" it away
    CHECK(peak < 3e-3);        // and it is an order of magnitude below the old sampler

    // (d) Bigger circles have proportionally less of it, because the boundary
    //     is a smaller fraction of the footprint.
    auto amplitude_at = [&](double sz) {
        double p = 0.0;
        for (int k = 1; k < 20; ++k) {
            const double frac = static_cast<double>(k) / 20.0;
            const Pixel2 truth{64.0 + frac, 64.0};
            c.clear();
            splat_emitter(c.buf, Canvas::W, Canvas::H, truth, sz, ShapeKind::Circle, 500.0f);
            p = std::max(p, std::abs(c.centroid().x - truth.x));
        }
        return p;
    };
    const double small = amplitude_at(5.0);
    const double large = amplitude_at(20.0);
    MESSAGE("circle CoM bias: 5 px -> " << small << " px, 20 px -> " << large << " px");
    CHECK(large < small);
}

TEST_CASE("an emitter partly off-sensor does not wrap, crash or vanish") {
    Canvas c;
    // Reference: the same emitter comfortably inside the sensor.
    splat_emitter(c.buf, Canvas::W, Canvas::H, Pixel2{64.0, 64.0}, 10.0,
                  ShapeKind::Square, 100.0f);
    const double whole = c.flux();
    CHECK(whole == doctest::Approx(100.0 * 100.0).epsilon(1e-5));   // intensity x area

    // Straddling the left edge: some flux lands, the rest is legitimately lost.
    // A 10 px square centred at x = -2 spans [-7, 3]. Pixel i covers
    // [i-0.5, i+0.5], so on-sensor columns 0, 1 and 2 are fully covered and
    // column 3 is half covered — 3.5 of the 10 columns survive.
    c.clear();
    splat_emitter(c.buf, Canvas::W, Canvas::H, Pixel2{-2.0, 64.0}, 10.0,
                  ShapeKind::Square, 100.0f);
    const double partial = c.flux();
    CHECK(partial > 0.0);
    CHECK(partial < whole);
    CHECK(partial == doctest::Approx(whole * 0.35).epsilon(1e-6));

    // No wraparound: the right-hand columns must be untouched. A missing clip
    // would show up here as light appearing on the opposite edge.
    for (int j = 0; j < Canvas::H; ++j) {
        CHECK(c.buf[static_cast<size_t>(j) * Canvas::W + (Canvas::W - 1)] == 0.0f);
    }

    // Entirely off-sensor contributes nothing and must not fault.
    c.clear();
    splat_emitter(c.buf, Canvas::W, Canvas::H, Pixel2{-50.0, -50.0}, 10.0,
                  ShapeKind::Square, 100.0f);
    CHECK(c.flux() == doctest::Approx(0.0));
}

TEST_CASE("degenerate inputs are ignored rather than faulting") {
    Canvas c;
    splat_emitter(c.buf, Canvas::W, Canvas::H, Pixel2{64, 64}, 0.0,  ShapeKind::Square, 100.0f);
    splat_emitter(c.buf, Canvas::W, Canvas::H, Pixel2{64, 64}, -5.0, ShapeKind::Square, 100.0f);
    splat_emitter(c.buf, Canvas::W, Canvas::H, Pixel2{64, 64}, 10.0, ShapeKind::Square, 100.0f, 0.0);
    CHECK(c.flux() == doctest::Approx(0.0));
    // CP 14.1 requires 5000 fuzzed scenarios with no crash; degenerate geometry
    // is the first thing a fuzzer will produce.
    splat_emitter({}, 0, 0, Pixel2{0, 0}, 10.0, ShapeKind::Square, 1.0f);
}

TEST_CASE("quantise_u8 rounds symmetrically and clips") {
    const std::vector<float> src{-10.0f, 0.0f, 0.4f, 0.5f, 0.6f, 127.5f, 254.4f, 255.0f, 1e9f};
    std::vector<uint8_t> dst(src.size());
    quantise_u8(src, dst);

    CHECK(dst[0] == 0);      // negative clips low
    CHECK(dst[1] == 0);
    CHECK(dst[2] == 0);      // 0.4 rounds down
    CHECK(dst[3] == 1);      // 0.5 rounds away from zero, as an ADC does
    CHECK(dst[4] == 1);
    CHECK(dst[5] == 128);
    CHECK(dst[7] == 255);
    CHECK(dst[8] == 255);    // clips high rather than wrapping

    // NaN must not reach the cast, where it would be undefined behaviour.
    const std::vector<float> nan_src{std::nanf("")};
    std::vector<uint8_t> nan_dst(1, 42);
    quantise_u8(nan_src, nan_dst);
    CHECK(nan_dst[0] == 0);
}

TEST_CASE("quantisation preserves the centroid to well under a tenth of a pixel") {
    // 8-bit quantisation is the last step of the damage chain and it is
    // unavoidable. What matters is that it does not BIAS the centre — rounding
    // half away from zero is symmetric, so the error it adds is noise, not a
    // systematic shift.
    Canvas c;
    const Pixel2 truth{64.37, 64.63};
    splat_emitter(c.buf, Canvas::W, Canvas::H, truth, 10.0, ShapeKind::Square, 200.0f);

    std::vector<uint8_t> q(c.buf.size());
    quantise_u8(c.buf, q);

    std::vector<float> back(q.size());
    for (size_t i = 0; i < q.size(); ++i) back[i] = static_cast<float>(q[i]);
    const Pixel2 got = intensity_centroid(back, Canvas::W, Canvas::H);

    INFO("after quantisation: (" << got.x << ", " << got.y << ")");
    CHECK(std::abs(got.x - truth.x) < 0.01);
    CHECK(std::abs(got.y - truth.y) < 0.01);
}

TEST_CASE("splat_emitters places emitters through the full screen->image transform") {
    // The end-to-end geometric path: a screen-space emitter, a boresight, and
    // the camera model in between. This is what the synthetic source will call.
    const auto cam = CameraGeometry::make(640, 480, 4.0, 3.0);
    const auto scr = ScreenGeometry::make(2000, 2000, cam);

    EmitterSoA e;
    // Place the beacon at a deliberately non-integer screen position.
    const Pixel2 screen_pos{1234.56, 1088.25};
    e.add(screen_pos.x, screen_pos.y, 500.0f, 10, ShapeKind::Square, EmitterKind::Target);

    // Point the camera so the emitter is comfortably inside the frame.
    const Angle2 bore = scr.to_angle(Pixel2{1200.0, 1100.0});

    std::vector<uint32_t> visible;
    e.query_visible(view_aabb(cam, scr, bore).expanded(e.max_extent_px()), visible);
    REQUIRE(visible.size() == 1);

    std::vector<float> img(static_cast<size_t>(cam.pixel_count()), 0.0f);
    splat_emitters(img, cam, scr, e, visible, bore);

    // The rendered centroid, converted back to screen coordinates, must be the
    // position we started from. This closes the loop through project/unproject.
    const Pixel2 img_c = intensity_centroid(img, cam.width, cam.height);
    const Pixel2 back  = image_to_screen(cam, scr, img_c, bore);

    INFO("screen truth (" << screen_pos.x << ", " << screen_pos.y
         << ") recovered (" << back.x << ", " << back.y << ")");
    CHECK(std::abs(back.x - screen_pos.x) < 1e-6);
    CHECK(std::abs(back.y - screen_pos.y) < 1e-6);
}

TEST_CASE("query_visible finds emitters that straddle the viewport edge") {
    const auto cam = CameraGeometry::make(640, 480, 4.0, 3.0);
    const auto scr = ScreenGeometry::make(2000, 2000, cam);
    const Angle2 bore{0.0, 0.0};
    const Aabb   box = view_aabb(cam, scr, bore);

    EmitterSoA e;
    e.add((box.x0 + box.x1) * 0.5, (box.y0 + box.y1) * 0.5, 1.0f, 10,
          ShapeKind::Square, EmitterKind::Target);      // 0: dead centre
    e.add(box.x0 - 3.0, (box.y0 + box.y1) * 0.5, 1.0f, 10,
          ShapeKind::Square, EmitterKind::Clutter);     // 1: centre outside, body inside
    e.add(box.x0 - 500.0, box.y0 - 500.0, 1.0f, 10,
          ShapeKind::Square, EmitterKind::Clutter);     // 2: far away

    std::vector<uint32_t> visible;
    e.query_visible(box.expanded(e.max_extent_px()), visible);

    REQUIRE(visible.size() == 2);
    CHECK(visible[0] == 0);
    CHECK(visible[1] == 1);   // straddling emitters must not be dropped

    // Order is index order, always. Design §9.4.6 depends on a fixed visit
    // order for deterministic blob labelling, and INV-3 forbids any that vary.
    CHECK(visible[0] < visible[1]);
}

// ---------------------------------------------------------------------------
// The factorised splat must be BIT-IDENTICAL to the per-pixel coverage
// functions, not merely close.
//
// splat_emitter no longer calls square_coverage / gaussian_coverage /
// circle_coverage in its fast paths: it factorises the footprint into 1-D
// tables (the separable shapes) or shares grid corners between neighbouring
// pixels (the circle). That is only a legitimate optimisation if the result is
// the same double, for two reasons:
//
//   INV-3   a run must reproduce bit for bit, and the rendered frame feeds the
//           snapshot hash;
//   CP 14.2 states the criterion for a faster kernel as "bit-identical to
//           scalar on random inputs", which is the standard this holds itself
//           to even though these are not SIMD kernels.
//
// So the check is exact equality of floats, with no tolerance at all.
// ---------------------------------------------------------------------------
TEST_CASE("splat is bit-identical to the per-pixel coverage functions") {
    using namespace sat;

    constexpr int W = 96, H = 96;

    // A reference splat written the obvious way: one 2-D coverage call per
    // pixel, exactly as splat_emitter used to be.
    auto reference = [](std::vector<float>& dst, Pixel2 c, double size,
                        ShapeKind shape, float intensity, double weight) {
        double reach = 0.5 * size;
        if (shape == ShapeKind::Gaussian) {
            constexpr double kFwhmToSigma = 1.0 / 2.3548200450309493;
            reach = 6.0 * size * kFwhmToSigma;
        }
        const int i0 = std::max(0,     static_cast<int>(std::floor(c.x - reach)) - 1);
        const int i1 = std::min(W - 1, static_cast<int>(std::ceil (c.x + reach)) + 1);
        const int j0 = std::max(0,     static_cast<int>(std::floor(c.y - reach)) - 1);
        const int j1 = std::min(H - 1, static_cast<int>(std::ceil (c.y + reach)) + 1);
        if (i0 > i1 || j0 > j1) return;
        const double scale = static_cast<double>(intensity) * weight;
        for (int j = j0; j <= j1; ++j) {
            for (int i = i0; i <= i1; ++i) {
                double cov = 0.0;
                switch (shape) {
                    case ShapeKind::Square:
                    case ShapeKind::Mask:
                        cov = square_coverage(i, j, c.x, c.y, size); break;
                    case ShapeKind::Circle:
                        cov = circle_coverage(i, j, c.x, c.y, size); break;
                    case ShapeKind::Gaussian:
                        cov = gaussian_coverage(i, j, c.x, c.y, size); break;
                }
                if (cov > 0.0) {
                    dst[static_cast<size_t>(j) * W + static_cast<size_t>(i)] +=
                        static_cast<float>(cov * scale);
                }
            }
        }
    };

    // Deterministic pseudo-random cases, including sub-pixel offsets, emitters
    // hanging off every edge, and sizes from sub-pixel to larger than the frame.
    Pcg32 rng{20260917u};

    const ShapeKind shapes[] = {ShapeKind::Square, ShapeKind::Circle,
                                ShapeKind::Gaussian, ShapeKind::Mask};

    int compared = 0;
    for (int trial = 0; trial < 400; ++trial) {
        const ShapeKind shape = shapes[rng.next_below(4)];
        // Deliberately spills past the edges: -20 .. W+20.
        const Pixel2 c{rng.next_range(-20.0, W + 20.0), rng.next_range(-20.0, H + 20.0)};
        const double size      = rng.next_range(0.3, 40.0);
        const float  intensity = static_cast<float>(rng.next_range(1.0, 200.0));
        const double weight    = rng.next_range(0.05, 1.0);

        std::vector<float> a(W * H, 0.0f), b(W * H, 0.0f);
        splat_emitter(a, W, H, c, size, shape, intensity, weight);
        reference(b, c, size, shape, intensity, weight);

        // Exact equality. A single ulp of difference is a failure. Counted
        // rather than REQUIREd per pixel: 400 trials x 9,216 pixels would be
        // 3.7 million assertions in the doctest summary, which drowns out every
        // other number in the suite.
        size_t mismatches = 0;
        size_t first_bad  = 0;
        for (size_t i = 0; i < a.size(); ++i) {
            if (a[i] != b[i]) {
                if (mismatches == 0) first_bad = i;
                ++mismatches;
            }
        }
        INFO("trial ", trial, " shape ", static_cast<int>(shape),
             " size ", size, " at (", c.x, ",", c.y, ")"
             " first bad pixel ", first_bad);
        REQUIRE(mismatches == 0);
        ++compared;
    }
    CHECK(compared == 400);
}
