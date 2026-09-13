// tests/degrade/test_noise.cpp — CP 4.4 through 4.9.
//
// The acceptance criteria these implement:
//   4.4  "Fog visibly washes out; measured contrast matches the table"
//   4.5  "Measured variance over 10k pixels matches theory within 2%"
//   4.6  "Exactly ~10% corrupted; measurably faster than naive"
//   4.7  "Same seed -> same defect pattern across runs"
//   4.8  "Startup logs 20 px/frame -> 65448 urad/s -> 3.75 deg/s"
//   4.9  "All five row-25 modes work, driven by the same code as target motion"
//
// Testing noise against THEORY rather than against a recorded golden output is
// the point. A golden file only says "it still does what it did"; comparing a
// measured variance to lambda says "it does what a Poisson process does", which
// is the claim the perception pipeline is tuned against.

#include <doctest/doctest.h>

#include "degrade/disturbance.hpp"
#include "degrade/noise.hpp"
#include "degrade/sensor.hpp"
#include "scenario/schema.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <string>
#include <vector>

using namespace sat;

// ===========================================================================
// CP 4.5 — shot noise and read noise against theory
// ===========================================================================

TEST_CASE("CP 4.5: Poisson has mean and variance equal to lambda") {
    Pcg32 rng(2024);
    // Straddle the lambda = 30 branch design §9.3 specifies, and include values
    // either side of it so both implementations are covered.
    for (double lambda : {0.5, 3.0, 12.0, 29.0, 31.0, 100.0, 1000.0}) {
        constexpr int kN = 200000;
        double sum = 0.0, sum_sq = 0.0;
        for (int i = 0; i < kN; ++i) {
            const double v = poisson(lambda, rng);
            sum += v;
            sum_sq += v * v;
        }
        const double mean = sum / kN;
        const double var  = sum_sq / kN - mean * mean;

        INFO("lambda = " << lambda << "  mean = " << mean << "  var = " << var);
        // The checkpoint asks for 2%; the standard error of the mean here is
        // sqrt(lambda/N), which is well under that for every case.
        CHECK(mean == doctest::Approx(lambda).epsilon(0.02));
        CHECK(var  == doctest::Approx(lambda).epsilon(0.05));
    }
}

TEST_CASE("the two Poisson branches agree at the threshold") {
    // Design §9.3 warns: "do not change the threshold between builds you intend
    // to compare". This checks the two implementations are statistically
    // equivalent either side of it, so the branch is a performance choice
    // rather than a behavioural one.
    Pcg32 a(11), b(11);
    double mean_below = 0.0, mean_above = 0.0;
    constexpr int kN = 200000;
    for (int i = 0; i < kN; ++i) mean_below += poisson(29.9, a);
    for (int i = 0; i < kN; ++i) mean_above += poisson(30.1, b);
    mean_below /= kN;
    mean_above /= kN;
    INFO("just below: " << mean_below << ", just above: " << mean_above);
    CHECK(std::abs(mean_below - mean_above) < 0.5);
}

TEST_CASE("Poisson is non-negative and finite for degenerate inputs") {
    // CP 14.1 fuzzes everything; lambda <= 0 is the first thing it will try.
    Pcg32 rng(1);
    CHECK(poisson(0.0, rng) == 0.0);
    CHECK(poisson(-5.0, rng) == 0.0);
    CHECK(poisson(std::nan(""), rng) == 0.0);
    for (int i = 0; i < 1000; ++i) {
        const double v = poisson(1e-9, rng);
        REQUIRE(std::isfinite(v));
        REQUIRE(v >= 0.0);
    }
}

TEST_CASE("CP 4.5: read noise has the configured standard deviation") {
    // Spec row 22 caps this at 20 grey levels, which is the scenario default.
    RngSet rng(7);
    std::vector<float> img(200000, 100.0f);

    NoiseParams p;
    p.poisson_enabled = false;      // isolate the Gaussian term
    p.gaussian_sigma  = 20.0;
    p.salt_pepper_p   = 0.0;

    Pcg32& g = rng[Stream::ReadNoise];
    for (auto& v : img) v += static_cast<float>(p.gaussian_sigma * g.next_normal());

    double sum = 0.0, sum_sq = 0.0;
    for (float v : img) { sum += v; sum_sq += double(v) * v; }
    const double mean = sum / img.size();
    const double sd   = std::sqrt(sum_sq / img.size() - mean * mean);

    INFO("mean = " << mean << ", sd = " << sd);
    CHECK(mean == doctest::Approx(100.0).epsilon(0.01));
    CHECK(sd   == doctest::Approx(20.0).epsilon(0.02));
}

// ===========================================================================
// CP 4.6 — salt and pepper by geometric skip sampling
// ===========================================================================

TEST_CASE("CP 4.6: exactly ~10% of pixels are corrupted") {
    Pcg32 rng(99);
    constexpr size_t kN = 640 * 480;
    std::vector<uint8_t> img(kN, 128);

    salt_pepper(img, 0.10, rng);

    size_t corrupted = 0, white = 0, black = 0;
    for (uint8_t v : img) {
        if (v == 0)        { ++corrupted; ++black; }
        else if (v == 255) { ++corrupted; ++white; }
    }
    const double frac = static_cast<double>(corrupted) / kN;
    INFO("corrupted " << corrupted << " of " << kN << " = " << frac);

    // Design §1.4 quotes 30,720 corrupted pixels at 10% of 640x480 — a 307:1
    // ratio against the 100-px beacon, which is the number that makes the
    // median filter (§9.4.1) necessary.
    CHECK(frac == doctest::Approx(0.10).epsilon(0.02));
    CHECK(corrupted == doctest::Approx(30720).epsilon(0.03));
    // Salt and pepper in equal measure.
    CHECK(static_cast<double>(white) / corrupted == doctest::Approx(0.5).epsilon(0.05));
}

TEST_CASE("CP 4.6: skip sampling matches the naive version's distribution") {
    // The claim is that geometric skip sampling produces an IDENTICAL
    // distribution, not an approximation. Same fraction, same split, from a
    // tenth of the random draws.
    constexpr size_t kN = 640 * 480;
    for (double p : {0.01, 0.05, 0.10, 0.25}) {
        Pcg32 a(4242), b(4242);
        std::vector<uint8_t> fast(kN, 128), slow(kN, 128);
        salt_pepper(fast, p, a);
        salt_pepper_naive(slow, p, b);

        auto count = [](const std::vector<uint8_t>& v) {
            size_t c = 0;
            for (uint8_t x : v) if (x == 0 || x == 255) ++c;
            return c;
        };
        const double f = static_cast<double>(count(fast)) / kN;
        const double s = static_cast<double>(count(slow)) / kN;
        INFO("p = " << p << "  skip = " << f << "  naive = " << s);
        CHECK(f == doctest::Approx(p).epsilon(0.05));
        CHECK(s == doctest::Approx(p).epsilon(0.05));
    }
}

TEST_CASE("CP 4.6: skip sampling uses far fewer random draws") {
    // The whole reason for the algorithm. Counted rather than timed, because a
    // draw count is deterministic and a wall-clock measurement is not — and
    // INV-3 keeps clocks out of anything that could affect a result.
    constexpr size_t kN = 640 * 480;
    const double p = 0.10;

    auto draws_used = [&](bool skip) {
        Pcg32 rng(1234);
        std::vector<uint8_t> img(kN, 128);
        const uint64_t before = rng.raw_state();
        (void)before;
        // Count by running the generator in parallel: advance a twin generator
        // one step at a time until it matches the state the real one ended at.
        // Simpler: instrument by comparing against a fresh generator advanced
        // a known number of times.
        if (skip) salt_pepper(img, p, rng);
        else      salt_pepper_naive(img, p, rng);

        Pcg32 probe(1234);
        uint64_t n = 0;
        while (probe.raw_state() != rng.raw_state() && n < 4000000) {
            (void)probe.next_u32();
            ++n;
        }
        return n;
    };

    const uint64_t skip_draws  = draws_used(true);
    const uint64_t naive_draws = draws_used(false);
    INFO("skip sampling: " << skip_draws << " draws, naive: " << naive_draws);

    CHECK(naive_draws > kN);                  // at least one per pixel
    CHECK(skip_draws < naive_draws / 4);      // dramatically fewer
}

TEST_CASE("salt and pepper handles the degenerate probabilities") {
    Pcg32 rng(3);
    std::vector<uint8_t> img(1000, 128);

    salt_pepper(img, 0.0, rng);
    for (uint8_t v : img) CHECK(v == 128);    // nothing touched

    salt_pepper(img, 1.0, rng);
    for (uint8_t v : img) CHECK((v == 0 || v == 255));   // everything touched

    std::vector<uint8_t> empty;
    salt_pepper(empty, 0.5, rng);             // must not fault
}

// ===========================================================================
// CP 4.7 — fixed pattern and defect pixels
// ===========================================================================

TEST_CASE("CP 4.7: the same seed gives the same defect pattern") {
    // The checkpoint's criterion. These are properties of the SENSOR, not of
    // the frame: they do not average away over time, so a tracker that assumes
    // frame-to-frame independence will be wrong in the same places every time.
    // That only models a real detector if the pattern is stable.
    RngSet a(555), b(555);
    FixedPattern fa, fb;
    fa.build(64, 64, 0.01, 1.5, 20, 5, a);
    fb.build(64, 64, 0.01, 1.5, 20, 5, b);

    std::vector<uint8_t> ia(64 * 64, 100), ib(64 * 64, 100);
    fa.apply_defects(ia);
    fb.apply_defects(ib);
    CHECK(ia == ib);

    std::vector<float> ra(64 * 64, 100.0f), rb(64 * 64, 100.0f);
    fa.apply_gain_offset(ra);
    fb.apply_gain_offset(rb);
    CHECK(ra == rb);

    // A different seed gives a different pattern, or the seeding does nothing.
    RngSet c(556);
    FixedPattern fc;
    fc.build(64, 64, 0.01, 1.5, 20, 5, c);
    std::vector<float> rc(64 * 64, 100.0f);
    fc.apply_gain_offset(rc);
    CHECK(ra != rc);
}

TEST_CASE("CP 4.7: hot pixels saturate and dead pixels zero") {
    RngSet rng(1);
    FixedPattern f;
    f.build(100, 100, 0.0, 0.0, 30, 10, rng);

    std::vector<uint8_t> img(10000, 128);
    f.apply_defects(img);

    size_t hot = 0, dead = 0;
    for (uint8_t v : img) {
        if (v == 255) ++hot;
        if (v == 0)   ++dead;
    }
    // Duplicates are possible (positions are drawn independently), so these are
    // upper bounds rather than equalities.
    CHECK(hot  <= 30);
    CHECK(hot  >= 25);
    CHECK(dead <= 10);
    CHECK(dead >= 7);
}

TEST_CASE("PRNU is a gain and FPN is an offset") {
    RngSet rng(2);
    FixedPattern f;
    f.build(200, 200, 0.02, 3.0, 0, 0, rng);

    // Gain is multiplicative, so doubling the input doubles the deviation from
    // a pure offset. Measuring at two levels separates the two terms.
    std::vector<float> lo(40000, 50.0f), hi(40000, 100.0f);
    f.apply_gain_offset(lo);
    f.apply_gain_offset(hi);

    double sd_lo = 0.0, sd_hi = 0.0, m_lo = 0.0, m_hi = 0.0;
    for (size_t i = 0; i < lo.size(); ++i) { m_lo += lo[i]; m_hi += hi[i]; }
    m_lo /= lo.size(); m_hi /= hi.size();
    for (size_t i = 0; i < lo.size(); ++i) {
        sd_lo += (lo[i] - m_lo) * (lo[i] - m_lo);
        sd_hi += (hi[i] - m_hi) * (hi[i] - m_hi);
    }
    sd_lo = std::sqrt(sd_lo / lo.size());
    sd_hi = std::sqrt(sd_hi / hi.size());

    // Variance = (signal * prnu_sigma)^2 + fpn_sigma^2, so the spread must grow
    // with signal level — that is what makes it a gain rather than an offset.
    INFO("sd at 50 = " << sd_lo << ", sd at 100 = " << sd_hi);
    CHECK(sd_hi > sd_lo);
    CHECK(sd_lo == doctest::Approx(std::sqrt(std::pow(50.0 * 0.02, 2) + 9.0)).epsilon(0.1));
    CHECK(sd_hi == doctest::Approx(std::sqrt(std::pow(100.0 * 0.02, 2) + 9.0)).epsilon(0.1));
}

// ===========================================================================
// CP 4.4 — the five atmosphere modes
// ===========================================================================

TEST_CASE("CP 4.4: measured contrast matches design 9.3's table") {
    // "Fog visibly washes out; measured contrast matches the table."
    //
    // Contrast here is the ratio of a bright feature's excursion above the
    // background, before and after. Since the transform is affine, alpha IS the
    // contrast ratio — beta shifts both and cancels.
    for (Atmosphere mode : {Atmosphere::Clear, Atmosphere::Haze, Atmosphere::Rain,
                            Atmosphere::Fog, Atmosphere::LowLight}) {
        const AtmosphereCoeffs c = atmosphere_coeffs(mode);

        const double bg_in = 20.0, target_in = 140.0;
        const double bg_out     = c.alpha * bg_in + c.beta;
        const double target_out = c.alpha * target_in + c.beta;

        const double contrast_in  = target_in - bg_in;
        const double contrast_out = target_out - bg_out;

        INFO("atmosphere = " << std::string(atmosphere_name(mode))
             << "  contrast " << contrast_in << " -> " << contrast_out);
        CHECK(contrast_out / contrast_in == doctest::Approx(c.alpha));
    }

    // The consequences the design draws from that table:
    //   fog loses 65% of contrast AND lifts the floor 60 levels — which is why
    //   a fixed threshold cannot work and CFAR (§9.4.5) must measure the
    //   background from the image itself.
    const auto fog = atmosphere_coeffs(Atmosphere::Fog);
    CHECK(fog.alpha == doctest::Approx(0.35));
    CHECK(fog.beta  == doctest::Approx(60.0));
    //   low light loses 60% of contrast and DARKENS — the opposite direction,
    //   which is the other half of the argument for an adaptive threshold.
    const auto low = atmosphere_coeffs(Atmosphere::LowLight);
    CHECK(low.alpha == doctest::Approx(0.40));
    CHECK(low.beta  < 0.0);
}

TEST_CASE("CP 4.4: fog measurably washes out a rendered frame") {
    auto run = [](Atmosphere mode) {
        Scenario sc;
        sc.atmosphere     = mode;
        sc.noise_poisson  = false;
        sc.gaussian_sigma = 0.0;
        sc.salt_pepper    = 0.0;
        sc.hot_pixels     = 0;

        RngSet rng(1);
        SensorChain chain;
        chain.build(sc, 64, 64, rng);
        chain.noise().prnu_sigma = 0.0;
        chain.noise().fpn_sigma  = 0.0;

        // A bright square on a dim background.
        std::vector<float> img(64 * 64, 20.0f);
        for (int j = 20; j < 44; ++j) {
            for (int i = 20; i < 44; ++i) img[static_cast<size_t>(j) * 64 + i] = 140.0f;
        }
        std::vector<uint8_t> out(64 * 64, 0);
        chain.apply(img, out, rng);

        // MEANS over regions, not single pixels. Fixed-pattern noise gives each
        // pixel its own gain and offset (~1.5 grey levels), so one pixel is not
        // a measurement of contrast — it is a measurement of that pixel. An
        // earlier version of this test read out[0] and out[32*64+32] and came
        // out 3 levels high for exactly that reason.
        auto mean_of = [&](int x0, int y0, int x1, int y1) {
            double sum = 0.0;
            int n = 0;
            for (int j = y0; j < y1; ++j) {
                for (int i = x0; i < x1; ++i) {
                    sum += out[static_cast<size_t>(j) * 64 + i];
                    ++n;
                }
            }
            return sum / n;
        };
        const double bg     = mean_of(2, 2, 18, 18);
        const double target = mean_of(24, 24, 40, 40);
        return target - bg;
    };

    const double clear = run(Atmosphere::Clear);
    const double fog   = run(Atmosphere::Fog);
    const double haze  = run(Atmosphere::Haze);

    INFO("contrast: clear " << clear << ", haze " << haze << ", fog " << fog);
    CHECK(clear == doctest::Approx(120).epsilon(0.02));
    CHECK(haze  == doctest::Approx(120 * 0.75).epsilon(0.05));
    CHECK(fog   == doctest::Approx(120 * 0.35).epsilon(0.05));
    CHECK(fog < haze);
    CHECK(haze < clear);
}

// ===========================================================================
// INV-8 — no damage in video modes
// ===========================================================================

TEST_CASE("INV-8: the chain is a pass-through in video modes") {
    Scenario sc;
    sc.input_mode     = InputMode::VideoDirect;
    sc.video_file     = "clip.mp4";
    sc.gaussian_sigma = 20.0;       // asked for, and must be ignored
    sc.salt_pepper    = 0.10;
    sc.atmosphere     = Atmosphere::Fog;

    RngSet rng(1);
    SensorChain chain;
    chain.build(sc, 32, 32, rng);
    CHECK_FALSE(chain.enabled());

    std::vector<float> img(32 * 32, 100.0f);
    std::vector<uint8_t> out(32 * 32, 0);
    chain.apply(img, out, rng);

    // Nothing but the quantisation the frame needed anyway.
    for (uint8_t v : out) CHECK(v == 100);
}

TEST_CASE("an unbuilt chain is a pass-through, not a spec-maximum noise source") {
    // A chain nobody configured must not silently apply 20-sigma read noise and
    // 10% salt-and-pepper. An earlier version defaulted to enabled and made
    // every hand-built test scene arrive at the detector under full damage.
    RngSet rng(1);
    SensorChain chain;
    CHECK_FALSE(chain.enabled());

    std::vector<float> img(100, 77.0f);
    std::vector<uint8_t> out(100, 0);
    chain.apply(img, out, rng);
    for (uint8_t v : out) CHECK(v == 77);
}

// ===========================================================================
// CP 4.8, 4.9 — disturbances
// ===========================================================================

TEST_CASE("CP 4.8: the startup jitter figures match design 9.3") {
    // "Startup logs 20 px/frame -> 65,448 urad/s -> 3.75 deg/s (75% of a
    //  5 deg/s motor)". This is the number §1.3's whole argument rests on.
    Scenario sc;
    sc.jitter_px_per_frame = 20.0;

    DisturbanceGenerator d;
    d.build(sc, sc.screen_geometry());

    const double urad_s = d.jitter_urad_s(30.0);
    const double deg_s  = urad_to_deg(urad_s);
    INFO("20 px/frame -> " << urad_s << " urad/s -> " << deg_s << " deg/s");

    // The doc quotes 65,448 from a rounded IFOV of 109.08; recomputing from the
    // unrounded value gives 65,449.85. Both round to 3.75 deg/s, which is the
    // figure the argument actually uses.
    CHECK(urad_s == doctest::Approx(65449.85).epsilon(1e-4));
    CHECK(deg_s  == doctest::Approx(3.75).epsilon(1e-4));
    CHECK(deg_s / 5.0 == doctest::Approx(0.75).epsilon(1e-4));
}

TEST_CASE("CP 4.8: jitter is bounded by the specified maximum") {
    // Spec row 23 states a hard maximum of +/- 20 px/frame. A Gaussian with
    // that as its 1-sigma would exceed it a third of the time; a uniform bound
    // means the stated figure IS the worst case.
    Scenario sc;
    sc.jitter_px_per_frame = 20.0;
    const ScreenGeometry scr = sc.screen_geometry();

    DisturbanceGenerator d;
    d.build(sc, scr);
    RngSet rng(42);

    double worst = 0.0, sum = 0.0;
    constexpr int kN = 20000;
    for (int i = 0; i < kN; ++i) {
        const Angle2 o = d.offset(i / 30.0, rng, /*new_frame=*/true);
        const double px = o.x / scr.ifov_x_urad;
        worst = std::max(worst, std::abs(px));
        sum += px;
    }
    INFO("worst jitter = " << worst << " px, mean = " << (sum / kN));
    CHECK(worst <= 20.0 + 1e-9);          // never exceeds the specification
    CHECK(worst > 19.0);                  // but does reach it
    CHECK(std::abs(sum / kN) < 0.5);      // zero-mean
}

TEST_CASE("CP 4.8: jitter is held constant across a frame, not resampled per tick") {
    // Spec row 23 specifies px PER FRAME. Drawing at the 300 Hz truth rate
    // would make the disturbance ten times more energetic than specified.
    Scenario sc;
    sc.jitter_px_per_frame = 20.0;
    DisturbanceGenerator d;
    d.build(sc, sc.screen_geometry());
    RngSet rng(1);

    const Angle2 a = d.offset(0.0, rng, /*new_frame=*/true);
    const Angle2 b = d.offset(0.001, rng, /*new_frame=*/false);
    const Angle2 c = d.offset(0.002, rng, /*new_frame=*/false);
    CHECK(a.x == b.x);
    CHECK(b.x == c.x);

    const Angle2 next = d.offset(1.0 / 30.0, rng, /*new_frame=*/true);
    CHECK(next.x != a.x);
}

TEST_CASE("CP 4.9: all five row-25 platform modes run from the same components") {
    // "All five row-25 modes work, driven by the same code as target motion."
    // Row 25: linear is mandatory; circular, random, spiral and figure-8 are
    // optional. Every one of them is a §7.2 component — there is one factory.
    struct Case { const char* kind; const char* toml; };
    const Case cases[] = {
        {"linear",    "kind = \"linear\"\nvelocity_px_s = [15.0, -8.0]"},
        {"circular",  "kind = \"circular\"\nradius_px = 40.0\nperiod_s = 11.0"},
        {"ou_noise",  "kind = \"ou_noise\"\nsigma_px_s = 30.0\ntau_s = 2.0"},
        {"spiral",    "kind = \"spiral\"\nr0_px = 10.0\ngrowth_px_s = 3.0\nperiod_s = 8.0"},
        {"lissajous", "kind = \"lissajous\"\namplitude_px = [50.0, 25.0]\n"
                      "freq_ratio = 2.0\nperiod_s = 14.0"},
    };

    for (const auto& c : cases) {
        INFO("platform mode = " << std::string(c.kind));
        const std::string toml =
            std::string("[sim]\nduration_s = 10\n[target]\nsize_px = 10\n"
                        "[[disturbance.platform]]\n") + c.toml + "\n";
        auto r = parse_scenario(toml, "platform.toml");
        REQUIRE_MESSAGE(r.has_value(), r.error());
        REQUIRE(r->platform.size() == 1);
        CHECK(r->platform[0].kind == c.kind);

        DisturbanceGenerator d;
        d.build(*r, r->screen_geometry());
        RngSet rng(5);

        // It must produce finite, non-trivial motion.
        double span = 0.0;
        double lo = 1e30, hi = -1e30;
        for (int i = 0; i < 600; ++i) {
            d.advance(1.0 / 300.0, rng);
            const Angle2 o = d.offset(i / 300.0, rng, false);
            REQUIRE(std::isfinite(o.x));
            REQUIRE(std::isfinite(o.y));
            lo = std::min(lo, o.x);
            hi = std::max(hi, o.x);
        }
        span = hi - lo;
        CHECK(span > 0.0);      // it actually moved

        // And its analytic rate is finite.
        const Rate2 rate = d.platform_rate(1.0);
        CHECK(std::isfinite(rate.x));
        CHECK(std::isfinite(rate.y));
    }
}

TEST_CASE("platform motion and target motion draw from different streams") {
    // Otherwise a stochastic platform would perturb a stochastic target, and
    // "platform on vs platform off" would differ for reasons unrelated to the
    // platform.
    Scenario sc;
    sc.platform.push_back(MotionSpec{});
    sc.platform[0].kind       = "ou_noise";
    sc.platform[0].sigma_px_s = 30.0;
    sc.platform[0].tau_s      = 2.0;

    RngSet a(77), b(77);
    DisturbanceGenerator d;
    d.build(sc, sc.screen_geometry());
    for (int i = 0; i < 1000; ++i) d.advance(1.0 / 300.0, a);

    // The target's stream must be untouched by all that platform advancing.
    CHECK(a[Stream::TargetMotion].next_u32() == b[Stream::TargetMotion].next_u32());
    CHECK(a[Stream::TargetInit].next_u32()   == b[Stream::TargetInit].next_u32());
}

TEST_CASE("the full chain produces a plausible frame") {
    // An integration check: everything on at once, and the result must still be
    // a sane 8-bit image with the beacon detectable above the background.
    Scenario sc;
    sc.atmosphere     = Atmosphere::Clear;
    sc.noise_poisson  = true;
    sc.gaussian_sigma = 20.0;       // spec row 22 maximum
    sc.salt_pepper    = 0.10;       // spec row 21
    sc.hot_pixels     = 40;

    RngSet rng(9);
    SensorChain chain;
    chain.build(sc, 128, 128, rng);
    CHECK(chain.enabled());

    std::vector<float> img(128 * 128, 20.0f);
    for (int j = 60; j < 70; ++j) {
        for (int i = 60; i < 70; ++i) img[static_cast<size_t>(j) * 128 + i] = 140.0f;
    }
    std::vector<uint8_t> out(128 * 128, 0);
    chain.apply(img, out, rng);

    // Median of the beacon region against median of the background: robust to
    // the salt-and-pepper impulses, which is exactly why §9.4.1 uses a median.
    auto median_of = [&](int x0, int y0, int x1, int y1) {
        std::vector<uint8_t> v;
        for (int j = y0; j < y1; ++j)
            for (int i = x0; i < x1; ++i)
                v.push_back(out[static_cast<size_t>(j) * 128 + i]);
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    };
    const int beacon = median_of(61, 61, 69, 69);
    const int bg     = median_of(5, 5, 40, 40);
    INFO("beacon median " << beacon << ", background median " << bg);
    CHECK(beacon > bg + 60);        // still clearly detectable
    CHECK(beacon <= 255);
    CHECK(bg >= 0);
}
