// tests/degrade/test_turbulence.cpp — audit P2-1, roadmap Phase 3.
//
// The roadmap states one acceptance criterion for this feature and it is a
// SPECTRUM, not a variance:
//
//     "At r0 = 5 cm, AoA PSD must follow f^(-11/3) over the resolved band,
//      asserted by a test."
//
// That is the right criterion and it is the one this file leads with. Getting
// the variance right is easy — any Gaussian scaled to sigma_AoA does that.
// Getting the SPECTRUM right is what separates a turbulence model from
// "Gaussian noise with a physics word in the variable name", and the -11/3
// exponent is not a free parameter: it follows from the Kolmogorov
// refractive-index spectrum. If this test's fitted slope drifts, the model has
// stopped being Kolmogorov and the claim in the report is no longer true.
//
// Everything is measured against theory rather than against a recorded golden
// output, for the reason test_noise.cpp gives at the top: a golden file says
// "it still does what it did", and a theory comparison says "it does what the
// physics does".

#include <doctest/doctest.h>

#include "degrade/disturbance.hpp"
#include "degrade/turbulence.hpp"
#include "scenario/schema.hpp"

#include <cmath>
#include <complex>
#include <vector>

using namespace sat;

namespace {

/// The bins the slope is fitted on: log-spaced, so each decade of frequency
/// carries the same weight. Fitting every bin in the range would weight the
/// top decade far more heavily than the bottom one purely because it contains
/// more bins, and the fitted slope would be a statement about bin counting
/// rather than about the spectrum.
std::vector<int> fit_bins(int lo, int hi) {
    std::vector<int> bins;
    for (double b = lo; b <= hi; b *= 1.25) {
        const int bi = static_cast<int>(b);
        if (bins.empty() || bins.back() != bi) bins.push_back(bi);
    }
    return bins;
}

/// Nuttall's 4-term window with a continuous first derivative.
///
/// THE WINDOW IS NOT OPTIONAL, and it is worth saying why, because the first
/// version of this test did not have one and reported f^(-2.51) for a process
/// that is f^(-3.64) by construction.
///
/// A rectangular window's spectral sidelobes fall only as f^(-2) in power.
/// Measuring a spectrum that falls as f^(-11/3) through one means that, past
/// the point where the true signal drops below the window's own skirt, the
/// periodogram stops measuring the signal and starts measuring the window —
/// it flattens out towards f^(-2), which is exactly the wrong answer, in
/// exactly the direction that would make a broken model look plausible.
///
/// Across the band here the true spectrum spans some 150 dB from the flat
/// low-frequency shelf to the top bin. This window gives -93 dB peak sidelobes
/// AND an 18 dB/octave rolloff, so leakage stays far below the signal
/// everywhere it is measured.
double nuttall(size_t i, size_t n) {
    constexpr double kTwoPi = 6.283185307179586476925286766559;
    const double x = kTwoPi * static_cast<double>(i) / static_cast<double>(n);
    return 0.3635819 - 0.4891775 * std::cos(x)
                     + 0.1365995 * std::cos(2.0 * x)
                     - 0.0106411 * std::cos(3.0 * x);
}

/// Naive DFT power at one frequency bin. O(N) per bin, and only the ~10 bins
/// that are actually fitted are ever asked for, which is cheaper and far
/// easier to read than pulling an FFT into a test.
double power_at_bin(const std::vector<double>& x, int bin) {
    const size_t n = x.size();
    const double w = -2.0 * 3.14159265358979323846 * static_cast<double>(bin)
                     / static_cast<double>(n);
    double re = 0.0, im = 0.0;
    for (size_t t = 0; t < n; ++t) {
        const double a = w * static_cast<double>(t);
        re += x[t] * std::cos(a);
        im += x[t] * std::sin(a);
    }
    return (re * re + im * im) / static_cast<double>(n);
}

/// Least-squares slope of log10(power) against log10(bin) over [lo, hi].
///
/// Averaged over `runs` independent realisations first. A single periodogram
/// of a stochastic process is chi-squared with 2 degrees of freedom — its
/// scatter is as large as its mean, whatever the length — so fitting one
/// realisation would produce a slope estimate whose error is dominated by that
/// scatter rather than by any property of the model. Averaging periodograms is
/// the standard cure and it is why this test is stable rather than flaky.
double fit_psd_slope(double beta, int lo_bin, int hi_bin, int n, int runs) {
    const std::vector<int> bins = fit_bins(lo_bin, hi_bin);
    std::vector<double> mean_power(bins.size(), 0.0);

    for (int r = 0; r < runs; ++r) {
        PowerLawNoise gen;
        gen.configure(beta, 1.0);
        Pcg32 g(static_cast<uint64_t>(1000 + r));

        // Discard a burn-in long enough for BOTH stages to reach stationarity.
        // The ring starts zeroed and the integrators start at zero, so the
        // opening samples are a ramp into the process; the integrators' time
        // constant is 163 frames, and 8 of those per stage is ample. Without
        // this the low-frequency end of every periodogram is the transient.
        for (int i = 0; i < 4000; ++i) (void)gen.next(g);

        std::vector<double> x(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) x[static_cast<size_t>(i)] = gen.next(g);

        // Remove the mean before windowing. An f^(-11/3) process wanders, so
        // any finite window has a large DC offset that the window would smear
        // across the low bins.
        double mean = 0.0;
        for (const double v : x) mean += v;
        mean /= static_cast<double>(n);
        for (size_t i = 0; i < x.size(); ++i) {
            x[i] = (x[i] - mean) * nuttall(i, x.size());
        }

        for (size_t bi = 0; bi < bins.size(); ++bi) {
            mean_power[bi] += power_at_bin(x, bins[bi]) / runs;
        }
    }

    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (size_t i = 0; i < bins.size(); ++i) {
        const double lx = std::log10(static_cast<double>(bins[i]));
        const double ly = std::log10(mean_power[i]);
        sx += lx; sy += ly; sxx += lx * lx; sxy += lx * ly;
    }
    const double m = static_cast<double>(bins.size());
    return (m * sxy - sx * sy) / (m * sxx - sx * sx);
}

Scenario clear_scenario() {
    Scenario sc;
    sc.jitter_px_per_frame = 0.0;   // isolate turbulence from row 23
    sc.platform.clear();            // and from row 25
    return sc;
}

}  // namespace

// ===========================================================================
// THE ACCEPTANCE CRITERION — the Kolmogorov exponent
// ===========================================================================

// The measurement band, and why it is these numbers.
//
// A discrete filter is not a continuum power law everywhere, and a test that
// ignores that is measuring the discretisation rather than the model:
//
//   * BELOW the integrators' corner (one cycle per 1024 samples) the spectrum
//     deliberately flattens — that is the outer-scale saturation the model
//     documents. Measuring there would report the flat part.
//   * NEAR Nyquist, |1 - e^(-iw)| = 2 sin(w/2) stops being w, so the
//     synthesised spectrum necessarily departs from the power law. Measuring
//     there would report the sampling.
//
// Bins 100-800 of an 8192-point transform put w in [0.077, 0.61] rad: a
// factor of eight above the corner and comfortably below Nyquist. Evaluating
// the cascade's exact analytic response over exactly this band gives a fitted
// slope of -3.640 against the ideal -3.667, so 0.027 of the tolerance below
// is known discretisation bias and the rest is estimator scatter.
constexpr int kFitN   = 8192;
constexpr int kFitLo  = 100;
constexpr int kFitHi  = 800;
constexpr int kFitRuns = 24;

TEST_CASE("P2-1: the synthesised AoA spectrum follows f^(-11/3)") {
    const double slope = fit_psd_slope(11.0 / 3.0, kFitLo, kFitHi, kFitN, kFitRuns);
    MESSAGE("fitted AoA PSD slope = " << slope << " (Kolmogorov: -3.6667)");

    // 0.15 is ~4% of the exponent: wide enough that the residual periodogram
    // scatter at 24 realisations plus the 0.027 of discretisation bias do not
    // make this flaky, and narrow enough that it rejects f^(-3) or f^(-4)
    // outright. Those are the confusions that actually matter — a plain random
    // walk lands at -2, a naive two-pole cascade at -4, and the first attempt
    // at this model (a direct 256-tap Kasdin FIR at beta = 11/3) landed at
    // -1.99. Every one of those would fail here, which is the point.
    CHECK(std::abs(slope - (-11.0 / 3.0)) < 0.15);
}

TEST_CASE("P2-1: the synthesiser is a power law, not one fixed shape") {
    // If the exponent were baked in — a cascade with hardcoded corners, say —
    // asking for a different beta would still give -11/3. Asking for f^(-2)
    // (Brownian) and getting -2 is what proves the exponent split and the
    // coefficient recursion are actually doing the work.
    const double slope2 = fit_psd_slope(2.0, kFitLo, kFitHi, kFitN, kFitRuns);
    MESSAGE("fitted slope for beta = 2: " << slope2);
    CHECK(std::abs(slope2 - (-2.0)) < 0.15);
}

// ===========================================================================
// Variance — the easy half, but it still has to be right
// ===========================================================================

TEST_CASE("P2-1: AoA variance matches the Fried-parameter formula") {
    // sigma^2 = 0.182 * (lambda/D)^2 * (D/r0)^(5/3).
    TurbulenceParams p;
    p.enabled       = true;
    p.r0_m          = 0.05;
    p.aperture_m    = 1.0;
    p.wavelength_nm = 1550.0;

    // Worked by hand so a refactor of the helper cannot quietly redefine it:
    //   lambda/D   = 1.55e-6
    //   (D/r0)^5/3 = 20^(5/3) = 147.03
    //   sigma^2    = 0.182 * 2.4025e-12 * 147.03 = 6.4287e-11 rad^2
    //   sigma      = 8.018e-6 rad = 8.018 urad
    CHECK(p.aoa_sigma_urad() == doctest::Approx(8.018).epsilon(0.002));

    // D^(-1/6) net dependence: a 4x bigger aperture averages tilt DOWN, and
    // the sign of that is the part that is easy to get backwards.
    TurbulenceParams big = p;
    big.aperture_m = 4.0;
    CHECK(big.aoa_sigma_urad() < p.aoa_sigma_urad());
    CHECK(big.aoa_sigma_urad() ==
          doctest::Approx(p.aoa_sigma_urad() * std::pow(4.0, -1.0 / 6.0)).epsilon(1e-6));

    // Worse seeing (smaller r0) must mean more jitter.
    TurbulenceParams bad = p;
    bad.r0_m = 0.02;
    CHECK(bad.aoa_sigma_urad() > p.aoa_sigma_urad());

    // Disabled is exactly zero, not "small".
    TurbulenceParams off = p;
    off.enabled = false;
    CHECK(off.aoa_sigma_urad() == 0.0);
}

TEST_CASE("P2-1: the realised AoA standard deviation is the configured one") {
    TurbulenceParams p;
    p.enabled       = true;
    p.r0_m          = 0.05;
    p.aperture_m    = 1.0;
    p.wavelength_nm = 1550.0;

    TurbulenceModel m;
    m.build(p, 30.0);
    RngSet rng(4242);

    // Burn in past the zeroed ring, then measure.
    for (int i = 0; i < 512; ++i) m.step(rng);

    double sx = 0.0, sxx = 0.0;
    constexpr int kN = 60000;
    for (int i = 0; i < kN; ++i) {
        m.step(rng);
        const double v = m.aoa_offset().x;
        sx += v;
        sxx += v * v;
    }
    const double mean = sx / kN;
    const double sd   = std::sqrt(sxx / kN - mean * mean);
    MESSAGE("realised sigma = " << sd << " urad, configured "
                                << p.aoa_sigma_urad());

    // 12% is a wide gate deliberately. An f^(-11/3) process has most of its
    // power at the lowest resolved frequencies, so the sample variance of even
    // a 60k-sample window is a high-variance estimator — that is a property of
    // the physics, not slack in the implementation. The tight statement about
    // this model is the SPECTRUM test above; this one exists to catch a
    // normalisation that is out by a factor, which is the realistic bug.
    CHECK(sd == doctest::Approx(p.aoa_sigma_urad()).epsilon(0.12));
    CHECK(std::abs(mean) < 0.35 * p.aoa_sigma_urad());   // zero-mean
}

// ===========================================================================
// Scintillation
// ===========================================================================

TEST_CASE("P2-1: scintillation is log-normal with unit mean and the stated index") {
    TurbulenceParams p;
    p.enabled             = true;
    p.r0_m                = 0.05;
    p.aperture_m          = 1.0;
    p.scintillation_index = 0.4;

    TurbulenceModel m;
    m.build(p, 30.0);
    RngSet rng(99);

    double s = 0.0, ss = 0.0;
    constexpr int kN = 200000;
    for (int i = 0; i < kN; ++i) {
        m.step(rng);
        const double g = m.irradiance_gain(0);
        CHECK(g > 0.0);            // irradiance cannot go negative
        s += g;
        ss += g * g;
    }
    const double mean = s / kN;
    const double var  = ss / kN - mean * mean;
    MESSAGE("scintillation mean = " << mean << ", index = " << var / (mean * mean));

    // E[I] = 1 is what makes the ablation honest: turning scintillation on
    // must not also brighten (or dim) the beacon, or the experiment would be
    // measuring two changes at once.
    CHECK(mean == doctest::Approx(1.0).epsilon(0.02));
    // Var(I)/E[I]^2 is the definition of the scintillation index.
    CHECK(var / (mean * mean) == doctest::Approx(0.4).epsilon(0.06));
}

TEST_CASE("P2-1: beacon and decoy scintillate independently") {
    // If they shared a gain, a decoy that faded in step with the beacon would
    // be a gift to the tracker — the two would stay perfectly distinguishable
    // by brightness ratio through every fade. Sources this far apart on the
    // screen are many isoplanatic angles apart and must be independent.
    TurbulenceParams p;
    p.enabled             = true;
    p.scintillation_index = 0.5;

    TurbulenceModel m;
    m.build(p, 30.0);
    RngSet rng(7);

    double sa = 0.0, sb = 0.0, sab = 0.0, saa = 0.0, sbb = 0.0;
    constexpr int kN = 100000;
    for (int i = 0; i < kN; ++i) {
        m.step(rng);
        const double a = m.irradiance_gain(0);
        const double b = m.irradiance_gain(1);
        sa += a; sb += b; sab += a * b; saa += a * a; sbb += b * b;
    }
    const double ma = sa / kN, mb = sb / kN;
    const double cov = sab / kN - ma * mb;
    const double corr = cov / std::sqrt((saa / kN - ma * ma) * (sbb / kN - mb * mb));
    MESSAGE("beacon/decoy gain correlation = " << corr);
    CHECK(std::abs(corr) < 0.02);
}

TEST_CASE("P2-1: a slot past the cap reads back as no scintillation at all") {
    // Failing open at 1.0 rather than reading out of bounds. A beacon that
    // stops scintillating is a far better failure than one that silently
    // shares another emitter's gain, or than undefined behaviour.
    TurbulenceParams p;
    p.enabled             = true;
    p.scintillation_index = 0.5;
    TurbulenceModel m;
    m.build(p, 30.0);
    RngSet rng(11);
    m.step(rng);
    CHECK(m.irradiance_gain(TurbulenceModel::kMaxGraded) == 1.0);
    CHECK(m.irradiance_gain(TurbulenceModel::kMaxGraded + 99) == 1.0);
}

TEST_CASE("P2-1: index 0 means exactly 1.0, not nearly") {
    TurbulenceParams p;
    p.enabled             = true;
    p.scintillation_index = 0.0;
    TurbulenceModel m;
    m.build(p, 30.0);
    RngSet rng(3);
    for (int i = 0; i < 100; ++i) {
        m.step(rng);
        CHECK(m.irradiance_gain(0) == 1.0);
    }
}

// ===========================================================================
// INV-3 and the off-by-default guarantee
// ===========================================================================

TEST_CASE("INV-3: turbulence is bit-reproducible for a given seed") {
    TurbulenceParams p;
    p.enabled             = true;
    p.r0_m                = 0.05;
    p.scintillation_index = 0.3;

    auto run = [&](uint64_t seed) {
        TurbulenceModel m;
        m.build(p, 30.0);
        RngSet rng(seed);
        std::vector<double> out;
        for (int i = 0; i < 500; ++i) {
            m.step(rng);
            out.push_back(m.aoa_offset().x);
            out.push_back(m.aoa_offset().y);
            out.push_back(m.irradiance_gain(0));
        }
        return out;
    };

    const auto a = run(12345);
    const auto b = run(12345);
    REQUIRE(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        // Bit-identical, not approximately equal. INV-3 is a bit-exactness
        // claim and testing it with a tolerance would not test it.
        CHECK(a[i] == b[i]);
    }

    // A different seed must actually change the answer, or the test above
    // would pass on a model that returns a constant.
    const auto c = run(54321);
    bool differs = false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != c[i]) { differs = true; break; }
    }
    CHECK(differs);
}

TEST_CASE("P2-1: turbulence is off by default and draws nothing when off") {
    // The guarantee that makes this feature safe to land: every scenario
    // written before turbulence existed, and every number committed from one,
    // is bit-identical with this code present. If a disabled model consumed
    // even one draw from Stream::Atmosphere, every historical run's shot noise
    // would shift and the whole RESULTS.md table would silently change.
    const Scenario sc = clear_scenario();
    CHECK_FALSE(sc.turbulence.enabled);

    TurbulenceModel m;
    m.build(sc.turbulence, 30.0);
    CHECK_FALSE(m.enabled());

    RngSet before(777);
    RngSet after(777);
    for (int i = 0; i < 100; ++i) m.step(after);

    // Same stream, same position: the disabled model advanced nothing.
    CHECK(before[Stream::Atmosphere].next_u32()
          == after[Stream::Atmosphere].next_u32());
    CHECK(m.aoa_offset().x == 0.0);
    CHECK(m.aoa_offset().y == 0.0);
    CHECK(m.irradiance_gain(0) == 1.0);
}

TEST_CASE("P2-1: turbulence reaches the boresight through DisturbanceGenerator") {
    // The integration claim, not the physics one: AoA must arrive as a
    // BORESIGHT offset, beside row 23's jitter and row 25's platform motion,
    // and not as a pixel shift. With jitter and platform both zeroed, any
    // motion at all in the returned offset can only have come from turbulence.
    Scenario sc = clear_scenario();
    sc.turbulence.enabled    = true;
    sc.turbulence.r0_m       = 0.02;     // bad seeing, so the effect is large
    sc.turbulence.aperture_m = 1.0;

    DisturbanceGenerator d;
    d.build(sc, sc.screen_geometry(), sc.camera_hz);
    RngSet rng(2024);

    double max_abs = 0.0;
    for (int i = 0; i < 2000; ++i) {
        const Angle2 o = d.offset(static_cast<double>(i) / sc.camera_hz, rng, true);
        max_abs = std::max(max_abs, std::abs(o.x));
    }
    MESSAGE("peak turbulence-only boresight offset = " << max_abs << " urad");
    CHECK(max_abs > 0.0);
    CHECK(std::isfinite(max_abs));

    // And with turbulence off, the same scenario produces exactly nothing.
    Scenario off = clear_scenario();
    DisturbanceGenerator d2;
    d2.build(off, off.screen_geometry(), off.camera_hz);
    RngSet rng2(2024);
    for (int i = 0; i < 100; ++i) {
        const Angle2 o = d2.offset(static_cast<double>(i) / off.camera_hz, rng2, true);
        CHECK(o.x == 0.0);
        CHECK(o.y == 0.0);
    }
}

TEST_CASE("P2-1: the tilt knee is where the wind and the aperture put it") {
    TurbulenceParams p;
    p.enabled    = true;
    p.aperture_m = 1.0;
    p.wind_ms    = 5.0;
    // f_T = 0.24 * V / D
    CHECK(p.tilt_knee_hz() == doctest::Approx(1.2).epsilon(1e-9));

    // A 10 cm aperture pushes the knee to 12 Hz, which at 30 Hz sampling is
    // almost at Nyquist — i.e. a small aperture resolves almost none of the
    // inertial band. This is exactly the nuance worth being able to state:
    // the f^(-11/3) regime is visible to a large terminal, not to a webcam.
    p.aperture_m = 0.1;
    CHECK(p.tilt_knee_hz() == doctest::Approx(12.0).epsilon(1e-9));
}
