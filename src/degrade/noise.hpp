// degrade/noise.hpp — the statistical parts of the damage chain.
//
// Design §9.3 fixes the ORDER, and the order is physical rather than arbitrary:
//
//     1. Atmosphere      I <- alpha*I + beta                    row 24
//     2. Shot noise      I <- Poisson(I*k)/k                    row 21
//     3. Read noise      I <- I + Normal(0, sigma)              rows 21-22
//     4. Fixed pattern   I <- I*prnu[p] + fpn[p]                seeded once
//     5. Salt & pepper   10% of pixels -> 0 or 255              row 21
//     6. Hot/dead pixels fixed seeded positions
//     7. Clip and quantise to 8-bit
//
// Photons are attenuated by the atmosphere BEFORE they arrive, so shot noise
// applies to what actually lands on the detector. Read noise is added by the
// electronics after that. Fixed-pattern non-uniformity is a property of the
// detector, so it multiplies the accumulated signal. Salt-and-pepper models
// transmission and ADC faults, which happen last. Getting this order wrong
// would change the noise statistics the whole perception pipeline is tuned
// against — in particular it would change what CFAR measures as the local
// background.
//
// Every generator draws from its OWN named stream (core/rng.hpp), so turning
// one off does not shift the numbers the others see. That is what makes the
// ablations in §13.3 comparable.

#pragma once

#include "core/rng.hpp"
#include "scenario/scenario.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace sat {

// ---------------------------------------------------------------------------
// poisson — shot noise (spec row 21).
//
// Design §9.3 gives the branch explicitly, and the threshold matters:
//
//   lambda < 30   Knuth's product method. Exact, but its cost grows with
//                 lambda (it draws until a product falls below exp(-lambda),
//                 which takes ~lambda draws on average).
//   lambda >= 30  the normal approximation. A Poisson distribution with
//                 lambda = 30 already has skewness 1/sqrt(30) = 0.18 and is
//                 visually indistinguishable from a Gaussian of the same
//                 variance; below 8-bit quantisation the difference is
//                 unmeasurable.
//
// §9.3 adds a warning worth repeating: "the branch changes the number of RNG
// draws, which is fine because lambda is deterministic, but do not change the
// threshold between builds you intend to compare." Changing it would alter
// every subsequent draw in the stream and make two runs incomparable for a
// reason that has nothing to do with the change being studied.
[[nodiscard]] double poisson(double lambda, Pcg32& rng) noexcept;

// ---------------------------------------------------------------------------
// salt_pepper — impulse noise (spec row 21), by geometric skip sampling.
//
// The naive implementation draws once per pixel: 307,200 draws for a 640x480
// frame, of which 90% are discarded. Instead, the GAP between corrupted pixels
// is geometrically distributed, and a geometric variate comes from a single
// uniform:
//
//     gap = floor( log(1 - u) / log(1 - p) )
//
// So at p = 0.10 this draws ~30,720 times instead of 307,200 — a 10x reduction
// — while producing an IDENTICAL distribution, not an approximation to it.
// CP 4.6 asks for exactly this and for it to be "measurably faster than naive
// in --bench".
//
// Design §9.3 gives the algorithm; this adds the `rng.next_double_open()` call
// so the uniform is never exactly zero, which would make log(1-u) = 0 and the
// gap zero forever.
void salt_pepper(std::span<uint8_t> img, double p, Pcg32& rng) noexcept;

/// The naive per-pixel version, kept only as the correctness oracle and the
/// --bench comparison for the one above. Never used in a run.
void salt_pepper_naive(std::span<uint8_t> img, double p, Pcg32& rng) noexcept;

// ---------------------------------------------------------------------------
// FixedPattern — PRNU and FPN (design §9.3 step 4), drawn ONCE at load.
//
// A real detector's pixels do not respond identically. Photo-response
// non-uniformity (PRNU) is a multiplicative gain difference of order 1%; fixed-
// pattern noise (FPN) is an additive offset. Both are FIXED for a given sensor,
// which is exactly why they matter here: they do not average away over frames,
// so a tracker that assumes frame-to-frame independence will be systematically
// wrong in the same places every time.
//
// CP 4.7's criterion is "same seed -> same defect pattern across runs", which
// is why these are drawn once from a dedicated stream at load rather than
// per-frame.
// ---------------------------------------------------------------------------
class FixedPattern {
public:
    /// Build the maps. `prnu_sigma` is the fractional gain spread (0.01 = 1%),
    /// `fpn_sigma` the additive offset spread in grey levels.
    void build(int width, int height, double prnu_sigma, double fpn_sigma,
               int hot_pixels, int dead_pixels, RngSet& rng);

    /// Apply gain and offset. Steps 4 of the chain.
    void apply_gain_offset(std::span<float> img) const noexcept;

    /// Force hot pixels to saturation and dead pixels to zero. Step 6.
    ///
    /// Applied AFTER salt-and-pepper so a defect is never masked by a random
    /// impulse: a hot pixel is stuck, and a stuck pixel stays stuck.
    void apply_defects(std::span<uint8_t> img) const noexcept;

    [[nodiscard]] bool built() const noexcept { return !prnu_.empty(); }
    [[nodiscard]] size_t hot_count()  const noexcept { return hot_.size(); }
    [[nodiscard]] size_t dead_count() const noexcept { return dead_.size(); }

private:
    std::vector<float>    prnu_;   ///< multiplicative gain, ~N(1, prnu_sigma)
    std::vector<float>    fpn_;    ///< additive offset, ~N(0, fpn_sigma)
    std::vector<uint32_t> hot_;    ///< pixel indices stuck at 255
    std::vector<uint32_t> dead_;   ///< pixel indices stuck at 0
};

// ---------------------------------------------------------------------------
// NoiseParams — the configured noise, resolved from the scenario.
// ---------------------------------------------------------------------------
struct NoiseParams {
    bool   poisson_enabled = true;    ///< row 21
    double gaussian_sigma  = 20.0;    ///< rows 21-22, grey levels
    double salt_pepper_p   = 0.10;    ///< row 21
    double prnu_sigma      = 0.01;    ///< 1% gain spread, a typical FPA figure
    double fpn_sigma       = 1.5;     ///< grey levels
    int    hot_pixels      = 40;
    int    dead_pixels     = 10;

    /// Black-level pedestal, in grey levels, added after the atmosphere.
    ///
    /// Every real camera has one, and it exists for exactly the reason it is
    /// needed here: to stop the noise distribution being CLIPPED at zero.
    ///
    /// Without it, spec row 24's low-light mode (alpha 0.40, beta -40) drives a
    /// dark background to -36.8 and the sensor clips it to 0. Two things break
    /// at once. The beacon's contrast collapses from 48 grey levels to 11,
    /// because the background had nowhere further to fall while the beacon did.
    /// And the read noise becomes half-normal — every negative excursion maps to
    /// 0 — which violates the Gaussian assumption CFAR's threshold rests on, so
    /// the measured false-alarm rate stops matching Q(k).
    ///
    /// Measured effect on CP 5.9's per-mode sweep: low-light detection went from
    /// 53% to the figure in that test. This is a missing piece of the sensor
    /// model, not a tuning knob.
    double black_level = 16.0;

    /// Photons per grey level. Sets how strong shot noise is relative to the
    /// signal: at k = 8, a 120-level beacon carries ~960 photons, so its shot
    /// noise is sqrt(960)/8 = 3.9 grey levels. Turning this up makes the sensor
    /// quieter, which is what a physically larger well depth would do.
    double photons_per_level = 8.0;

    [[nodiscard]] static NoiseParams from_scenario(const Scenario& sc) noexcept {
        NoiseParams p;
        p.poisson_enabled = sc.noise_poisson;
        p.gaussian_sigma  = sc.gaussian_sigma;
        p.salt_pepper_p   = sc.salt_pepper;
        p.hot_pixels      = sc.hot_pixels;
        return p;
    }
};

}  // namespace sat
