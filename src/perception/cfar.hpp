// perception/cfar.hpp — constant false alarm rate detection (design §9.4.5).
//
// ---------------------------------------------------------------------------
// WHY NOT A FIXED THRESHOLD
// ---------------------------------------------------------------------------
// Design §9.4.5 gives two reasons and both are decisive:
//
//   * Five weather modes change the background level and contrast by 3x (§9.3:
//     fog is alpha 0.35 with beta +60; low light is alpha 0.40 with beta -40).
//     A threshold tuned in clear air finds nothing in fog; one tuned for fog
//     fires everywhere in clear air.
//   * The evaluators' videos have statistics nobody here has ever seen. There
//     is no value to tune a fixed threshold TO.
//
// CFAR measures the background from the image itself, at every pixel, so it
// adapts automatically. There is no weather-dependent constant anywhere in the
// detection path.
//
// ---------------------------------------------------------------------------
// AND WHY THE THRESHOLD IS A PROBABILITY, NOT A MAGIC NUMBER
// ---------------------------------------------------------------------------
// The detector fires when a cell exceeds its local mean by k standard
// deviations. For Gaussian background that makes the false-alarm probability
//
//     Pfa = Q(k) = 0.5 * erfc(k / sqrt(2))
//
// so §9.4.5's default k = 3.9 is not a number someone tuned until it looked
// right — it is a stated Pfa of about 4.8e-5, which over a 640x480 frame means
// roughly 15 false alarms per frame BEFORE the shape gate (§9.4.7) removes the
// ones that are not target-shaped. That is a figure that can be defended in a
// Q&A and predicted in a report, which a tuned constant cannot.
//
// ---------------------------------------------------------------------------
// THE GUARD BAND IS NOT OPTIONAL
// ---------------------------------------------------------------------------
// §9.4.5: "Guard band is essential — without it a bright beacon contaminates
// its own background estimate." A 10 px beacon inside a 61x61 training window
// raises the estimated mean and, far worse, inflates the estimated standard
// deviation — so the brighter the target, the higher the bar it has to clear.
// The effect is self-defeating precisely for the targets that matter most.
// Excluding a 31x31 guard region around the cell under test removes it.
//
// Both the training window and the guard are box sums, so the whole thing is
// 16 summed-area lookups per pixel regardless of window size (§9.4.3).

#pragma once

#include "perception/sat.hpp"

#include <cstdint>
#include <span>

namespace sat {

/// CFAR parameters. Defaults are design §9.4.5's: T = 61, G = 31, k = 3.9.
struct CfarParams {
    int   train = 61;    ///< outer training window, odd
    int   guard = 31;    ///< excluded guard region, odd, < train
    float k     = 3.9f;  ///< threshold in local standard deviations

    /// The false-alarm probability `k` corresponds to, for Gaussian background.
    /// Reported in run.json so the claim is on the record with the results.
    [[nodiscard]] double pfa() const noexcept;

    /// The k that gives a desired Pfa — the inverse, so a scenario can ask for
    /// a false-alarm rate rather than for a threshold.
    [[nodiscard]] static float k_for_pfa(double pfa) noexcept;
};

/// One CFAR evaluation.
struct CfarResult {
    float snr       = 0.0f;   ///< (cell - local mean) / local sd
    float local_mean = 0.0f;
    float local_sd   = 0.0f;
    bool  detected   = false;
};

/// Evaluate CFAR for the cell at (x, y) against its surrounding annulus.
///
/// `cell_sum` is the value being tested — typically the matched-filter response
/// or a single pixel — and `cell_pixels` how many image pixels it covers, so the
/// comparison is like-for-like with the per-pixel background statistics.
[[nodiscard]] CfarResult cfar_at(const SummedArea& sa, int x, int y,
                                 const CfarParams& p) noexcept;

/// Run CFAR over the whole image, writing a sparse mask.
///
/// The output is a mask rather than a list because §9.4.6's run-length grouping
/// consumes exactly that, and a mask is what lets the grouping pass be a single
/// linear scan.
void cfar_mask(const SummedArea& sa, int width, int height,
               const CfarParams& p,
               std::span<uint8_t> mask, std::span<float> snr) noexcept;

}  // namespace sat
