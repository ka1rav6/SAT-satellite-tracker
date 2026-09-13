// perception/matched.hpp — multi-scale matched filter (design §9.4.4).
//
// ---------------------------------------------------------------------------
// WHY A BOX SUM IS THE OPTIMAL DETECTOR HERE
// ---------------------------------------------------------------------------
// The matched-filter theorem says the detector that maximises output SNR for a
// KNOWN signal in additive noise is correlation with that signal. Design
// decision 16 draws the consequence and it is worth restating, because it is the
// project's main argument for not using a learned object detector:
//
//     The beacon is a uniform square with no texture and no shape variety. Its
//     matched filter IS a box of the same size. Correlating with a box is a box
//     SUM, which the summed-area table makes four lookups — about 1000x cheaper
//     than a CNN, and provably optimal rather than empirically good.
//
// There is nothing for a network to learn here that the mathematics does not
// already give exactly. §4.1 says to state that reasoning in the report as
// evidence of judgement under "Selection of Algorithms".
//
// ---------------------------------------------------------------------------
// WHY SIX SCALES
// ---------------------------------------------------------------------------
// Spec row 10 lets the beacon be 5-20 px, "user-defined". Worse, in video mode
// (§8) the size is genuinely UNKNOWN — the evaluators' MP4s come with no
// metadata about the beacon in them. A filter matched to the wrong size loses
// SNR twice over: too small and it integrates only part of the signal, too large
// and it integrates background that is pure noise.
//
// So the filter runs at {5, 8, 11, 14, 17, 20} and takes the maximum. Because
// each scale is O(1), six of them cost 24 lookups rather than six convolutions.
// The winning scale is a free size estimate, which §9.4.4 notes feeds the
// centroid window and which §10.1.4 uses for the uncertainty estimate.
//
// ---------------------------------------------------------------------------
// THE 1/sqrt(k*k) NORMALISATION
// ---------------------------------------------------------------------------
// Design §9.4.4 divides the box sum by sqrt(k*k) = k, and the reason is that
// without it the scales are not comparable. Over a region of independent noise
// with per-pixel standard deviation sigma, a k x k box sum has standard
// deviation sigma*k — so a larger box always produces a larger raw number, and
// "take the max over scales" would always pick scale 20 regardless of the
// signal. Dividing by k makes the noise floor identical at every scale, so the
// maximum is chosen by the SIGNAL and the winning scale means something.

#pragma once

#include "perception/sat.hpp"

#include <array>
#include <cstdint>
#include <span>

namespace sat {

/// The six scales of design §9.4.4, spanning spec row 10's 5-20 px range.
inline constexpr std::array<int, 6> kMatchedScales{5, 8, 11, 14, 17, 20};

/// Response of the matched filter at one point and one scale.
///
/// The box is centred on (x, y) and clipped to the image, with the response
/// normalised by the ACTUAL pixel count rather than the nominal k*k — the same
/// argument as §9.4.5's "clamp boxes to the image and adjust n". Using the
/// nominal count near an edge would scale the response down and make a beacon
/// leaving the frame progressively harder to see, which is the opposite of what
/// is wanted.
[[nodiscard]] float matched_response(const SummedArea& sa, int x, int y, int k) noexcept;

/// The best response over all six scales at one point.
struct MatchedPeak {
    float response = 0.0f;
    int   scale    = 0;      ///< the winning k: a free size estimate
};

[[nodiscard]] MatchedPeak matched_best(const SummedArea& sa, int x, int y) noexcept;

/// Run the filter over the whole image, writing the best response and the
/// winning scale per pixel.
///
/// `response` and `scale` are width*height. Both are outputs because CFAR
/// (§9.4.5) thresholds the response while the grouping stage (§9.4.6) and the
/// centroid window (§10.1.2) want the scale.
void matched_filter(const SummedArea& sa, int width, int height,
                    std::span<float> response, std::span<uint8_t> scale) noexcept;

/// Response at ONE fixed scale, over the whole image.
///
/// Used for the detection threshold, while matched_filter()'s max-over-scales
/// supplies the size estimate. The distinction matters statistically: a maximum
/// over six correlated scales is an ORDER STATISTIC, so its background has a
/// higher mean and a noticeably higher variance than any single scale's. CFAR
/// then raises its threshold to match and the target is missed — measured at 23
/// frames in 120 on CP 5.9's worst case, against 0 for a single scale.
///
/// A single scale is a plain linear filter whose background stays as Gaussian
/// as the input, which is the assumption CFAR's Pfa = Q(k) rests on.
void matched_filter_at_scale(const SummedArea& sa, int width, int height, int k,
                             std::span<float> response) noexcept;

/// The scale from kMatchedScales closest to a given target size.
[[nodiscard]] int nearest_scale(int target_size_px) noexcept;

}  // namespace sat
