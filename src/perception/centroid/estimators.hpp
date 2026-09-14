// perception/centroid/estimators.hpp — CP 9.1. Sixty percent of the marks.
//
// Design §10.1.2 names four estimators and requires all of them, runtime
// switchable. That is not indecision: they fail in different ways, and §10.6's
// supervisor picks between them from measured conditions.
//
//   CoM          fast, unbiased with perfect background removal
//                — a background residual pulls it
//   WindowedCoM  less noise, because it stops summing before the profile does
//                — stronger PIXEL-LOCKING bias, which is §10.1.3's S-curve
//   SurfaceFit   robust to asymmetry and nearby clutter
//                — slower
//   MatchedPeak  best at low SNR, the noise is already integrated away
//                — limited by the response grid's resolution
//
// ---------------------------------------------------------------------------
// WHY THE WINDOW IS THE WHOLE STORY
// ---------------------------------------------------------------------------
// A centre-of-mass over an infinite, noiseless, background-free profile is
// exactly unbiased. Every departure from that is a source of systematic error,
// and the dominant one is the finite window: a beacon whose true centre sits
// left of a pixel's centre has more of its right-hand tail inside the window
// than its left-hand tail, so the estimate is pulled right. The size of the
// pull depends on where in the pixel the centre is, which is why the error
// curve is a repeating S-shape and why averaging does not remove it.
//
// §10.1.3 calls correcting that "the highest-leverage single item" in the
// project and expects 2-5x at high SNR. estimators.hpp produces the raw
// estimates; bias.hpp measures and removes the curve.

#pragma once

#include "core/units.hpp"
#include "perception/grouping.hpp"
#include "perception/sat.hpp"

#include <cstdint>
#include <span>
#include <string_view>

namespace sat {

// ---------------------------------------------------------------------------
// CentroidKind — §10.1.2's enum, logged in run.json and switchable at runtime.
// ---------------------------------------------------------------------------
enum class CentroidKind : uint8_t {
    CoM = 0,        ///< first moment over the whole blob
    WindowedCoM,    ///< first moment over a fixed window about the peak
    SurfaceFit,     ///< 2-D quadratic fit to log intensity
    MatchedPeak,    ///< parabolic interpolation of the matched-filter response
    Learned,        ///< Stage 11; falls back to WindowedCoM until then (INV-7)
    kCount
};

[[nodiscard]] const char* centroid_kind_name(CentroidKind k) noexcept;
/// Parse a name from a scenario or a CLI flag. Returns false on an unknown one
/// rather than silently defaulting, because a typo that quietly selects a
/// different estimator would make an ablation table meaningless.
[[nodiscard]] bool parse_centroid_kind(std::string_view s, CentroidKind& out) noexcept;

// ---------------------------------------------------------------------------
// The estimators.
//
// All four take BACKGROUND-SUBTRACTED data (the top-hat), because every one of
// them assumes the pedestal is gone. §9.4.2's van Herk top-hat is what makes
// that true, and CoM's listed weakness — "a background residual pulls it" — is
// precisely what happens when it is not.
// ---------------------------------------------------------------------------

/// First moment over the blob's own pixels, from the moments grouping already
/// accumulated. Free: no second pass over the image.
[[nodiscard]] Pixel2 centroid_com(const BlobAccum& b) noexcept;

/// First moment over a (2*win+1) square centred on `peak`.
///
/// `win` should be about the beacon's radius: large enough to contain the
/// profile, small enough to exclude its neighbours. Too large and it is CoM
/// with extra steps; too small and the truncation bias grows without bound.
[[nodiscard]] Pixel2 centroid_windowed(std::span<const int16_t> tophat,
                                       int width, int height,
                                       Pixel2 peak, int win) noexcept;

/// 2-D quadratic fit to the LOG of intensity over a 5x5 about the peak.
///
/// A Gaussian's log is exactly a quadratic, so for a Gaussian-ish beacon this
/// recovers the centre with no window-truncation bias at all — which is the
/// point. It is more expensive and it degrades where the log is ill-defined
/// (a pixel at or below the background), so those are excluded from the fit.
[[nodiscard]] Pixel2 centroid_surface_fit(std::span<const int16_t> tophat,
                                          int width, int height,
                                          Pixel2 peak) noexcept;

/// Parabolic interpolation of the matched-filter response about its peak.
///
/// The response has already integrated the noise over the beacon's own scale,
/// so at low SNR this is the most stable of the four — at the cost of being
/// quantised to the response grid, which is why it is not the default.
[[nodiscard]] Pixel2 centroid_matched_peak(std::span<const float> response,
                                           int width, int height,
                                           Pixel2 peak) noexcept;

/// Dispatch. `Learned` falls back to WindowedCoM (INV-7: the system must run
/// fully without AI, and a missing model is a warning, not a crash).
[[nodiscard]] Pixel2 centroid_estimate(CentroidKind kind,
                                       const BlobAccum& b,
                                       std::span<const int16_t> tophat,
                                       std::span<const float> response,
                                       int width, int height,
                                       Pixel2 peak, int win) noexcept;

}  // namespace sat
