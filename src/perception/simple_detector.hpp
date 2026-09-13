// perception/simple_detector.hpp — CP 1.5, the brightest-pixel detector.
//
// This is a DELIBERATE STRAW MAN. It is the simplest thing that closes the loop,
// so that Stage 1 can prove the camera follows the beacon because our own code
// told it to, before any of the real perception pipeline exists.
//
// It is also kept permanently, for two reasons:
//
//   1. CP 4.11's acceptance criterion is "with 120 clutter sources, the
//      brightest-pixel detector demonstrably locks onto the wrong thing". That
//      is a real result — it is the evidence that the CFAR/matched-filter
//      pipeline of §9.4 is necessary rather than decorative, and the report is
//      much stronger for showing the failure than for asserting it.
//   2. It is the floor of the ablation table. Every later estimator has to beat
//      something, and "brightest pixel" is the honest baseline.
//
// It must never be the default. INV-9 and the whole of §9.4 exist because this
// approach does not survive contact with salt-and-pepper noise: a single 255
// impulse pixel beats a real 10x10 beacon on peak brightness every time, and
// there are ~30,720 of them per frame at spec row 21's 10%.

#pragma once

#include "core/units.hpp"

#include <cstdint>
#include <span>

namespace sat {

/// What a single-detection detector returns.
struct SimpleDetection {
    Pixel2  centre{};          ///< image coordinates, sub-pixel where available
    float   peak      = 0.0f;  ///< brightest value found
    float   integrated = 0.0f; ///< sum over the analysis window
    bool    found     = false;

    /// INV-9 in miniature: when nothing was found, callers must not invent a
    /// position. `found` is the only thing that may be trusted.
    explicit operator bool() const noexcept { return found; }
};

/// Return the brightest pixel in the image.
///
/// Ties go to the FIRST pixel in raster order. That is arbitrary but it must be
/// deterministic — INV-3 forbids any tie-break that could vary between runs, and
/// ties are common on a saturated or heavily quantised frame.
[[nodiscard]] SimpleDetection detect_brightest(std::span<const uint8_t> img,
                                               int width, int height) noexcept;

/// Brightest pixel, refined to sub-pixel accuracy by a centre-of-mass over a
/// window around it.
///
/// `window` is the half-width, so 5 means an 11x11 box. `floor_value` is
/// subtracted from every sample before weighting, which is a crude stand-in for
/// the top-hat background removal of §9.4.2: without it, a uniform background
/// pedestal pulls the centre of mass toward the middle of the WINDOW rather than
/// the middle of the BEACON, and the result is biased toward wherever the peak
/// pixel happened to be.
///
/// Still a straw man — the real estimators live in perception/centroid/ from
/// Stage 5 — but sub-pixel enough for CP 1.8's closed loop to settle.
[[nodiscard]] SimpleDetection detect_brightest_subpixel(std::span<const uint8_t> img,
                                                        int width, int height,
                                                        int window = 5,
                                                        float floor_value = 0.0f) noexcept;

}  // namespace sat
