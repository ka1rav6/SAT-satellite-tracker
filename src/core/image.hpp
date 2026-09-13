// core/image.hpp — generic operations on a pixel buffer.
//
// These know nothing about cameras, emitters or scenarios: they take a span of
// pixels and a width. That is why they live in core rather than in camera/ —
// the degradation chain needs quantise_u8 and has no business depending on the
// camera module to get it.

#pragma once

#include "core/units.hpp"

#include <cstdint>
#include <span>

namespace sat {

/// Convert a float render to 8-bit, clipping at 255.
///
/// Design §9.3 step 7. Rounding is round-half-away-from-zero, which is what a
/// real ADC does and, more importantly, is SYMMETRIC — round-half-even or
/// truncation would bias every pixel in one direction and shift the centroid by
/// a fraction of a pixel, directly into the metric worth 60% of the marks.
void quantise_u8(std::span<const float> src, std::span<uint8_t> dst) noexcept;

/// The intensity-weighted centre of a float image, in pixels.
///
/// This is the SIMULATOR's check on itself — CP 1.3's acceptance test. It is not
/// the tracker's centroid estimator: that one lives in perception/, works on a
/// degraded 8-bit image with background removal and bias correction, and never
/// sees this function.
[[nodiscard]] Pixel2 intensity_centroid(std::span<const float> img,
                                        int width, int height) noexcept;

/// Same, over an 8-bit image with a background floor subtracted.
[[nodiscard]] Pixel2 intensity_centroid_u8(std::span<const uint8_t> img,
                                           int width, int height,
                                           double floor_value = 0.0) noexcept;

/// Sum of all pixels. Used by tests to assert a splat conserves flux.
[[nodiscard]] double total_flux(std::span<const float> img) noexcept;

/// Write a binary PGM.
///
/// The CP 1.4 acceptance surface after the §14.0 amendment: the numeric test is
/// intensity_centroid(), and this exists so a human can look at a frame when
/// something is obviously wrong.
[[nodiscard]] bool write_pgm(const char* path, std::span<const uint8_t> img,
                             int width, int height) noexcept;

}  // namespace sat
