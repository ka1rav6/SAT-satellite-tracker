// camera/splat.hpp — draw emitters onto the sensor.
//
// The render target is a float buffer, not uint8. Quantising to 8 bits happens
// once, at the very end of the degradation chain (design §9.3 step 7). Doing it
// earlier would round every emitter's contribution before they are summed, and
// with 120 clutter sources plus a beacon that is a lot of accumulated rounding
// sitting directly on top of the graded metric.
//
// Motion blur (design §9.2) is handled by calling splat_emitters once per
// exposure substep with interpolated positions and a weight of 1/N. Doing it
// that way rather than as a post-process means the blur is physically what it
// should be: the integral of the scene over the exposure, including the
// boresight's own motion during that exposure, which is what actually smears a
// fast slew.

#pragma once

#include "camera/coverage.hpp"
#include "core/frames.hpp"
#include "world/emitters.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace sat {

/// Add one emitter's contribution to `dst`, at a continuous sub-pixel position
/// in IMAGE coordinates.
///
/// `weight` scales the whole contribution; the motion-blur loop passes 1/N.
///
/// Only the pixels the emitter can actually touch are visited — the bounding box
/// of its footprint, clipped to the sensor. A 10 px beacon touches ~144 pixels,
/// not 307,200, which is why the whole splat stage fits in design §15's 0.04 ms
/// budget even with clutter.
void splat_emitter(std::span<float> dst, int width, int height,
                   Pixel2 centre, double size_px, ShapeKind shape,
                   float intensity, double weight = 1.0) noexcept;

/// Splat every emitter in `visible` (indices into `emitters`) onto `dst`.
///
/// `boresight` is the pointing this sub-exposure is rendered at, so the caller
/// can interpolate it across the exposure for blur.
void splat_emitters(std::span<float> dst,
                    const CameraGeometry& cam, const ScreenGeometry& scr,
                    const EmitterSoA& emitters, std::span<const uint32_t> visible,
                    Angle2 boresight, double weight = 1.0) noexcept;

/// The intensity-weighted centre of a float image, in image pixels.
///
/// This is the simulator's own check on itself, and the direct subject of CP
/// 1.3's acceptance test. It is NOT the tracker's centroid estimator — that one
/// lives in perception/, works on a degraded 8-bit image with background
/// removal and bias correction, and never sees this function.
[[nodiscard]] Pixel2 intensity_centroid(std::span<const float> img,
                                        int width, int height) noexcept;

/// Sum of all pixels. Used by tests to assert that a splat conserves flux.
[[nodiscard]] double total_flux(std::span<const float> img) noexcept;

/// Convert a float render to 8-bit, clipping at 255.
///
/// This is design §9.3 step 7 in isolation; the full chain inserts atmosphere,
/// noise and defects before it. Rounding is round-half-away-from-zero, which is
/// what a real ADC does and, more importantly, is symmetric — a round-half-even
/// or a truncation would bias every pixel in one direction and shift the
/// centroid by a fraction of a pixel.
void quantise_u8(std::span<const float> src, std::span<uint8_t> dst) noexcept;

/// Write a binary PGM. The CP 1.4 acceptance surface after the §14.0 amendment:
/// the numeric test is intensity_centroid(), and this exists so a human can also
/// look at a frame when something is obviously wrong.
[[nodiscard]] bool write_pgm(const char* path, std::span<const uint8_t> img,
                             int width, int height) noexcept;

}  // namespace sat
