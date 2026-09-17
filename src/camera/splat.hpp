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
#include "core/image.hpp"
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
/// `reach_sigmas` is how far a GAUSSIAN is drawn, in units of its own sigma.
/// It has no effect on the other shapes, which are bounded.
///
/// ---------------------------------------------------------------------------
/// WHY THIS IS A PARAMETER AND NOT THE CONSTANT 6
/// ---------------------------------------------------------------------------
/// Six sigma is required for the GRADED beacon, and the argument is in the
/// implementation: truncating a Gaussian is asymmetric about a sub-pixel
/// centre, so it SHIFTS the rendered centroid, and that centroid is the
/// reference the 60%-weighted metric is measured against. Measured worst-case
/// bias runs 3.5e-3 px at three sigma down to 8.7e-9 at six.
///
/// None of that applies to CLUTTER. A clutter source's rendered position is
/// never scored against anything; it exists to be rejected. The only thing its
/// tail has to do is disappear below what an 8-bit sensor can express, and at
/// 3.5 sigma a Gaussian is at exp(-6.125) = 0.0022 of its peak — 0.42 grey
/// levels for the brightest clutter §9.1 draws, which rounds to nothing.
///
/// The saving is the square of the ratio: (6/3.5)^2 = 2.9x fewer pixels, on the
/// 75% of clutter that §9.1 makes Gaussian and on the 120 sources per frame
/// that spec row 1's screen carries.
void splat_emitter(std::span<float> dst, int width, int height,
                   Pixel2 centre, double size_px, ShapeKind shape,
                   float intensity, double weight = 1.0,
                   double reach_sigmas = 6.0) noexcept;

/// The reach a source that is never scored needs. See splat_emitter.
inline constexpr double kClutterReachSigmas = 3.5;

/// Splat every emitter in `visible` (indices into `emitters`) onto `dst`.
///
/// `boresight` is the pointing this sub-exposure is rendered at, so the caller
/// can interpolate it across the exposure for blur.
void splat_emitters(std::span<float> dst,
                    const CameraGeometry& cam, const ScreenGeometry& scr,
                    const EmitterSoA& emitters, std::span<const uint32_t> visible,
                    Angle2 boresight, double weight = 1.0) noexcept;

// intensity_centroid, total_flux, quantise_u8 and write_pgm moved to
// core/image.hpp: they operate on a plain pixel buffer and the degradation
// chain needs them too, without depending on the camera module.

}  // namespace sat
