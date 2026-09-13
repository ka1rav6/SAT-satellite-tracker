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

// intensity_centroid, total_flux, quantise_u8 and write_pgm moved to
// core/image.hpp: they operate on a plain pixel buffer and the degradation
// chain needs them too, without depending on the camera module.

}  // namespace sat
