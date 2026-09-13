// tracking/measurement.hpp — CP 6.1. Where pixels stop and angles begin.
//
// Design §6.2 step B16: "Unproject to angles, add commanded boresight → world
// angular frame." That one line is the boundary between everything that is
// specific to this camera and everything that is not, and it is worth being
// precise about why it has to be here rather than anywhere else.
//
// ---------------------------------------------------------------------------
// WHY THE FILTER RUNS IN ANGLES, NOT PIXELS  (INV-5)
// ---------------------------------------------------------------------------
// A Kalman filter's state has to live in a frame that does not move, otherwise
// the constant-velocity model is a lie. Image pixels move: the camera slews
// under the target constantly, so a stationary beacon sweeps across the sensor
// at up to the mount's full 10 deg/s. A filter fed raw pixel coordinates would
// spend its entire life modelling the GIMBAL's motion instead of the TARGET's,
// and its velocity estimate — which is what feeds the velocity feedforward in
// §10.4, "the highest-value ten lines in the project" — would be meaningless.
//
// Adding the boresight makes the measurement absolute. In the world angular
// frame a stationary emitter has a stationary state, whatever the camera does.
// CP 6.1's acceptance test is exactly that sentence, mechanised.
//
// ---------------------------------------------------------------------------
// COMMANDED, NOT TRUE
// ---------------------------------------------------------------------------
// The boresight added here is the COMMANDED one — strictly, the quantised
// encoder reading, which is all the real system can observe. Using the true
// boresight would silently cancel jitter and platform drift and make the whole
// system look perfect for the wrong reason. It is also unavailable: INV-1 means
// this module cannot even name it.
//
// The consequence is real and is not a bug: the angular measurement carries the
// pointing error as an additive disturbance. That is why §13.1 reports centroid
// error in both frames (INV-6) and why the filter's R has a floor — see
// measurement_sigma_urad below.

#pragma once

#include "core/frames.hpp"
#include "core/units.hpp"
#include "perception/pipeline.hpp"

#include <cstdint>

namespace sat {

// ---------------------------------------------------------------------------
// Measurement — one gated detection, expressed in the world angular frame.
//
// This is the ONLY thing the tracker sees. Note what is absent: no pixel
// coordinates, no image, no frame pointer. A measurement that still carried its
// image position would invite someone to "just check" it against something, and
// the point of the conversion is that after it the camera is irrelevant.
// ---------------------------------------------------------------------------
struct Measurement {
    Angle2 angle{};            ///< world-frame direction, microradians
    double sigma_urad = 0.0;   ///< 1-sigma, both axes (R = sigma^2 * I)
    float  snr        = 0.0f;  ///< carried through for adaptive R and priority
    uint16_t size_est_px = 0;  ///< winning matched-filter scale
    uint16_t area_px     = 0;
    int      source_index = -1; ///< index into the detection list it came from

    /// CP 6.3 needs to say "this measurement was consumed by track k" without
    /// erasing it from the list, because the unassociated ones still count as
    /// evidence for §10.5's negative information.
    bool associated = false;
};

// ---------------------------------------------------------------------------
// measurement_sigma_urad — the R that goes into the filter.
//
// Two terms, and the second one is the part that is easy to forget:
//
//   1. The centroiding uncertainty, in pixels, times the instantaneous field of
//      view. §10.1.4's sigma ~ size / (2 * SNR) already lives in
//      ClassicalPerception::centroid_sigma, so this just changes its units.
//
//   2. A FLOOR equal to the pointing uncertainty. The measurement is
//      centroid + commanded boresight, and the commanded boresight is wrong by
//      the encoder quantisation plus whatever jitter and platform drift have
//      done since. Even a perfect centroid therefore lands in the wrong place
//      in the world frame. Without the floor, a high-SNR detection claims an
//      accuracy the measurement cannot possibly have, the filter trusts it far
//      more than its own prediction, and the estimate inherits the full jitter
//      spectrum — which is precisely the noise the filter exists to suppress.
//
//      This was visible immediately: with no floor the filtered angle tracked
//      the jitter almost exactly, because R was ~0.05 px worth of variance
//      against a jitter of several microradians.
//
// The floor is passed in rather than assumed, because it differs by mode: in
// video_screen INV-8 disables the disturbances entirely and the honest floor is
// just the encoder LSB.
// ---------------------------------------------------------------------------
[[nodiscard]] inline double measurement_sigma_urad(float centroid_sigma_px,
                                                   double ifov_urad,
                                                   double pointing_sigma_urad) noexcept {
    const double centroid = static_cast<double>(centroid_sigma_px) * ifov_urad;
    // Added in quadrature: the two errors are independent, one being a property
    // of the detector and the other of the mount.
    return std::sqrt(centroid * centroid + pointing_sigma_urad * pointing_sigma_urad);
}

// ---------------------------------------------------------------------------
// to_measurement — B16, for one detection.
//
// `boresight` is the commanded/encoder angle at the instant the frame was
// exposed. `pointing_sigma_urad` is the floor described above.
// ---------------------------------------------------------------------------
[[nodiscard]] inline Measurement to_measurement(const Detection& d,
                                                const CameraGeometry& cam,
                                                Angle2 boresight,
                                                double pointing_sigma_urad,
                                                int source_index = -1) noexcept {
    Measurement m;
    // unproject: image pixel -> angular offset from the boresight. The camera
    // model's job; this function does not know or care whether it is linear or
    // equidistant (core/frames.hpp documents that deviation).
    m.angle        = boresight + cam.unproject(d.centroid_image);
    m.sigma_urad   = measurement_sigma_urad(d.centroid_sigma_est, cam.ifov_urad(),
                                            pointing_sigma_urad);
    m.snr          = d.snr;
    m.size_est_px  = d.size_est_px;
    m.area_px      = d.area_px;
    m.source_index = source_index;
    return m;
}

}  // namespace sat
