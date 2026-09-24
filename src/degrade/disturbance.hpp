// degrade/disturbance.hpp — jitter and platform motion (spec rows 23 and 25).
//
// ---------------------------------------------------------------------------
// THESE PERTURB THE TRUE BORESIGHT, NEVER THE PIXELS
// ---------------------------------------------------------------------------
// Design §9.3 is emphatic about this and gives two reasons, both of which
// matter:
//
//   1. It is physically correct. A platform that shakes moves where the camera
//      is POINTING; it does not smear the image by some separate mechanism.
//      Modelling it at the boresight means motion blur follows automatically
//      from the exposure integration (§9.2) rather than having to be faked.
//   2. "It prevents the tracker from implicitly knowing its own pointing
//      error." If jitter were applied as a pixel shift, the commanded boresight
//      would still be the true one, and the system would be solving a much
//      easier problem than the real one — with no way to tell from the outside.
//
// ---------------------------------------------------------------------------
// WHY THIS IS THE HARD PART OF THE WHOLE PROJECT
// ---------------------------------------------------------------------------
// Design §1.3, restated in numbers this file produces:
//
//     camera authority at 5 deg/s, 30 Hz    26.7 px/frame
//     jitter alone (row 23)                 20   px/frame   = 75% of authority
//     jitter + platform (rows 23 + 25)      40   px/frame   = 150% of authority
//
// The disturbance can EXCEED what the motor can do. That is why reactive
// control cannot meet the 10 px budget and why the system must predict — the
// conclusion the whole design rests on.

#pragma once

#include "core/frames.hpp"
#include "core/rng.hpp"
#include "core/units.hpp"
#include "degrade/turbulence.hpp"
#include "scenario/scenario.hpp"
#include "world/motion_component.hpp"

#include <cstddef>
#include <memory>

namespace sat {

// ---------------------------------------------------------------------------
// DisturbanceGenerator
//
// Produces the boresight offset to add to the commanded pointing before a frame
// is rendered. Two independent sources:
//
//   jitter    high-frequency, zero-mean, uncorrelated frame to frame. Models
//             vibration. Spec row 23 caps it at +/- 20 px/frame.
//   platform  low-frequency, correlated, and DRIVEN BY THE SAME MOTION ALGEBRA
//             as the target (design §7.2). Spec row 25 asks for linear as
//             mandatory and circular/random/spiral/figure-8 as optional, which
//             is exactly the menu §7.2 already provides — CP 4.9's criterion is
//             that all five work "driven by the same code as target motion".
// ---------------------------------------------------------------------------
class DisturbanceGenerator {
public:
    void build(const Scenario& sc, const ScreenGeometry& scr, double camera_hz);

    /// Advance the platform's stochastic components by one truth tick.
    void advance(double dt, RngSet& rng);

    /// The total boresight offset at time t, in microradians.
    ///
    /// `jitter_tick` should change once per CAMERA frame, not once per truth
    /// tick: spec row 23 specifies jitter in px PER FRAME, so drawing it at the
    /// truth rate would make it ten times more energetic than specified.
    [[nodiscard]] Angle2 offset(double t_s, RngSet& rng, bool new_frame);

    /// The platform's analytic velocity, in microradians per second.
    ///
    /// This is what CP 10.3's platform-drift cancellation would ideally know.
    /// The controller does NOT get this — it has to estimate it — but the
    /// metrics use it to report how much of the residual error is platform
    /// motion the estimator failed to remove.
    [[nodiscard]] Rate2 platform_rate(double t_s) const;

    void reset();

    // -----------------------------------------------------------------------
    // LIVE OVERRIDES for spec rows 23 and 25.
    //
    // Rows 21, 22 and 24 have been live-adjustable from the dashboard since
    // the damage panel existed, and the scenario panel reads the LIVE value
    // for each of them against the file's. Rows 23 and 25 were in that table
    // too — and in the panel's heading — with no control behind them, so the
    // comparison could never differ and the panel's claim that "every one of
    // these is live-adjustable" was false for the two that matter most.
    //
    // They matter most because row 17's graded tracking error is dominated by
    // row 23. Jitter is drawn fresh every frame and added to the TRUE
    // boresight, so no controller can reject it and it puts row 17 on a
    // 16.33 px floor (design §1.3, docs/RESULTS.md §1). A viewer looking at
    // 17 px of tracking error has no way to tell that from a loop that is
    // failing, and the "Clean" preset told them the loop "should track to a
    // few pixels" while leaving the jitter on.
    //
    // With these, dragging jitter to zero drops the error to ~4 px in front of
    // the viewer, which is the demonstration that separates the two.
    // -----------------------------------------------------------------------
    void set_jitter_px_per_frame(double px) noexcept { jitter_px_ = px > 0.0 ? px : 0.0; }

    /// Row 25's platform motion, on or off. It suppresses the OFFSET only,
    /// never the stochastic components' advance() — a component that stopped
    /// drawing would desynchronise Stream::PlatformMotion and the toggle would
    /// produce a different run rather than the same run without the drift.
    void set_platform_enabled(bool on) noexcept { platform_on_ = on; }
    [[nodiscard]] bool platform_enabled() const noexcept { return platform_on_; }
    [[nodiscard]] bool has_platform() const noexcept { return platform_count_ > 0; }

    /// Startup diagnostics. Design §9.3 asks for this exact line to be logged:
    ///     20 px/frame -> 65449 urad/s -> 3.75 deg/s (75% of a 5 deg/s motor)
    [[nodiscard]] double jitter_urad_s(double camera_hz) const noexcept;
    [[nodiscard]] double jitter_px_per_frame() const noexcept { return jitter_px_; }

    /// The turbulence model, for the renderer's scintillation gain and for the
    /// startup diagnostics. A THIRD source alongside jitter and platform
    /// motion, and like both of them it moves the BORESIGHT, never the pixels
    /// (degrade/turbulence.hpp says why at length).
    [[nodiscard]] const TurbulenceModel& turbulence() const noexcept { return turb_; }

private:
    TurbulenceModel turb_{};
    double          jitter_px_    = 0.0;
    double          ifov_x_       = 1.0;
    double          ifov_y_       = 1.0;
    CompositeMotion platform_;
    bool            platform_on_    = true;
    size_t          platform_count_ = 0;   ///< components built, for the GUI
    Angle2          jitter_held_{};   ///< resampled once per camera frame
};

}  // namespace sat
