// engine/pipeline.hpp — the per-frame orchestrator. The loop itself.
//
// Design §6.2 lays out steps B1 through B31. At Stage 1 this implements the
// skeleton of that sequence:
//
//   B1-B2   advance world and disturbances, 10 sub-ticks per frame
//   B3      step the gimbal with the commanded rate  <- closes the loop
//   B4      acquire a frame at the TRUE boresight
//   B5-B15  perception (Stage 1: brightest pixel; Stage 5: the real pipeline)
//   B16-B21 tracking (Stage 6)
//   B22-B27 control
//   B28-B30 metrics and snapshot (Stage 7)
//
// The stages that do not exist yet are absent, not stubbed with fake behaviour.
// A stub that returns a plausible-looking value is how a pipeline ends up
// passing its tests while doing nothing.
//
// ---------------------------------------------------------------------------
// INV-2: THE LOOP IS CLOSED
// ---------------------------------------------------------------------------
// "The controller's output determines the boresight at which the next frame is
// produced. There must be no code path where frames are produced independently
// of the control output."
//
// In this file that is the single line in step() that feeds `cmd_rate_` into
// gimbal_.step() before the frame is acquired. CP 1.8's acceptance test
// verifies it by construction: with the controller disabled the camera must NOT
// follow, and with it enabled it must. A simulation that renders a nicely
// centred beacon regardless of the controller looks identical on screen and
// proves nothing.

#pragma once

#include "control/controller.hpp"
#include "core/profile.hpp"
#include "core/triple_buffer.hpp"
#include "engine/snapshot.hpp"
#include "engine/synthetic_source.hpp"
#include "scenario/scenario.hpp"
#include "perception/simple_detector.hpp"
#include "plant/gimbal.hpp"

#include <cstdint>
#include <vector>

namespace sat {

/// Per-frame record the engine publishes. Stage 7 grows this into the full
/// SimSnapshot that the triple buffer carries to the GUI.
struct FrameRecord {
    int64_t frame          = 0;
    double  time_s         = 0.0;

    Angle2  boresight_cmd{};      ///< what the controller asked for
    Angle2  boresight_true{};     ///< where the camera actually pointed
    Rate2   cmd_rate{};

    bool    detected       = false;
    Pixel2  detection_img{};      ///< image coordinates
    Pixel2  detection_screen{};   ///< screen coordinates — the graded column
    float   detection_peak = 0.0f;

    // --- metrics (need truth; INV-6 keeps these two strictly separate) -----
    bool    truth_valid       = false;
    Pixel2  truth_screen{};
    bool    truth_in_fov      = false;
    // -----------------------------------------------------------------------
    // CENTROIDING ERROR, IN BOTH FRAMES — and they are not the same number.
    //
    // Design §13.2's centroid.csv carries both cx_cam/cy_cam and
    // cx_screen/cy_screen, with the note "you do not know which they want".
    // Working through it, they measure genuinely different things:
    //
    //   IMAGE frame   |detection - true position in the image|. This is the
    //                 DETECTOR's accuracy and nothing else. It is the quantity
    //                 §10.1.1's theoretical bound sigma >= w/(2*SNR) applies to,
    //                 and the one §10.1.3's bias correction improves.
    //
    //   SCREEN frame  the same, after converting through the COMMANDED
    //                 boresight — which is all the system knows. It therefore
    //                 also contains the unmeasured pointing error: jitter
    //                 (row 23) and platform drift (row 25) move the true
    //                 boresight, and §10.3's encoder does not see them.
    //
    // So the screen-frame number can never beat the jitter amplitude, and
    // reporting it alone would make the 60%-weighted metric dominated by a
    // disturbance the detector has no influence over — exactly the conflation
    // INV-6 forbids. Both are reported, labelled, and plotted separately.
    //
    // In VIDEO modes the two coincide, because INV-8 disables the disturbances
    // and the crop offset is known exactly (design §8.3 requirement 6). That is
    // the mode Benchmark Performance-2 grades centroiding in.
    // -----------------------------------------------------------------------
    double  centroid_error_px        = 0.0;   ///< IMAGE frame: the detector alone
    double  centroid_error_screen_px = 0.0;   ///< SCREEN frame: + pointing knowledge
    bool    centroid_error_valid     = false;

    /// A detection was reported while the beacon was NOT in the field of view.
    /// Feeds §13.1's false_track_rate; it is not a centroiding error.
    bool    false_alarm = false;

    /// |boresight - true target angle| in px. Independent of whether the beacon
    /// was detected this frame — it measures the CONTROL LOOP, not the detector
    /// (INV-6).
    double  tracking_error_px = 0.0;
};

/// Stage 1 pipeline configuration. Stage 3 folds this into the Scenario struct.
struct PipelineConfig {
    SyntheticConfig synthetic{};
    GimbalParams    pan{};
    GimbalParams    tilt{};
    ControlGains    gains = ControlGains::proportional(6.0);

    /// Where the camera starts. Spec row 6: the centre of the screen, which is
    /// the angular origin.
    Angle2 initial_boresight{};

    /// CP 1.8 (b): "Commenting out the line applying the command stops the
    /// following; uncommenting restores it." Rather than asking anyone to edit
    /// and rebuild, this makes it a flag, so the acceptance criterion is a test
    /// that runs on every commit instead of a manual ritual.
    bool control_enabled = true;

    /// Straw-man detector parameters: analysis window half-width, and the
    /// level it subtracts before computing a centre of mass.
    ///
    /// The floor MUST sit above the rendered background, which is the scene
    /// pedestal plus the sensor's black level (degrade/noise.hpp). It was 12
    /// when the background was 8; adding a black-level pedestal of 16 put the
    /// background at ~24 and the floor stopped excluding it, so background
    /// pixels started pulling the centre of mass and the measured centroiding
    /// error rose from 0.7 px to 1.55 px.
    ///
    /// That fragility is exactly why this detector is a straw man: a fixed
    /// floor cannot survive a change in scene level, which is the same argument
    /// §9.4.5 makes for CFAR over a fixed threshold. ClassicalPerception
    /// estimates the background per frame and has no such parameter.
    int   detector_window = 7;
    float detector_floor  = 40.0f;

    /// Publish a SimSnapshot every frame. Costs one frame copy (~300 KB), so
    /// the headless benchmark path turns it off. Reproducibility verification
    /// turns it ON, because the fingerprint is computed from the snapshot.
    bool publish_snapshots = true;
};

// ---------------------------------------------------------------------------
// Pipeline
// ---------------------------------------------------------------------------
class Pipeline {
public:
    void build(const PipelineConfig& cfg, EmitterSoA emitters);

    /// Build everything from a parsed scenario: world, clutter, decoys, damage
    /// chain, disturbances, plant and controller. Design §6.1's startup
    /// sequence, in one call.
    void build_from_scenario(const Scenario& sc);

    /// Run one camera frame: sub-ticks, acquire, detect, control.
    /// Returns false when the source is exhausted.
    [[nodiscard]] bool step();

    /// Run to completion, collecting a record per frame.
    void run(std::vector<FrameRecord>& out);

    [[nodiscard]] const FrameRecord& last()     const noexcept { return last_; }

    /// The per-frame fingerprints (CP 2.5). Empty unless publish_snapshots.
    [[nodiscard]] const std::vector<FrameFingerprint>& fingerprints() const noexcept {
        return fingerprints_;
    }

    /// The triple buffer the display thread reads. Exposed so CP 2.3's "slowing
    /// the display does not slow the simulation" can be demonstrated, and so
    /// CP 15.0's dashboard has something to attach to without touching any of
    /// this.
    [[nodiscard]] TripleBuffer<SimSnapshot>& snapshots() noexcept { return snapshots_; }
    [[nodiscard]] const Gimbal&      gimbal()   const noexcept { return gimbal_; }
    [[nodiscard]] SyntheticSource&   source()         noexcept { return source_; }
    [[nodiscard]] const StageTimers& timers()   const noexcept { return timers_; }

    /// Flip the loop open or closed mid-run. Used by the CP 1.8 test and by the
    /// live demo (§14.1, 5:00-6:30), where turning it off and watching the error
    /// trace blow up is the most convincing moment in the presentation.
    void set_control_enabled(bool on) noexcept { cfg_.control_enabled = on; }
    [[nodiscard]] bool control_enabled() const noexcept { return cfg_.control_enabled; }

    [[nodiscard]] const PipelineConfig& config() const noexcept { return cfg_; }

private:
    PipelineConfig  cfg_{};
    SyntheticSource source_{};
    Gimbal          gimbal_{};
    Controller      control_{};
    StageTimers     timers_{};

    Rate2       cmd_rate_{};      ///< the value that closes the loop (INV-2)
    FrameRecord last_{};
    int64_t     frame_ = 0;

    TripleBuffer<SimSnapshot>    snapshots_{};
    std::vector<FrameFingerprint> fingerprints_;
};

}  // namespace sat
