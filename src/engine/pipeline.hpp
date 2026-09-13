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
    double  centroid_error_px = 0.0;   ///< |reported centre − true centre|, SCREEN px
    double  tracking_error_px = 0.0;   ///< |boresight − true target angle|, px
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

    /// Detector analysis window half-width, and the background level it
    /// subtracts before computing a centre of mass.
    int   detector_window = 7;
    float detector_floor  = 12.0f;

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

    /// Flip the loop open or closed mid-run. Used by the CP 1.8 test and, later,
    /// by the live demo (CP 15.2).
    void set_control_enabled(bool on) noexcept { cfg_.control_enabled = on; }

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
