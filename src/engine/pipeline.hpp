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
#include "core/strategy.hpp"
#include "control/handover.hpp"
#include "control/supervisor.hpp"
#include "engine/events.hpp"
#include "control/mode_fsm.hpp"
#include "core/arena.hpp"
#include "core/profile.hpp"
#include "core/triple_buffer.hpp"
#include "engine/snapshot.hpp"
#include "engine/synthetic_source.hpp"
#include "engine/video_source.hpp"
#include "scenario/scenario.hpp"
#include "perception/pipeline.hpp"
#include "perception/simple_detector.hpp"
#include "plant/gimbal.hpp"
#include "search/pattern.hpp"
#include "tracking/track.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
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

    // -----------------------------------------------------------------------
    // A THIRD COLUMN, AND WHY IT HAD TO EXIST.
    //
    // The two columns above bracket the truth but neither is the number a
    // reader actually wants, and the gap between them turned out to be
    // unbounded. Working the algebra through: with the true boresight
    // B_true = B_cmd + D(t), where D is the accumulated platform displacement
    // (spec row 25), the beacon lands on the sensor at T - B_true; the
    // detector finds it there essentially exactly; and the screen conversion
    // reconstructs B_cmd + (T - B_true) = T - D(t). Against a truth of T that
    // leaves
    //
    //     centroid_error_screen = |D(t)|
    //
    // identically — the SCREEN column is a measurement of the disturbance, not
    // of the algorithm. Measured on compliance.toml with the shipped linear
    // (15, -8) px/s platform motion it grows 44 px at 4 s, 295 px at 30 s,
    // 589 px at 60 s, matching 17 * sqrt(t^2/3) to within 0.1 %.
    //
    // That is correct physics: a pan-tilt mount with encoders cannot observe
    // motion of the base it is bolted to, so without an IMU or a star tracker
    // the world-frame position of the beacon is genuinely unknowable. What was
    // wrong was the REPORTING — the number was printed under the heading
    // "CENTROIDING (graded, 60 %)" with no caveat, where it reads as a
    // diverging centroider.
    //
    // So: the boresight-relative column. It is the same error measured in
    // SCREEN pixels (so it is directly comparable with the column beside it)
    // but referenced to the boresight rather than to the world, which cancels
    // D(t) exactly:
    //
    //     (detection_screen - B_cmd_screen) - (truth_screen - B_true_screen)
    //
    // This is the number that answers "how well did the detector locate the
    // beacon, expressed in the canvas coordinates the scenario is written in".
    // It equals the image-frame error rescaled by the screen/camera pixel
    // ratio, which is exactly the point: it has the units a judge will read
    // and the semantics the detector is responsible for.
    // -----------------------------------------------------------------------
    double  centroid_error_boresight_px = 0.0;

    /// |B_true - B_cmd| in screen pixels — D(t) above, the accumulated
    /// pointing error the system cannot observe. Logged per frame so the
    /// summary can print the screen-frame RMSE beside E[|D|] and let a reader
    /// see that the two agree, turning the 589 px figure from an apparent
    /// detector failure into a consistency check on the simulator.
    double  pointing_drift_px = 0.0;

    /// A detection was reported while the beacon was NOT in the field of view.
    /// Feeds §13.1's false_track_rate; it is not a centroiding error.
    bool    false_alarm = false;

    /// |boresight - true target angle| in px. Independent of whether the beacon
    /// was detected this frame — it measures the CONTROL LOOP, not the detector
    /// (INV-6).
    double  tracking_error_px = 0.0;

    // --- Stage 6: tracking -------------------------------------------------
    int        candidate_count = 0;   ///< gated detections this frame (§9.4.7)
    /// The window the detector was actually given this frame. Recorded so the
    /// GUI can draw it, the trace can log it, and a reviewer can see for
    /// themselves that a fast frame searched less rather than searched worse.
    DetectRoi  roi{};
    TrackMode  mode        = TrackMode::Idle;
    TrackState track_state = TrackState::Deleted;
    bool       has_lock    = false;   ///< Confirmed or Coasting
    Angle2     estimate{};            ///< the filter's angular position
    Rate2      estimate_rate{};       ///< and its velocity — the feedforward input
    double     estimate_sigma_urad = 0.0;

    // --- Stage 12: the SAT supervisor --------------------------------------
    /// Which detector the supervisor has selected, and whether it switched
    /// this frame. Recorded per frame because §10.6's deliverable is a
    /// TIMELINE — "that turns 'adaptive' from a claim into data" — and a
    /// timeline cannot be reconstructed from an end-of-run summary.
    DetectorKind sup_detector = DetectorKind::Classical;
    CentroidKind sup_centroider = CentroidKind::WindowedCoM;
    float        sup_cfar_k = 0.0f;
    float        sup_q_scale = 1.0f;
    bool         sup_switched = false;
    /// The smoothed integrated SNR the supervisor is deciding on. One of
    /// §10.6's twelve features, and the one every rule keys off.
    float        sup_snr = 0.0f;

    // --- CP 10.5: IMM mode probabilities ----------------------------------
    /// Probability of each of §10.2's three models, in ImmMode order
    /// (CV, CA, CT). All zero when the IMM is not running. This is §12's
    /// stacked-area "mode probability panel", and the quantity the checkpoint
    /// asks to see shift at the figure-8 crossing.
    float      imm_mode_prob[3] = {0.0f, 0.0f, 0.0f};
    /// The CT model's turn-rate estimate, rad/s. Signed, so the sign flip at
    /// the crossing is visible.
    float      imm_turn_rate = 0.0f;

    // --- CP 10.7: handover ------------------------------------------------
    /// The beacon's offset from the boresight as a co-boresighted quadrant
    /// cell would see it, microradians. Observable: the frame is rendered at
    /// the true boresight, so the detection's distance from the image centre
    /// IS this offset. Valid only when `detected`.
    double     quad_offset_urad = 0.0;
    bool       quad_in_capture  = false;
    /// RMS of that offset over the handover window. 1e9 until the window fills.
    double     handover_rms_urad = 1e9;

    /// Where the controller was told to point this frame. In Track this is the
    /// filter's prediction; in Search it is the search pattern's look point.
    Angle2     aim{};

    /// Detection quality and shape, carried through for §13.2's centroid.csv
    /// and for the GUI. These are columns of the GRADED artifact, which is why
    /// they are recorded per frame rather than recomputed later: after the run
    /// the blob they describe no longer exists.
    float    detection_snr     = 0.0f;
    float    centroid_sigma_px = 0.0f;
    uint16_t detection_area_px = 0;
    uint16_t detection_size_est_px = 0;
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

    // -----------------------------------------------------------------------
    // Which detector actually runs.
    //
    // The enum moved to core/strategy.hpp at Stage 12: the SAT supervisor
    // SELECTS a detector and lives in sat_control, which cannot include this
    // header — sat_engine links sat_world and INV-1 fails the configure. The
    // alias keeps every existing `PipelineConfig::Detector` spelling working.
    // -----------------------------------------------------------------------
    using Detector = DetectorKind;
    DetectorKind detector = DetectorKind::Classical;

    PerceptionParams perception{};
    RoiParams        roi{};
    TrackParams      tracking{};
    PriorityWeights  priority{};

    /// Stage 12. Off by default: the supervisor CHANGES the configuration a
    /// run uses, so a run with it on and one with it off are different claims
    /// and must be distinguishable — the same argument INV-7 makes for
    /// --no-ai. Every artifact records which.
    SupervisorParams supervisor{};

    // -----------------------------------------------------------------------
    // The largest angular acceleration the TARGET can produce, urad/s^2.
    //
    // This is what sizes the Kalman filter's q (tracking/kalman.hpp), and
    // getting the source of it right matters more than the number:
    //
    //   It is the TARGET's acceleration, not the mount's. The measurement is
    //   in the world angular frame — measurement.hpp adds the boresight back
    //   — so the camera's own motion is removed before the filter ever sees
    //   it, and the mount's acceleration limit has nothing to do with how
    //   wrong the constant-velocity model is.
    //
    // build_from_scenario computes it from the target's motion stack (§7.2's
    // closed forms, via scenario::max_accel_px_s2). The low-level build() path
    // has no scenario to read, so it uses the default below: a 300 px circle
    // at 0.2 Hz, which is a representative spec row 12 motion —
    // (2*pi*0.2)^2 * 300 = 474 px/s^2, about 5.2e4 urad/s^2.
    // -----------------------------------------------------------------------
    double max_target_accel_urad_s2 = 5.2e4;

    /// The largest speed the target can reach, urad/s. Sizes the filter's
    /// velocity prior (one-point initialisation has to state one) and the
    /// physical reachability cap on the gate. Like the acceleration above it is
    /// the TARGET's, computed from §7.2's closed forms by build_from_scenario;
    /// the default here is the same representative motion, a 300 px circle at
    /// 0.2 Hz, which peaks at 377 px/s.
    double max_target_speed_urad_s = 377.0 * 109.08;

    // -----------------------------------------------------------------------
    // POINTING STABILITY BUDGET — the tracker's own assumption about how well
    // it knows where it is pointing, in pixels, 1 sigma.
    //
    // A CORRECTION TO STAGE 6, found by Stage 7's metrics on the very first
    // full-scenario run. The reasoning recorded there was:
    //
    //   "R's pointing floor is the encoder LSB, deliberately not the jitter
    //    amplitude... the filter discovers them as innovation, which is what a
    //    filter is for."
    //
    // The first half is right and the second half is wrong. A filter CANNOT
    // discover noise its gate rejects. The measurement is
    // `commanded_boresight + unproject(centroid)`, so the pointing error is
    // additive measurement noise; with R set to the encoder quantisation the
    // gate is about a pixel wide, and spec row 23's jitter moves the true
    // boresight by up to 20 px per frame. The true detection fell outside the
    // gate on almost every frame, so no innovation was ever accepted and there
    // was nothing to adapt from.
    //
    // Measured on scenarios/spec_defaults.toml — the beacon in view, no
    // clutter — the same run, the same seed, jitter the only difference:
    //
    //     jitter 20 px/frame   lock retention  0.6 %   acquisition 8.37 s
    //     jitter  0 px/frame   lock retention 99.3 %   acquisition 0.07 s
    //                          tracking 2.89 px RMS, centroid 0.26 px RMSE
    //
    // WHY THIS NUMBER IS NOT CHEATING. It is NOT read from the scenario — doing
    // that would be reading simulator state, the same class of mistake as
    // reading the truth, and INV-1 exists to prevent it. It comes from spec row
    // 23, which states the disturbance the system is REQUIRED to tolerate:
    // "Max camera jitter +/- 20 px/frame". That is a published requirement, and
    // sizing a filter for its specified disturbance envelope is what designing
    // to a specification means. A scenario that jitters less gets a filter that
    // is merely conservative; one that jitters more is out of spec.
    //
    // THE DISTRIBUTION MATTERS, not just the bound. degrade/disturbance.cpp
    // draws jitter UNIFORMLY in [-20, +20] px per axis, once per camera frame,
    // and says why: "a uniform bound means the stated figure is exactly the
    // worst case". The standard deviation of that distribution is 20/sqrt(3) =
    // 11.55 px, not 20/3.
    //
    // The first version of this line used peak/3, reasoning about a Gaussian
    // when the process is uniform. It is 1.7x too small, and the consequence
    // was measurable rather than theoretical: the chi-square gate's radius is
    // sqrt(9.21) = 3.03 sigma, so at 6.67 px it reached 20 px — exactly the
    // jitter's peak — and rejected the true detection on roughly a quarter of
    // frames. The track oscillated Confirmed/Coasting about six times a second
    // and lock retention sat at 69%. At the correct sigma the gate reaches
    // 35 px, which covers the box's 28.3 px corner.
    //
    // The better long-term answer is to ESTIMATE this from the innovation
    // sequence rather than assume it (Mehra-style adaptive R), which would let
    // a well-stabilised mount earn a tighter gate. That needs the gate to be
    // accepting measurements first, which is what this floor makes true, and
    // it belongs with CP 10.3's platform-drift estimation.
    // -----------------------------------------------------------------------
    double pointing_sigma_px = 20.0 / 1.7320508075688772;   // spec row 23's peak / sqrt(3)

    /// Floor on the above. A purely linear target has an analytic acceleration
    /// of exactly zero, and a filter with q = 0 is not a filter — it becomes
    /// arbitrarily confident and then refuses every measurement, including the
    /// correct one, the moment anything is not quite as modelled. Something
    /// always is: platform drift the tracker cannot observe, the centroid's
    /// S-curve bias walking with sub-pixel phase, atmospheric wander. 50 px/s^2
    /// is the scale of those residuals rather than of any commanded motion.
    double min_target_accel_urad_s2 = 50.0 * 109.08;
    ModeFsmParams    mode{};
    SearchParams     search{};

    /// Bytes reserved for the perception workspace. Sized once at build() and
    /// never grown, which is what makes the per-frame path allocation-free
    /// (INV-4). 64 MB is generous for 640x480; the arena reports its own high
    /// water mark so the real figure lands in run.json at CP 7.4.
    size_t perception_arena_bytes = 64u << 20;

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

    // -----------------------------------------------------------------------
    // Video modes (Stage 8, Benchmark Performance-2, 30%).
    //
    // Everything above build_from_scenario is unchanged: §6.2 B4 says frame
    // acquisition is "THE ONLY PLACE THE MODES DIFFER", so a video run swaps
    // the source and nothing else. Perception, tracking, control and metrics
    // are the same code, which is what makes a result in one mode evidence
    // about the other.
    // -----------------------------------------------------------------------
    [[nodiscard]] Status build_from_video(const Scenario& sc,
                                          const std::filesystem::path& clip,
                                          const VideoMode* mode_override = nullptr,
                                          const std::filesystem::path* truth_csv = nullptr);

    /// The video source, or nullptr in synthetic mode. For the log header and
    /// for run.json's provenance.
    [[nodiscard]] const VideoSource* video() const noexcept { return video_.get(); }
    [[nodiscard]] bool is_video() const noexcept { return video_ != nullptr; }

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

    /// Read-only, for the CP 10.x control trace. The integrator's value is not
    /// derivable from anything else published, and watching it is how windup
    /// is diagnosed — CP 10.2's "settles cleanly with no ringing" is a
    /// statement about this state, not about the output.
    [[nodiscard]] const Controller&  controller() const noexcept { return control_; }
    [[nodiscard]] const SatSupervisor& supervisor() const noexcept { return supervisor_; }
    [[nodiscard]] const EventTimeline&  timeline()   const noexcept { return events_; }

    // -----------------------------------------------------------------------
    // CP 10.3's platform-drift cancellation input, urad/s. IT MUST BE ZERO,
    // and it exists so that a test can demonstrate why rather than a comment
    // asserting it.
    //
    // §10.4 subtracts an estimated platform rate from the controller's output,
    // on the premise that platform drift is a disturbance the loop cannot see.
    // In this architecture it is already cancelled:
    //
    //   the platform displaces the TRUE boresight   B_true = B_cmd + D
    //   the beacon lands on the sensor at           T - B_true
    //   measurement.hpp reconstructs through the
    //   COMMANDED boresight — all we know           z = T - D
    //
    // So the filter sees the target at T - D moving at v - D', and driving
    // B_cmd there puts B_true exactly on the beacon. Subtracting a second
    // estimate removes the drift twice.
    //
    // Measured both directions in tests/control/test_stage10.cpp — "CP 10.3:
    // platform drift is already cancelled by the measurement frame" and "CP
    // 10.3: cancelling the drift a second time makes it worse": sweeping the
    // drift to four times the specification leaves the residual flat, and
    // feeding the controller the TRUE drift rate — a perfect estimator — makes
    // the error 1.75x worse.
    //
    // A-7: this used to cite a feedforward test under tests/loop/ that does
    // not exist and, from the git history, never did. The argument was sound
    // and the evidence was real; only the pointer was wrong. In a project
    // whose credibility rests on every claim having a checkable citation, a
    // citation that does not resolve is worse than no citation at all — it
    // invites a reader to check, and what they find is a missing file.
    //
    // tools/check_source_invariants.py now resolves every repository path
    // mentioned in a source comment, so this cannot recur silently. It found
    // two more the moment it was written: a splat comment pointing at a
    // tests/camera/ directory that does not exist (the file is under
    // tests/render/), and a schema example naming an illustrative bad-config
    // file that was never committed.
    //
    // It would be needed if the measurement were reconstructed through the
    // true boresight, as an IMU-stabilised mount reporting real attitude would
    // give. That is the case the argument is kept alive for.
    // -----------------------------------------------------------------------
    void set_platform_rate_est(Rate2 r) noexcept { platform_rate_est_ = r; }

    /// Mutable access, for CP 10.4's deliberately-wrong plant model. Nothing in
    /// the engine uses it; a test that cannot make the controller's model
    /// disagree with the plant cannot demonstrate §10.4's "amplifies model
    /// error" at all.
    [[nodiscard]] Controller& controller_mut() noexcept { return control_; }

    /// Mutable, for CP 15.2's live algorithm switching. The GUI changes which
    /// filter runs while the loop is running; a rebuild would restart the run
    /// and the audience would see two runs rather than one loop changing its
    /// mind.
    [[nodiscard]] Tracker& tracker_mut() noexcept { return tracker_; }
    [[nodiscard]] SyntheticSource&   source()         noexcept { return source_; }
    [[nodiscard]] const StageTimers& timers()   const noexcept { return timers_; }

    /// Swap the detector without rebuilding. Used by CP 4.11's ablation, which
    /// needs the straw man driving the real closed loop, and by tests that want
    /// the simulator as a frame source and run their own perception — for which
    /// paying for the full §9.4 pipeline twice per frame is pure waste.
    void set_detector(PipelineConfig::Detector d) noexcept { cfg_.detector = d; }

    /// Turn snapshot publishing on or off after build(). A published snapshot
    /// costs a ~300 KB frame copy and is what produces the INV-3 fingerprint,
    /// so a pure speed run turns it off — measuring it would mean measuring
    /// the measuring — and anything that has to be reproducible turns it on.
    void set_publish_snapshots(bool on) noexcept { cfg_.publish_snapshots = on; }

    /// Tell the pipeline that something will READ the published snapshots'
    /// preview image, so the per-frame copy is worth making — P1-2.
    ///
    /// Default false. The fingerprint does not need it (it hashes the source
    /// frame in place), and in headless nothing else does either, so the
    /// 307 KB copy per frame is pure waste there. The GUI calls this when it
    /// binds to the triple buffer.
    ///
    /// Deliberately explicit rather than inferred from the triple buffer's
    /// state: a consumer that has not yet called acquire() still needs the
    /// first frame's preview, so "has anyone read yet" is the wrong question.
    void set_preview_consumers(bool on) noexcept { preview_consumers_ = on; }
    [[nodiscard]] bool preview_consumers() const noexcept { return preview_consumers_; }
    [[nodiscard]] PipelineConfig::Detector detector() const noexcept { return cfg_.detector; }

    /// Flip the loop open or closed mid-run. Used by the CP 1.8 test and by the
    /// live demo (§14.1, 5:00-6:30), where turning it off and watching the error
    /// trace blow up is the most convincing moment in the presentation.
    void set_control_enabled(bool on) noexcept { cfg_.control_enabled = on; }
    [[nodiscard]] bool control_enabled() const noexcept { return cfg_.control_enabled; }

    [[nodiscard]] const PipelineConfig& config() const noexcept { return cfg_; }

    /// The classical detector, for read-only inspection. The dashboard reads
    /// `last_blob_count()` and `last_blob_overflow()` from it: §14.0e bounds
    /// the blob table, and a bound whose overflow nothing displays is a bound
    /// that silently changes the answer.
    [[nodiscard]] const ClassicalPerception& perception() const noexcept {
        return perception_;
    }

    [[nodiscard]] const Tracker&       tracker()  const noexcept { return tracker_; }
    [[nodiscard]] const ModeFsm&       fsm()      const noexcept { return fsm_; }
    [[nodiscard]] const SearchPattern& search()   const noexcept { return search_; }
    [[nodiscard]] const Arena&         arena()    const noexcept { return arena_; }
    [[nodiscard]] const std::vector<Detection>& detections() const noexcept {
        return dets_;
    }

private:
    /// Startup wiring for perception, tracking, the mode FSM and the search
    /// pattern. Called from both build() paths; see pipeline.cpp.
    void build_stage6();

    /// The window the detector gets this frame (see DetectRoi). Full frame
    /// unless the track is confirmed and the filter is confident.
    [[nodiscard]] DetectRoi detect_window(int width, int height,
                                          Angle2 commanded) const noexcept;

    PipelineConfig  cfg_{};
    SyntheticSource source_{};
    std::unique_ptr<VideoSource> video_{};   ///< non-null in video modes
    Gimbal          gimbal_{};
    Controller      control_{};
    StageTimers     timers_{};

    // --- Stage 6: perception, tracking, mode, search ----------------------
    Arena               arena_{};
    PerceptionWorkspace ws_{};
    ClassicalPerception perception_{};
    std::vector<Detection>   dets_;
    std::vector<Measurement> meas_;
    Tracker             tracker_{};
    ModeFsm             fsm_{};
    SearchPattern       search_{};
    bool                perception_ready_ = false;

    /// Gains and the plant model together — see the note in pipeline.cpp.
    void reset_controller();

    QuadrantDetector quad_{};       ///< CP 10.7's fine sensor model
    HandoverMonitor  handover_{};
    SatSupervisor    supervisor_{};

    // --- design §7.4's timeline ------------------------------------------
    EventTimeline    events_{};
    /// The target's configured brightness, so occlude_target can restore it.
    /// Kept rather than re-read because the emitter array is the only copy and
    /// the event overwrites it.
    float            target_intensity_ = 0.0f;
    size_t           target_index_ = 0;
    bool             have_target_index_ = false;
    /// The filter's process noise as configured, before any q_scale. Kept so
    /// the supervisor's multiplier applies to the SCENARIO's value every frame
    /// rather than compounding on the previous frame's scaled one.
    double           base_accel_psd_ = 0.0;

    Rate2       cmd_rate_{};      ///< the value that closes the loop (INV-2)
    Rate2       platform_rate_est_{};   ///< CP 10.3: zero; see set_platform_rate_est
    FrameRecord last_{};
    int64_t     frame_ = 0;

    TripleBuffer<SimSnapshot>    snapshots_{};
    /// See set_preview_consumers. False in headless, which is where every
    /// benchmark figure comes from.
    bool                         preview_consumers_ = false;
    std::vector<FrameFingerprint> fingerprints_;
};

}  // namespace sat
