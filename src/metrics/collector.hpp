// metrics/collector.hpp — CP 7.1.
//
// Design §13.1's definitions, implemented one for one. The definitions
// themselves are reproduced verbatim in docs/METRICS.md, which the checkpoint
// requires; this file is where they become arithmetic.
//
// ---------------------------------------------------------------------------
// WHY THIS STAGE IS "THE PIVOT"
// ---------------------------------------------------------------------------
// §14's roadmap says of Stage 7: "After this stage, every change is measurable.
// Before it, you are guessing." That is not rhetoric. Everything up to here was
// justified by a checkpoint's acceptance test — a single number from a single
// scenario, read once by a person. From here on, a change is accompanied by a
// distribution over hundreds of runs, and a regression is something the build
// reports rather than something someone notices.
//
// ---------------------------------------------------------------------------
// INV-6, MECHANISED
// ---------------------------------------------------------------------------
// "Centroiding error and tracking error are distinct." CP 7.1 restates it:
// "centroiding and tracking error STRICTLY SEPARATE". They are separate fields
// here, computed from different inputs, over different frame sets, and they
// cannot be combined by accident because no code path adds them:
//
//   centroiding   detector vs truth, IN THE IMAGE and on the screen, on frames
//                 WITH A DETECTION and with the beacon actually visible.
//   tracking      boresight vs truth, on frames WHILE CONFIRMED (§13.1 is
//                 explicit about that restriction, and it matters — scoring
//                 the mount's aim during Search would measure the search
//                 pattern, not the loop).
//
// ---------------------------------------------------------------------------
// THE FRAME SETS ARE DIFFERENT, AND THAT IS THE POINT
// ---------------------------------------------------------------------------
// Four different denominators appear below and each is chosen deliberately:
//
//   every frame            saturation_frac, fps
//   frames beacon in FOV   lock_retention_rate  (§13.1's own wording)
//   frames while Confirmed tracking_error
//   frames with detection  centroiding_error
//
// Using one denominator for all of them is the single easiest way to produce a
// flattering number: a run that loses the beacon for half its length and tracks
// beautifully for the other half has excellent tracking error and terrible lock
// retention, and reporting either alone is a misrepresentation.

#pragma once

#include "core/mode.hpp"
#include "core/profile.hpp"
#include "engine/pipeline.hpp"
#include "metrics/series.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace sat {

// ---------------------------------------------------------------------------
// RunMetrics — the finished numbers for one run.
//
// A plain struct, because it is what run.json serialises (CP 7.4), what the
// compliance matrix reads (CP 7.7) and what the sweep aggregates (CP 7.5).
// ---------------------------------------------------------------------------
struct RunMetrics {
    // --- centroiding — GRADED, 60% of BP-1/BP-2 ---------------------------
    // §13.1 says "in SCREEN pixels". Both frames are kept anyway, because
    // INV-6's note in engine/pipeline.hpp applies: the screen figure also
    // contains the pointing error, which the detector has no influence over.
    // Reporting only the screen number would make the graded metric dominated
    // by a disturbance, and reporting only the image number would overstate
    // what the system actually delivers. Both, labelled.
    double centroid_rmse_screen_px = 0.0;
    double centroid_bias_x_px      = 0.0;   ///< mean SIGNED — reveals a fixable offset
    double centroid_bias_y_px      = 0.0;
    double centroid_p95_screen_px  = 0.0;
    double centroid_max_screen_px  = 0.0;

    double centroid_rmse_image_px  = 0.0;   ///< the detector alone
    double centroid_p95_image_px   = 0.0;
    double centroid_max_image_px   = 0.0;
    int64_t centroid_frames        = 0;     ///< the denominator, stated

    /// The third column — see the note on FrameRecord::centroid_error_boresight_px.
    /// Screen pixels, but with the unobservable platform displacement D(t)
    /// cancelled, so it stays bounded where the screen column grows as
    /// integral|D|dt. This is the figure a reader should compare against a
    /// sub-pixel expectation.
    double centroid_rmse_boresight_px = 0.0;
    double centroid_p95_boresight_px  = 0.0;
    double centroid_max_boresight_px  = 0.0;

    /// RMS of |B_true - B_cmd| over every truth-bearing frame: the disturbance
    /// the screen column is really measuring. Printed beside the screen RMSE so
    /// the two can be seen to agree.
    double pointing_drift_rms_px = 0.0;
    double pointing_drift_max_px = 0.0;

    // --- tracking — spec row 17 -------------------------------------------
    double tracking_rms_px    = 0.0;
    double tracking_p95_px    = 0.0;
    double tracking_max_px    = 0.0;
    double tracking_rms_urad  = 0.0;        ///< §13.1: "in px AND urad"
    int64_t tracking_frames   = 0;          ///< frames scored, i.e. Confirmed

    /// The PS's Performance Log deliverable asks in so many words for
    /// "average and maximum tracking error". §13.1 reports RMS, p95 and max
    /// and argues correctly that the mean hides a heavy tail — but the
    /// deliverable is a literal requirement and the two numbers are different
    /// enough to be worth printing together. On a jitter-free compliance run
    /// the mean is ~0.5 px where the RMS is 3.36 px, and the gap IS the
    /// acquisition transient; seeing both side by side is what makes that
    /// legible. Reported, never graded — row 17 is graded on the steady-state
    /// RMS below.
    double tracking_mean_px   = 0.0;

    // --- transient vs steady state — P1-9 ---------------------------------
    //
    // A 20 s jitter-free compliance run used to print:
    //
    //     TRACKING  RMS 3.357 px   p95 0.973   max 45.886
    //
    // RMS greater than p95 is arithmetically fine for a heavy-tailed sample —
    // a handful of 45 px samples from the acquisition slew dominate the sum of
    // squares — but it reads as a bug, and the honest reading is that two
    // different regimes are being averaged into one number. The first frames
    // after a lock are the mount still slewing onto the target; they measure
    // the SLEW, not the loop.
    //
    // So the sample is split at a fixed settle window after the first
    // Confirmed frame (`kSettleFrames` in collector.cpp) and both halves are
    // reported. Row 17 is graded on the steady-state figure, with the split
    // stated inline so the choice cannot be mistaken for cherry-picking.
    double  tracking_rms_steady_px    = 0.0;
    double  tracking_p95_steady_px    = 0.0;
    double  tracking_max_steady_px    = 0.0;
    double  tracking_mean_steady_px   = 0.0;
    int64_t tracking_frames_steady    = 0;
    double  tracking_rms_transient_px = 0.0;
    double  tracking_max_transient_px = 0.0;
    int64_t tracking_frames_transient = 0;
    /// How many frames after the first lock are counted as transient.
    int64_t settle_frames             = 0;

    // --- acquisition — spec rows 16 and 19 --------------------------------
    // Two acquisition numbers, always reported together and always labelled.
    // §10.5 derives that a cold sweep of the screen cannot meet row 16's 2 s,
    // and instructs: report both, clearly labelled. A single "acquisition
    // time" would be either a bound-breaking claim or a needlessly bad one
    // depending on which was chosen, and neither is honest.
    bool   acquired            = false;
    double acquisition_cold_s  = 0.0;       ///< run start -> first Confirmed
    bool   acquired_in_fov     = false;
    double acquisition_in_fov_s = 0.0;      ///< beacon enters FOV -> first Confirmed

    Series reacquisition_s;                 ///< one sample per lost-and-refound episode
    double reacquisition_mean_s = 0.0;
    double reacquisition_p95_s  = 0.0;
    double reacquisition_max_s  = 0.0;
    int64_t reacquisitions      = 0;

    // --- lock and losses — spec row 18 ------------------------------------
    double  lock_retention_rate = 0.0;      ///< Confirmed frames / in-FOV frames
    double  target_loss_frac    = 0.0;      ///< 1 - lock_retention_rate
    int64_t frames_in_fov       = 0;
    int64_t frames_confirmed    = 0;   ///< every Confirmed frame, in view or not
    int64_t frames_held_in_fov  = 0;   ///< Confirmed AND in view — the numerator

    // --- FOV containment — THE PROBLEM STATEMENT'S OWN OBJECTIVE ----------
    //
    // PS 26169 asks the system to "first locate and MAINTAIN the remote
    // terminal within its camera Field-of-View". That sentence names a
    // quantity, and until this block existed the project did not report it.
    //
    // Why it has to be its own metric rather than an inference from the two
    // above: lock_retention_rate is normalised over in-FOV frames ONLY, which
    // is the right denominator for the question "when the beacon was there to
    // be seen, did we hold it?" but is silent on the question "was it there to
    // be seen at all?". The two questions come apart badly. A 60 s run of the
    // old baseline scenario had the beacon inside the FOV for 79 of 1800
    // frames and reported "7.59 % target loss" — a PASS against row 18's 5 %
    // — while the mount spent 95.6 % of the run pointing at empty sky. The
    // denominator was hiding the failure, not causing it.
    //
    // So: three numbers, each with its denominator printed beside it.
    //
    //   fov_containment_frac      in-FOV frames / ALL frames. The blunt,
    //                             whole-run answer. Includes the initial
    //                             search, during which the beacon genuinely
    //                             cannot be in view, so it is pessimistic on
    //                             a cold start by construction.
    //   fov_containment_post_acq  in-FOV frames / frames after first lock.
    //                             The fair form: it asks how well the system
    //                             MAINTAINED containment once it had achieved
    //                             it, which is the verb the PS uses.
    //   target_loss_post_acq      1 - (held frames / frames after first lock).
    //                             Row 18 is graded on THIS, because it is the
    //                             only one of the three loss figures that
    //                             cannot be flattered by never acquiring.
    //
    // Both post-acquisition figures are undefined before the first Confirmed
    // frame; `post_acq_valid` says so rather than letting a zero denominator
    // print as a perfect score.
    double  fov_containment_frac     = 0.0;
    double  fov_containment_post_acq = 0.0;
    double  target_loss_post_acq     = 0.0;
    bool    post_acq_valid           = false;
    int64_t frames_post_acq          = 0;   ///< frames from the first lock onward
    int64_t frames_in_fov_post_acq   = 0;
    int64_t frames_held_post_acq     = 0;

    double  false_track_rate_per_min = 0.0; ///< §13.1, per minute
    int64_t false_tracks            = 0;

    // --- handover — CP 10.7 ------------------------------------------------
    //
    // The deliverable of COARSE alignment is a handover: the moment the loop
    // can tell a fine sensor "the beam is inside your capture range, take it".
    // A coarse tracker with no such moment has no definition of success beyond
    // "the error looks small", which is why this is reported alongside the
    // graded rows rather than as a curiosity.
    //
    // Reached-or-not per run; a sweep turns that into the success RATE the
    // checkpoint asks for, the same way it turns per-run acquisition times
    // into a distribution.
    bool    handover_reached = false;
    double  handover_time_s  = 0.0;     ///< from run start; 0 when never reached
    double  handover_rms_urad_best = 1e9;  ///< closest the loop ever got

    // --- plant and timing --------------------------------------------------
    double  saturation_frac = 0.0;
    double  fps_mean        = 0.0;          ///< sustained closed-loop
    double  fps_p5          = 0.0;          ///< the LOW tail: slow frames are the failure
    double  frame_ms_p50    = 0.0;
    double  frame_ms_p95    = 0.0;
    double  frame_ms_p99    = 0.0;

    /// False in video_direct: the frame does not follow the controller, so
    /// |boresight - target| measures nothing about the loop. §13.1's
    /// tracking_error is undefined there and is reported as such rather than
    /// as a large number that looks like a failure.
    bool pointing_supported = true;

    /// Frames whose truth was known at all. Zero in a video run with no
    /// --truth CSV, which is a completely different situation from a run that
    /// tracked nothing — and the summary has to say which, or a perfectly good
    /// video run reads as a total failure.
    int64_t frames_with_truth = 0;

    int64_t frames_total   = 0;
    double  duration_s     = 0.0;
    double  wall_time_s    = 0.0;           ///< measured OUTSIDE the sim (INV-3 safe)

    // --- provenance --------------------------------------------------------
    std::string scenario_name;
    uint64_t    seed = 0;
    bool        ai_enabled = false;
};

// ---------------------------------------------------------------------------
// MetricCollector
// ---------------------------------------------------------------------------
class MetricCollector {
public:
    /// Reserve for `expected_frames` so that add() never allocates (INV-4).
    void begin(const std::string& scenario_name, uint64_t seed,
               double ifov_urad, size_t expected_frames,
               bool pointing_supported = true);

    /// One frame. Called after Pipeline::step().
    void add(const FrameRecord& r);

    /// Finalise. `timers` supplies §13.1's processing percentiles and
    /// `wall_time_s` the sustained frame rate — both measured outside the
    /// simulation, so INV-3 is untouched.
    [[nodiscard]] RunMetrics finish(const StageTimers& timers,
                                    double wall_time_s,
                                    double saturation_frac) const;

    [[nodiscard]] int64_t frames() const noexcept { return frames_; }

private:
    // Per-frame series. Magnitudes for RMSE/p95/max, signed components for bias.
    Series centroid_screen_, centroid_image_, centroid_boresight_;
    Series centroid_dx_, centroid_dy_;
    Series drift_px_;
    /// Every Confirmed frame's pointing error, and the subset of those frames
    /// that fall outside the post-lock settle window. Two series rather than
    /// one plus an index because Series is append-only and the split point is
    /// known at push time — the frame index is compared against the settle
    /// deadline as each frame arrives.
    Series tracking_px_, tracking_steady_px_, tracking_transient_px_;
    Series reacq_s_;

    std::string name_;
    uint64_t    seed_ = 0;
    double      ifov_ = 1.0;
    bool        pointing_ = true;

    int64_t frames_          = 0;
    int64_t frames_in_fov_   = 0;
    int64_t frames_with_truth_ = 0;
    int64_t frames_confirmed_ = 0;
    /// Confirmed AND in view — the retention numerator. See collector.cpp.
    int64_t frames_held_in_fov_ = 0;

    // --- FOV containment, post-acquisition counters ------------------------
    // "Post-acquisition" starts on the FIRST Confirmed frame and runs to the
    // end of the run — it is never re-opened by a later re-acquisition,
    // because the question is "once you had it, how much of the remaining run
    // did you keep it?" and restarting the window on each re-lock would make
    // every dropout invisible.
    int64_t frames_post_acq_        = 0;
    int64_t frames_in_fov_post_acq_ = 0;
    int64_t frames_held_post_acq_   = 0;

    // CP 10.7. First entry only — see the note at the call site.
    bool    handover_reached_   = false;
    double  handover_time_s_    = 0.0;
    double  handover_best_urad_ = 1e9;
    int64_t false_tracks_    = 0;

    // --- acquisition and re-acquisition state ------------------------------
    bool   have_first_confirm_ = false;
    double first_confirm_s_    = 0.0;
    /// Frame index of the first Confirmed frame. The transient/steady split
    /// and the post-acquisition window both hang off this.
    int64_t first_confirm_frame_ = 0;
    bool   have_first_in_fov_  = false;
    double first_in_fov_s_     = 0.0;

    /// A re-acquisition episode is open once a CONFIRMED track has been lost.
    /// §13.1: "Confirmed -> lost -> Confirmed again". The clock starts at the
    /// loss, not at the moment the beacon becomes visible again — that is what
    /// spec row 19 is asking about, since the system does not know when the
    /// target came back.
    bool   lost_open_   = false;
    double lost_at_s_   = 0.0;
    bool   was_confirmed_ = false;

    double last_time_s_ = 0.0;
};

/// Render the §13.1 numbers as a human-readable block. Shared by --headless's
/// stdout summary and the report.
[[nodiscard]] std::string format_summary(const RunMetrics& m);

/// CP 14.4: "p50/p95/p99 per stage reportable from the SHIPPED binary."
///
/// Against §15's per-stage budget, so the table says not just how long each
/// stage took but how far from its target it is — which is the only form in
/// which the number leads anywhere.
[[nodiscard]] std::string format_stage_timings(const StageTimers& t);

}  // namespace sat
