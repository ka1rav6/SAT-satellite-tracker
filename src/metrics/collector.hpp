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

    // --- tracking — spec row 17 -------------------------------------------
    double tracking_rms_px    = 0.0;
    double tracking_p95_px    = 0.0;
    double tracking_max_px    = 0.0;
    double tracking_rms_urad  = 0.0;        ///< §13.1: "in px AND urad"
    int64_t tracking_frames   = 0;          ///< frames scored, i.e. Confirmed

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
    int64_t frames_confirmed    = 0;

    double  false_track_rate_per_min = 0.0; ///< §13.1, per minute
    int64_t false_tracks            = 0;

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
    Series centroid_screen_, centroid_image_;
    Series centroid_dx_, centroid_dy_;
    Series tracking_px_;
    Series reacq_s_;

    std::string name_;
    uint64_t    seed_ = 0;
    double      ifov_ = 1.0;
    bool        pointing_ = true;

    int64_t frames_          = 0;
    int64_t frames_in_fov_   = 0;
    int64_t frames_with_truth_ = 0;
    int64_t frames_confirmed_ = 0;
    int64_t false_tracks_    = 0;

    // --- acquisition and re-acquisition state ------------------------------
    bool   have_first_confirm_ = false;
    double first_confirm_s_    = 0.0;
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
