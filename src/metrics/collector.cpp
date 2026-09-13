// metrics/collector.cpp

#include "metrics/collector.hpp"

#include <cstdio>
#include <cmath>

namespace sat {

void MetricCollector::begin(const std::string& scenario_name, uint64_t seed,
                            double ifov_urad, size_t expected_frames) {
    *this = MetricCollector{};
    name_ = scenario_name;
    seed_ = seed;
    ifov_ = (ifov_urad > 0.0) ? ifov_urad : 1.0;

    const size_t n = expected_frames + 2;
    centroid_screen_.reserve(n);
    centroid_image_.reserve(n);
    centroid_dx_.reserve(n);
    centroid_dy_.reserve(n);
    tracking_px_.reserve(n);
    // Re-acquisitions are rare; 64 is far more than any sane run produces and
    // still costs nothing.
    reacq_s_.reserve(64);
}

void MetricCollector::add(const FrameRecord& r) {
    ++frames_;
    last_time_s_ = r.time_s;

    const bool confirmed = (r.track_state == TrackState::Confirmed);

    // -----------------------------------------------------------------------
    // Denominator one: frames with the beacon inside the field of view.
    //
    // §13.1 defines lock_retention_rate as "frames Confirmed / frames beacon
    // within FOV". Using total frames instead would penalise the system for
    // the part of a run where the target is genuinely not there to be seen —
    // which is what the ACQUISITION metrics measure, separately.
    // -----------------------------------------------------------------------
    if (r.truth_valid && r.truth_in_fov) {
        ++frames_in_fov_;
        if (!have_first_in_fov_) {
            have_first_in_fov_ = true;
            first_in_fov_s_    = r.time_s;
        }
    }

    if (confirmed) ++frames_confirmed_;

    // --- centroiding, GRADED ----------------------------------------------
    // Only on frames with a detection AND with the beacon visible. The second
    // condition is the one §13.1 leaves implicit and engine/pipeline.cpp
    // already argues for: scoring a reported centroid against a beacon that is
    // not there measures nothing, and whatever was reported is a false alarm,
    // which is a different metric.
    if (r.centroid_error_valid) {
        centroid_screen_.push(r.centroid_error_screen_px);
        centroid_image_.push(r.centroid_error_px);
        // Signed components, for §13.1's "bias (mean signed)". A bias is a
        // systematic offset and is therefore removable; an RMSE alone cannot
        // tell you whether you have one.
        centroid_dx_.push(r.detection_screen.x - r.truth_screen.x);
        centroid_dy_.push(r.detection_screen.y - r.truth_screen.y);
    }

    // --- false tracks -------------------------------------------------------
    // §13.1: "Confirmed tracks on non-beacons, per minute". Counted as frames
    // where the system claims a confirmed lock while the beacon is not in view
    // — which is the observable form of that definition. A frame where
    // perception reported something with no lock is not a false TRACK; it is a
    // candidate the gate correctly threw away, and counting it would punish the
    // pipeline for the thing it does right.
    if (confirmed && r.truth_valid && !r.truth_in_fov) ++false_tracks_;

    // --- tracking error, ONLY WHILE CONFIRMED ------------------------------
    if (confirmed && r.truth_valid) {
        tracking_px_.push(r.tracking_error_px);
    }

    // --- acquisition --------------------------------------------------------
    if (confirmed && !have_first_confirm_) {
        have_first_confirm_ = true;
        first_confirm_s_    = r.time_s;
    }

    // --- re-acquisition episodes -------------------------------------------
    // The edge, not the level: an episode opens on the Confirmed -> not
    // Confirmed transition and closes on the next Confirmed frame. Written as
    // a transition rather than as "while not confirmed" so that a run which
    // ends mid-episode contributes nothing rather than a truncated sample —
    // a half-finished re-acquisition is not a fast one.
    if (was_confirmed_ && !confirmed) {
        lost_open_ = true;
        lost_at_s_ = r.time_s;
    } else if (lost_open_ && confirmed) {
        reacq_s_.push(r.time_s - lost_at_s_);
        lost_open_ = false;
    }
    was_confirmed_ = confirmed;
}

RunMetrics MetricCollector::finish(const StageTimers& timers,
                                   double wall_time_s,
                                   double saturation_frac) const {
    RunMetrics m;
    m.scenario_name = name_;
    m.seed          = seed_;
    m.frames_total  = frames_;
    m.duration_s    = last_time_s_;
    m.wall_time_s   = wall_time_s;

    m.centroid_rmse_screen_px = centroid_screen_.rms();
    m.centroid_p95_screen_px  = centroid_screen_.p95();
    m.centroid_max_screen_px  = centroid_screen_.max();
    m.centroid_bias_x_px      = centroid_dx_.mean();
    m.centroid_bias_y_px      = centroid_dy_.mean();
    m.centroid_rmse_image_px  = centroid_image_.rms();
    m.centroid_p95_image_px   = centroid_image_.p95();
    m.centroid_max_image_px   = centroid_image_.max();
    m.centroid_frames         = static_cast<int64_t>(centroid_screen_.count());

    m.tracking_rms_px   = tracking_px_.rms();
    m.tracking_p95_px   = tracking_px_.p95();
    m.tracking_max_px   = tracking_px_.max();
    m.tracking_rms_urad = m.tracking_rms_px * ifov_;
    m.tracking_frames   = static_cast<int64_t>(tracking_px_.count());

    m.acquired = have_first_confirm_;
    m.acquisition_cold_s = have_first_confirm_ ? first_confirm_s_ : 0.0;
    // In-FOV acquisition is only defined if the beacon was ever in view AND was
    // acquired after it arrived. A run where the beacon started in view has the
    // two clocks coincide, which is correct and is why baseline scenarios show
    // a cold time as good as the in-view one.
    m.acquired_in_fov = have_first_confirm_ && have_first_in_fov_
                     && first_confirm_s_ >= first_in_fov_s_;
    m.acquisition_in_fov_s = m.acquired_in_fov ? (first_confirm_s_ - first_in_fov_s_) : 0.0;

    m.reacquisition_mean_s = reacq_s_.mean();
    m.reacquisition_p95_s  = reacq_s_.p95();
    m.reacquisition_max_s  = reacq_s_.max();
    m.reacquisitions       = static_cast<int64_t>(reacq_s_.count());

    m.frames_in_fov    = frames_in_fov_;
    m.frames_confirmed = frames_confirmed_;
    // Guarded: a run where the beacon is never in view has no denominator, and
    // 0/0 reported as either 0% or 100% retention would be a lie in opposite
    // directions. It is reported as zero with frames_in_fov = 0 beside it, so
    // the reader can see the ratio is undefined rather than bad.
    m.lock_retention_rate = frames_in_fov_
        ? static_cast<double>(frames_confirmed_) / static_cast<double>(frames_in_fov_)
        : 0.0;
    // Clamped: Confirmed frames can exceed in-FOV frames when the filter coasts
    // correctly through a brief occlusion, which would otherwise report a
    // retention above 100% and a NEGATIVE loss fraction.
    if (m.lock_retention_rate > 1.0) m.lock_retention_rate = 1.0;
    m.target_loss_frac = frames_in_fov_ ? (1.0 - m.lock_retention_rate) : 0.0;

    m.false_tracks = false_tracks_;
    m.false_track_rate_per_min = (last_time_s_ > 0.0)
        ? (static_cast<double>(false_tracks_) * 60.0 / last_time_s_) : 0.0;

    m.saturation_frac = saturation_frac;

    const LatencyHistogram& total = timers[Stage::FrameTotal];
    m.frame_ms_p50 = total.p50() * 1e-3;    // the histogram is in microseconds
    m.frame_ms_p95 = total.p95() * 1e-3;
    m.frame_ms_p99 = total.p99() * 1e-3;
    // fps from the p50 and p95 frame times rather than from frames/wall_time:
    // the mean hides exactly what §13.1's "NEVER the mean" note is about, and
    // spec row 20's >= 20 FPS is a floor, so the figure that matters is the
    // slow tail. fps_p5 is therefore derived from the 95th percentile frame.
    m.fps_mean = (m.frame_ms_p50 > 0.0) ? (1000.0 / m.frame_ms_p50) : 0.0;
    m.fps_p5   = (m.frame_ms_p95 > 0.0) ? (1000.0 / m.frame_ms_p95) : 0.0;

    return m;
}

// ---------------------------------------------------------------------------
// format_summary — the §13.1 numbers as a block a person reads.
//
// Built in pieces rather than as one giant format string. The acquisition lines
// are conditional (a run that never acquired has no time to report, and
// printing 0.00 s there would read as instantaneous success), and the
// denominators are printed beside every ratio so a reader can see when one is
// undefined rather than merely small.
// ---------------------------------------------------------------------------
std::string format_summary(const RunMetrics& m) {
    std::string out;
    char b[512];
    auto line = [&](const char* fmt, auto... args) {
        std::snprintf(b, sizeof b, fmt, args...);
        out += b;
    };

    line("scenario            %s  (seed %llu, %lld frames, %.2f s simulated)\n\n",
         m.scenario_name.c_str(), static_cast<unsigned long long>(m.seed),
         static_cast<long long>(m.frames_total), m.duration_s);

    line("CENTROIDING  (graded, 60%%)          frames scored %lld\n",
         static_cast<long long>(m.centroid_frames));
    if (m.centroid_frames > 0) {
        line("  screen RMSE       %8.4f px   p95 %8.4f   max %8.4f\n",
             m.centroid_rmse_screen_px, m.centroid_p95_screen_px, m.centroid_max_screen_px);
        line("  image  RMSE       %8.4f px   p95 %8.4f   max %8.4f\n",
             m.centroid_rmse_image_px, m.centroid_p95_image_px, m.centroid_max_image_px);
        line("  bias (mean signed) %+7.4f, %+.4f px\n",
             m.centroid_bias_x_px, m.centroid_bias_y_px);
    } else {
        out += "  no frame had both a detection and the beacon in view\n";
    }

    line("\nTRACKING     (row 17, <= 10 px)      frames scored %lld (Confirmed only)\n",
         static_cast<long long>(m.tracking_frames));
    if (m.tracking_frames > 0) {
        line("  RMS               %8.3f px   p95 %8.3f   max %8.3f\n",
             m.tracking_rms_px, m.tracking_p95_px, m.tracking_max_px);
        line("  RMS               %8.1f urad\n", m.tracking_rms_urad);
    } else {
        out += "  the track was never Confirmed\n";
    }

    out += "\nACQUISITION  (row 16, <= 2 s)\n";
    if (m.acquired_in_fov) {
        line("  in-view           %8.3f s\n", m.acquisition_in_fov_s);
    } else {
        out += "  in-view                n/a   (beacon in view from t=0, or never acquired)\n";
    }
    if (m.acquired) {
        line("  cold              %8.3f s   <- bound derived, design 10.5\n",
             m.acquisition_cold_s);
    } else {
        out += "  cold                 never   <- bound derived, design 10.5\n";
    }

    line("REACQUISITION (row 19, <= 1 s)      episodes %lld\n",
         static_cast<long long>(m.reacquisitions));
    if (m.reacquisitions > 0) {
        line("  mean              %8.3f s    p95 %8.3f   max %8.3f\n",
             m.reacquisition_mean_s, m.reacquisition_p95_s, m.reacquisition_max_s);
    }

    out += "\nLOCK\n";
    if (m.frames_in_fov > 0) {
        line("  retention         %8.2f %%   (%lld confirmed / %lld in-FOV frames)\n",
             100.0 * m.lock_retention_rate,
             static_cast<long long>(m.frames_confirmed),
             static_cast<long long>(m.frames_in_fov));
        line("  target loss       %8.2f %%   (row 18, < 5 %%)\n", 100.0 * m.target_loss_frac);
    } else {
        out += "  retention              n/a   (the beacon was never in view)\n";
    }
    line("  false tracks      %8.2f /min (%lld frames confirmed with no beacon in view)\n",
         m.false_track_rate_per_min, static_cast<long long>(m.false_tracks));
    line("  gimbal saturation %8.2f %%\n", 100.0 * m.saturation_frac);

    out += "\nSPEED        (row 20, >= 20 FPS)\n";
    line("  frame time        p50 %.3f ms   p95 %.3f ms   p99 %.3f ms\n",
         m.frame_ms_p50, m.frame_ms_p95, m.frame_ms_p99);
    line("  fps               %8.1f (from p50)  %8.1f (from p95)\n", m.fps_mean, m.fps_p5);
    line("  wall time         %8.2f s for %.2f s simulated  (%.2fx real time)\n",
         m.wall_time_s, m.duration_s,
         (m.wall_time_s > 0.0) ? (m.duration_s / m.wall_time_s) : 0.0);

    return out;
}

}  // namespace sat
