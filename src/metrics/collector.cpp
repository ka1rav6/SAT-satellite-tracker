// metrics/collector.cpp

#include "metrics/collector.hpp"

#include <cstdio>
#include <cmath>

namespace sat {

// ---------------------------------------------------------------------------
// THE TRANSIENT/STEADY-STATE SPLIT POINT — P1-9.
//
// 30 frames at the 30 Hz spec minimum is one second after the first lock.
//
// Why one second and not a tuned value: the mount's authority is 5 deg/s
// (spec row 13) across a 4 deg x 3 deg field (row 4), so a worst-case slew
// from one field edge to the other takes 0.8 s. One second is therefore the
// smallest round number that certainly contains the slew, and it is derived
// from the specification rather than from the data — which matters, because a
// split point chosen by looking at the error curve would be indistinguishable
// from cherry-picking however well it was argued.
//
// A frame count rather than a duration because the split has to be identical
// between a 30 Hz and a 120 Hz run of the same scenario for the two to be
// comparable, and because it keeps the arithmetic integral and therefore
// bit-exact (INV-3).
// ---------------------------------------------------------------------------
static constexpr int64_t kSettleFrames = 30;

void MetricCollector::begin(const std::string& scenario_name, uint64_t seed,
                            double ifov_urad, size_t expected_frames,
                            bool pointing_supported) {
    *this = MetricCollector{};
    pointing_ = pointing_supported;
    name_ = scenario_name;
    seed_ = seed;
    ifov_ = (ifov_urad > 0.0) ? ifov_urad : 1.0;

    const size_t n = expected_frames + 2;
    centroid_screen_.reserve(n);
    centroid_image_.reserve(n);
    centroid_boresight_.reserve(n);
    centroid_dx_.reserve(n);
    centroid_dy_.reserve(n);
    drift_px_.reserve(n);
    tracking_px_.reserve(n);
    // Both halves of the split get the full reservation. Together that is 2n
    // rather than n, but the split point is not known at begin() time and
    // INV-4 forbids an allocation inside add(); n extra doubles per run is the
    // right side of that trade.
    tracking_steady_px_.reserve(n);
    tracking_transient_px_.reserve(n);
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
    if (r.truth_valid) {
        ++frames_with_truth_;
        // D(t), on every truth-bearing frame regardless of detection. See the
        // note on FrameRecord::pointing_drift_px: this is what the screen-frame
        // centroiding column is actually measuring, and reporting it beside
        // that column is what makes the column legible.
        drift_px_.push(r.pointing_drift_px);
    }
    if (r.truth_valid && r.truth_in_fov) {
        ++frames_in_fov_;
        // The retention NUMERATOR, and it is deliberately inside this branch.
        // See the note at the ratio below for why counting every Confirmed
        // frame reports the most flattering possible number in exactly the
        // worst case.
        if (confirmed) ++frames_held_in_fov_;
        if (!have_first_in_fov_) {
            have_first_in_fov_ = true;
            first_in_fov_s_    = r.time_s;
        }
    }

    if (confirmed) ++frames_confirmed_;

    // -----------------------------------------------------------------------
    // Denominator two: frames from the first lock onward — the PS's own
    // objective, "MAINTAIN the remote terminal within its camera FOV".
    //
    // Opened by the first Confirmed frame and never closed. A run that never
    // locks contributes nothing here and reports the post-acquisition figures
    // as undefined, which is the only honest thing to say about "how well did
    // you maintain a lock you never had".
    //
    // Note the ordering: have_first_confirm_ is set further down, so on the
    // very frame of the first lock this test is still false and that frame is
    // excluded. That is deliberate — the window is "after acquisition", and
    // including the acquiring frame itself would put a frame that belongs to
    // the search into the retention figure.
    // -----------------------------------------------------------------------
    if (have_first_confirm_) {
        ++frames_post_acq_;
        if (r.truth_valid && r.truth_in_fov) {
            ++frames_in_fov_post_acq_;
            if (confirmed) ++frames_held_post_acq_;
        }
    }

    // --- CP 10.7: handover -------------------------------------------------
    // First entry only. A run that drops back to Track and re-enters would
    // otherwise report the LAST handover time, and the question being asked is
    // how long coarse alignment took, not how many times it succeeded.
    if (!handover_reached_ && r.mode == TrackMode::Handover) {
        handover_reached_ = true;
        handover_time_s_  = r.time_s;
    }
    if (r.handover_rms_urad < handover_best_urad_) {
        handover_best_urad_ = r.handover_rms_urad;
    }

    // --- centroiding, GRADED ----------------------------------------------
    // Only on frames with a detection AND with the beacon visible. The second
    // condition is the one §13.1 leaves implicit and engine/pipeline.cpp
    // already argues for: scoring a reported centroid against a beacon that is
    // not there measures nothing, and whatever was reported is a false alarm,
    // which is a different metric.
    if (r.centroid_error_valid) {
        centroid_screen_.push(r.centroid_error_screen_px);
        centroid_image_.push(r.centroid_error_px);
        centroid_boresight_.push(r.centroid_error_boresight_px);
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
    if (confirmed && r.truth_valid && pointing_) {
        tracking_px_.push(r.tracking_error_px);
        // P1-9: the same sample, additionally routed into one of two halves.
        // The split is a frame count from the first lock (kSettleFrames), so a
        // frame lands in "transient" only during the settle window that
        // immediately follows acquisition. Frames before any lock cannot reach
        // here at all — the track is not Confirmed — so the transient series
        // contains exactly the post-lock slew and nothing else.
        const bool transient =
            have_first_confirm_ && (frames_ - first_confirm_frame_) <= kSettleFrames;
        if (transient) tracking_transient_px_.push(r.tracking_error_px);
        else           tracking_steady_px_.push(r.tracking_error_px);
    }

    // --- acquisition --------------------------------------------------------
    if (confirmed && !have_first_confirm_) {
        have_first_confirm_  = true;
        first_confirm_s_     = r.time_s;
        first_confirm_frame_ = frames_;
        // The acquiring frame itself belongs to the transient by definition,
        // and the test above ran before have_first_confirm_ was set, so it was
        // classified as steady. Move it: one sample, but it is the largest one
        // in the run and leaving it in the steady series defeats the split.
        if (r.truth_valid && pointing_ && tracking_steady_px_.count() > 0) {
            tracking_steady_px_.pop_back();
            tracking_transient_px_.push(r.tracking_error_px);
        }
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
    m.frames_with_truth = frames_with_truth_;
    m.pointing_supported = pointing_;
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

    m.centroid_rmse_boresight_px = centroid_boresight_.rms();
    m.centroid_p95_boresight_px  = centroid_boresight_.p95();
    m.centroid_max_boresight_px  = centroid_boresight_.max();

    m.pointing_drift_rms_px = drift_px_.rms();
    m.pointing_drift_max_px = drift_px_.max();

    m.tracking_rms_px   = tracking_px_.rms();
    m.tracking_p95_px   = tracking_px_.p95();
    m.tracking_max_px   = tracking_px_.max();
    m.tracking_mean_px  = tracking_px_.mean();   // the PS's literal "average"
    m.tracking_rms_urad = m.tracking_rms_px * ifov_;
    m.tracking_frames   = static_cast<int64_t>(tracking_px_.count());

    // P1-9: the two regimes, each with its own denominator.
    m.settle_frames             = kSettleFrames;
    m.tracking_rms_steady_px    = tracking_steady_px_.rms();
    m.tracking_p95_steady_px    = tracking_steady_px_.p95();
    m.tracking_max_steady_px    = tracking_steady_px_.max();
    m.tracking_mean_steady_px   = tracking_steady_px_.mean();
    m.tracking_frames_steady    = static_cast<int64_t>(tracking_steady_px_.count());
    m.tracking_rms_transient_px = tracking_transient_px_.rms();
    m.tracking_max_transient_px = tracking_transient_px_.max();
    m.tracking_frames_transient = static_cast<int64_t>(tracking_transient_px_.count());

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
    m.frames_confirmed   = frames_confirmed_;
    m.frames_held_in_fov = frames_held_in_fov_;
    // -----------------------------------------------------------------------
    // §13.1: "lock_retention_rate = frames Confirmed / frames beacon in view".
    //
    // The numerator is frames that are Confirmed AND IN VIEW, not every
    // Confirmed frame, and the difference is not pedantry — it was reporting
    // 100% retention on a run that had lost the beacon entirely.
    //
    // A 60 s compliance run with 120 clutter sources ends with the tracker
    // holding a clutter source while the beacon has left the field: 1798
    // Confirmed frames against 727 in-view frames, a ratio of 2.47. The old
    // code clamped that to 1.0 and printed "retention 100.00 %" directly above
    // "false tracks 1073.60 /min". Both numbers were computed correctly and
    // together they were a contradiction, with the clamp turning the system's
    // worst failure into its best-looking metric.
    //
    // The clamp's own justification was real — a filter coasting correctly
    // through an occlusion does produce Confirmed frames while the beacon is
    // out of view — but intersecting the numerator with the denominator's
    // condition handles that case properly rather than papering over it: those
    // frames are in neither, so they neither inflate nor penalise. The ratio
    // now cannot exceed 1 by construction, so there is nothing to clamp.
    //
    // Guarded for the empty denominator: a run where the beacon is never in
    // view has no ratio, and 0/0 as either 0% or 100% would be a lie in
    // opposite directions. Zero, with frames_in_fov = 0 printed beside it.
    // -----------------------------------------------------------------------
    m.lock_retention_rate = frames_in_fov_
        ? static_cast<double>(frames_held_in_fov_) / static_cast<double>(frames_in_fov_)
        : 0.0;
    m.target_loss_frac = frames_in_fov_ ? (1.0 - m.lock_retention_rate) : 0.0;

    // -----------------------------------------------------------------------
    // FOV CONTAINMENT — the PS's stated objective, finally reported.
    //
    // "first locate and maintain the remote terminal within its camera
    // Field-of-View." The retention figure above answers a narrower question
    // (given that it was visible, did we hold it?) and on a run that never
    // acquires it answers it flatteringly: the old baseline scenario put the
    // beacon in the FOV for 79 of 1800 frames and reported 7.59 % target loss,
    // a PASS against row 18, while pointing at empty sky for 95.6 % of the run.
    //
    // The whole-run fraction is the blunt answer and it is deliberately
    // pessimistic during a cold search. The post-acquisition pair is the fair
    // one, and row 18 is graded on target_loss_post_acq because it is the only
    // loss figure that a run which never acquires cannot flatter.
    // -----------------------------------------------------------------------
    m.frames_post_acq        = frames_post_acq_;
    m.frames_in_fov_post_acq = frames_in_fov_post_acq_;
    m.frames_held_post_acq   = frames_held_post_acq_;
    m.fov_containment_frac   = frames_
        ? static_cast<double>(frames_in_fov_) / static_cast<double>(frames_)
        : 0.0;
    m.post_acq_valid = (frames_post_acq_ > 0);
    if (m.post_acq_valid) {
        m.fov_containment_post_acq =
            static_cast<double>(frames_in_fov_post_acq_) / static_cast<double>(frames_post_acq_);
        m.target_loss_post_acq = 1.0 -
            static_cast<double>(frames_held_post_acq_) / static_cast<double>(frames_post_acq_);
    }

    m.false_tracks = false_tracks_;
    m.false_track_rate_per_min = (last_time_s_ > 0.0)
        ? (static_cast<double>(false_tracks_) * 60.0 / last_time_s_) : 0.0;

    m.saturation_frac = saturation_frac;
    m.handover_reached = handover_reached_;
    m.handover_time_s  = handover_time_s_;
    m.handover_rms_urad_best = handover_best_urad_;

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

void MetricCollector::fill_latency(RunMetrics& m, double exposure_s,
                                   double transport_s) {
    // The frame is an INTEGRAL over the exposure window, so its effective
    // instant is the middle of that window, not its end. Half the exposure is
    // therefore already in the past by the time the frame exists, before any
    // processing has happened at all.
    m.latency_exposure_ms  = 0.5 * exposure_s * 1e3;
    m.latency_compute_ms   = m.frame_ms_p50;
    m.latency_transport_ms = transport_s * 1e3;
    m.latency_total_ms     = m.latency_exposure_ms + m.latency_compute_ms
                           + m.latency_transport_ms;
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
    // __attribute__((format)) cannot be applied to a generic lambda, and
    // without it GCC cannot see that `fmt` is always a literal at the call
    // site — it only sees snprintf handed a runtime pointer, which is
    // -Wformat-security's whole point. The format strings below are all
    // literals in this function, so the check is satisfied by inspection;
    // the pragma says so at the narrowest scope that works rather than
    // turning the warning off for the translation unit.
#if defined(__GNUC__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wformat-security"
#endif
    auto line = [&](const char* fmt, auto... args) {
        std::snprintf(b, sizeof b, fmt, args...);
        out += b;
    };
#if defined(__GNUC__)
#  pragma GCC diagnostic pop
#endif

    line("scenario            %s  (seed %llu, %lld frames, %.2f s simulated)\n\n",
         m.scenario_name.c_str(), static_cast<unsigned long long>(m.seed),
         static_cast<long long>(m.frames_total), m.duration_s);

    line("CENTROIDING  (graded, 60%%)          frames scored %lld\n",
         static_cast<long long>(m.centroid_frames));
    if (m.centroid_frames > 0) {
        line("  image  RMSE       %8.4f px   p95 %8.4f   max %8.4f   <- the detector alone\n",
             m.centroid_rmse_image_px, m.centroid_p95_image_px, m.centroid_max_image_px);
        line("  bore   RMSE       %8.4f px   p95 %8.4f   max %8.4f   <- screen px, boresight-referenced\n",
             m.centroid_rmse_boresight_px, m.centroid_p95_boresight_px,
             m.centroid_max_boresight_px);
        line("  screen RMSE       %8.4f px   p95 %8.4f   max %8.4f   <- + accumulated pointing error\n",
             m.centroid_rmse_screen_px, m.centroid_p95_screen_px, m.centroid_max_screen_px);
        line("  bias (mean signed) %+7.4f, %+.4f px\n",
             m.centroid_bias_x_px, m.centroid_bias_y_px);
        // -------------------------------------------------------------------
        // The caveat that makes the screen column readable — P1-6.
        //
        // screen error == image error + |B_true - B_cmd|, and the second term
        // is the platform displacement D(t), which a pan-tilt mount with
        // encoders physically cannot observe. With row 25 platform motion
        // active it grows as integral|D|dt without bound: 44 px at 4 s, 295 px
        // at 30 s, 589 px at 60 s on compliance.toml.
        //
        // Printing the measured RMS of D beside it turns the screen figure
        // from an apparent detector divergence into a consistency check on the
        // simulator, which is what it actually is. Only emitted when the drift
        // is large enough to dominate — on a video run (INV-8, no
        // disturbances) and on a jitter-free scenario the three columns agree
        // and the note would be noise.
        // -------------------------------------------------------------------
        if (m.pointing_drift_rms_px > 1.0) {
            line("  NOTE: screen-frame error = boresight-frame error + accumulated pointing\n"
                 "        error. Row 25 platform motion is active, so it grows as int|D|dt and\n"
                 "        is NOT a detector metric. Measured RMS |B_true - B_cmd| = %.2f px\n"
                 "        (max %.2f), which is what the screen column above is reporting.\n"
                 "        Grade the detector on the IMAGE or BORE rows.\n",
                 m.pointing_drift_rms_px, m.pointing_drift_max_px);
        }
    } else if (m.frames_with_truth == 0) {
        out += "  n/a — no ground truth was supplied. The detector ran and its\n"
               "        output is in centroid.csv; scoring it needs --truth.\n";
    } else {
        out += "  no frame had both a detection and the beacon in view\n";
    }

    line("\nTRACKING     (row 17, <= 10 px)      frames scored %lld (Confirmed only)\n",
         static_cast<long long>(m.tracking_frames));
    if (m.tracking_frames > 0) {
        // -------------------------------------------------------------------
        // Three rows, because one number was hiding two regimes — P1-9.
        //
        // A jitter-free 20 s compliance run used to print "RMS 3.357 px,
        // p95 0.973" and an RMS above the p95 reads as a bug. It is not: a
        // handful of 45 px samples from the post-lock slew dominate the sum of
        // squares. Splitting at a settle window derived from the mount's own
        // authority (see kSettleFrames) separates the slew from the loop, and
        // row 17 is graded on the steady-state row — stated here inline so the
        // choice is visible rather than buried.
        // -------------------------------------------------------------------
        line("  steady state      %8.3f px   p95 %8.3f   max %8.3f   mean %7.3f   <- ROW 17 GRADED HERE\n",
             m.tracking_rms_steady_px, m.tracking_p95_steady_px,
             m.tracking_max_steady_px, m.tracking_mean_steady_px);
        if (m.tracking_frames_transient > 0) {
            line("  acq transient     %8.3f px                 max %8.3f              "
                 "  <- first %lld frames after lock\n",
                 m.tracking_rms_transient_px, m.tracking_max_transient_px,
                 static_cast<long long>(m.settle_frames));
        }
        line("  whole run         %8.3f px   p95 %8.3f   max %8.3f   mean %7.3f\n",
             m.tracking_rms_px, m.tracking_p95_px, m.tracking_max_px, m.tracking_mean_px);
        line("  RMS               %8.1f urad  (whole run)\n", m.tracking_rms_urad);
        line("  split             %lld steady / %lld transient frames\n",
             static_cast<long long>(m.tracking_frames_steady),
             static_cast<long long>(m.tracking_frames_transient));
    } else if (!m.pointing_supported) {
        out += "  n/a — this source cannot be pointed (video_direct), so there is\n"
               "        no pointing error to measure. Centroiding above is the\n"
               "        graded metric in this mode.\n";
    } else if (m.frames_with_truth == 0) {
        out += "  n/a — no ground truth was supplied, so pointing error cannot be\n"
               "        scored. In video mode, pass --truth to enable it (CP 8.7).\n";
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

    // -----------------------------------------------------------------------
    // FOV CONTAINMENT — printed ABOVE lock, deliberately.
    //
    // This is the quantity PS 26169's objective sentence names: "locate and
    // MAINTAIN the remote terminal within its camera Field-of-View". Every
    // other number in this block is conditional on it, so it goes first.
    // -----------------------------------------------------------------------
    out += "\nFOV CONTAINMENT  (the PS coarse-alignment objective)\n";
    if (m.frames_with_truth > 0) {
        line("  time on target    %8.2f %%   (%lld / %lld frames with the beacon inside the FOV)\n",
             100.0 * m.fov_containment_frac,
             static_cast<long long>(m.frames_in_fov),
             static_cast<long long>(m.frames_total));
        if (m.post_acq_valid) {
            line("  post-acquisition  %8.2f %%   (%lld / %lld frames after the first lock)\n",
                 100.0 * m.fov_containment_post_acq,
                 static_cast<long long>(m.frames_in_fov_post_acq),
                 static_cast<long long>(m.frames_post_acq));
        } else {
            out += "  post-acquisition       n/a   (the track was never Confirmed)\n";
        }
    } else {
        out += "  n/a — no ground truth was supplied, so containment is unknowable.\n";
    }

    out += "\nLOCK\n";
    if (m.frames_in_fov > 0) {
        line("  retention         %8.2f %%   (%lld held / %lld in-FOV frames)\n",
             100.0 * m.lock_retention_rate,
             static_cast<long long>(m.frames_held_in_fov),
             static_cast<long long>(m.frames_in_fov));
        // -------------------------------------------------------------------
        // Row 18, reported TWICE — P0-2.
        //
        // The two definitions answer different questions and only one of them
        // can be gamed by never acquiring:
        //
        //   in-FOV        1 - held/in-FOV. Detector-centric: "when it was
        //                 there to be seen, did we hold it?" Silent about
        //                 whether it was ever there.
        //   post-acq      1 - held/frames-after-first-lock. System-centric:
        //                 "once you had it, how much of the rest of the run
        //                 did you keep it?" This is the PS's verb, and it is
        //                 the one row 18 is graded on here.
        //
        // On the old baseline scenario the first reads 7.59 % (PASS) and the
        // second reads 95.9 % (FAIL), on the same run. The second is right.
        // -------------------------------------------------------------------
        line("  target loss       %8.2f %%   (over in-FOV frames — detector-centric)\n",
             100.0 * m.target_loss_frac);
        if (m.post_acq_valid) {
            line("  target loss       %8.2f %%   (over post-acquisition frames — ROW 18 GRADED HERE, < 5 %%)\n",
                 100.0 * m.target_loss_post_acq);
        }
        // Printed whenever they differ, because the gap IS the false-track
        // count and seeing it next to the retention is what makes the two
        // numbers legible together rather than contradictory.
        if (m.frames_confirmed != m.frames_held_in_fov) {
            line("                              %lld further Confirmed frames with "
                 "the beacon out of view\n",
                 static_cast<long long>(m.frames_confirmed - m.frames_held_in_fov));
        }
    } else if (m.frames_with_truth == 0) {
        out += "  retention              n/a   (no ground truth supplied)\n";
    } else {
        out += "  retention              n/a   (the beacon was never in view)\n";
    }
    line("  false tracks      %8.2f /min (%lld frames confirmed with no beacon in view)\n",
         m.false_track_rate_per_min, static_cast<long long>(m.false_tracks));
    line("  gimbal saturation %8.2f %%\n", 100.0 * m.saturation_frac);

    // CP 10.7. Reported as its own block because it is a different KIND of
    // claim from the rows above: those grade how well the loop points, this
    // says whether coarse alignment ever finished.
    line("\nHANDOVER     (CP 10.7, quadrant cell, 1 mrad capture)\n");
    if (m.handover_reached) {
        line("  reached             at %6.3f s   (RMS offset held under "
             "capture/3 for the required window)\n", m.handover_time_s);
    } else if (m.handover_rms_urad_best >= 1e8) {
        line("  not reached          the window never filled — no sustained "
             "run of detections\n");
    } else {
        line("  not reached          best sustained RMS offset %8.1f urad "
             "(needed under %.1f)\n",
             m.handover_rms_urad_best, 1000.0 / 3.0);
    }

    out += "\nSPEED        (row 20, >= 20 FPS)\n";
    line("  frame time        p50 %.3f ms   p95 %.3f ms   p99 %.3f ms\n",
         m.frame_ms_p50, m.frame_ms_p95, m.frame_ms_p99);
    line("  fps               %8.1f (from p50)  %8.1f (from p95)\n", m.fps_mean, m.fps_p5);
    line("  wall time         %8.2f s for %.2f s simulated  (%.2fx real time)\n",
         m.wall_time_s, m.duration_s,
         (m.wall_time_s > 0.0) ? (m.duration_s / m.wall_time_s) : 0.0);

    // -----------------------------------------------------------------------
    // END-TO-END LATENCY — P2-9.
    //
    // The block above is COMPUTE time. This is the number a pointing system
    // cares about: how stale the command is when it reaches the mount. The
    // three terms are printed rather than just the sum, because which one
    // dominates is the actionable part — on the synthetic path the transport
    // delay is several times the compute time, so making the tracker faster
    // would barely move the latency.
    // -----------------------------------------------------------------------
    if (m.latency_total_ms > 0.0) {
        out += "\nLATENCY      (sensor to command; reported, the PS sets no requirement)\n";
        line("  exposure/2        %8.3f ms   the frame's effective instant is mid-exposure\n",
             m.latency_exposure_ms);
        line("  compute           %8.3f ms   Pipeline::step, p50\n", m.latency_compute_ms);
        line("  transport delay   %8.3f ms   the mount's dead time (design 10.3)\n",
             m.latency_transport_ms);
        line("  total             %8.3f ms\n", m.latency_total_ms);
    }

    // --- resources — P2-8 --------------------------------------------------
    // "292 FPS" on an unstated number of cores is a throughput figure divided
    // by an unknown, and "what hardware does this need?" was a question the
    // project could not answer.
    if (m.resources_known) {
        out += "\nRESOURCES\n";
        const double cpu = m.cpu_user_s + m.cpu_system_s;
        line("  cpu time          %8.2f s   (%.2f user + %.2f system)\n",
             cpu, m.cpu_user_s, m.cpu_system_s);
        if (m.wall_time_s > 0.0) {
            line("  cores used        %8.2f     (cpu time / wall time, of %u available)\n",
                 cpu / m.wall_time_s, m.hardware_threads);
        }
        line("  peak memory       %8.1f MB  (high-water RSS, not the value at exit)\n",
             static_cast<double>(m.peak_rss_bytes) / (1024.0 * 1024.0));
    }

    return out;
}

// ---------------------------------------------------------------------------
// format_stage_timings — CP 14.4.
//
// §15's table gives a scalar budget per stage and a total of ~0.85 ms. Printing
// the measurement beside the budget is what turns a profile into a decision:
// "SummedArea 4.2 ms" is a number, "SummedArea 4.2 ms against 0.35 ms, 12x
// over" is a place to start.
// ---------------------------------------------------------------------------
std::string format_stage_timings(const StageTimers& t) {
    // Design §15's scalar column, microseconds. Stages the design does not
    // budget separately get 0, which prints as a blank rather than as a
    // target they are failing.
    struct Budget { Stage stage; double us; };
    static constexpr Budget kBudget[] = {
        {Stage::WorldAdvance,   20.0},   // "World, 10 sub-ticks"
        {Stage::Disturbance,     0.0},
        {Stage::GimbalStep,     10.0},   // part of "Control + plant"
        {Stage::FrameAcquire,  740.0},   // PARENT: background + splat + damage
        {Stage::BackgroundRender, 150.0},
        {Stage::EmitterSplat,    40.0},
        {Stage::DamageChain,    550.0},
        {Stage::Perception,    1390.0},  // PARENT: the sum of the leaves below
        {Stage::Median,        350.0},
        {Stage::TopHat,        300.0},
        {Stage::SummedArea,    350.0},   // "SAT x2"
        {Stage::MatchedFilter, 120.0},
        {Stage::Cfar,          200.0},
        {Stage::Grouping,       50.0},
        {Stage::Centroid,       20.0},
        {Stage::Tracking,       10.0},   // "IMM"
        {Stage::Supervisor,      5.0},
        {Stage::Control,        10.0},
        {Stage::Metrics,         0.0},
        {Stage::Snapshot,       30.0},
        {Stage::FrameTotal,    850.0},   // "Total (synthetic) ~ 0.85 ms"
    };

    std::string out =
        "STAGE TIMINGS  (design section 15 budget; percentiles, never means)\n"
        "stage             count      p50       p95       p99    budget    over\n"
        "-----------------------------------------------------------------------\n";
    char b[256];
    for (const Budget& e : kBudget) {
        const LatencyHistogram& h = t[e.stage];
        if (h.count() == 0) continue;
        const double p50 = h.p50();
        if (e.us > 0.0) {
            std::snprintf(b, sizeof b,
                          "%-16s %7llu %8.1f  %8.1f  %8.1f  %8.1f  %6.1fx\n",
                          stage_name(e.stage),
                          static_cast<unsigned long long>(h.count()),
                          p50, h.p95(), h.p99(), e.us, p50 / e.us);
        } else {
            std::snprintf(b, sizeof b,
                          "%-16s %7llu %8.1f  %8.1f  %8.1f\n",
                          stage_name(e.stage),
                          static_cast<unsigned long long>(h.count()),
                          p50, h.p95(), h.p99());
        }
        out += b;
    }
    out += "(microseconds)\n";
    return out;
}

}  // namespace sat
