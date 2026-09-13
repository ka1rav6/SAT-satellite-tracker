// engine/pipeline.cpp

#include "engine/pipeline.hpp"

#include <cmath>

namespace sat {

void Pipeline::build(const PipelineConfig& cfg, EmitterSoA emitters) {
    cfg_ = cfg;
    source_.build(cfg_.synthetic, std::move(emitters));
    gimbal_.reset(cfg_.pan, cfg_.tilt, cfg_.initial_boresight);
    control_.reset(cfg_.gains);
    cmd_rate_ = Rate2{};
    frame_    = 0;
    last_     = FrameRecord{};

    // Pre-size all three snapshot slots from a prototype, so that publishing a
    // frame never allocates (INV-4). This is the one place it is allowed.
    SimSnapshot proto;
    proto.reserve_preview(cfg_.synthetic.camera.width, cfg_.synthetic.camera.height);
    snapshots_.reset(proto);

    fingerprints_.clear();
    if (cfg_.publish_snapshots) {
        fingerprints_.reserve(
            static_cast<size_t>(cfg_.synthetic.duration_s * cfg_.synthetic.camera_hz) + 2);
    }
}

void Pipeline::build_from_scenario(const Scenario& sc) {
    PipelineConfig cfg;
    cfg.synthetic.camera        = sc.camera_geometry();
    cfg.synthetic.screen        = sc.screen_geometry();
    cfg.synthetic.truth_hz      = sc.truth_hz;
    cfg.synthetic.camera_hz     = sc.camera_hz;
    cfg.synthetic.control_hz    = sc.control_hz;
    cfg.synthetic.duration_s    = sc.duration_s;
    cfg.synthetic.seed          = sc.seed;
    cfg.synthetic.blur_substeps = sc.blur_substeps;
    cfg.synthetic.exposure_s    = sc.exposure_ms * 1e-3;

    // Spec rows 13-15. from_dps converts to microradians (INV-5); the remaining
    // non-idealities come straight from the scenario.
    cfg.pan  = GimbalParams::from_dps(sc.max_pan_dps,  sc.max_accel_dps2);
    cfg.tilt = GimbalParams::from_dps(sc.max_tilt_dps, sc.max_accel_dps2);
    cfg.pan.tau_s  = cfg.tilt.tau_s  = sc.time_constant_s;
    cfg.pan.latency_s = cfg.tilt.latency_s = sc.latency_s;
    cfg.pan.encoder_lsb_urad = cfg.tilt.encoder_lsb_urad = sc.encoder_lsb_urad;
    cfg.pan.resonance_hz = cfg.tilt.resonance_hz = sc.resonance_hz;

    cfg.initial_boresight = sc.initial_boresight();   // spec row 6

    cfg_ = cfg;
    source_.build_from_scenario(sc);
    gimbal_.reset(cfg_.pan, cfg_.tilt, cfg_.initial_boresight);
    control_.reset(cfg_.gains);
    cmd_rate_ = Rate2{};
    frame_    = 0;
    last_     = FrameRecord{};

    SimSnapshot proto;
    proto.reserve_preview(cfg_.synthetic.camera.width, cfg_.synthetic.camera.height);
    snapshots_.reset(proto);
    fingerprints_.clear();
}

bool Pipeline::step() {
    SAT_ZONE(timers_, Stage::FrameTotal);

    const Clock& clk = source_.clock();
    const double truth_dt = clk.truth_dt();
    const int    subticks = clk.camera_divisor();

    // -----------------------------------------------------------------------
    // B1-B3: sub-tick loop. The world and the plant run at 300 Hz while frames
    // arrive at 30. Design §6.2 makes this explicit, and it matters: a gimbal
    // integrated once per frame at 33 ms would misrepresent its own dynamics,
    // and the acceleration limit in particular would be unobservable.
    // -----------------------------------------------------------------------
    for (int s = 0; s < subticks; ++s) {
        {
            SAT_ZONE(timers_, Stage::WorldAdvance);
            source_.advance_world(truth_dt);
        }
        {
            SAT_ZONE(timers_, Stage::GimbalStep);
            // ===============================================================
            // INV-2 LIVES HERE. This is the line that closes the loop: the
            // rate the controller produced at the end of the LAST frame is what
            // moves the mount before THIS frame is rendered.
            //
            // With control_enabled false the mount is commanded to hold still,
            // which is what CP 1.8 (b) checks — the camera must then fail to
            // follow, proving the following was caused by our controller and
            // not by the simulator conveniently centring the target.
            // ===============================================================
            const Rate2 applied = cfg_.control_enabled ? cmd_rate_ : Rate2{};
            gimbal_.step(applied, truth_dt);
        }
    }

    // -----------------------------------------------------------------------
    // B4: acquire. The source renders at the true boresight; we hand it the
    // commanded one and it adds the disturbance itself.
    // -----------------------------------------------------------------------
    const Angle2 commanded = gimbal_.true_position();
    SourceFrame frame;
    {
        SAT_ZONE(timers_, Stage::FrameAcquire);
        // The mount's real slew, for the exposure smear. Gimbal rate plus the
        // platform's analytic rate — both smooth, both physical. Jitter is
        // excluded on purpose; see SyntheticSource::render_frame.
        const Rate2 gr = gimbal_.rate();
        const Rate2 pr = source_.disturbance().platform_rate(frame_ / 30.0);
        source_.set_blur_rate(Rate2{gr.x + pr.x, gr.y + pr.y});
        if (!source_.next(commanded, frame)) return false;
    }

    FrameRecord rec{};
    rec.frame          = frame_;
    rec.time_s         = frame.timestamp_s;
    rec.boresight_cmd  = commanded;
    rec.boresight_true = frame.truth.boresight_true;
    rec.cmd_rate       = cmd_rate_;

    // -----------------------------------------------------------------------
    // B5-B15: perception.
    //
    // Note what is passed: frame.pixels and nothing else. The detector cannot
    // reach frame.truth even though it is sitting right there in the same
    // struct, because it is never handed the enclosing SourceFrame. That is
    // INV-1 enforced at the call site, on top of the link-level enforcement in
    // cmake/modules.cmake.
    // -----------------------------------------------------------------------
    SimpleDetection det;
    {
        SAT_ZONE(timers_, Stage::Centroid);
        det = detect_brightest_subpixel(frame.pixels, frame.width, frame.height,
                                        cfg_.detector_window, cfg_.detector_floor);
    }

    // -----------------------------------------------------------------------
    // B16: image pixels -> world angle, via the COMMANDED boresight.
    //
    // Commanded, not true: the tracker does not know the true one. Using truth
    // here would silently cancel the pointing error and make the whole system
    // look perfect for the wrong reason — the single easiest way to accidentally
    // cheat in this project.
    // -----------------------------------------------------------------------
    Angle2 aim = commanded;
    if (det.found) {
        rec.detected        = true;
        rec.detection_img   = det.centre;
        rec.detection_peak  = det.peak;

        const Angle2 offset = cfg_.synthetic.camera.unproject(det.centre);
        const Angle2 world  = commanded + offset;
        rec.detection_screen = cfg_.synthetic.screen.to_pixel(world);
        aim = world;
    }

    // -----------------------------------------------------------------------
    // B22-B27: control.
    // -----------------------------------------------------------------------
    {
        SAT_ZONE(timers_, Stage::Control);
        const Angle2 measured = gimbal_.position();   // the QUANTISED encoder
        const bool az_sat = std::fabs(gimbal_.az().rate())
                          >= gimbal_.az().params().max_rate_urad_s * 0.999;
        const bool el_sat = std::fabs(gimbal_.el().rate())
                          >= gimbal_.el().params().max_rate_urad_s * 0.999;

        if (det.found) {
            cmd_rate_ = control_.compute(aim, measured,
                                         Rate2{}, Rate2{},   // FF arrives at CP 10.1
                                         source_.clock().control_dt(),
                                         az_sat, el_sat);
        } else {
            // INV-9's spirit: with no detection there is nothing to aim at, and
            // inventing a command would be worse than holding. Real coasting on
            // a filter prediction arrives at CP 6.4.
            cmd_rate_ = Rate2{};
        }
    }

    // -----------------------------------------------------------------------
    // B28: metrics. THE ONLY PLACE TRUTH IS READ.
    //
    // INV-6: centroiding error and tracking error are different quantities and
    // are computed, stored and reported separately. Conflating them would make
    // both meaningless — one measures the detector, the other the control loop.
    // -----------------------------------------------------------------------
    {
        SAT_ZONE(timers_, Stage::Metrics);
        if (const FrameTruth::Target* t = frame.truth.primary()) {
            rec.truth_valid  = true;
            rec.truth_screen = t->screen_pos;
            rec.truth_in_fov = t->in_fov;

            // -------------------------------------------------------------
            // Centroiding error: how well we located the beacon IN THE IMAGE.
            //
            // Design §13.1 says "computed ONLY on frames with a detection", and
            // there is a second condition it leaves implicit: the beacon has to
            // be in the field of view. Scoring a reported centroid against a
            // beacon that is off-screen is not a measurement of centroiding
            // accuracy — the detector cannot locate something that is not
            // there, and whatever it reported is a FALSE ALARM, which is a
            // different metric (§13.1's false_track_rate).
            //
            // Without this condition the number is dominated by frames where
            // the camera was pointed somewhere else entirely: an early ablation
            // measured 38 px of "centroiding error" that was really the
            // distance to an off-screen target.
            // -------------------------------------------------------------
            if (rec.detected && t->in_fov) {
                // IMAGE frame: truth.image_pos is where the beacon actually
                // landed on the sensor, computed at the TRUE boresight. The
                // difference is the detector's error and nothing else.
                rec.centroid_error_px = (rec.detection_img - t->image_pos).norm();
                // SCREEN frame: additionally carries the pointing error, since
                // detection_screen was converted through the commanded
                // boresight.
                rec.centroid_error_screen_px =
                    (rec.detection_screen - t->screen_pos).norm();
                rec.centroid_error_valid = true;
            } else if (rec.detected) {
                rec.false_alarm = true;
            }
            // Tracking error: how well the mount is pointed at the beacon.
            // Independent of whether we detected it this frame.
            const Pixel2 bore_screen = cfg_.synthetic.screen.to_pixel(rec.boresight_true);
            rec.tracking_error_px = (bore_screen - t->screen_pos).norm();
        }
    }

    // -----------------------------------------------------------------------
    // B30: publish the snapshot.
    //
    // This is the seam the GUI attaches to at CP 15.0. Building and exercising
    // it now — rather than when the dashboard is written — is the entire
    // justification for the §14.0 amendment that deferred the GUI, and it is
    // also where the reproducibility fingerprint comes from, since the snapshot
    // is by construction everything about a frame that is externally
    // observable.
    // -----------------------------------------------------------------------
    if (cfg_.publish_snapshots) {
        SAT_ZONE(timers_, Stage::Snapshot);
        SimSnapshot& snap = snapshots_.write_slot();
        snap.frame             = rec.frame;
        snap.time_s            = rec.time_s;
        snap.boresight_cmd     = rec.boresight_cmd;
        snap.boresight_true    = rec.boresight_true;
        snap.cmd_rate          = cmd_rate_;
        snap.gimbal_rate       = gimbal_.rate();
        snap.detected          = rec.detected;
        snap.detection_img     = rec.detection_img;
        snap.detection_screen  = rec.detection_screen;
        snap.detection_snr     = rec.detection_peak;
        snap.candidate_count   = rec.detected ? 1 : 0;
        snap.mode              = rec.detected ? TrackMode::Track : TrackMode::Search;
        snap.truth_valid       = rec.truth_valid;
        snap.truth_screen      = rec.truth_screen;
        snap.truth_in_fov      = rec.truth_in_fov;
        snap.centroid_error_px = rec.centroid_error_px;
        snap.tracking_error_px = rec.tracking_error_px;
        snap.set_preview(frame.pixels);

        fingerprints_.push_back(fingerprint(snap));
        snapshots_.publish();
    }

    last_ = rec;
    ++frame_;
    return true;
}

void Pipeline::run(std::vector<FrameRecord>& out) {
    out.clear();
    while (step()) out.push_back(last_);
}

}  // namespace sat
