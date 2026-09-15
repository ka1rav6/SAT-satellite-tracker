// engine/pipeline.cpp

#include "engine/pipeline.hpp"

#include "engine/truth_csv.hpp"
#include "scenario/schema.hpp"

#include <cstdio>

#include <algorithm>
#include <cmath>

namespace sat {

// ---------------------------------------------------------------------------
// build_stage6 — allocate and reset perception, tracking, the mode FSM and the
// search pattern.
//
// Everything here happens ONCE, at startup (design §6.1). That is the whole
// point: the arena is reserved, every per-frame buffer is carved out of it, and
// the candidate and measurement vectors are reserved to their maximum size, so
// that a frame never allocates (INV-4). If this function is not called, or is
// called with a mismatched frame size, perception_ready_ stays false and the
// pipeline falls back to the straw-man detector rather than reading past the
// end of a buffer.
// ---------------------------------------------------------------------------
void Pipeline::build_stage6() {
    const CameraGeometry& cam = cfg_.synthetic.camera;

    arena_.reserve(cfg_.perception_arena_bytes);
    perception_ready_ = ws_.allocate(
        arena_, cam.width, cam.height,
        structuring_element_size(cfg_.perception.target_size_px));
    perception_.configure(cfg_.perception);

    dets_.clear();
    meas_.clear();
    dets_.reserve(static_cast<size_t>(std::max(1, cfg_.perception.max_candidates)));
    meas_.reserve(static_cast<size_t>(std::max(1, cfg_.perception.max_candidates)));

    // -----------------------------------------------------------------------
    // The filter's process noise.
    //
    // A CORRECTION. The first version of this derived q from the MOUNT's
    // acceleration limit, reasoning that the angular measurement's
    // acceleration is bounded by the target's plus the camera's. That is
    // wrong, and wrong by nearly two orders of magnitude: measurement.hpp adds
    // the boresight back, so the measurement is in the world angular frame and
    // the camera's motion is already removed. Only the TARGET's acceleration
    // enters.
    //
    // The symptom was unmistakable once the whole loop ran. A mount limit of
    // ~3.5e5 urad/s^2 gives q = a^2*dt = 4e9 urad^2/s^3, which inflates the
    // position uncertainty to about 80 px after eight coasted frames. The
    // chi-square gate scales with that, so it grew to ~240 px across — and the
    // CFAR stage legitimately reports several noise candidates per frame, at
    // arbitrary positions. The track was then kept alive indefinitely by noise
    // and dragged 370 px off the target while still reporting a lock. Every
    // component was behaving exactly as specified; the number connecting them
    // was nonsense.
    //
    // The right bound is computable, which is the point of §7.2's analytic
    // motion algebra: build_from_scenario reads the target's motion stack and
    // calls max_accel_px_s2. See pipeline.hpp for the floor and the default.
    // -----------------------------------------------------------------------
    const double dt = 1.0 / std::max(1.0, static_cast<double>(cfg_.synthetic.camera_hz));
    const double a_max = std::max(cfg_.max_target_accel_urad_s2,
                                  cfg_.min_target_accel_urad_s2);
    if (cfg_.tracking.kf.accel_psd_urad2_s3 <= 0.0) {
        cfg_.tracking.kf = KalmanParams::from_max_accel(a_max, dt);
    }
    // -----------------------------------------------------------------------
    // The velocity prior, and the gate's physical cap.
    //
    // ANOTHER CORRECTION IN THE SAME FAMILY AS q's. KalmanParams' default
    // velocity prior is the mount's full slew rate (175,000 urad/s = 1,600
    // px/s), on the reasoning that with no velocity information the honest
    // prior is "anything up to the fastest thing that can happen". That is the
    // fastest thing the CAMERA can do, and the filter's state is in the world
    // frame where the camera's motion has already been removed.
    //
    // The cost was not subtle. A 1,600 px/s prior makes the predicted position
    // uncertain by ~53 px after a single frame, so for the first several frames
    // of a track the chi-square gate is over 150 px wide — and a CFAR noise
    // blob 81 px from the beacon walked straight into it and captured the
    // track. Sizing the prior from the target's own computable maximum speed
    // closes that window.
    //
    // The reachability cap is belt and braces for the same failure, and it is
    // the one that keeps holding later in a run: see TrackParams.
    // -----------------------------------------------------------------------
    cfg_.tracking.kf.initial_rate_sigma_urad_s = cfg_.max_target_speed_urad_s;
    cfg_.tracking.max_target_speed_urad_s      = cfg_.max_target_speed_urad_s;
    // The cap's padding is the measurement's own uncertainty, so a correct
    // detection is never excluded by it: the pointing budget plus a couple of
    // pixels of centroiding slack.
    cfg_.tracking.gate_pad_urad =
        (cfg_.pointing_sigma_px * 3.0 + 2.0) * cam.ifov_urad();

    tracker_.reset(cfg_.tracking);

    cfg_.mode.ifov_urad = cam.ifov_urad();
    fsm_.reset(cfg_.mode);

    if (cfg_.search.step.x <= 0.0 || cfg_.search.step.y <= 0.0) {
        const SearchStrategy keep = cfg_.search.strategy;
        cfg_.search = SearchParams::from_camera(cam, cfg_.synthetic.screen,
                                                cfg_.search.overlap);
        cfg_.search.strategy = keep;
    }
    search_.reset(cfg_.search, cfg_.initial_boresight);
}

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

    build_stage6();
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

    // Design §10.4's gains, from the scenario rather than from a header
    // default. Before CP 10.1 this line did not exist and every run used
    // ControlGains::proportional(6.0) — a pure P loop with ki, kd and k_ff all
    // at zero, which meant the feedforward path built at Stage 6 was plumbed
    // but never energised. Reading them from the scenario is what makes the
    // checkpoint's on/off comparison a `--set control.k_ff=0` away.
    cfg.gains.kp      = sc.control.kp;
    cfg.gains.ki      = sc.control.ki;
    cfg.gains.kd      = sc.control.kd;
    cfg.gains.k_ff    = sc.control.k_ff;
    cfg.gains.i_limit = sc.control.i_limit;
    cfg.gains.anti_windup = sc.control.anti_windup;

    // §7.2's closed forms turned into the filter's q. The largest acceleration
    // over every target, because the tracker does not know which one it will
    // end up on and must be able to follow any of them.
    double a_px = 0.0, v_px = 0.0;
    for (const TargetSpec& t : sc.targets) {
        a_px = std::max(a_px, max_accel_px_s2(t, sc.duration_s));
        v_px = std::max(v_px, max_speed_px_s(t, sc.duration_s));
    }
    cfg.max_target_accel_urad_s2 = a_px * cfg.synthetic.camera.ifov_urad();
    cfg.min_target_accel_urad_s2 = 50.0 * cfg.synthetic.camera.ifov_urad();
    // A floor here too, and for the same reason as the acceleration's: a
    // stationary target has an analytic speed of exactly zero, and a filter
    // that believes the target cannot move at all will reject the first frame
    // where anything is slightly not as modelled. 50 px/s is well under the
    // slowest interesting motion and well over the residual wander.
    cfg.max_target_speed_urad_s =
        std::max(v_px, 50.0) * cfg.synthetic.camera.ifov_urad();

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

    build_stage6();
}

// ---------------------------------------------------------------------------
// build_from_video — Stage 8. The source changes; nothing else does.
// ---------------------------------------------------------------------------
Status Pipeline::build_from_video(const Scenario& sc,
                                  const std::filesystem::path& clip,
                                  const VideoMode* mode_override,
                                  const std::filesystem::path* truth_csv) {
    // The synthetic world is still built, and deliberately so: it owns the
    // clock, and the clock owns the sub-tick structure that the gimbal and the
    // controller run on. What it does NOT do in video mode is render — step()
    // takes its frames from the video source instead — so its emitters are
    // irrelevant and its degradation chain is never called, which is INV-8
    // enforced by the code path rather than by a flag.
    build_from_scenario(sc);

    auto v = VideoSource::open(clip, sc, mode_override);
    if (!v) return Err(v.error());
    video_ = std::move(*v);

    if (!video_->inv8_warning().empty()) {
        std::fprintf(stderr, "%s", video_->inv8_warning().c_str());
    }

    // §8.3 requirement 3. A rate that does not divide truth_hz cleanly would
    // advance the world by a different amount between consecutive frames — a
    // slow drift in every metric, invisible in any single frame. Reported, not
    // worked around, because the fix is to raise truth_hz and that is the
    // caller's decision.
    if (video_->camera_divisor() <= 0) {
        std::fprintf(stderr,
            "sat-tracker: warning: the clip runs at %.4f fps, which does not divide\n"
            "  the scenario's truth_hz of %d cleanly. The world will advance by a\n"
            "  rounded number of sub-ticks per frame. Set sim.truth_hz to a multiple\n"
            "  of the clip's rate to remove the drift.\n",
            video_->fps(), sc.truth_hz);
    }

    if (truth_csv) {
        auto t = load_truth_csv(*truth_csv, sc.screen_geometry());
        if (!t) return Err(t.error());
        video_->set_truth(std::move(*t));
    }

    // The geometry the metrics report in is the FILE's, not the scenario's
    // nominal 2000x2000 — in screen mode because the video IS the canvas, and
    // in direct mode because there is no canvas at all and screen == image.
    const FrameGeometry g = video_->geometry();
    cfg_.synthetic.screen = ScreenGeometry::make(g.screen_w, g.screen_h,
                                                 cfg_.synthetic.camera);

    // ------------------------------------------------------------------
    // In DIRECT mode the frames are the file's size, which need not be the
    // camera's. build_stage6 sized the perception workspace from
    // cfg_.synthetic.camera — 640x480 — so a 641x481 clip would have written
    // past the end of every buffer in the pipeline.
    //
    // CP 8.8 ships odd_641x481.mp4 for exactly this, and the generator's own
    // comment predicted it: "a bicubic crop that assumes even dimensions will
    // read past the last row". The prediction was right about the risk and
    // wrong about where — the crop handles any size, and it was the workspace
    // behind it that was fixed at the camera's.
    //
    // So in direct mode the camera geometry IS the file's, and everything
    // downstream is rebuilt around it. The field of view is kept, because that
    // is a property of the optics the clip was shot with and nothing in the
    // file tells us otherwise.
    // ------------------------------------------------------------------
    if (!video_->supports_pointing()
        && (g.width != cfg_.synthetic.camera.width
         || g.height != cfg_.synthetic.camera.height)) {
        std::fprintf(stderr,
            "sat-tracker: the clip is %dx%d but the scenario's camera is %dx%d;\n"
            "  in direct mode the frame IS the camera, so the camera geometry is\n"
            "  taken from the file (field of view kept at %.2f x %.2f degrees).\n",
            g.width, g.height, cfg_.synthetic.camera.width, cfg_.synthetic.camera.height,
            cfg_.synthetic.camera.fov_x_deg, cfg_.synthetic.camera.fov_y_deg);
        cfg_.synthetic.camera = CameraGeometry::make(g.width, g.height,
                                                     cfg_.synthetic.camera.fov_x_deg,
                                                     cfg_.synthetic.camera.fov_y_deg);
        cfg_.synthetic.screen = ScreenGeometry::make(g.width, g.height,
                                                     cfg_.synthetic.camera);
        build_stage6();          // re-allocate the workspace at the real size
    }
    if (!video_->supports_pointing()) {
        // video_direct: the frame does not follow the controller. INV-2's
        // closed-loop requirement carves out this mode explicitly — the
        // controller still runs and still reports where it WOULD aim, and the
        // metrics still score the centroid, which is the whole of BP-2 under
        // this reading of the requirement.
        cfg_.control_enabled = false;
    }
    return Ok();
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
            // Not in video mode: there is no world to advance, the clip is the
            // world. Calling it anyway would burn emitters' RNG streams and
            // make a video run's fingerprint depend on a simulation nobody
            // asked for.
            if (!video_) source_.advance_world(truth_dt);
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
    // In video mode the COMMANDED angle is what the crop uses, and there is no
    // true-vs-commanded distinction: INV-8 disables the disturbances, so the
    // camera points exactly where it was told. That is precisely why video mode
    // is the honest place to grade centroiding — the screen-frame number does
    // not carry a pointing error the detector cannot influence (INV-6).
    const Angle2 commanded = video_ ? gimbal_.position() : gimbal_.true_position();
    SourceFrame frame;
    {
        SAT_ZONE(timers_, Stage::FrameAcquire);
        if (video_) {
            if (!video_->next(commanded, frame)) return false;   // EOF: clean
        } else {
            // The mount's real slew, for the exposure smear. Gimbal rate plus
            // the platform's analytic rate — both smooth, both physical.
            // Jitter is excluded on purpose; see SyntheticSource::render_frame.
            const Rate2 gr = gimbal_.rate();
            const Rate2 pr = source_.disturbance().platform_rate(frame_ / 30.0);
            source_.set_blur_rate(Rate2{gr.x + pr.x, gr.y + pr.y});
            if (!source_.next(commanded, frame)) return false;
        }
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
    {
        SAT_ZONE(timers_, Stage::Centroid);
        if (cfg_.detector == PipelineConfig::Detector::Classical && perception_ready_) {
            perception_.process(frame.pixels, frame.width, frame.height, ws_, dets_,
                                &timers_);
        } else {
            // The CP 4.11 ablation arm. §9.4 is explicit that this "must never
            // be the default", and it is not — but it has to be runnable
            // through the whole engine, because the result being demonstrated
            // is that it locks onto the wrong thing in clutter, and that is a
            // statement about the closed loop, not about one frame.
            dets_.clear();
            const SimpleDetection sd = detect_brightest_subpixel(
                frame.pixels, frame.width, frame.height,
                cfg_.detector_window, cfg_.detector_floor);
            if (sd.found) {
                Detection d;
                d.centroid_image     = sd.centre;
                d.peak               = sd.peak;
                d.integrated         = sd.integrated;
                // The straw man has no background estimate, so it cannot report
                // an SNR. Reporting a fabricated one would flow straight into
                // the filter's adaptive R (CP 6.5) and make the ablation
                // compare two different filters rather than two detectors, so
                // it declares a fixed, mediocre confidence instead.
                d.snr                = 6.0f;
                d.size_est_px        = static_cast<uint16_t>(cfg_.perception.target_size_px);
                d.centroid_sigma_est = centroid_sigma(d.snr, d.size_est_px);
                dets_.push_back(d);
            }
        }
    }
    rec.candidate_count = static_cast<int>(dets_.size());

    // -----------------------------------------------------------------------
    // B16: image pixels -> world angle, via the COMMANDED boresight.
    //
    // Commanded, not true: the tracker does not know the true one. Using truth
    // here would silently cancel the pointing error and make the whole system
    // look perfect for the wrong reason — the single easiest way to accidentally
    // cheat in this project.
    // -----------------------------------------------------------------------
    //
    // The pointing-uncertainty floor on R. Two independent contributions, added
    // in quadrature:
    //
    //   the encoder quantisation, which the mount's own datasheet states, and
    //   the pointing stability budget (spec row 23), which is the disturbance
    //   the system is required to tolerate.
    //
    // See PipelineConfig::pointing_sigma_px for why the second one is taken
    // from the SPECIFICATION and never from the scenario, and for the
    // measurement that showed it has to be there at all.
    const double encoder = std::max(cfg_.pan.encoder_lsb_urad,
                                    cfg_.tilt.encoder_lsb_urad);
    // INV-8: no disturbance is applied in video modes, so there is no pointing
    // error to allow for and the budget collapses to the encoder alone. Leaving
    // the spec-row-23 allowance in would widen the gate by ~35 px on exactly
    // the benchmark where the measurement is cleanest, letting clutter in for
    // no reason.
    const double budget  = video_ ? 0.0
        : cfg_.pointing_sigma_px * cfg_.synthetic.camera.ifov_urad();
    const double pointing_sigma = std::sqrt(encoder * encoder + budget * budget);
    meas_.clear();
    for (size_t i = 0; i < dets_.size(); ++i) {
        meas_.push_back(to_measurement(dets_[i], cfg_.synthetic.camera, commanded,
                                       pointing_sigma, static_cast<int>(i)));
    }

    // Report the strongest candidate as "the detection" for the metrics and the
    // log. Which candidate the TRACKER chose is recorded separately below; the
    // two differ exactly when the gate rejected the brightest thing in frame,
    // which is the behaviour CP 6.3 exists to produce.
    if (!dets_.empty()) {
        rec.detected          = true;
        rec.detection_img     = dets_[0].centroid_image;
        rec.detection_peak    = dets_[0].peak;
        rec.detection_snr     = dets_[0].snr;
        rec.centroid_sigma_px = dets_[0].centroid_sigma_est;
        rec.detection_area_px = dets_[0].area_px;
        rec.detection_size_est_px = dets_[0].size_est_px;
        rec.detection_screen  = cfg_.synthetic.screen.to_pixel(
            commanded + cfg_.synthetic.camera.unproject(dets_[0].centroid_image));
    }

    // -----------------------------------------------------------------------
    // B17-B21: gate, associate, filter, lifecycle.
    // -----------------------------------------------------------------------
    const double frame_dt = 1.0 / std::max(1.0, static_cast<double>(cfg_.synthetic.camera_hz));
    int associated = -1;
    {
        SAT_ZONE(timers_, Stage::Tracking);
        associated = tracker_.step(frame_dt, std::span<Measurement>(meas_), frame_);
    }
    if (associated >= 0) {
        // The tracker's choice, not the brightest one. This is what the log and
        // the GUI should show as "the target".
        const Detection& d = dets_[static_cast<size_t>(meas_[static_cast<size_t>(associated)].source_index)];
        rec.detection_img     = d.centroid_image;
        rec.detection_peak    = d.peak;
        rec.detection_snr     = d.snr;
        rec.centroid_sigma_px = d.centroid_sigma_est;
        rec.detection_area_px = d.area_px;
        rec.detection_size_est_px = d.size_est_px;
        rec.detection_screen  = cfg_.synthetic.screen.to_pixel(
            commanded + cfg_.synthetic.camera.unproject(d.centroid_image));
    }

    const Track& trk = tracker_.track();
    rec.track_state         = trk.state();
    rec.has_lock            = trk.drivable();
    rec.estimate            = trk.position();
    rec.estimate_rate       = trk.rate();
    rec.estimate_sigma_urad = trk.filter().position_sigma_urad();

    // -----------------------------------------------------------------------
    // B24: mode FSM.
    // -----------------------------------------------------------------------
    ModeFsmInputs fsm_in;
    fsm_in.running         = true;
    fsm_in.have_track      = tracker_.has_track();
    fsm_in.track_state     = trk.state();
    fsm_in.candidate_count = rec.candidate_count;
    fsm_in.rms_error_px    = 1e9;      // CP 10.7 feeds the real figure
    rec.mode = fsm_.step(fsm_in, frame_, frame.timestamp_s);

    // -----------------------------------------------------------------------
    // B25: the aim point. THE ONE PLACE THE MODE ACTUALLY DOES SOMETHING.
    //
    // In Track/Reacquire the aim is the filter's estimate AT THIS FRAME'S
    // TIMESTAMP. It is not predicted a frame ahead, and CP 10.1 is where that
    // changed — the previous version aimed at predict_position(frame_dt), and
    // the reasoning behind it was right for a loop with no feedforward and
    // wrong for one that has it. The correction is written out here because
    // "we removed a lead term" reads like a regression otherwise.
    //
    //   A one-frame-ahead aim is A FEEDFORWARD, implemented in the setpoint.
    //   Pushing the setpoint ahead by v*dt makes the proportional term produce
    //   an extra kp*v*dt of rate, which is a velocity-proportional lead by
    //   another name. When k_ff was zero that was the only lead in the loop and
    //   it was worth having. With k_ff = 1 the velocity is fed forward
    //   explicitly, and keeping both feeds it TWICE.
    //
    // The algebra, at constant target velocity v, ignoring the slow integrator:
    //
    //   steady state needs   kp*e + k_ff*v = v,  so  e = v*(1 - k_ff)/kp
    //   with the lead aim    error metric = v*dt - e
    //   at k_ff = 1          e = 0, and the mount LEADS the target by v*dt
    //
    // At 200 px/s and 30 Hz that is 6.7 px of lead — pointing ahead of the
    // beacon instead of behind it, which is not an improvement, just a sign
    // change. It was visible as a k_ff sweep whose minimum sat at 0.75 rather
    // than 1.0, and 1 - kp*dt = 1 - 8*0.0333 = 0.733 predicts exactly that.
    // Tuning k_ff down to 0.75 would have "fixed" the number while leaving a
    // gain that silently depends on kp and the frame rate.
    //
    // Aiming at the current estimate makes k_ff = 1 the correct value for the
    // reason it is supposed to be correct, and makes the k_ff = 0 ablation
    // measure the pure feedback lag v/kp rather than v/kp minus a hidden lead.
    //
    // In Search the pattern owns the aim. Entering Search restarts it from the
    // last known position rather than from the screen centre: the prediction is
    // the best prior available and CP 6.7 is the measurement of what that is
    // worth.
    // -----------------------------------------------------------------------
    Angle2 aim;
    if (fsm_.tracking_active() && trk.drivable()) {
        aim = trk.position();
        if (fsm_.changed_this_frame()) search_.recentre(aim);
    } else {
        if (fsm_.changed_this_frame() && rec.mode == TrackMode::Search) {
            // Coming out of a lost track: search outward from where it was
            // last believed to be, which is where it is most likely to
            // reappear. On a cold start this is the initial boresight, which is
            // the screen centre — the same thing, correctly.
            search_.recentre(tracker_.has_track() ? trk.position() : search_.centre());
            search_.restart();
        }
        aim = search_.step(frame_dt, commanded);
    }
    rec.aim = aim;

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

        // The velocity feedforward input is now real: the Kalman filter
        // estimates the target's angular rate directly (CP 6.2). The GAIN is
        // still whatever the scenario set — ControlGains::proportional leaves
        // k_ff at zero — so this changes nothing until CP 10.1 turns it on,
        // which is the point: the plumbing and the tuning are separate
        // commits, and the on/off comparison CP 10.1 asks for is then a
        // one-line change rather than a rewrite.
        const Rate2 ff = trk.drivable() ? trk.rate() : Rate2{};

        // A note on what replaced the old behaviour. Until Stage 6 this read
        // `if (det.found) ... else cmd_rate_ = Rate2{}` — hold still whenever a
        // single frame missed. That was correct for a system with no memory,
        // and INV-9 still forbids inventing a CENTROID. But a filter prediction
        // is not an invented measurement: it is an estimate with a stated
        // covariance, and refusing to use it is what made a one-frame dropout
        // stop the mount dead. Search is now the only state that holds, and it
        // holds by aiming at a search pattern rather than by zeroing the rate.
        // Integral action is switched off unless the loop is actually holding
        // something. See controller.hpp for the measurement that forced this:
        // integrating against a scanning search pattern wound the mount up hard
        // enough to smear a false candidate into the image.
        const bool integral_ok = fsm_.tracking_active() && trk.drivable();

        // And on the way OUT of tracking the accumulated integral describes a
        // target that no longer exists, so it is discarded rather than carried
        // into the search. clear_state() was written at Stage 1 with the
        // comment "used when the FSM re-enters Search" and, until CP 10.1,
        // nothing called it — the documentation was right and the wiring was
        // missing. Nothing noticed because ki was zero in every run.
        if (fsm_.changed_this_frame() && !integral_ok) control_.clear_state();

        cmd_rate_ = control_.compute(aim, measured, ff, Rate2{},
                                     source_.clock().control_dt(),
                                     az_sat, el_sat, integral_ok);
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
        snap.detection_snr     = rec.detection_snr;
        snap.centroid_sigma_px = rec.centroid_sigma_px;
        snap.candidate_count   = rec.candidate_count;
        snap.mode              = rec.mode;
        snap.track_state       = static_cast<int>(rec.track_state);
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
