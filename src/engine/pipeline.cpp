// engine/pipeline.cpp

#include "engine/pipeline.hpp"

#include <chrono>

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
    tracker_.weights() = cfg_.priority;

    dets_.clear();
    dets_band_.clear();
    meas_.clear();
    // -----------------------------------------------------------------------
    // INV-4 (CP 14.3): sized for the PRE-TRUNCATION count, not for
    // max_candidates.
    //
    // ClassicalPerception::process pushes every blob that survives the shape
    // gate and truncates to max_candidates afterwards, so `dets_` transiently
    // holds more than the cap. Reserving the cap was enough until Stage 12:
    // the supervisor lowers the CFAR threshold in poor conditions, more blobs
    // survive, and the vector reallocated inside the frame loop at 48
    // detections against a reserve of 24. The trap caught it on the first run
    // with the supervisor switching.
    //
    // The bound is the blob reserve — in principle every blob can pass the
    // gate. 4096 Detections is about 300 KB, reserved once.
    //
    // `meas_` keeps the cap: measurements are built only from the detections
    // that survive truncation, so max_candidates really is its bound.
    // -----------------------------------------------------------------------
    dets_.reserve(ClassicalPerception::kMaxDetections);
    // P1-7's sweep runs a SECOND process() pass, and process() transiently
    // holds the pre-truncation blob list in whatever vector it is handed. The
    // band's vector therefore needs the same bound as dets_, for the same
    // reason, and it is a member rather than a local so it only pays for it
    // once instead of on every frame.
    dets_band_.reserve(ClassicalPerception::kMaxDetections);

    // -----------------------------------------------------------------------
    // TWICE max_candidates, not max_candidates.
    //
    // One measurement is built per surviving detection, and with P1-7's sweep
    // enabled `dets_` is the merge of TWO independent process() calls — the
    // tracking window and the row-band — each of which truncates to
    // max_candidates on its own, afterwards and separately. 2 * the cap is
    // therefore the real bound on the merged list, and a reserve has to be
    // made against the bound.
    //
    // HONESTY ABOUT WHAT WAS AND WAS NOT OBSERVED. No fixture in this
    // repository has actually driven the merge past the single cap. Probed
    // across 150-500 clutter sources and 2, 3 and 7 bands, the observed
    // maximum is exactly 24 — the cap itself — and it comes from frames where
    // the track is NOT confirmed, on which the window is the whole frame and
    // refresh_band() skips the sweep entirely. On the banded frames the window
    // is small, yields few, and the merge stays well under the cap.
    //
    // The reserve is against the bound anyway, for the same reason `dets_`
    // above reserves 16,384 against an observed maximum of 24: INV-4 is a
    // statement about what the code CAN do, and CP 14.1's fuzzer exists
    // because legal scenarios reach places no hand-written fixture does. The
    // factor is unconditional so that toggling the sweep at runtime cannot
    // invalidate a reserve made at build time.
    // -----------------------------------------------------------------------
    meas_.reserve(2 * static_cast<size_t>(std::max(1, cfg_.perception.max_candidates)));

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

    // CP 10.5: the IMM's parameters, derived from the same bound as the plain
    // filter's q so the two are answering the same question about the target.
    // Done here rather than in build_from_scenario because the bound is only
    // resolved at this point — and because the IMM must pick up the same
    // corrected velocity prior the paragraphs above exist to justify.
    cfg_.tracking.imm_params = ImmParams::from_max_accel(a_max, dt);
    cfg_.tracking.imm_params.kf = cfg_.tracking.kf;

    tracker_.reset(cfg_.tracking);

    // -----------------------------------------------------------------------
    // Stage 12: the supervisor's BASE strategy is the configuration this run
    // would use with no supervisor at all.
    //
    // Every rule in §10.6's table is expressed as a modification of this rather
    // than as an absolute, so that switching the supervisor on cannot silently
    // discard a deliberate scenario setting — a scenario that asks for the IMM
    // and a middle-band SNR must still get the IMM.
    // -----------------------------------------------------------------------
    base_accel_psd_ = cfg_.tracking.kf.accel_psd_urad2_s3;
    Strategy base;
    base.perception = cfg_.detector;
    base.centroider = cfg_.perception.centroid_kind;
    base.cfar_k     = cfg_.perception.cfar.k;
    base.min_snr_factor = cfg_.perception.min_snr_factor;
    base.filter     = cfg_.tracking.imm ? FilterKind::Imm : FilterKind::Cv;
    base.predictor  = cfg_.gains.smith ? PredictorKind::Smith : PredictorKind::None;
    base.gains      = cfg_.gains;
    base.search     = cfg_.search.strategy;
    base.q_scale    = 1.0;
    supervisor_.reset(cfg_.supervisor, base);

    cfg_.mode.ifov_urad = cam.ifov_urad();
    fsm_.reset(cfg_.mode);

    // CP 10.7: the handover sensor and the window its criterion is written
    // against. The window comes from the FSM's own parameter so the two cannot
    // disagree about how many frames "30 consecutive frames" means.
    quad_.reset(QuadrantParams{cfg_.mode.handover_capture_urad,
                               cfg_.mode.handover_capture_urad * 0.1});
    handover_.reset(cfg_.mode.handover_frames);

    if (cfg_.search.step.x <= 0.0 || cfg_.search.step.y <= 0.0) {
        const SearchStrategy keep = cfg_.search.strategy;
        cfg_.search = SearchParams::from_camera(cam, cfg_.synthetic.screen,
                                                cfg_.search.overlap);
        cfg_.search.strategy = keep;
    }
    search_.reset(cfg_.search, cfg_.initial_boresight, cfg_.synthetic.screen);
}

// ---------------------------------------------------------------------------
// reset_controller — gains AND the plant model, together.
//
// One function because they were two lines in two places and CP 10.4 found out
// the hard way: build() got the plant model and build_from_scenario() did not,
// so every scenario-driven run used the DEFAULT model — no rate ceiling, no
// acceleration limit — and the Smith predictor's numbers were identical with
// the limits added and without them. Identical numbers after a change that
// should have mattered is the tell; the code was never running.
// ---------------------------------------------------------------------------
void Pipeline::reset_controller() {
    control_.reset(cfg_.gains);
    // The controller's BELIEF about the mount, copied from the same
    // configuration the plant was built from. That is the honest starting
    // point — a real system would take these from a datasheet or a calibration
    // and they would be slightly wrong, which is the risk control/smith.hpp is
    // about and tests/control/test_stage10.cpp deliberately exercises.
    control_.set_plant_model(PlantModel{cfg_.pan.latency_s, cfg_.pan.tau_s,
                                        cfg_.pan.max_rate_urad_s,
                                        cfg_.pan.max_accel_urad_s2},
                             source_.clock().control_dt());
}

void Pipeline::build(const PipelineConfig& cfg, EmitterSoA emitters) {
    cfg_ = cfg;
    source_.build(cfg_.synthetic, std::move(emitters));
    gimbal_.reset(cfg_.pan, cfg_.tilt, cfg_.initial_boresight);
    reset_controller();
    cmd_rate_ = Rate2{};
    frame_    = 0;
    last_     = FrameRecord{};

    // Pre-size all three snapshot slots from a prototype, so that publishing a
    // frame never allocates (INV-4). This is the one place it is allowed.
    SimSnapshot proto;
    proto.reserve_preview(cfg_.synthetic.camera.width, cfg_.synthetic.camera.height);
    snapshots_.reset(proto);

    fingerprints_.clear();
    // INV-4 (CP 14.3): one fingerprint per frame, and the run length is known
    // here. Without the reserve the vector doubles its way up inside the frame
    // loop — a handful of allocations over a run, invisible in a profile, and
    // exactly what the Debug trap exists to refuse.
    fingerprints_.reserve(
        static_cast<size_t>(cfg_.synthetic.duration_s
                            * std::max(1, cfg_.synthetic.camera_hz)) + 8);
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
    cfg.gains.smith       = sc.control.smith;
    cfg.gains.smith_rate_blend = sc.control.smith_rate_blend;
    cfg.tracking.imm      = sc.tracking_imm;
    cfg.priority.enabled        = sc.priority_enabled;
    cfg.priority.motion         = static_cast<float>(sc.priority_motion_w);
    cfg.priority.min_commit_score = static_cast<float>(sc.priority_min_commit);
    cfg.priority.promote_min_age  = sc.priority_min_frames;
    cfg.priority.drop_frames      = sc.priority_drop_frames;
    cfg.priority.switch_frames    = sc.priority_switch_frames;
    cfg.priority.switch_ratio     = static_cast<float>(sc.priority_switch_ratio);

    cfg.perception.min_snr_factor = static_cast<float>(sc.min_snr_factor);

    cfg.roi.enabled        = sc.roi_enabled;
    cfg.roi.min_half_px    = sc.roi_min_half_px;
    cfg.roi.sigma_margin   = sc.roi_sigma_margin;
    cfg.roi.refresh_frames = sc.roi_refresh_frames;
    cfg.search.strategy =
          sc.search_strategy == "raster"        ? SearchStrategy::Raster
        : sc.search_strategy == "probabilistic" ? SearchStrategy::Probabilistic
        : sc.search_strategy == "camp_and_wait" ? SearchStrategy::CampAndWait
                                                : SearchStrategy::Spiral;
    cfg.supervisor.enabled          = sc.supervisor_enabled;
    cfg.supervisor.min_dwell_frames = sc.supervisor_dwell;
    cfg.supervisor.ema_tau_frames   = sc.supervisor_ema_tau;
    cfg.ai_enabled      = sc.ai_enabled;
    cfg.motion_net_path = sc.motion_net;

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

    // -----------------------------------------------------------------------
    // Design §7.4's timeline. Built from the SCENARIO, which is why it is here
    // and not in build_stage6: the PipelineConfig path has no scenario to take
    // events from, and a timeline is a property of the run's description
    // rather than of its resolved configuration.
    // -----------------------------------------------------------------------
    motion_net_.reset();
    last_forecast_ = MotionForecast{};
    motion_prior_applies_ = 0;
    motion_coast_uses_ = 0;
    if (sc.ai_enabled && !sc.motion_net.empty()) {
        auto loaded = MotionNet::load(sc.motion_net);
        if (!loaded) {
            std::fprintf(stderr,
                         "sat-tracker: MotionNet '%s': %s — using IMM (INV-7)\n",
                         sc.motion_net.c_str(), loaded.error().c_str());
        } else {
            motion_net_ = std::make_unique<MotionNet>(std::move(*loaded));
        }
    }

    events_.build(sc);
    have_target_index_ = false;
    {
        const EmitterSoA& em = source_.emitters();
        for (size_t i = 0; i < em.n; ++i) {
            if (em.kind_of(i) == EmitterKind::Target) {
                target_index_      = i;
                target_intensity_  = em.intensity[i];
                have_target_index_ = true;
                break;
            }
        }
    }
    gimbal_.reset(cfg_.pan, cfg_.tilt, cfg_.initial_boresight);
    reset_controller();
    cmd_rate_ = Rate2{};
    frame_    = 0;
    last_     = FrameRecord{};

    SimSnapshot proto;
    proto.reserve_preview(cfg_.synthetic.camera.width, cfg_.synthetic.camera.height);
    snapshots_.reset(proto);
    fingerprints_.clear();
    // INV-4 (CP 14.3): one fingerprint per frame, and the run length is known
    // here. Without the reserve the vector doubles its way up inside the frame
    // loop — a handful of allocations over a run, invisible in a profile, and
    // exactly what the Debug trap exists to refuse.
    fingerprints_.reserve(
        static_cast<size_t>(cfg_.synthetic.duration_s
                            * std::max(1, cfg_.synthetic.camera_hz)) + 8);

    build_stage6();
}

// ---------------------------------------------------------------------------
// build_from_video — Stage 8. The source changes; nothing else does.
// ---------------------------------------------------------------------------
// detect_window — where the detector is asked to look this frame.
//
// See DetectRoi in perception/pipeline.hpp for why, and docs/SAT-DESIGN.md
// §14.0b for the design amendment. The rules, in order:
//
//   1. Windowing off, or no confirmed track           -> the whole frame.
//   2. Otherwise: centred on the PREDICTED image position, half-size the
//      larger of the configured floor and the filter's own position sigma
//      times the configured margin.
//
// There used to be a rule between those two: "a refresh frame -> the whole
// frame". P1-7 removed it. The background sweep it implemented is now
// refresh_band() below, which runs ALONGSIDE this window instead of replacing
// it for a frame; see RoiParams::refresh_frames for the measurement that
// motivated the change.
//
// Rule 2's two halves do different jobs. The floor is about CFAR, not about the
// target: §9.4.5's training annulus is 61 px across, and a window narrower than
// that estimates the background from almost nothing. The sigma term is about
// the target: a filter that is losing confidence widens its own window, so a
// track that starts to drift gets more frame back before it drops out of
// Confirmed and gets all of it.
//
// COASTING deliberately does NOT get a window. drivable() is true for Coasting
// as well as Confirmed, and the coasting case is exactly the one where the
// prediction is least trustworthy — the detector has already failed to find
// the target somewhere, and narrowing its search would be the wrong response.
// ---------------------------------------------------------------------------
DetectRoi Pipeline::detect_window(int width, int height,
                                  Angle2 commanded) const noexcept {
    const DetectRoi full = DetectRoi::full(width, height);
    if (!cfg_.roi.enabled) return full;

    const Track& trk = tracker_.track();
    if (trk.state() != TrackState::Confirmed) return full;

    // Where the tracker thinks the target will be when this frame is exposed,
    // expressed in the camera's pixels via the COMMANDED boresight — the same
    // boresight B16 uses to go the other way. Truth is not available here and
    // must not be: INV-1.
    const double  dt  = 1.0 / std::max(1.0, static_cast<double>(cfg_.synthetic.camera_hz));
    const Angle2  ang = trk.predict_position(dt);
    const Pixel2  c   = cfg_.synthetic.camera.project(
                            Angle2{ang.x - commanded.x, ang.y - commanded.y});
    if (!std::isfinite(c.x) || !std::isfinite(c.y)) return full;

    // The filter's own uncertainty, converted from microradians to pixels.
    const double ifov = cfg_.synthetic.camera.ifov_urad();
    const double sig_px = ifov > 0.0 ? trk.position_sigma_urad() / ifov : 0.0;

    double half = std::max(static_cast<double>(cfg_.roi.min_half_px),
                           cfg_.roi.sigma_margin * sig_px);

    // P1-10 rung 2: halve the window when the deadline model says we are
    // behind. The cost is real and is stated rather than hidden — §9.4.5's
    // CFAR training annulus is 61 px across, so a halved floor starts clipping
    // the ring the background estimate is built from, and the detector gets
    // noisier exactly when it is under the most pressure. It is rung TWO for
    // that reason: the median (rung 1) is cheaper to give up.
    if (deadline_.decision().shrink_roi) half = std::max(8.0, half * 0.5);
    // A window that has grown past the frame is just the frame; saying so here
    // keeps the crop out of the hot path entirely on those frames.
    if (half * 2.0 >= static_cast<double>(std::min(width, height))) return full;

    const int x0 = static_cast<int>(std::floor(c.x - half));
    const int y0 = static_cast<int>(std::floor(c.y - half));
    const int x1 = static_cast<int>(std::ceil (c.x + half));
    const int y1 = static_cast<int>(std::ceil (c.y + half));

    const int cx0 = std::clamp(x0, 0, width  - 1);
    const int cy0 = std::clamp(y0, 0, height - 1);
    const int cx1 = std::clamp(x1, 0, width  - 1);
    const int cy1 = std::clamp(y1, 0, height - 1);
    const int w   = cx1 - cx0 + 1;
    const int h   = cy1 - cy0 + 1;
    // The prediction has left the sensor: there is nothing to window around.
    if (w < 8 || h < 8) return full;

    return DetectRoi{cx0, cy0, w, h};
}

// ---------------------------------------------------------------------------
// frame_cost_us — P1-10: what the governor is told this frame cost.
//
// The whole determinism argument for the deadline model lives in these few
// lines, so they are worth stating plainly.
//
// In Injected mode the wall clock is NEVER READ. The cost is whatever the
// injected schedule says and nothing else, so a run's sequence of deadline
// misses, shed levels and therefore detections is a pure function of the
// scenario and the injection — INV-3 holds exactly as it does with the model
// switched off, and a test can assert that frame 45 missed and frames 46-53
// ran degraded.
//
// In Realtime mode it is the measured time, and the run is NOT reproducible.
// That is not a defect of this mode, it is what the mode MEANS: a real-time
// system's behaviour depends on how fast the machine it is on happens to be.
// It is reported (Pipeline::reproducible()) rather than left for a reader to
// discover.
// ---------------------------------------------------------------------------
double Pipeline::frame_cost_us(double measured_us) const noexcept {
    switch (deadline_mode_) {
        case DeadlineMode::Off:
            return 0.0;
        case DeadlineMode::Injected:
            // Nominal zero plus the injection. A frame with no stall against
            // it can never miss, which is precisely what makes the test's
            // expected miss count exact rather than machine-dependent.
            return (frame_ == stall_frame_) ? stall_us_ : 0.0;
        case DeadlineMode::Realtime:
            return measured_us + ((frame_ == stall_frame_) ? stall_us_ : 0.0);
    }
    return 0.0;
}

// ---------------------------------------------------------------------------
// refresh_band — P1-7's background sweep, one horizontal band per frame.
//
// WHAT THIS IS FOR. The tracking window is a bet that the target is where the
// filter says it is. When that bet is wrong — the classic case is a lock onto
// a decoy — nothing outside the window is ever examined again, so nothing
// outside the window can ever be recovered. The sweep is the hedge: it
// guarantees that every row of the sensor is looked at at least once every N
// frames no matter where the window is.
//
// WHY A BAND AND NOT A FRAME. See RoiParams::refresh_frames for the full
// amendment note. In one line: a full-frame sweep costs 19.0 ms of perception
// on 640x480 against a 1.39 ms budget, and paying that on one frame in N puts
// a 14x spike into the p99 of every run that enables it. Splitting the same
// coverage across N frames costs 1/N of that, every frame, with no spike.
//
// THE BAND IS ADDITIONAL, NOT ALTERNATIVE. The window still runs; this is a
// second pass. Sweeping instead of windowing would show the target to the
// tracker only once every N frames, and tracking/track.hpp's M-of-N promotion
// wants 3 hits in 5 frames — it would never confirm. Acquisition is graded by
// specification row 16 and the perception tail is graded by nothing, so
// trading the former for the latter would be the wrong way round.
//
// WHICH BAND. frame_ % N, so the bands rotate in raster order and the sweep is
// a pure function of the frame index. INV-3: no state, no randomness, nothing
// that depends on what the previous frames found.
// ---------------------------------------------------------------------------
DetectRoi Pipeline::refresh_band(int width, int height,
                                 DetectRoi window) const noexcept {
    const int n = cfg_.roi.refresh_frames;
    if (n <= 0) return DetectRoi{0, 0, 0, 0};

    // The window is already the whole frame — during search, or whenever the
    // track is not confirmed. Every row is being examined this frame anyway,
    // so a band would be a second pass over pixels that were just processed.
    if (window.is_full(width, height)) return DetectRoi{0, 0, 0, 0};

    // Integer band edges that tile the frame EXACTLY: band i covers
    // [i*H/N, (i+1)*H/N). Computing the height as H/N and multiplying would
    // leave the last H%N rows never swept, which is the kind of gap that only
    // shows up as "the one time it failed, the target was near the bottom".
    const int i  = static_cast<int>(frame_ % static_cast<uint64_t>(n));
    const int y0 = static_cast<int>(static_cast<int64_t>(i)     * height / n);
    const int y1 = static_cast<int>(static_cast<int64_t>(i + 1) * height / n);
    const int h  = y1 - y0;

    // A band thinner than the CFAR training annulus estimates its background
    // from almost nothing and reports noise as detections. Refusing to sweep
    // is better than sweeping badly: the caller treats an invalid band as "no
    // sweep this frame", and the scenario schema caps refresh_frames so a
    // legal scenario cannot silently land here on every frame.
    if (h < 8 || width < 8) return DetectRoi{0, 0, 0, 0};

    return DetectRoi{0, y0, width, h};
}

// ---------------------------------------------------------------------------
Status Pipeline::build_from_video(const Scenario& sc,
                                  const std::filesystem::path& clip,
                                  const VideoMode* mode_override,
                                  const std::filesystem::path* truth_csv,
                                  int decode_threads) {
    // The synthetic world is still built, and deliberately so: it owns the
    // clock, and the clock owns the sub-tick structure that the gimbal and the
    // controller run on. What it does NOT do in video mode is render — step()
    // takes its frames from the video source instead — so its emitters are
    // irrelevant and its degradation chain is never called, which is INV-8
    // enforced by the code path rather than by a flag.
    build_from_scenario(sc);

    auto v = VideoSource::open(clip, sc, mode_override, decode_threads);
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

    // P1-10. Read only in Realtime mode — see the observe() call at the bottom
    // of this function. Taken here rather than inside the branch so that the
    // span covers the whole frame including the sub-tick loop, which is what a
    // real deadline would have to cover.
    const auto frame_started = (deadline_mode_ == DeadlineMode::Realtime)
                                   ? std::chrono::steady_clock::now()
                                   : std::chrono::steady_clock::time_point{};

    // -----------------------------------------------------------------------
    // CP 14.3: the window INV-4 is about.
    //
    // In a Debug build the global operator new aborts inside this scope. The
    // window is deliberately narrow — it covers per-frame processing and not
    // the setup that precedes a run, because building the world, sizing the
    // arena and opening the logs all allocate and all of them should. INV-4 is
    // about the STEADY state, not about a program that never calls malloc.
    //
    // Costs nothing in Release: FrameScope sets a bool, and with SAT_ALLOC_TRAP
    // undefined nothing ever reads it.
    // -----------------------------------------------------------------------
    FrameScope frame_scope;

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
    // -----------------------------------------------------------------------
    // B3a: design §7.4's timeline.
    //
    // Fired BEFORE the frame is rendered, so an event scheduled at t takes
    // effect in the frame timestamped t rather than the one after it. The
    // alternative reads the same in a config file and is off by one frame in
    // the artifact, which is exactly the kind of discrepancy that makes a
    // measured "reaction time" wrong by a fixed amount nobody can find.
    //
    // Video modes are excluded: INV-8 forbids adding damage in a video mode,
    // and set_atmosphere is damage. A clip's weather is whatever was in front
    // of the camera.
    // -----------------------------------------------------------------------
    if (!video_) {
        const double t_now = static_cast<double>(frame_)
                           / std::max(1.0, static_cast<double>(cfg_.synthetic.camera_hz));
        for (const ScheduledEvent* e : events_.due(t_now)) {
            switch (e->action) {
                case EventAction::SetAtmosphere:
                    // ramp_s is accepted and applied as a step at t_s + ramp/2,
                    // which is where a linear ramp crosses its own midpoint.
                    // Atmosphere is an ENUM in degrade/, not a severity, so
                    // there is nothing continuous to interpolate; pretending
                    // otherwise would be a ramp in name only. Recorded here
                    // rather than silently dropping the field.
                    source_.sensor().set_atmosphere(e->atmosphere);
                    break;

                case EventAction::OccludeTarget:
                    // Handled below as an INTERVAL, not here as an edge. An
                    // edge-triggered occlusion never ends.
                    break;

                case EventAction::SpawnDecoy: {
                    // A second near-identical target, at the stated offset from
                    // the real one. §9.1's decoys are placed at build time;
                    // this is the same thing arriving mid-run, which is the
                    // harder case for the tracker because the association gate
                    // is already narrow around a confirmed track.
                    if (have_target_index_) {
                        EmitterSoA& em = source_.emitters();
                        const Pixel2 p0 = em.position(target_index_);
                        em.add(p0.x + e->offset_px[0], p0.y + e->offset_px[1],
                               target_intensity_, em.size_px[target_index_],
                               em.shape_of(target_index_), EmitterKind::Decoy);
                    }
                    break;
                }

                case EventAction::PlatformGust:
                case EventAction::Unknown:
                    // Gusts are intervals too; see below.
                    break;
            }
        }

        // The interval-valued events, evaluated every frame rather than fired.
        //
        // Guarded on the timeline actually CONTAINING an occlusion. Writing the
        // intensity unconditionally — "not occluded, so restore it" — makes
        // the timeline the owner of that field for the whole run and
        // overwrites anything else that sets it. CP 6.7's dropout tests blank
        // the beacon by zeroing exactly this field, and they stopped seeing a
        // dropout: the timeline handed the beacon back every frame. A
        // component must not write what it does not own.
        if (have_target_index_ && events_.has_occlusions()) {
            const bool hidden = events_.target_occluded(t_now);
            source_.emitters().intensity[target_index_] =
                hidden ? 0.0f : target_intensity_;
        }
    }

    // B4: acquire. The source renders at the true boresight; we hand it the
    // commanded one and it adds the disturbance itself.
    // -----------------------------------------------------------------------
    // In video mode the COMMANDED angle is what the crop uses, and there is no
    // true-vs-commanded distinction: INV-8 disables the disturbances, so the
    // camera points exactly where it was told. That is precisely why video mode
    // is the honest place to grade centroiding — the screen-frame number does
    // not carry a pointing error the detector cannot influence (INV-6).
    // -----------------------------------------------------------------------
    // TWO ANGLES, AND THEY ARE NOT INTERCHANGEABLE — A-4.
    //
    // There used to be one variable here, `commanded`, defined as
    // `video_ ? gimbal_.position() : gimbal_.true_position()`, and it was used
    // for two jobs that need different answers.
    //
    //   aim_angle  WHERE THE CAMERA IS BOLTED. The optics point where the
    //              mount physically is, not where its encoder thinks it is, so
    //              this is true_position() and the source adds the row-23/25
    //              disturbance on top of it. An encoder is a sensor; it does
    //              not move the telescope.
    //
    //   sensed     WHAT THE SYSTEM KNOWS. A real encoder reports a quantised
    //              angle, and every reconstruction perception performs — the
    //              B16 measurement, the screen-frame centroid conversion, the
    //              ROI window, the priority policy's centrality term, the
    //              search pattern's frame of reference — can only use that.
    //
    // gimbal.hpp states the rule in so many words: "position() is what the
    // controller is allowed to read... true_position() is what the metrics
    // use. Mixing them up is the bug §10.3 exists to make impossible." Design
    // §14 CP 4.10's acceptance criterion is "the controller reads only the
    // quantised value". The synthetic path used true_position() for BOTH jobs,
    // and was inconsistent with the video path, which used position() for both.
    //
    // WHY IT MATTERED. Sweeping the encoder LSB on the spec defaults, 10 s,
    // before the fix:
    //
    //     encoder_lsb_urad   in px     screen RMSE   image RMSE   tracking RMS
    //                   20    0.18       99.3061       0.2028        17.585
    //                  500    4.58       99.3086       0.1881        17.613
    //                 4000   36.67       99.3060       0.2038        20.383
    //
    // A 36.67 px quantisation — 3.7x the entire row-17 budget — changed the
    // reported measurement error by ZERO. The encoder model reached only the
    // control feedback path; the measurement path was immune to it, so the
    // question "what happens if your encoder is coarse?" got a misleadingly
    // good answer.
    //
    // WHAT DOES AND DOES NOT MOVE NOW. The IMAGE-frame centroiding error is
    // |detection - truth| within the sensor and is correctly unaffected: the
    // encoder cannot change where light lands on the focal plane. What the
    // encoder now contaminates is everything that converts OUT of the image
    // frame — the screen-frame centroid, the angular measurement, and through
    // it the filter and the loop. That is the physically correct set.
    //
    // In video mode the two coincide: INV-8 disables the disturbances, so
    // true_position() is the commanded angle and only the quantisation
    // separates them.
    //
    // At the shipped 20 urad LSB the effect is ~0.05 px, and pipeline.cpp
    // already budgets `encoder` into the measurement noise R, so the filter
    // was conservative either way. This is a correctness fix to the MODEL, not
    // a correction to a wrong headline number.
    //
    // This necessarily changes the INV-3 digest — perception now consumes a
    // quantised angle, so every downstream value moves. That is the fix
    // working, not a determinism failure.
    // -----------------------------------------------------------------------
    const Angle2 aim_angle = gimbal_.true_position();
    const Angle2 commanded = gimbal_.position();
    SourceFrame frame;
    {
        SAT_ZONE(timers_, Stage::FrameAcquire);
        if (video_) {
            // Video: INV-8 disables the disturbances, so aim_angle and
            // `commanded` differ only by the encoder quantisation. The crop
            // is where the optics are, same as the synthetic path.
            if (!video_->next(aim_angle, frame)) return false;   // EOF: clean
        } else {
            // The mount's real slew, for the exposure smear. Gimbal rate plus
            // the platform's analytic rate — both smooth, both physical.
            // Jitter is excluded on purpose; see SyntheticSource::render_frame.
            const Rate2 gr = gimbal_.rate();
            // -----------------------------------------------------------
            // A-3. This read `frame_ / 30.0` — the camera rate hardcoded.
            //
            // `camera_hz` is schema-legal from 30 to 1000 (scenario/schema.cpp,
            // spec row 5 gives 30 as a MINIMUM, not a fixed value). At
            // camera_hz = 60 the platform rate for the exposure smear was
            // therefore sampled at twice the true elapsed time, so for any
            // periodic platform motion — circular, figure-8, sinusoidal — the
            // blur direction and magnitude were simply wrong, and wrong in a
            // way that grows with the run.
            //
            // Latent until now only because every shipped scenario uses 30.
            // A judge raising row 5 above its minimum, which the PS explicitly
            // permits, would have hit it immediately.
            //
            // The clock already knows the answer and is in scope.
            // -----------------------------------------------------------
            const Rate2 pr = source_.disturbance().platform_rate(
                static_cast<double>(frame_) * source_.clock().camera_dt());
            source_.set_timers(&timers_);
            source_.set_blur_rate(Rate2{gr.x + pr.x, gr.y + pr.y});
            if (!source_.next(aim_angle, frame)) return false;
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
        // PARENT zone. This used to be labelled Stage::Centroid, which made the
        // stage table read "centroid 17,309 us against a 20 us budget" — 865x
        // over for a stage whose whole job is a weighted mean over a 25-pixel
        // window. It was never measuring the centroid: every perception
        // sub-stage nests inside it, so the number was the detector's total.
        // Stage::Centroid is now the leaf it was always meant to be, timed
        // around B12/B13 in perception/pipeline.cpp, and this is `perception`.
        SAT_ZONE(timers_, Stage::Perception);

        // ---------------------------------------------------------------
        // P1-10's load shedding, applied here because this is where the
        // money is: perception is 19.0 ms of a 20.3 ms full-frame budget.
        // The rungs are applied in the order engine/deadline.hpp derives
        // them, cheapest accuracy cost first.
        //
        // `shed` is read BEFORE the frame is processed and the governor is
        // told the cost AFTER, which is the ordering the shed_frames count
        // in DeadlineGovernor::observe depends on.
        // ---------------------------------------------------------------
        const ShedDecision shed = deadline_.decision();
        rec.shed_level      = shed.level;
        rec.deadline_missed = deadline_.last_missed();
        perception_.set_median_enabled(!shed.skip_median);

        // No shed rung can take the classical detector away. See ShedDecision
        // in engine/deadline.hpp for the rung that used to and why it went.
        const bool use_classical =
            cfg_.detector == PipelineConfig::Detector::Classical
            && perception_ready_;

        if (use_classical) {
            rec.roi = detect_window(frame.width, frame.height, commanded);
            perception_.process(frame.pixels, frame.width, frame.height, rec.roi,
                                ws_, dets_, &timers_);

            // -------------------------------------------------------------
            // P1-7's background sweep. A second detector pass over one
            // horizontal band, so that the rows the tracking window is not
            // looking at are still examined once every N frames. Off unless
            // the scenario sets perception.roi_refresh_frames.
            // -------------------------------------------------------------
            rec.roi_band = refresh_band(frame.width, frame.height, rec.roi);
            if (rec.roi_band.valid()) {
                perception_.process(frame.pixels, frame.width, frame.height,
                                    rec.roi_band, ws_, dets_band_, &timers_);

                // The band and the window overlap wherever the window's rows
                // fall inside this band, and a target sitting in the overlap
                // is found by BOTH passes. Handing the tracker the same blob
                // twice would let it associate one of them to the track and
                // treat the other as a rival — a self-inflicted decoy, in the
                // feature whose entire purpose is to survive decoys.
                //
                // The window's copy is the one that is kept. It is not an
                // arbitrary choice: the window is centred on the prediction,
                // so a target in the overlap sits near the middle of the
                // window with its CFAR annulus complete, while in the band it
                // may be hard against a horizontal edge with half its
                // background context clipped away.
                //
                // Detections come back in FULL-FRAME coordinates from both
                // passes (see ClassicalPerception::process), so this compares
                // like with like.
                for (const Detection& d : dets_band_) {
                    const double x = d.centroid_image.x;
                    const double y = d.centroid_image.y;
                    const bool inside_window =
                        x >= rec.roi.x0 && x < rec.roi.x0 + rec.roi.width &&
                        y >= rec.roi.y0 && y < rec.roi.y0 + rec.roi.height;
                    if (!inside_window) dets_.push_back(d);
                }
            }
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
        // §10.2's priority policy needs two things the tracker cannot know on
        // its own: where the camera is pointing (for the centrality term) and
        // how fast the target is allowed to move (to normalise the motion
        // term). Both come from the COMMANDED boresight and the scenario, never
        // from truth.
        PriorityContext pc;
        pc.boresight       = commanded;
        pc.half_fov_urad   = cfg_.synthetic.camera.half_fov_urad().norm();
        pc.speed_ref_urad_s = tracker_.params().max_target_speed_urad_s
                            * static_cast<double>(tracker_.weights().speed_frac);
        pc.speed_max_urad_s = tracker_.params().max_target_speed_urad_s;
        tracker_.set_priority_context(pc);
        associated = tracker_.step(frame_dt, std::span<Measurement>(meas_), frame_);
        // B19: MotionNet beside IMM. Priors on a hit; gated forecast only
        // while coasting. Locked Track aim stays IMM at the Smith horizon
        // (INV-2) and never writes a forecast into the centroid (INV-9).
        last_forecast_ = MotionForecast{};
        if (motion_net_) {
            float hist[Track::kHistory][4];
            tracker_.track().history_tensor(hist);
            last_forecast_ = motion_net_->run(hist);
            if (last_forecast_.valid && associated >= 0
                && tracker_.track().drivable() && tracker_.track().uses_imm()) {
                tracker_.track().set_regime_prior(
                    last_forecast_.regime, last_forecast_.confidence);
                ++motion_prior_applies_;
            }
        }
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
    rec.estimate_sigma_urad = trk.position_sigma_urad();
    if (trk.uses_imm()) {
        rec.imm_mode_prob[0] = static_cast<float>(trk.imm().mode_prob(ImmMode::CV));
        rec.imm_mode_prob[1] = static_cast<float>(trk.imm().mode_prob(ImmMode::CA));
        rec.imm_mode_prob[2] = static_cast<float>(trk.imm().mode_prob(ImmMode::CT));
        rec.imm_turn_rate    = static_cast<float>(trk.imm().turn_rate_rad_s());
    }

    // -----------------------------------------------------------------------
    // B23a: the SAT supervisor (design §10.6, Stage 12).
    //
    // Runs AFTER tracking and BEFORE control, and the position is the whole
    // design. It needs this frame's detection quality and the filter's own
    // consistency, which do not exist until tracking has run; and what it
    // decides has to reach the controller this frame, not next.
    //
    // Everything handed to it is OBSERVABLE. There is no truth in an
    // Observation, which is not an accident of what happened to be nearby: the
    // supervisor is part of the shipped loop, and a supervisor that consulted
    // truth would make every result it produced meaningless.
    // -----------------------------------------------------------------------
    {
        Observation obs;
        obs.detected = rec.detected;
        if (rec.detected) {
            obs.integrated_snr    = rec.detection_snr;
            obs.centroid_sigma_px = rec.centroid_sigma_px;
            // Contrast above the local background, in grey levels. The
            // top-hat has already removed the pedestal, so the peak IS the
            // contrast; bg_sigma is recovered from the pair the detector
            // reports, because SNR is contrast over sigma by construction.
            obs.target_contrast = rec.detection_peak;
            obs.bg_sigma = (rec.detection_snr > 0.0f)
                         ? rec.detection_peak / rec.detection_snr : 0.0f;
        }
        obs.have_track = trk.drivable();
        if (obs.have_track) {
            obs.innovation_nis = static_cast<float>(
                trk.uses_imm() ? trk.imm().last_nis() : trk.filter().last_nis());
            obs.track_age_s = static_cast<float>(trk.age_frames())
                            * static_cast<float>(frame_dt);
            // How much of the gate the accepted innovation used. Near 1 means
            // the filter is only just explaining what it sees.
            obs.gate_utilisation = static_cast<float>(
                std::min(1.0, obs.innovation_nis / cfg_.tracking.gate_chi2));
            obs.imm_mode_ca = rec.imm_mode_prob[1];
            obs.imm_mode_ct = rec.imm_mode_prob[2];
        }
        obs.saturation_frac = static_cast<float>(gimbal_.saturation_frac());

        const Strategy& st = supervisor_.update(obs, frame_, frame.timestamp_s);
        rec.sup_detector   = st.perception;
        rec.sup_centroider = st.centroider;
        rec.sup_cfar_k     = st.cfar_k;
        rec.sup_q_scale    = st.q_scale;
        rec.sup_switched   = supervisor_.changed_this_frame();
        rec.sup_snr        = supervisor_.conditions().integrated_snr;

        if (cfg_.supervisor.enabled && supervisor_.changed_this_frame()) {
            // -------------------------------------------------------------
            // Applying a switch. Three of these are ordinary and one is not.
            // -------------------------------------------------------------
            cfg_.detector = st.perception;
            cfg_.perception.centroid_kind = st.centroider;
            cfg_.perception.cfar.k          = st.cfar_k;
            cfg_.perception.min_snr_factor  = st.min_snr_factor;
            perception_.configure(cfg_.perception);

            // §10.6's property 2, BUMPLESS SWITCHING: "carry the integrator
            // across a gain change, or the switch kicks the loop".
            // set_gains rescales the integrator by the ki ratio so the
            // CONTRIBUTED TERM stays continuous — which is what the plant
            // feels — rather than carrying the raw accumulator, which would
            // step the output by the ratio of the gains. reset() here would
            // zero it and the mount would jump.
            control_.set_gains(st.gains);
            cfg_.gains = st.gains;

            // q_scale multiplies the SCENARIO's process noise, not the
            // previous frame's scaled value. Compounding a 1.4x every switch
            // would walk q away by orders of magnitude over a long run, and
            // nothing downstream would look wrong until the gate was metres
            // wide.
            tracker_.params().kf.accel_psd_urad2_s3 = base_accel_psd_ * st.q_scale;

            // The filter selection reaches only NEW tracks. Swapping a live
            // four-state filter for a six-state one mid-track would have to
            // invent two states, and INV-9's principle applies to a state
            // estimate as much as to a centroid: an invented value that looks
            // like a measurement is worse than an admitted gap.
            tracker_.params().imm = (st.filter == FilterKind::Imm);
        }
    }

    // -----------------------------------------------------------------------
    // B24: mode FSM.
    // -----------------------------------------------------------------------
    // -----------------------------------------------------------------------
    // CP 10.7: the handover sensor.
    //
    // The offset a co-boresighted quadrant cell would see. It is OBSERVABLE
    // and not truth-derived: the frame is rendered at the true boresight, so
    // the detection's distance from the image centre is exactly that offset.
    // Feeding the metrics' tracking error here instead would have the mode FSM
    // deciding to hand over using information the real system does not have,
    // which is the conflation INV-6 exists to prevent.
    //
    // A frame with no detection feeds nothing, rather than feeding zero. Zero
    // is the most flattering possible value for the quantity being tested, and
    // INV-9 makes the same point about the centroid log: a missing measurement
    // is a gap, never a placeholder.
    // -----------------------------------------------------------------------
    if (rec.detected) {
        const Angle2 off = cfg_.synthetic.camera.unproject(rec.detection_img);
        rec.quad_offset_urad = std::hypot(off.x, off.y);
        rec.quad_in_capture  = quad_.in_capture(off);
        handover_.push(rec.quad_offset_urad);
    }
    if (!trk.drivable()) handover_.clear();
    rec.handover_rms_urad = handover_.rms_urad();

    ModeFsmInputs fsm_in;
    fsm_in.running         = true;
    fsm_in.have_track      = tracker_.has_track();
    // Not trk.state(): see Tracker::fsm_state() for why a live hypothesis has
    // to count as Acquire, and for the frame of transient overshoot that
    // telling the FSM otherwise cost.
    fsm_in.track_state     = tracker_.fsm_state();
    fsm_in.candidate_count = rec.candidate_count;
    fsm_in.rms_error_px    = rec.handover_rms_urad / cfg_.synthetic.camera.ifov_urad();
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
        // CP 10.4: advanced by the controller's prediction horizon, which is
        // ZERO unless the Smith predictor is on. Both sides of the error move
        // together or neither does — control/smith.hpp has the argument, and
        // CP 10.1's double lead is what happens when only one of them moves.
        aim = trk.predict_position(control_.horizon_s());
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
        // Coast + valid MotionNet: recentre search/reacquire on the gated
        // multi-step forecast. Locked Track never uses this path (INV-2).
        if (last_forecast_.valid && trk.state() == TrackState::Coasting) {
            const int k = std::min(std::max(trk.consecutive_misses() - 1, 0), 14);
            const Angle2 mn = trk.position() + last_forecast_.step_urad[k];
            const Angle2 imm_p = trk.predict_position(frame_dt * static_cast<double>(k + 1));
            const double gate = static_cast<double>(k + 1) * trk.position_sigma_urad();
            if ((mn - imm_p).norm() <= gate) {
                search_.recentre(mn);
                ++motion_coast_uses_;
            }
        }
        // CP 13.1/13.2: the closed-loop strategies need to know what this look
        // saw and how fast the target can move. The open-loop ones ignore both
        // and step_informed forwards to step() for them, so there is one call
        // site rather than a branch that can get out of step with the enum.
        aim = search_.step_informed(frame_dt, commanded, cfg_.synthetic.camera,
                                    rec.detected, cfg_.max_target_speed_urad_s
                                        / std::max(1e-9, cfg_.synthetic.camera.ifov_urad()),
                                    std::min(cfg_.pan.max_rate_urad_s,
                                             cfg_.tilt.max_rate_urad_s));
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

        cmd_rate_ = control_.compute(aim, measured, ff, platform_rate_est_,
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
                // BORESIGHT-RELATIVE: the same error, still in screen pixels,
                // but with each side referenced to the boresight it was formed
                // through. detection_screen was built from the COMMANDED
                // boresight and truth_screen sits at the TRUE one, so
                // subtracting each from its own reference cancels the
                // unobservable platform displacement D(t) exactly and leaves
                // the detector's own error. See the long note on the field in
                // pipeline.hpp for the algebra.
                const Pixel2 cmd_screen  = cfg_.synthetic.screen.to_pixel(rec.boresight_cmd);
                const Pixel2 true_screen = cfg_.synthetic.screen.to_pixel(rec.boresight_true);
                rec.centroid_error_boresight_px =
                    ((rec.detection_screen - cmd_screen) - (t->screen_pos - true_screen)).norm();
                rec.centroid_error_valid = true;
            } else if (rec.detected) {
                rec.false_alarm = true;
            }
            // Tracking error: how well the mount is pointed at the beacon.
            // Independent of whether we detected it this frame.
            const Pixel2 bore_screen = cfg_.synthetic.screen.to_pixel(rec.boresight_true);
            rec.tracking_error_px = (bore_screen - t->screen_pos).norm();
            // D(t): what the screen-frame centroiding column is actually
            // measuring whenever row 25 platform motion is active. Recorded on
            // every truth-bearing frame, not just detected ones, so the
            // expectation the summary prints is over the whole run.
            rec.pointing_drift_px =
                (bore_screen - cfg_.synthetic.screen.to_pixel(rec.boresight_cmd)).norm();
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
        // ------------------------------------------------------------------
        // P1-2: the preview copy happens ONLY for a consumer that will read it.
        //
        // This stage was measured at 813 us per frame against §15's 30 us
        // budget — 27x over and 24 % of the whole frame — for a memcpy and a
        // hash that produce provenance and would not exist on real hardware.
        // It is on by default in headless, so every FPS figure this project
        // has quoted included it.
        //
        // Two separate savings, and they are worth distinguishing:
        //
        //   The COPY (307,200 bytes, ~25 us). Needed only when something reads
        //   the snapshot asynchronously, because such a reader cannot borrow a
        //   buffer the next frame overwrites. In headless nothing does, so it
        //   is skipped entirely. `preview_consumers_` is set by whoever
        //   attaches; the GUI sets it when it binds to the triple buffer.
        //
        //   The HASH (~790 us). The larger half, and it was never about the
        //   copy at all: FNV-1a is a serial multiply chain, one byte at a
        //   time. It now runs over frame.pixels IN PLACE through fnv1a_bulk,
        //   which is eight independent FNV chains. See core/hash.hpp.
        //
        // Hashing the source buffer instead of the copy must produce the same
        // digest — the bytes are identical — and tests/repro asserts exactly
        // that rather than leaving it to argument.
        // ------------------------------------------------------------------
        if (preview_consumers_) snap.set_preview(frame.pixels);

        fingerprints_.push_back(fingerprint(snap, frame.pixels));
        snapshots_.publish();
    }

    // -----------------------------------------------------------------------
    // P1-10: tell the governor what this frame cost, AFTER processing it and
    // before the frame counter moves. The ordering matters twice over —
    // DeadlineGovernor::observe charges the frame just finished to the level
    // it actually ran at, and frame_cost_us() addresses the injected schedule
    // by the frame index that is still current.
    //
    // The clock is read only when the model is on. In the default Off mode
    // nothing here executes beyond a branch, so a run that does not ask for a
    // deadline pays nothing for one and — more to the point — cannot have its
    // behaviour influenced by a clock it never read.
    // -----------------------------------------------------------------------
    if (deadline_mode_ != DeadlineMode::Off) {
        double measured_us = 0.0;
        if (deadline_mode_ == DeadlineMode::Realtime) {
            measured_us = std::chrono::duration<double, std::micro>(
                              std::chrono::steady_clock::now() - frame_started)
                              .count();
        }
        // `may_shed`: only while there is a lock to keep fed. See
        // DeadlineGovernor::observe for the run this parameter's absence
        // destroyed. drivable() covers Confirmed and Coasting — Coasting is
        // included deliberately, because a coasting track is exactly the one
        // that needs its next measurement to arrive on time.
        deadline_.observe(frame_cost_us(measured_us),
                          tracker_.track().drivable());
        // The record carries the DECISION the frame ran under (set at the
        // perception stage), not the one this observation just produced. A
        // miss is attributed to the frame that missed.
        rec.deadline_missed = deadline_.last_missed();
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
