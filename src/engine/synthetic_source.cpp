// engine/synthetic_source.cpp

#include "engine/synthetic_source.hpp"

#include <cmath>
#include <algorithm>

namespace sat {

void SyntheticSource::build(const SyntheticConfig& cfg, EmitterSoA emitters) {
    cfg_      = cfg;
    cfg_.screen = ScreenGeometry::make(cfg.screen.width  ? cfg.screen.width  : 2000,
                                       cfg.screen.height ? cfg.screen.height : 2000,
                                       cfg.camera);
    world_ = World{};
    world_.emitters = std::move(emitters);

    auto clk = Clock::make(cfg_.truth_hz, cfg_.camera_hz, cfg_.control_hz);
    // The scenario loader validates these before we get here (design §6.1 A1),
    // so a failure at this point is a programming error, not user input. Fall
    // back to the spec defaults rather than leaving the clock unconfigured.
    clock_ = clk.has_value() ? *clk : *Clock::make(300, 30, 30);

    rng_.seed_all(cfg_.seed);

    const size_t px = static_cast<size_t>(cfg_.camera.pixel_count());
    radiance_.assign(px, 0.0f);
    frame_.assign(px, 0);
    visible_.reserve(world_.emitters.capacity());

    frame_index_    = 0;
    max_frames_     = static_cast<int64_t>(cfg_.duration_s * cfg_.camera_hz + 0.5);
    have_prev_bore_ = false;
    disturbance_    = Angle2{};

    // -----------------------------------------------------------------------
    // EVERY piece of per-run state, not just the obvious ones.
    //
    // build() is not only "construct a source"; it is also "start this run
    // over", and the GUI's Reset button is the caller that makes the
    // difference visible. Reset calls Pipeline::build_from_scenario on the
    // SAME Pipeline object, so this SyntheticSource is reused rather than
    // reconstructed, and anything left behind here is carried silently into
    // the next run.
    //
    // `sim_time_s_` was the one that mattered. §7.2's motion algebra evaluates
    // every emitter's position at ABSOLUTE time t rather than integrating a
    // velocity — which is exactly what makes the analytic velocity reportable
    // as truth — so a run that starts at t = 20 s puts the beacon wherever the
    // previous run left it, on a phase of its figure-8 that has nothing to do
    // with frame 0. The RNG streams were reseeded correctly, the world was
    // rebuilt correctly, and the simulation still came out different every
    // time Reset was pressed. Pipeline::step() meanwhile timestamps frames as
    // frame_ / camera_hz, which DOES restart at zero, so the world clock and
    // the event timeline also disagreed by a whole previous run.
    //
    // `blur_rate_` is the same class of bug one order smaller: the first frame
    // after a Reset would be smeared along the slew the PREVIOUS run ended on
    // until the engine set it again.
    //
    // `graded_slot_` is cleared rather than trusted: ensure_scintillation_slots
    // rebuilds it only when the emitter COUNT changes, and two different
    // scenarios can have the same count with different kinds.
    // -----------------------------------------------------------------------
    sim_time_s_     = 0.0;
    blur_rate_      = Rate2{};
    prev_true_bore_ = Angle2{};
    graded_slot_.clear();
    // build() installs a bare emitter set, so there is no compiled world until
    // build_from_scenario says otherwise. Leaving this true would run the
    // motion algebra over an empty World.
    have_world_         = false;
    manual_disturbance_ = false;
}

void SyntheticSource::advance_world(double dt) noexcept {
    // -----------------------------------------------------------------------
    // THE CLOCK, not an accumulator.
    //
    // This line used to be `sim_time_s_ += dt`, which is the one thing
    // core/time.hpp forbids in so many words: "seconds = tick / truth_hz, NOT
    // from an accumulated t += dt. Accumulation drifts". The Clock existed,
    // was configured in build(), and nothing in the entire program ever called
    // tick() on it — so `clock_.tick_index()` was zero on every frame of every
    // run, and the `tick` column of FrameTruth was a constant 0.
    //
    // Ticking it here makes the Clock the single authority the file says it
    // is. The time is then an exact integer division, identical on every
    // machine, and the truth tick means something.
    //
    // `dt` is still honoured for the benefit of the world and the disturbance
    // integrators, which take a step size rather than an instant, and it is
    // the caller's truth_dt in every path that exists. When it is NOT the
    // clock's own truth_dt — a test stepping the world by hand — the clock
    // would disagree with the requested step, so the accumulator is kept for
    // exactly that case and the clock is used whenever they agree.
    // -----------------------------------------------------------------------
    if (std::fabs(dt - clock_.truth_dt()) < 1e-15) {
        clock_.tick();
        sim_time_s_ = clock_.seconds();
    } else {
        sim_time_s_ += dt;
    }

    if (have_world_) {
        // Design §7.2's motion algebra: each emitter's position comes from
        // evaluating its stack at absolute time t, not from integrating a
        // velocity. That is what makes the position exact for an accelerating
        // target and what lets the analytic velocity be reported as truth.
        world_.advance(sim_time_s_, dt, rng_);
        disturb_.advance(dt, rng_);
        return;
    }

    // No scenario: emitters carry a constant configured velocity. Kept for the
    // tests and benchmarks that build a world by hand rather than from TOML.
    for (size_t i = 0; i < world_.emitters.n; ++i) {
        world_.emitters.x[i] += world_.emitters.vx[i] * dt;
        world_.emitters.y[i] += world_.emitters.vy[i] * dt;
    }
}

void SyntheticSource::build_from_scenario(const Scenario& sc) {
    // Design §6.1's startup sequence, in order.
    SyntheticConfig cfg;
    cfg.camera        = sc.camera_geometry();               // A2
    cfg.screen        = sc.screen_geometry();
    cfg.truth_hz      = sc.truth_hz;
    cfg.camera_hz     = sc.camera_hz;
    cfg.control_hz    = sc.control_hz;
    cfg.duration_s    = sc.duration_s;
    cfg.seed          = sc.seed;                            // A3
    cfg.blur_substeps = sc.blur_substeps;
    cfg.exposure_s    = sc.exposure_ms * 1e-3;

    build(cfg, EmitterSoA{});

    // A5: emitters, compiled motion stacks, clutter and decoys. They live
    // inside `world_` and stay there — World::advance writes into exactly the
    // arrays the renderer reads, so there is never a second copy to keep in
    // step.
    world_      = build_world(sc, rng_);
    have_world_ = true;

    sensor_.build(sc, cfg.camera.width, cfg.camera.height, rng_);
    disturb_.build(sc, cfg.screen, static_cast<double>(cfg.camera_hz));
    visible_.reserve(world_.emitters.capacity());
    ensure_scintillation_slots();
    manual_disturbance_ = false;
}

void SyntheticSource::ensure_scintillation_slots() {
    // Audit P2-1: each graded emitter gets its own independent scintillation
    // slot. Anything past the cap is parked on kMaxGraded, which
    // TurbulenceModel::irradiance_gain() answers 1.0 for — a beacon that
    // stops scintillating is a far better failure than one that silently
    // shares a decoy's gain, and neither is an out-of-bounds read.
    if (graded_slot_.size() == world_.emitters.n) return;

    graded_slot_.assign(world_.emitters.n,
                        static_cast<uint8_t>(TurbulenceModel::kMaxGraded));
    size_t slot = 0;
    for (size_t i = 0; i < world_.emitters.n; ++i) {
        if (world_.emitters.kind_of(i) == EmitterKind::Clutter) continue;
        if (slot >= TurbulenceModel::kMaxGraded) break;
        graded_slot_[i] = static_cast<uint8_t>(slot++);
    }
}

void SyntheticSource::render_frame(Angle2 true_bore, double /*t_s*/) {
    // Start from the background pedestal rather than zero. A real sensor always
    // has one, and starting at zero would flatter every detector downstream.
    //
    // Timed as §15's "Background render" line. It is a fill rather than a
    // procedural texture because the screen the beacon sits on is specified as
    // a uniform field (spec rows 1 and 8); a texture would be scenery, not
    // signal, and every detector downstream would be measured against it.
    ensure_scintillation_slots();

    {
        SAT_ZONE_OPT(timers_, Stage::BackgroundRender);
        std::fill(radiance_.begin(), radiance_.end(), cfg_.background);
    }

    const int substeps = std::max(1, cfg_.blur_substeps);
    const double w = 1.0 / static_cast<double>(substeps);

    // -----------------------------------------------------------------------
    // Motion blur (design §9.2): integrate the scene over the EXPOSURE,
    // interpolating both the emitters and the boresight.
    //
    // Two things here are easy to get wrong and both bias the graded metric:
    //
    // (1) The exposure is `exposure_ms` long (5 ms by default), NOT the whole
    //     frame interval (33 ms at 30 Hz). A shutter is open for a fraction of
    //     the frame; smearing across the full interval would exaggerate the
    //     blur almost sevenfold. During a full-rate slew that is the difference
    //     between a 4 px smear and a 27 px one.
    //
    // (2) The exposure is centred on the frame's timestamp, not ended at it.
    //     The centroid of a motion-blurred image sits at the target's position
    //     at the exposure MIDPOINT, so if truth were reported at the end of the
    //     exposure every frame would carry a systematic centroid error of
    //     v * exposure / 2 — small (0.05 px at 20 px/s) but systematic, and
    //     therefore exactly the kind of bias §10.1.3 warns cannot be averaged
    //     away. Centring the exposure on the timestamp makes the reported truth
    //     the thing the image actually shows.
    //
    // Interpolating only the emitters and not the boresight would miss the
    // dominant effect during a fast slew, which is the camera moving rather
    // than the target.
    // -----------------------------------------------------------------------
    const double exposure_s = std::max(0.0, cfg_.exposure_s);

    // -----------------------------------------------------------------------
    // The boresight rate used for blur is the mount's PHYSICAL SLEW — the
    // gimbal's rate plus the platform's analytic rate — supplied by the engine.
    // It deliberately EXCLUDES jitter.
    //
    // Why: spec row 23 specifies jitter as "+/- 20 px per FRAME", which is a
    // frame-to-frame pointing displacement. Differencing consecutive true
    // boresights would fold that displacement into a rate of up to
    // 40 px / 33 ms = 1200 px/s and smear every frame by ~6 px, inventing a
    // difficulty the specification does not describe. (An early version did
    // exactly that and produced 38 px of centroiding error on a beacon that was
    // sitting in plain view.)
    //
    // Treating jitter as a pointing offset held constant across the 5 ms
    // exposure is the standard "vibration is slow compared to the shutter"
    // assumption. It is also the conservative direction: it does not make the
    // problem artificially easier, because the jitter still moves the boresight
    // by its full amplitude between frames, which is what row 23 states.
    // -----------------------------------------------------------------------
    const Angle2 bore_rate{blur_rate_.x, blur_rate_.y};

    // Hoisted out of the splat loop: with turbulence off — the default, and
    // every scenario committed before it existed — this is the only test paid
    // for scintillation anywhere in the frame.
    const bool scintillating = disturb_.turbulence().enabled();

    {
    SAT_ZONE_OPT(timers_, Stage::EmitterSplat);
    for (int s = 0; s < substeps; ++s) {
        // Substep CENTRES, offset so the samples straddle the timestamp:
        // f runs over (-0.5, +0.5) x exposure. Sampling at the interval edges
        // would double-weight the endpoints and shift the blurred centroid by
        // half a substep.
        const double f = ((static_cast<double>(s) + 0.5)
                          / static_cast<double>(substeps)) - 0.5;
        const double dt_s = f * exposure_s;

        const Angle2 bore{true_bore.x + bore_rate.x * dt_s,
                          true_bore.y + bore_rate.y * dt_s};

        const Aabb box = view_aabb(cfg_.camera, cfg_.screen, bore)
                             .expanded(world_.emitters.max_extent_px());
        world_.emitters.query_visible(box, visible_);

        for (const uint32_t idx : visible_) {
            const size_t i = idx;
            const Pixel2 screen_pos{world_.emitters.x[i] + world_.emitters.vx[i] * dt_s,
                                    world_.emitters.y[i] + world_.emitters.vy[i] * dt_s};
            const Pixel2 img = screen_to_image(cfg_.camera, cfg_.screen, screen_pos, bore);
            // Clutter is drawn to 3.5 sigma and the beacon to 6. The
            // difference is not a shortcut: six sigma exists to stop the
            // truncation shifting a rendered centroid, and a clutter source's
            // centroid is never scored against anything. See splat.hpp.
            const bool graded = world_.emitters.kind_of(i) != EmitterKind::Clutter;
            // Audit P2-1: scintillation multiplies the BEACON'S IRRADIANCE
            // here, before the splat, so it flows through the exposure
            // integration, the damage chain, the SNR gate and the detector
            // exactly as a real irradiance fluctuation would — rather than
            // being a brightness knob applied to the finished frame.
            //
            // Graded emitters only, and each with its own independent gain.
            // Sources further apart than the isoplanatic angle (~10 urad)
            // scintillate independently, and every emitter on a 12.5 deg
            // screen is far past that, so one common gain would be wrong and
            // a decoy that faded in step with the beacon would be a gift to
            // the tracker. Clutter is scenery, not a propagating beam; see
            // degrade/turbulence.hpp for why it is left alone.
            const double gain = (scintillating && graded)
                ? disturb_.turbulence().irradiance_gain(graded_slot_[i])
                : 1.0;
            splat_emitter(radiance_, cfg_.camera.width, cfg_.camera.height, img,
                          static_cast<double>(world_.emitters.size_px[i]),
                          world_.emitters.shape_of(i),
                          world_.emitters.intensity[i] * gain, w,
                          graded ? 6.0 : kClutterReachSigmas);
        }
    }
    }

    // The full damage chain (design §9.3): atmosphere, shot noise, read noise,
    // fixed pattern, salt and pepper, defects, clip and quantise. In a video
    // mode it degenerates to the quantisation alone (INV-8).
    {
        SAT_ZONE_OPT(timers_, Stage::DamageChain);
        sensor_.apply(radiance_, frame_, rng_);
    }

    prev_true_bore_ = true_bore;
    have_prev_bore_ = true;
}

void SyntheticSource::fill_truth(Angle2 true_bore, Angle2 commanded_bore,
                                 FrameTruth& t) const {
    t.tick                = clock_.tick_index();
    t.boresight_true      = true_bore;
    t.boresight_commanded = commanded_bore;
    t.n                   = 0;

    for (size_t i = 0; i < world_.emitters.n && t.n < kMaxTargets; ++i) {
        const EmitterKind k = world_.emitters.kind_of(i);
        // Clutter is not reported: truth exists to score the beacon, and a
        // hundred clutter entries would just make FrameTruth expensive to copy.
        if (k != EmitterKind::Target && k != EmitterKind::Decoy) continue;

        FrameTruth::Target& e = t.targets[t.n];
        e.id         = world_.emitters.id[i];
        e.screen_pos = world_.emitters.position(i);
        e.world_ang  = cfg_.screen.to_angle(e.screen_pos);
        e.image_pos  = screen_to_image(cfg_.camera, cfg_.screen, e.screen_pos, true_bore);
        // Velocity in angle space. Screen pixels and angle share the IFOV
        // (core/frames.hpp), so this is a pure scale.
        e.world_rate = Rate2{world_.emitters.vx[i] * cfg_.screen.ifov_x_urad,
                             world_.emitters.vy[i] * cfg_.screen.ifov_y_urad};
        e.in_fov     = cfg_.camera.contains(e.image_pos);
        e.is_primary = (k == EmitterKind::Target);
        ++t.n;
    }
}

bool SyntheticSource::next(Angle2 commanded_boresight, SourceFrame& out) {
    if (max_frames_ > 0 && frame_index_ >= max_frames_) return false;

    // -----------------------------------------------------------------------
    // INV-2 and the honesty of the whole simulation: the frame is rendered at
    // the TRUE boresight, which is the commanded one plus whatever the
    // disturbances did. The tracker is handed only the commanded value, and the
    // difference is the pointing error it cannot observe directly.
    //
    // Jitter is resampled here rather than in advance_world because spec row 23
    // specifies it in px PER FRAME; drawing it at the 300 Hz truth rate would
    // make it ten times more energetic than the specification describes.
    // -----------------------------------------------------------------------
    if (!manual_disturbance_) {
        disturbance_ = disturb_.offset(sim_time_s_, rng_, /*new_frame=*/true);
    }
    const Angle2 true_bore{commanded_boresight.x + disturbance_.x,
                           commanded_boresight.y + disturbance_.y};

    // -----------------------------------------------------------------------
    // THE FRAME'S TIMESTAMP IS THE TIME OF THE WORLD IT SHOWS.
    //
    // This read `frame_index_ / camera_hz`, and it was wrong by exactly one
    // camera period on every frame of every run.
    //
    // Design §6.2's order is B1 advance the world, B2 advance the
    // disturbances, B3 step the gimbal, B4 acquire — so by the time this
    // function runs, Pipeline::step has already called advance_world()
    // camera_divisor times and the world is at t = (frame_index + 1) / camera_hz.
    // That is the instant the emitters are splatted at (the splat reads
    // `world_.emitters`, which World::advance has already moved) and the
    // instant the boresight disturbance below is evaluated at. Labelling it
    // frame_index / camera_hz claimed the frame showed the world 33 ms before
    // the one it actually shows.
    //
    // Measured, with jitter and platform motion off so the beacon's position
    // is purely its own motion, on spec_defaults' 22 px/s linear target:
    //
    //   frame 0, reported t = 0.0000, truth_screen.x = 1040.733
    //   analytic x(0.0000) = 1040.000        analytic x(0.0333) = 1040.733
    //
    // Everything downstream that reads a timestamp inherited the error:
    // centroid.csv and trace.csv, the acquisition time row 16 is graded on,
    // and — the one that changes behaviour rather than just labels — §7.4's
    // event timeline and the platform rate the exposure smear is integrated
    // along, both of which Pipeline::step was computing from the same wrong
    // `frame_ / camera_hz` expression. An event scheduled at t fired against a
    // world that was already a frame past t, which is precisely the off-by-one
    // that pipeline.cpp's own comment says it exists to avoid.
    //
    // The clock is asked rather than the arithmetic repeated: advance_world()
    // ticks it, so it is the same integer division for every consumer.
    // -----------------------------------------------------------------------
    const double t_s = sim_time_s_;
    render_frame(true_bore, t_s);

    out.pixels              = frame_;
    out.width               = cfg_.camera.width;
    out.height              = cfg_.camera.height;
    out.frame_index         = frame_index_;
    out.timestamp_s         = t_s;
    out.commanded_boresight = commanded_boresight;
    out.has_truth           = true;
    fill_truth(true_bore, commanded_boresight, out.truth);

    ++frame_index_;
    return true;
}

FrameGeometry SyntheticSource::geometry() const {
    return FrameGeometry{cfg_.camera.width, cfg_.camera.height,
                         static_cast<double>(cfg_.camera_hz),
                         cfg_.screen.width, cfg_.screen.height};
}

}  // namespace sat
