// engine/synthetic_source.cpp

#include "engine/synthetic_source.hpp"

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
    visible_.reserve(world_.emitters.n);

    frame_index_    = 0;
    max_frames_     = static_cast<int64_t>(cfg_.duration_s * cfg_.camera_hz + 0.5);
    have_prev_bore_ = false;
    disturbance_    = Angle2{};
}

void SyntheticSource::advance_world(double dt) noexcept {
    sim_time_s_ += dt;

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
    disturb_.build(sc, cfg.screen);
    visible_.reserve(world_.emitters.n);
    manual_disturbance_ = false;
}

void SyntheticSource::render_frame(Angle2 true_bore, double /*t_s*/) {
    // Start from the background pedestal rather than zero. A real sensor always
    // has one, and starting at zero would flatter every detector downstream.
    //
    // Timed as §15's "Background render" line. It is a fill rather than a
    // procedural texture because the screen the beacon sits on is specified as
    // a uniform field (spec rows 1 and 8); a texture would be scenery, not
    // signal, and every detector downstream would be measured against it.
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
            splat_emitter(radiance_, cfg_.camera.width, cfg_.camera.height, img,
                          static_cast<double>(world_.emitters.size_px[i]),
                          world_.emitters.shape_of(i), world_.emitters.intensity[i], w);
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

    const double t_s = static_cast<double>(frame_index_) / static_cast<double>(cfg_.camera_hz);
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
