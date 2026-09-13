// engine/synthetic_source.cpp

#include "engine/synthetic_source.hpp"

#include <algorithm>

namespace sat {

void SyntheticSource::build(const SyntheticConfig& cfg, EmitterSoA emitters) {
    cfg_      = cfg;
    cfg_.screen = ScreenGeometry::make(cfg.screen.width  ? cfg.screen.width  : 2000,
                                       cfg.screen.height ? cfg.screen.height : 2000,
                                       cfg.camera);
    emitters_ = std::move(emitters);

    auto clk = Clock::make(cfg_.truth_hz, cfg_.camera_hz, cfg_.control_hz);
    // The scenario loader validates these before we get here (design §6.1 A1),
    // so a failure at this point is a programming error, not user input. Fall
    // back to the spec defaults rather than leaving the clock unconfigured.
    clock_ = clk.has_value() ? *clk : *Clock::make(300, 30, 30);

    rng_.seed_all(cfg_.seed);

    const size_t px = static_cast<size_t>(cfg_.camera.pixel_count());
    radiance_.assign(px, 0.0f);
    frame_.assign(px, 0);
    visible_.reserve(emitters_.n);

    frame_index_    = 0;
    max_frames_     = static_cast<int64_t>(cfg_.duration_s * cfg_.camera_hz + 0.5);
    have_prev_bore_ = false;
    disturbance_    = Angle2{};
}

void SyntheticSource::advance_world(double dt) noexcept {
    // Stage 1: emitters move at their constant configured velocity. The full
    // motion algebra (design §7.2) replaces this at CP 3.3-3.4, which is why
    // this integrates rather than evaluating a closed form — the positions are
    // about to become a function of t, and the interface should not change.
    for (size_t i = 0; i < emitters_.n; ++i) {
        emitters_.x[i] += emitters_.vx[i] * dt;
        emitters_.y[i] += emitters_.vy[i] * dt;
    }
}

void SyntheticSource::render_frame(Angle2 true_bore, double /*t_s*/) {
    // Start from the background pedestal rather than zero. A real sensor always
    // has one, and starting at zero would flatter every detector downstream.
    std::fill(radiance_.begin(), radiance_.end(), cfg_.background);

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

    // Boresight rate, estimated from the previous frame. Zero on the first
    // frame, which is correct: the mount starts at rest.
    Angle2 bore_rate{};
    if (have_prev_bore_ && clock_.camera_dt() > 0.0) {
        bore_rate = Angle2{(true_bore.x - prev_true_bore_.x) / clock_.camera_dt(),
                           (true_bore.y - prev_true_bore_.y) / clock_.camera_dt()};
    }

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
                             .expanded(emitters_.max_extent_px());
        emitters_.query_visible(box, visible_);

        for (const uint32_t idx : visible_) {
            const size_t i = idx;
            const Pixel2 screen_pos{emitters_.x[i] + emitters_.vx[i] * dt_s,
                                    emitters_.y[i] + emitters_.vy[i] * dt_s};
            const Pixel2 img = screen_to_image(cfg_.camera, cfg_.screen, screen_pos, bore);
            splat_emitter(radiance_, cfg_.camera.width, cfg_.camera.height, img,
                          static_cast<double>(emitters_.size_px[i]),
                          emitters_.shape_of(i), emitters_.intensity[i], w);
        }
    }

    // Stage 4 inserts the full damage chain (design §9.3) between here and the
    // quantisation below.
    quantise_u8(radiance_, frame_);

    prev_true_bore_ = true_bore;
    have_prev_bore_ = true;
}

void SyntheticSource::fill_truth(Angle2 true_bore, Angle2 commanded_bore,
                                 FrameTruth& t) const {
    t.tick                = clock_.tick_index();
    t.boresight_true      = true_bore;
    t.boresight_commanded = commanded_bore;
    t.n                   = 0;

    for (size_t i = 0; i < emitters_.n && t.n < kMaxTargets; ++i) {
        const EmitterKind k = emitters_.kind_of(i);
        // Clutter is not reported: truth exists to score the beacon, and a
        // hundred clutter entries would just make FrameTruth expensive to copy.
        if (k != EmitterKind::Target && k != EmitterKind::Decoy) continue;

        FrameTruth::Target& e = t.targets[t.n];
        e.id         = emitters_.id[i];
        e.screen_pos = emitters_.position(i);
        e.world_ang  = cfg_.screen.to_angle(e.screen_pos);
        e.image_pos  = screen_to_image(cfg_.camera, cfg_.screen, e.screen_pos, true_bore);
        // Velocity in angle space. Screen pixels and angle share the IFOV
        // (core/frames.hpp), so this is a pure scale.
        e.world_rate = Rate2{emitters_.vx[i] * cfg_.screen.ifov_x_urad,
                             emitters_.vy[i] * cfg_.screen.ifov_y_urad};
        e.in_fov     = cfg_.camera.contains(e.image_pos);
        e.is_primary = (k == EmitterKind::Target);
        ++t.n;
    }
}

bool SyntheticSource::next(Angle2 commanded_boresight, SourceFrame& out) {
    if (max_frames_ > 0 && frame_index_ >= max_frames_) return false;

    // INV-2 and the honesty of the whole simulation: the frame is rendered at
    // the TRUE boresight, which is the commanded one plus whatever the
    // disturbances did. The tracker is handed only the commanded value.
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
