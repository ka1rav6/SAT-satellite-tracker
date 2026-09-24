#include "degrade/disturbance.hpp"

#include "world/motion_factory.hpp"

#include <cmath>

namespace sat {
namespace {

}  // namespace

void DisturbanceGenerator::build(const Scenario& sc, const ScreenGeometry& scr,
                                 double camera_hz) {
    jitter_px_ = sc.jitter_px_per_frame;
    ifov_x_    = scr.ifov_x_urad;
    ifov_y_    = scr.ifov_y_urad;

    platform_ = CompositeMotion{};
    for (const MotionSpec& m : sc.platform) {
        if (auto c = build_motion_component(m)) platform_.add(std::move(c));
    }
    jitter_held_ = Angle2{};

    // Audit P2-1. Off by default, so a scenario with no
    // [atmosphere.turbulence] block draws nothing and is bit-identical.
    turb_.build(sc.turbulence, camera_hz);
}

void DisturbanceGenerator::advance(double dt, RngSet& rng) {
    // Platform stochastic components draw from Stream::PlatformMotion, which is
    // distinct from the target's Stream::TargetMotion — so an `ou_noise`
    // platform does not perturb an `ou_noise` target.
    platform_.advance(dt, rng[Stream::PlatformMotion]);
}

Angle2 DisturbanceGenerator::offset(double t_s, RngSet& rng, bool new_frame) {
    // --- jitter (spec row 23) ----------------------------------------------
    // Resampled once per CAMERA frame. Row 23 specifies "+/- 20 px per frame",
    // so drawing at the 300 Hz truth rate would make it ten times more
    // energetic than the specification describes.
    //
    // Uniform in [-jitter, +jitter] rather than Gaussian: the specification
    // states a hard maximum, and a Gaussian with that as its 1-sigma would
    // exceed it a third of the time while a Gaussian scaled to keep it at 3
    // sigma would be far gentler than specified. A uniform bound means the
    // stated figure is exactly the worst case.
    if (new_frame && jitter_px_ > 0.0) {
        Pcg32& g = rng[Stream::Jitter];
        jitter_held_ = Angle2{g.next_range(-jitter_px_, jitter_px_) * ifov_x_,
                              g.next_range(-jitter_px_, jitter_px_) * ifov_y_};
    } else if (jitter_px_ <= 0.0) {
        jitter_held_ = Angle2{};
    }

    // --- atmospheric angle of arrival (audit P2-1) -------------------------
    // Stepped once per CAMERA frame for the same reason jitter is: the
    // synthesised spectrum is defined against the camera's sample rate, and
    // driving it at the 300 Hz truth tick would make it ten times wider than
    // the band a 30 Hz camera can resolve.
    if (new_frame) turb_.step(rng);
    const Angle2 aoa = turb_.aoa_offset();

    // --- platform motion (spec row 25) -------------------------------------
    const MotionState p = platform_.eval(t_s);
    return Angle2{jitter_held_.x + p.x * ifov_x_ + aoa.x,
                  jitter_held_.y + p.y * ifov_y_ + aoa.y};
}

Rate2 DisturbanceGenerator::platform_rate(double t_s) const {
    const MotionState p = platform_.eval(t_s);
    return Rate2{p.vx * ifov_x_, p.vy * ifov_y_};
}

void DisturbanceGenerator::reset() {
    platform_.reset();
    turb_.reset();
    jitter_held_ = Angle2{};
}

double DisturbanceGenerator::jitter_urad_s(double camera_hz) const noexcept {
    // Design §9.3: rate_urad_s = px_per_frame * ifov_urad * camera_hz
    return jitter_px_ * 0.5 * (ifov_x_ + ifov_y_) * camera_hz;
}

}  // namespace sat
