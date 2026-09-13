// world/motion_factory.cpp — MotionSpec (parsed TOML) -> IMotionComponent.
//
// One factory, used by BOTH the target builder (spec row 12) and the platform
// disturbance builder (spec row 25). Design §7.2 claims "the same components
// drive [[disturbance.platform]]", and CP 4.9's criterion is that the five
// platform modes are "driven by the same code as target motion" — which is only
// true if there is literally one function, so there is.

#include "world/motion_factory.hpp"

namespace sat {

/// Build one IMotionComponent from a parsed MotionSpec.
///
/// Shared by the target builder and the platform builder — design §7.2's claim
/// that rows 12 and 25 use "the same component system" is only true if there is
/// literally one factory, so there is.
std::unique_ptr<IMotionComponent> build_motion_component(const MotionSpec& m) {
    if (m.kind == "constant") {
        return std::make_unique<ConstantMotion>(m.offset_px[0], m.offset_px[1]);
    }
    if (m.kind == "linear") {
        return std::make_unique<LinearMotion>(m.velocity_px_s[0], m.velocity_px_s[1]);
    }
    if (m.kind == "accel") {
        return std::make_unique<AccelMotion>(m.accel_px_s2[0], m.accel_px_s2[1]);
    }
    if (m.kind == "sinusoid") {
        const double amp = m.axis == 0 ? m.amplitude_px[0] : m.amplitude_px[1];
        return std::make_unique<SinusoidMotion>(m.axis, amp, m.period_s, m.phase_deg);
    }
    if (m.kind == "circular") {
        return std::make_unique<CircularMotion>(m.radius_px, m.period_s, m.phase_deg);
    }
    if (m.kind == "lissajous") {
        return std::make_unique<LissajousMotion>(m.amplitude_px[0], m.amplitude_px[1],
                                                 m.freq_ratio, m.period_s, m.phase_deg);
    }
    if (m.kind == "spiral") {
        return std::make_unique<SpiralMotion>(m.r0_px, m.growth_px_s, m.period_s, m.phase_deg);
    }
    if (m.kind == "ou_noise") {
        return std::make_unique<OuNoiseMotion>(m.sigma_px_s, m.tau_s);
    }
    if (m.kind == "waypoints") {
        std::vector<WaypointMotion::Waypoint> pts;
        pts.reserve(m.points.size());
        for (const auto& p : m.points) pts.push_back({p[0], p[1], p[2]});
        return std::make_unique<WaypointMotion>(std::move(pts));
    }
    // The schema rejects unknown kinds before this point, so reaching here means
    // a kind was added to the parser and not to this factory.
    return nullptr;
}


}  // namespace sat
