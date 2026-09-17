// tracking/priority.cpp — see the header for why the motion term exists.

#include "tracking/priority.hpp"

#include "tracking/track.hpp"

namespace sat {

namespace {
/// Clamp to [0, 1]. Every term is normalised this way so the weights mean
/// what they look like they mean.
[[nodiscard]] float unit(double v) noexcept {
    if (!(v > 0.0)) return 0.0f;                // also catches NaN
    return static_cast<float>(v < 1.0 ? v : 1.0);
}
}  // namespace

float priority_score(const Track& t, const PriorityContext& ctx,
                     const PriorityWeights& w) noexcept {
    if (!t.alive()) return 0.0f;

    // --- brightness -------------------------------------------------------
    const float snr = unit(static_cast<double>(t.mean_snr())
                           / std::max(1.0f, w.snr_ref));

    // --- stability --------------------------------------------------------
    // The RECENT hit ratio, not the lifetime one: a track with a thousand good
    // frames behind it should not be able to coast on its reputation.
    const float stab = unit(static_cast<double>(t.recent_hit_ratio()));

    // --- centrality -------------------------------------------------------
    // A candidate near the middle of the field is cheaper to keep than one at
    // the edge, which is about to leave. Weighted lightly: it is a tiebreaker,
    // and weighting it heavily is precisely how the old policy convinced
    // itself that the clutter source the controller had just centred was the
    // target.
    const Angle2 p = t.position();
    const double d = std::hypot(p.x - ctx.boresight.x, p.y - ctx.boresight.y);
    const float  central = 1.0f - unit(d / std::max(1.0, ctx.half_fov_urad));

    // --- age --------------------------------------------------------------
    const float age = unit(static_cast<double>(t.age_frames())
                           / std::max(1.0f, w.age_ref));

    // --- motion -----------------------------------------------------------
    const double ref = ctx.speed_ref_urad_s > 0.0 ? ctx.speed_ref_urad_s
                                                  : w.speed_ref_fallback_urad_s;
    // Ego-motion compensated: what this candidate does DIFFERENTLY from the
    // static world, not what it does. See Track::relative_velocity_urad_s and
    // priority.hpp for the measurement showing the uncompensated version
    // prefers the clutter.
    const double rel = t.relative_speed_urad_s();
    float motion = unit(rel / std::max(1.0, ref));
    // The upper bound. See PriorityContext::speed_max_urad_s: a candidate that
    // moves faster than the target physically can is not the target, and
    // without this the motion term is a prize for erratic tracks.
    if (ctx.speed_max_urad_s > 0.0 &&
        rel > ctx.speed_max_urad_s * static_cast<double>(w.speed_max_slack)) {
        motion = 0.0f;
    }

    return w.snr        * snr
         + w.stability  * stab
         + w.centrality * central
         + w.age        * age
         + w.motion     * motion;
}

}  // namespace sat
