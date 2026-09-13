// world/motion_component.cpp — the one component too long to inline.

#include "world/motion_component.hpp"

#include <algorithm>

namespace sat {

// ---------------------------------------------------------------------------
// Catmull-Rom spline through the waypoints.
//
// For a segment between p1 and p2, with neighbours p0 and p3, and local
// parameter u in [0, 1]:
//
//     p(u) = 0.5 * ( (2p1)
//                  + (-p0 + p2) u
//                  + (2p0 - 5p1 + 4p2 - p3) u²
//                  + (-p0 + 3p1 - 3p2 + p3) u³ )
//
// and the derivative with respect to u is the term-by-term differentiation:
//
//     p'(u) = 0.5 * ( (-p0 + p2)
//                   + 2(2p0 - 5p1 + 4p2 - p3) u
//                   + 3(-p0 + 3p1 - 3p2 + p3) u² )
//
// Converting to a velocity in px/s needs the chain rule: du/dt = 1/(t2 - t1).
// Forgetting that factor is the classic mistake here, and it would make the
// reported velocity wrong by the segment duration — which CP 3.5's finite-
// difference test catches immediately.
//
// Endpoints are handled by duplicating the first and last points, which is the
// standard clamped boundary condition and makes the spline start and end
// travelling toward its neighbour rather than flying off.
// ---------------------------------------------------------------------------
MotionState WaypointMotion::eval(double t) const {
    if (pts_.empty()) return {};
    if (pts_.size() == 1) return {pts_[0].x, pts_[0].y, 0.0, 0.0};

    // Before the first waypoint or after the last, hold position with zero
    // velocity. Extrapolating a cubic outside its range diverges fast, and a
    // target that flew off the canvas because the run outlasted its waypoint
    // list would be a confusing failure.
    if (t <= pts_.front().t) return {pts_.front().x, pts_.front().y, 0.0, 0.0};
    if (t >= pts_.back().t)  return {pts_.back().x,  pts_.back().y,  0.0, 0.0};

    // Find the segment [i, i+1] containing t. Linear scan: waypoint lists are
    // short (tens of points) and this runs once per emitter per tick, so a
    // binary search would cost more in branch misprediction than it saves.
    size_t i = 0;
    while (i + 2 < pts_.size() && pts_[i + 1].t <= t) ++i;

    const auto& p1 = pts_[i];
    const auto& p2 = pts_[i + 1];
    const auto& p0 = (i == 0) ? pts_[0] : pts_[i - 1];
    const auto& p3 = (i + 2 < pts_.size()) ? pts_[i + 2] : pts_[i + 1];

    const double span = p2.t - p1.t;
    if (span <= 0.0) return {p1.x, p1.y, 0.0, 0.0};   // duplicate timestamps

    const double u  = (t - p1.t) / span;
    const double u2 = u * u;
    const double u3 = u2 * u;

    auto position = [&](double a0, double a1, double a2, double a3) {
        return 0.5 * ((2.0 * a1)
                    + (-a0 + a2) * u
                    + (2.0 * a0 - 5.0 * a1 + 4.0 * a2 - a3) * u2
                    + (-a0 + 3.0 * a1 - 3.0 * a2 + a3) * u3);
    };
    auto velocity = [&](double a0, double a1, double a2, double a3) {
        // d/du, then chain rule du/dt = 1/span.
        return 0.5 * ((-a0 + a2)
                    + 2.0 * (2.0 * a0 - 5.0 * a1 + 4.0 * a2 - a3) * u
                    + 3.0 * (-a0 + 3.0 * a1 - 3.0 * a2 + a3) * u2) / span;
    };

    return {position(p0.x, p1.x, p2.x, p3.x),
            position(p0.y, p1.y, p2.y, p3.y),
            velocity(p0.x, p1.x, p2.x, p3.x),
            velocity(p0.y, p1.y, p2.y, p3.y)};
}

}  // namespace sat
