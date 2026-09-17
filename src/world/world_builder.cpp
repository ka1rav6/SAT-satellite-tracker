#include "world/world_builder.hpp"

#include "world/motion_factory.hpp"

#include <algorithm>
#include <cmath>

namespace sat {

double fold_edge(double p, double& v, double extent, EdgeBehaviour e) noexcept {
    if (e == EdgeBehaviour::Exit || !(extent > 0.0) || !std::isfinite(p)) return p;

    if (e == EdgeBehaviour::Wrap) {
        double q = std::fmod(p, extent);
        if (q < 0.0) q += extent;
        return q;
    }

    // Bounce. Fold into [0, 2*extent), then reflect the upper half down. The
    // number of reflections is odd exactly in that upper half, which is where
    // the velocity's sign flips.
    const double period = 2.0 * extent;
    double q = std::fmod(p, period);
    if (q < 0.0) q += period;
    if (q > extent) {
        v = -v;
        return period - q;
    }
    return q;
}

void World::advance(double t_s, double dt, RngSet& rng) {
    // Stochastic components must be advanced in order, once per tick, before
    // the stack is evaluated.
    for (auto& m : motion) {
        if (m->has_stochastic()) m->advance(dt, rng[Stream::TargetMotion]);
    }

    for (size_t k = 0; k < motion.size(); ++k) {
        const uint32_t i = motion_owner[k];
        const MotionState s = motion[k]->eval(t_s);
        emitters.x[i]  = motion_base[k].x + s.x;
        emitters.y[i]  = motion_base[k].y + s.y;
        // Analytic velocity, straight from the component (design §7.2). This is
        // what FrameTruth reports as world_rate and what CP 6.2 compares the
        // filter's estimate against, so it must not be a difference.
        emitters.vx[i] = s.vx;
        emitters.vy[i] = s.vy;

        // Spec row 8. See the note in world_builder.hpp for why this is a fold
        // of the evaluated coordinate rather than a reflected velocity.
        emitters.x[i] = fold_edge(emitters.x[i], emitters.vx[i], canvas_w, edge);
        emitters.y[i] = fold_edge(emitters.y[i], emitters.vy[i], canvas_h, edge);
    }
}

void World::reset() {
    for (auto& m : motion) m->reset();
}

World build_world(const Scenario& sc, RngSet& rng) {
    World w;
    const ScreenGeometry scr = sc.screen_geometry();
    w.edge     = sc.edge_behaviour;              // spec row 8
    w.canvas_w = static_cast<double>(scr.width);
    w.canvas_h = static_cast<double>(scr.height);

    const size_t expected = sc.targets.size()
                          + static_cast<size_t>(std::max(0, sc.decoy_beacons))
                          + static_cast<size_t>(std::max(0, sc.static_sources));
    w.emitters.reserve(expected);

    auto shape_of = [](const std::string& s) {
        if (s == "circle")   return ShapeKind::Circle;
        if (s == "gaussian") return ShapeKind::Gaussian;
        if (s == "mask")     return ShapeKind::Mask;
        return ShapeKind::Square;
    };

    // --- targets (spec rows 7-12) ------------------------------------------
    for (const TargetSpec& t : sc.targets) {
        // Spec row 11: "random" or explicit. The random draw comes from
        // Stream::TargetInit so that enabling clutter cannot move the target.
        Pixel2 pos{t.initial_px[0], t.initial_px[1]};
        if (t.random_initial) {
            Pcg32& g = rng[Stream::TargetInit];
            // Kept a margin inside the canvas so a target never starts half off
            // the edge, which would make the first frame's truth ambiguous.
            const double margin = static_cast<double>(t.size_px);
            pos.x = g.next_range(margin, sc.canvas_px[0] - margin);
            pos.y = g.next_range(margin, sc.canvas_px[1] - margin);
        }

        const size_t idx = w.emitters.add(pos.x, pos.y, static_cast<float>(t.intensity),
                                          static_cast<uint16_t>(t.size_px),
                                          shape_of(t.shape_type), EmitterKind::Target);

        if (!t.motion.empty()) {
            auto stack = std::make_unique<CompositeMotion>();
            for (const MotionSpec& m : t.motion) {
                if (auto c = build_motion_component(m)) stack->add(std::move(c));
            }
            if (!stack->empty()) {
                w.motion.push_back(std::move(stack));
                w.motion_owner.push_back(static_cast<uint32_t>(idx));
                // A stack containing a `constant` component is expressing an
                // absolute position, so its base is the origin; otherwise the
                // stack is a displacement from wherever the emitter started.
                bool absolute = false;
                for (const MotionSpec& m : t.motion) {
                    if (m.kind == "constant") { absolute = true; break; }
                }
                w.motion_base.push_back(absolute ? Pixel2{0.0, 0.0} : pos);
            }
        }
    }

    // --- decoys (design §9.1) ----------------------------------------------
    // "A second near-identical target." Near-identical is the point: it must be
    // indistinguishable on appearance so that only BEHAVIOUR can separate it
    // from the real beacon.
    if (!sc.targets.empty() && sc.decoy_beacons > 0) {
        Pcg32& g = rng[Stream::DecoyLayout];
        const TargetSpec& t = sc.targets[0];
        const Pixel2 primary = w.emitters.position(0);
        for (int d = 0; d < sc.decoy_beacons; ++d) {
            // Placed 60-300 px away: close enough to fall inside the same field
            // of view and compete for the association gate (CP 6.3 uses 60 px),
            // far enough not to merge into one blob.
            const double angle = g.next_range(0.0, 2.0 * kPi);
            const double dist  = g.next_range(60.0, 300.0);
            const double x = clamp(primary.x + dist * std::cos(angle),
                                   10.0, sc.canvas_px[0] - 10.0);
            const double y = clamp(primary.y + dist * std::sin(angle),
                                   10.0, sc.canvas_px[1] - 10.0);
            // Within 10% of the real beacon's brightness and within one pixel
            // of its size: a detector cannot tell them apart.
            w.emitters.add(x, y,
                           static_cast<float>(t.intensity * g.next_range(0.9, 1.1)),
                           static_cast<uint16_t>(std::max(5, t.size_px +
                                                 static_cast<int>(g.next_below(3)) - 1)),
                           shape_of(t.shape_type), EmitterKind::Decoy);
        }
    }

    // --- static clutter (design §9.1) --------------------------------------
    if (sc.static_sources > 0) {
        Pcg32& g = rng[Stream::ClutterLayout];
        for (int i = 0; i < sc.static_sources; ++i) {
            const double x = g.next_range(0.0, static_cast<double>(sc.canvas_px[0]));
            const double y = g.next_range(0.0, static_cast<double>(sc.canvas_px[1]));

            // A spread of brightnesses, some of them BRIGHTER than the beacon.
            // That is deliberate and it is what CP 4.11 needs: if every clutter
            // source were dimmer, a brightest-pixel detector would still work
            // and the case for CFAR would be unproven.
            const double intensity = g.next_range(0.35, 1.6) * 120.0;

            // Sizes spanning and exceeding the beacon's range, so the shape gate
            // (§9.4.7) has both things to keep and things to reject.
            const uint16_t size = static_cast<uint16_t>(3 + g.next_below(22));

            // Mostly Gaussian: real background sources are diffuse, and a field
            // of hard-edged squares would make the shape gate look better than
            // it is.
            const ShapeKind shape = (g.next_below(4) == 0) ? ShapeKind::Square
                                                           : ShapeKind::Gaussian;
            w.emitters.add(x, y, static_cast<float>(intensity), size,
                           shape, EmitterKind::Clutter);
        }
    }

    return w;
}

}  // namespace sat
