// world/world_builder.hpp — turn a Scenario into a populated world.
//
// Design §6.1 step A5: "Build world. Emitters, compiled motion stacks, shape
// masks, clutter, decoys, initial target position from Stream::TargetInit."
//
// ---------------------------------------------------------------------------
// CLUTTER IS MANDATORY, NOT DECORATIVE
// ---------------------------------------------------------------------------
// Design §9.1: "Clutter is mandatory for credibility: 50-500 static bright
// sources, >=1 decoy beacon (a second near-identical target), bright edges and
// gradients."
//
// Without it the detection problem is trivial — the beacon is the only bright
// thing in an empty field, and a brightest-pixel detector would score perfectly.
// CP 4.11's acceptance criterion is precisely that with 120 clutter sources the
// brightest-pixel detector "demonstrably locks onto the wrong thing", which is
// the evidence that §9.4's CFAR pipeline is necessary rather than decorative.
//
// The decoy is the sharper test: a second, near-identical beacon. It cannot be
// rejected on appearance — only on behaviour, which is what the Mahalanobis gate
// and the track lifecycle (§10.2) are for. CP 6.3 requires the tracker to stay
// on the real beacon with a decoy 60 px away.

#pragma once

#include "core/rng.hpp"
#include "scenario/scenario.hpp"
#include "world/emitters.hpp"
#include "world/motion_component.hpp"

#include <memory>
#include <vector>

namespace sat {

/// A world: emitters plus the motion stack that drives each of them.
///
/// Motion stacks are stored parallel to the emitters rather than inside
/// EmitterSoA, because most emitters (clutter) have none and a vector of null
/// unique_ptrs in the hot SoA would be waste.
struct World {
    EmitterSoA emitters;

    /// motion[i] drives emitters index motion_owner[i]. Only emitters that
    /// actually move appear here.
    std::vector<std::unique_ptr<CompositeMotion>> motion;
    std::vector<uint32_t>                         motion_owner;

    /// Base position for each moving emitter, so a motion stack expressing a
    /// pure displacement (like `linear`) is applied relative to where the
    /// emitter started rather than to the canvas origin.
    std::vector<Pixel2> motion_base;

    // -----------------------------------------------------------------------
    // Spec row 8's edge behaviour, applied to the evaluated position.
    //
    // This was parsed, validated, echoed into run.json — and read by nothing,
    // exactly like design §7.4's events before they were wired up. The
    // consequence is not cosmetic. On scenarios/baseline.toml with the
    // specification's own default seed, spec row 11's random initial position
    // puts the beacon at (1936, 1831) on a 2000 x 2000 screen and row 12's
    // linear motion carries it off the canvas within three seconds:
    //
    //     frame   0   truth = (1936, 1831)
    //     frame 100   truth = (2009, 1794)   <- already outside
    //     frame 800   truth = (2523, 1537)
    //
    // The tracker then spent the remaining 886 frames of the run failing to
    // find a beacon that was not in the world, and every metric the run
    // produced was a measurement of that rather than of anything else. It is
    // the single most misleading kind of defect: everything downstream works
    // perfectly and reports a catastrophe.
    //
    // Applied as a TRANSFORM of the analytically evaluated position, not by
    // reflecting a velocity and integrating. Design §7.2 is explicit that
    // positions come from evaluating the stack at absolute time t rather than
    // from integrating, because that is what makes the position exact for an
    // accelerating target and what lets the analytic velocity be reported as
    // truth. A bounce implemented by flipping a stored velocity would have to
    // become stateful and would break both properties; the triangle-wave fold
    // below is a pure function of the evaluated coordinate and keeps them.
    // -----------------------------------------------------------------------
    EdgeBehaviour edge = EdgeBehaviour::Bounce;
    double        canvas_w = 0.0, canvas_h = 0.0;   ///< 0 disables the fold

    /// Evaluate every motion stack at time t and write the results into the
    /// emitter arrays. Called once per truth tick.
    void advance(double t_s, double dt, RngSet& rng);

    void reset();
};

// ---------------------------------------------------------------------------
// fold_edge — one axis of spec row 8, as a pure function.
//
// `bounce`  a triangle wave: the coordinate reflects off 0 and `extent`, and
//           the velocity's sign flips on every other fold. Exact for any
//           position however far outside, and with no accumulated state.
// `wrap`    a sawtooth: the coordinate is taken modulo `extent`. The velocity
//           is unchanged; the emitter reappears on the other side.
// `exit`    unchanged. A target that leaves has left, which is a legitimate
//           thing for a scenario to ask for and is what CP 8.8's "beacon
//           exits" clip tests.
//
// Returns the folded coordinate and writes the (possibly negated) velocity.
// ---------------------------------------------------------------------------
[[nodiscard]] double fold_edge(double p, double& v, double extent,
                               EdgeBehaviour e) noexcept;

/// Build the world described by a scenario.
///
/// Every random choice draws from a named stream, so changing the clutter count
/// cannot move the target's initial position and vice versa — which is what
/// makes two scenarios that differ in one setting actually comparable.
[[nodiscard]] World build_world(const Scenario& sc, RngSet& rng);

}  // namespace sat
