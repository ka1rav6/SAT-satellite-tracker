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

    /// Evaluate every motion stack at time t and write the results into the
    /// emitter arrays. Called once per truth tick.
    void advance(double t_s, double dt, RngSet& rng);

    void reset();
};

/// Build the world described by a scenario.
///
/// Every random choice draws from a named stream, so changing the clutter count
/// cannot move the target's initial position and vice versa — which is what
/// makes two scenarios that differ in one setting actually comparable.
[[nodiscard]] World build_world(const Scenario& sc, RngSet& rng);

}  // namespace sat
