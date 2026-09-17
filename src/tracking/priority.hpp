// tracking/priority.hpp — design §10.2's priority policy, and the term the
// design did not have.
//
// ---------------------------------------------------------------------------
// THE PROBLEM THIS SOLVES, WITH THE MEASUREMENT THAT FOUND IT
// ---------------------------------------------------------------------------
// Until this existed, Tracker::step() seeded a new track like this:
//
//     "No track. The strongest candidate seeds a new Tentative one."
//
// The strongest candidate. On the specification's own default scenario there
// are 120 static clutter sources (design §9.1, "mandatory for credibility")
// whose intensities are drawn over 0.35x to 1.6x the beacon's, so roughly half
// of them are BRIGHTER than the beacon. The camera sees 9.8% of the screen, the
// beacon starts at a random position (spec row 11), and so on almost every run
// the first thing the tracker ever locks onto is a clutter source.
//
// Measured on scenarios/baseline.toml over 30 s, before this file existed:
//
//     retention   n/a — the beacon was never in view
//     false tracks   1798 /min — every frame of the run
//     tracking RMS   1272 px
//
// The tracker locked onto clutter in the first two frames, the controller
// faithfully drove the mount to centre it, the search stopped because the FSM
// had a confirmed track, and the beacon was never looked for again. Six seconds
// of run time hides this completely, which is why it survived so long.
//
// ---------------------------------------------------------------------------
// WHAT DISTINGUISHES A BEACON FROM A BRIGHT ROCK
// ---------------------------------------------------------------------------
// Design §10.2 gives the priority score as
//
//     w.snr * norm(mean_snr) + w.stability * norm(hit_ratio)
//   + w.centrality * (1 - norm(dist_from_boresight)) + w.age * norm(age_s)
//
// and every one of those four terms is something a bright static source scores
// WELL on. It is bright, it is perfectly stable, the controller has just
// centred it, and it gets older every frame. The design's own weights, applied
// to the design's own scenario, choose the clutter.
//
// The missing term is MOTION, and it is not a heuristic — it is the physical
// definition of the problem. Clutter is specified as static; the target is
// specified as moving, in all four of row 12's mandatory modes.
//
// ---------------------------------------------------------------------------
// WHY RAW MOTION IS NOT ENOUGH: SPEC ROW 25
// ---------------------------------------------------------------------------
// The first version of this scored raw apparent speed and it did not work, for
// a reason worth writing down because it inverts the discriminator rather than
// weakening it.
//
// The tracker converts pixels to world angles through the COMMANDED boresight,
// because it cannot know the true one. Row 25's platform motion moves the true
// boresight and not the commanded one, so an object at a fixed world angle
// theta is reported at theta - disturbance(t) and therefore appears to move at
// MINUS THE PLATFORM RATE.
//
// On scenarios/baseline.toml, with the specification's own numbers:
//
//     platform      15, -8 px/s      ->  clutter appears to move at 1854 urad/s
//     beacon        22, -11 px/s     ->  beacon  appears to move at  831 urad/s
//
// The clutter appears to be moving more than twice as fast as the beacon. A
// tracker scoring raw speed would prefer the rock, confidently, every time.
//
// The fix is ego-motion compensation and it is exact rather than approximate:
// every static source shares the SAME apparent velocity vector, so that vector
// is recoverable as the median over the live hypotheses — most of which, in a
// field of 120 clutter sources, are static. Subtracting it restores the true
// picture:
//
//     clutter  |v - v_common| = 0
//     beacon   |v - v_common| = 2682 urad/s, which is its real speed
//
// See Tracker's common-mode estimate for how the median is formed, and
// Track::mean_velocity_urad_s() for why the term uses lifetime displacement
// rather than the filter's instantaneous rate: spec row 23's jitter makes the
// instantaneous rate useless for the first dozen frames.
//
// ---------------------------------------------------------------------------
// HONEST LIMITS
// ---------------------------------------------------------------------------
//   * A genuinely STATIONARY target scores zero on the motion term. It can
//     still be acquired — motion is one weighted term among five, not a gate —
//     but it competes on equal footing with clutter, which is the situation
//     that existed before. A scenario that wants that behaviour back sets
//     `motion = 0` in the weights.
//   * A moving decoy (spec's decoy beacon) is NOT separated by this. It moves,
//     it is bright, it is stable. Separating a decoy needs a signature the
//     simulation does not model; design §11's CandidateNet is where that lives.
//   * The weights are a policy, not a law. They are data so a scenario can
//     change them and so the ablation in docs/RESULTS.md can run both arms.

#pragma once

#include "core/frames.hpp"

#include <algorithm>
#include <cmath>

namespace sat {

// Declared, not included: tracking/track.hpp includes THIS header so that
// Tracker can hold a PriorityWeights, and including it back would be a cycle.
class Track;

// ---------------------------------------------------------------------------
// PriorityWeights — §10.2's four, plus motion. They sum to 1 by convention so
// a score is directly comparable between runs, but nothing enforces it.
// ---------------------------------------------------------------------------
struct PriorityWeights {
    /// Off reverts to the pre-§10.2 behaviour: the strongest candidate in the
    /// first frame that has one becomes the track, with no hypotheses and no
    /// score. It is the ablation arm, and it is what docs/RESULTS.md measures
    /// this policy against.
    bool  enabled = true;

    // -----------------------------------------------------------------------
    // The four non-motion weights sum to 0.45, and that is a deliberate
    // structural choice rather than a taste.
    //
    // A settled bright clutter source scores the MAXIMUM on all four: it is
    // bright, it never misses, the controller has just centred it, and it gets
    // older every frame. So 0.45 is the ceiling on what anything can score
    // without moving — and min_commit_score below sits above that ceiling.
    // A candidate therefore cannot be committed to on brightness and stability
    // alone, however bright and however stable, which is precisely the failure
    // this whole file exists to fix.
    //
    // Measured before the weights were structured this way: with the four
    // summing to 0.60 the ceiling touched the threshold, a bright rock scored
    // 0.629 for one frame, was promoted, and the run was lost.
    // -----------------------------------------------------------------------
    float snr        = 0.15f;
    float stability  = 0.20f;
    float centrality = 0.05f;
    float age        = 0.05f;
    /// The largest weight, because it is the only term that separates the
    /// beacon from a bright static source and that separation is the whole
    /// reason this file exists.
    float motion     = 0.55f;

    // --- normalisation references -----------------------------------------
    /// SNR at which the brightness term saturates. 40 is roughly the beacon's
    /// clear-air detection SNR, so clutter brighter than the beacon cannot
    /// score higher on this term than the beacon does.
    float snr_ref     = 40.0f;
    /// Frames at which the age term saturates — two seconds at 30 Hz, which is
    /// spec row 16's whole acquisition budget.
    float age_ref     = 60.0f;
    /// Fraction of max_target_speed_urad_s at which the motion term saturates.
    /// A target dawdling at a fifth of its top speed still scores 1.
    float speed_frac  = 0.20f;
    /// Slack on the upper bound, for estimation error. A candidate whose
    /// measured relative speed exceeds speed_max x this cannot be the target.
    float speed_max_slack = 1.5f;
    /// Fallback when the scenario gives no speed bound: one pixel per frame at
    /// the default IFOV, times thirty. Deliberately small, so that a scenario
    /// which forgot to state a speed still discriminates.
    double speed_ref_fallback_urad_s = 3272.0;

    // --- switching hysteresis (§10.2, "mandatory") -------------------------
    /// "Switch ONLY if score_new > 1.25 x score_current, sustained 15 frames."
    float switch_ratio  = 1.25f;
    int   switch_frames = 15;

    // -----------------------------------------------------------------------
    // The score a candidate must reach before the mount is COMMITTED to it.
    //
    // Without this the policy still fails on the one case it most needs to get
    // right: a field of view containing nothing but clutter. The best of a bad
    // set is still promoted, the FSM stops searching because it now has a
    // confirmed track, and the beacon is never looked for. Measured on
    // scenarios/baseline.toml over 30 s before this existed: the beacon was
    // never once in view, 1,798 false-track frames per minute, 1,272 px of
    // tracking error.
    //
    // The number sits above the 0.45 ceiling the four non-motion terms can
    // reach, so it cannot be met without motion:
    //
    //     the best possible rock        0.15 + 0.20 + 0.05 + 0.05 + 0.00 = 0.45
    //     a dim beacon, twelve frames old
    //                                   0.045 + 0.20 + 0.045 + 0.01 + 0.55 = 0.85
    //
    // At 0.60 a candidate needs (0.60 - 0.45) / 0.55 = 0.27 of the motion
    // reference, which at speed_frac = 0.20 means moving at 5.5% of the
    // target's top speed. Anything slower than that is indistinguishable from
    // the scenery by the only evidence available.
    // -----------------------------------------------------------------------
    float min_commit_score = 0.60f;

    // -----------------------------------------------------------------------
    // ...and the symmetric rule, which is what actually rescues a bad lock.
    //
    // A threshold on promotion is not enough on its own: once anything is
    // committed the mode FSM stops searching, the controller centres it, every
    // other candidate leaves the field of view, and there is no rival left to
    // out-score it. The committed track has to keep EARNING the mount.
    //
    // Measured on scenarios/baseline.toml: a clutter source promoted at frame
    // 14 held the mount for the remaining 886 frames of the run with no
    // challenger ever appearing, because the camera was pointed away from
    // everything else by then.
    //
    // drop_frames is generous — a second and a half — because the cost of
    // dropping a good track is a full re-acquisition, and because a target
    // momentarily slowed or occluded must not be thrown away. 0 disables it.
    // -----------------------------------------------------------------------
    int   drop_frames = 45;

    /// Frames of MOTION EVIDENCE a hypothesis must have before it may be
    /// promoted at all — frames with both a measurement and a usable ego-motion
    /// estimate (Track::relative_frames()).
    ///
    /// Not the same thing as M-of-N. M-of-N asks "is this a real detection";
    /// this asks "has the motion term had time to mean anything". Twelve frames
    /// was not enough: during a search sweep the mount is accelerating, the
    /// ego-motion median is still settling, and everything in the frame has a
    /// large transient residual. Thirty frames is one second, against spec row
    /// 16's two-second acquisition budget, and it is one second of frames that
    /// actually carried information rather than one second of wall time.
    int   promote_min_age = 30;
};

/// Everything the score needs that is not in the Track itself.
struct PriorityContext {
    Angle2 boresight{};             ///< where the camera is pointing now
    double half_fov_urad = 1.0;     ///< for the centrality term
    double speed_ref_urad_s = 0.0;  ///< 0 -> the weights' fallback
    // -----------------------------------------------------------------------
    // The fastest the TARGET can possibly go, from the scenario's own motion
    // stack (§7.2's closed forms, the same number that sizes the Kalman q and
    // Track::reach_urad's physical gate). 0 means unbounded.
    //
    // The motion term needs an upper bound as well as a scale, and the reason
    // is measurable. A weak track that jumps between different noise blobs
    // reports a huge apparent speed — 10,165 urad/s was observed against a
    // beacon that can only do 2,682 — and on the motion term alone it outscored
    // everything real in the frame. Something moving four times faster than the
    // target can move is not the target, whatever else is true about it, and
    // the scenario states exactly how fast that is.
    // -----------------------------------------------------------------------
    double speed_max_urad_s = 0.0;
};

/// §10.2's priority_score, with the motion term. Range [0, 1] when the weights
/// sum to 1. Higher is a better candidate for the mount to be aimed at.
[[nodiscard]] float priority_score(const Track& t, const PriorityContext& ctx,
                                   const PriorityWeights& w) noexcept;

}  // namespace sat
