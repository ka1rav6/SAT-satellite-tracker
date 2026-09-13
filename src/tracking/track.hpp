// tracking/track.hpp — CP 6.3, 6.4 and 6.5. Gating, association, lifecycle.
//
// Design §10.2's lifecycle diagram, verbatim:
//
//     Tentative ──3 hits in 5 frames──▶ Confirmed ──miss──▶ Coasting
//         │                                 ▲                   │
//      5 frames, no confirm                 └────any hit────────┤
//         │                                                15 misses
//         ▼                                                     │
//      Deleted ◀────────────────────────────────────────────────┘
//
// ---------------------------------------------------------------------------
// WHY THIS IS A STATE MACHINE AND NOT A BOOLEAN
// ---------------------------------------------------------------------------
// The naive tracker is `if (detected) follow(detection); else stop;` — which is
// literally what engine/pipeline.cpp did up to Stage 5. It fails in both
// directions and both failures are graded:
//
//   A single false detection makes it jump.       -> §13.1 false_track_rate
//   A single missed frame makes it stop dead.     -> §13.1 lock_retention_rate,
//                                                    graded in BP-2
//
// M-of-N confirmation fixes the first: a track has to prove itself over several
// frames before the controller is allowed to chase it. Coasting fixes the
// second: the filter keeps predicting through a dropout and the camera keeps
// moving sensibly, which is exactly CP 6.4's acceptance criterion ("blanking
// detection for 8 frames does NOT delete the track").
//
// And the two interact in a way that is worth stating, because it is the reason
// no special case is needed for reacquisition: while coasting, P grows every
// frame (predict with no update), so S grows, so the Mahalanobis gate WIDENS
// automatically. A target that reappears somewhere slightly unexpected is
// accepted precisely because we have become less certain. §10.2 makes this
// point explicitly and it is the nicest property in the whole module.

#pragma once

#include "core/units.hpp"
#include "tracking/kalman.hpp"
#include "tracking/measurement.hpp"

#include <cstdint>
#include <span>

namespace sat {

// ---------------------------------------------------------------------------
// TrackState — §10.2's four states. The integer values are logged and hashed
// (snapshot.hpp's track_state), so they are fixed.
// ---------------------------------------------------------------------------
enum class TrackState : uint8_t {
    Tentative = 0,   ///< seen at least once, not yet trusted by the controller
    Confirmed = 1,   ///< M-of-N satisfied; this is what "locked" means
    Coasting  = 2,   ///< was confirmed, currently missing, still predicting
    Deleted   = 3    ///< gone; the slot is free
};

[[nodiscard]] const char* track_state_name(TrackState s) noexcept;

// ---------------------------------------------------------------------------
// TrackParams
// ---------------------------------------------------------------------------
struct TrackParams {
    KalmanParams kf{};

    /// Gate threshold, chi^2(2, 0.99). Kept as data rather than a constant
    /// because §10.6's supervisor widens it in poor conditions.
    double gate_chi2 = kGateChi2_2dof_99;

    // --- M-of-N confirmation (§10.2) ---------------------------------------
    int confirm_hits   = 3;    ///< M
    int confirm_window = 5;    ///< N — also the Tentative timeout

    // --- coasting ----------------------------------------------------------
    /// §10.2's "15 misses". At 30 Hz that is exactly half a second, and spec
    /// row 19 allows 1 s for re-acquisition — so a track survives a dropout for
    /// half the re-acquisition budget before the system gives up and re-earns
    /// the lock from scratch. Deleting sooner would throw away a perfectly good
    /// velocity estimate; deleting later would keep chasing a target that has
    /// genuinely gone.
    int coast_max_misses = 15;

    // -----------------------------------------------------------------------
    // CP 6.5 — adaptive R.
    //
    // When true, R comes from the detection itself: §10.1.4's
    // sigma ~ size / (2 * SNR), floored by the pointing uncertainty. A weak,
    // smeared detection in fog therefore carries a large R, the Kalman gain
    // falls, and the filter leans on its own prediction instead — which is
    // precisely CP 6.5's acceptance criterion, stated as an observable.
    //
    // When false, every measurement gets fixed_r_sigma_urad regardless. That is
    // not a fallback, it is the ABLATION: the checkpoint asks for "error lower
    // than with fixed R", which is only a claim if the fixed-R configuration
    // can actually be run.
    // -----------------------------------------------------------------------
    bool   adaptive_r          = true;
    double fixed_r_sigma_urad  = 110.0;   ///< ~1 px at the default ifov

    /// Clamp on the adaptive sigma. A detection reporting an absurd SNR must
    /// not be able to drive R to zero (the filter would then ignore its own
    /// prediction entirely and follow a single frame) or to infinity (the
    /// update would become a no-op and the track would coast while claiming a
    /// hit, which is the worst of both).
    double min_r_sigma_urad = 5.0;
    double max_r_sigma_urad = 5.0e3;

    // -----------------------------------------------------------------------
    // A PHYSICAL CAP ON THE GATE, on top of the chi-square one.
    //
    // Added at Stage 7, because the metrics found the failure the chi-square
    // gate cannot prevent on its own. The trace, from scenarios/baseline.toml
    // with the beacon pinned in view:
    //
    //   frame 3   the beacon is briefly missed during the approach slew
    //             (motion blur at ~18 px/frame), the only candidates are CFAR
    //             noise blobs, and one 81 px away is INSIDE the gate — because
    //             three frames after a one-point initialisation the position
    //             covariance is still dominated by the velocity prior.
    //   frame 4+  the corrupted velocity estimate drives the mount to its rate
    //             limit, the real beacon falls outside the gate for good, and
    //             the camera slews away for the remaining 446 frames.
    //
    // A chi-square gate is a statement about the filter's OWN uncertainty, so
    // it is only as good as that uncertainty is honest — and right after
    // initialisation it is deliberately not, because a wide prior is how you
    // admit you know nothing about the velocity yet. That is the moment a
    // track is most vulnerable and the moment the statistical gate is weakest.
    //
    // This cap is a different kind of claim, and one that stays true: the
    // TARGET CANNOT HAVE MOVED FURTHER THAN ITS OWN MAXIMUM SPEED ALLOWS.
    // §7.2 makes that speed computable (scenario::max_speed_px_s), and the
    // budget below is
    //
    //     max_target_speed * elapsed_since_last_update + pointing allowance
    //
    // so it widens honestly while coasting — at the target's speed, not at the
    // covariance's — and never admits a candidate that no physical target
    // could have reached.
    // -----------------------------------------------------------------------
    double max_target_speed_urad_s = 0.0;   ///< 0 disables the cap
    double gate_pad_urad           = 0.0;   ///< pointing + centroid allowance
};

// ---------------------------------------------------------------------------
// Track — one target.
// ---------------------------------------------------------------------------
class Track {
public:
    /// Start a new Tentative track from a measurement.
    void start(const Measurement& m, const TrackParams& p, int64_t frame) noexcept;

    /// Advance the estimate by `dt` with no measurement applied yet.
    void predict(double dt) noexcept;

    /// d^2 of a measurement against this track's prediction, using the R this
    /// track would apply to it. Returns a huge number for a dead track so that
    /// "minimum distance" logic never selects one.
    [[nodiscard]] double distance2(const Measurement& m) const noexcept;

    /// Is this measurement inside the gate? BOTH gates: the chi-square one and
    /// the physical reachability cap. See TrackParams::max_target_speed_urad_s.
    [[nodiscard]] bool gates(const Measurement& m) const noexcept {
        return distance2(m) < p_.gate_chi2 && within_reach(m);
    }

    /// Could a target moving at its maximum speed have got here since the last
    /// accepted measurement? Always true when the cap is disabled.
    [[nodiscard]] bool within_reach(const Measurement& m) const noexcept;

    /// The reachability radius this frame, microradians. Exposed so the GUI can
    /// draw it and a test can assert it grows with the miss count.
    [[nodiscard]] double reach_urad() const noexcept;

    // -----------------------------------------------------------------------
    // assoc_score — what nearest neighbour actually minimises.
    //
    //     score = d^2 + ln|S|      ( = -2 ln L, up to a constant )
    //
    // NOT d^2 on its own, and the difference is not cosmetic once CP 6.5's
    // adaptive R is switched on.
    //
    // d^2 is a ratio: displacement over uncertainty. A garbage low-SNR
    // detection reports a large sigma, which inflates S, which DIVIDES its d^2
    // down — so a plain nearest-in-d^2 rule systematically prefers the least
    // trustworthy candidate in the frame. Two candidates the same distance from
    // the prediction, one crisp and one smeared, and it takes the smeared one.
    //
    // The likelihood has a normalisation term that fixes exactly this: a wide
    // distribution has a low peak, so ln|S| charges a candidate for the
    // uncertainty it claims. The result is the standard NN score and it picks
    // the crisp detection, which is what CP 6.5's test asserts.
    //
    // Gating still uses d^2 alone, because the gate is a chi-square
    // probability statement about one measurement and 9.21 means nothing on a
    // scale that includes ln|S|.
    // -----------------------------------------------------------------------
    [[nodiscard]] double assoc_score(const Measurement& m) const noexcept;

    /// Apply an associated measurement and run the lifecycle transition.
    void update(const Measurement& m) noexcept;

    /// No measurement was associated this frame. Runs the lifecycle transition.
    void miss() noexcept;

    // --- state -------------------------------------------------------------
    [[nodiscard]] TrackState state()     const noexcept { return state_; }
    [[nodiscard]] bool       alive()     const noexcept { return state_ != TrackState::Deleted; }
    [[nodiscard]] bool       confirmed() const noexcept { return state_ == TrackState::Confirmed; }

    /// Should the controller aim at this track? Confirmed and Coasting both
    /// qualify — coasting on a prediction is the entire point of CP 6.4. A
    /// Tentative track must NOT drive the mount: it might be a noise blob, and
    /// chasing it is how a false detection becomes a lost lock.
    [[nodiscard]] bool drivable() const noexcept {
        return state_ == TrackState::Confirmed || state_ == TrackState::Coasting;
    }

    [[nodiscard]] const KalmanFilter& filter() const noexcept { return kf_; }
    [[nodiscard]] Angle2 position() const noexcept { return kf_.position(); }
    [[nodiscard]] Rate2  rate()     const noexcept { return kf_.rate(); }
    [[nodiscard]] Angle2 predict_position(double dt) const noexcept {
        return kf_.predict_position(dt);
    }

    [[nodiscard]] int  age_frames()          const noexcept { return age_; }
    [[nodiscard]] int  hits()                const noexcept { return hits_; }
    [[nodiscard]] int  consecutive_misses()  const noexcept { return misses_; }
    [[nodiscard]] float mean_snr()           const noexcept { return mean_snr_; }
    [[nodiscard]] double last_sigma_urad()   const noexcept { return last_sigma_; }

    /// hits / age. §10.2's priority_score reads it as `hit_ratio`, and it is
    /// also the per-track form of §13.1's lock_retention_rate.
    [[nodiscard]] float hit_ratio() const noexcept {
        return age_ > 0 ? static_cast<float>(hits_) / static_cast<float>(age_) : 0.0f;
    }

private:
    /// R for this measurement, honouring adaptive_r and the clamps.
    [[nodiscard]] double sigma_for(const Measurement& m) const noexcept;

    void record_hit(bool hit) noexcept;
    [[nodiscard]] int hits_in_window() const noexcept;

    KalmanFilter kf_{};
    TrackParams  p_{};
    TrackState   state_ = TrackState::Deleted;

    int      age_    = 0;      ///< frames since start, hits and misses alike
    int      hits_   = 0;      ///< total updates applied
    int      misses_ = 0;      ///< CONSECUTIVE misses; reset by any hit
    uint32_t window_ = 0;      ///< bitmask of the last 32 frames, 1 = hit
    int64_t  start_frame_ = 0;

    float  mean_snr_   = 0.0f;
    double last_sigma_ = 0.0;
    double dt_         = 1.0 / 30.0;   ///< frame interval, for reach_urad()
};

// ---------------------------------------------------------------------------
// Tracker — the single-target policy (§10.2's TargetPolicy::SingleLock).
//
// One track at a time. Multi-target and the Hungarian assignment of §10.2 are
// a Stage 12 concern; building them now would be speculative, and the
// single-track case is what every graded metric in §3.2 is defined against.
//
// What it does each frame, in §6.2's order:
//
//   B17  predict, then gate every measurement against the prediction
//   B18  associate — nearest neighbour by Mahalanobis distance
//   B19  update, or miss
//   B20  lifecycle transition (inside Track)
// ---------------------------------------------------------------------------
class Tracker {
public:
    void reset(const TrackParams& p) noexcept;

    /// One frame. `meas` is modified: the associated element is flagged, so the
    /// caller can tell which candidates went unexplained — §10.5's search grid
    /// treats those as evidence rather than noise.
    ///
    /// Returns the index of the associated measurement, or -1.
    int step(double dt, std::span<Measurement> meas, int64_t frame) noexcept;

    [[nodiscard]] const Track& track() const noexcept { return track_; }
    [[nodiscard]] bool has_track()  const noexcept { return track_.alive(); }
    [[nodiscard]] bool has_lock()   const noexcept { return track_.drivable(); }

    /// Gated but unassociated this frame. Diagnostic, and the seed of the
    /// multi-target extension.
    [[nodiscard]] int gated_count() const noexcept { return gated_; }

    [[nodiscard]] const TrackParams& params() const noexcept { return p_; }
    [[nodiscard]] TrackParams& params() noexcept { return p_; }

    /// Force the track away — used when the mode FSM decides the lock is bad.
    void drop() noexcept { track_ = Track{}; }

private:
    TrackParams p_{};
    Track       track_{};
    int         gated_ = 0;
};

}  // namespace sat
