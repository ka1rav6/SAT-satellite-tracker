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
#include "tracking/imm.hpp"
#include "tracking/kalman.hpp"
#include "tracking/measurement.hpp"
#include "tracking/priority.hpp"

#include <cstdint>
#include <array>
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

    // -----------------------------------------------------------------------
    // CP 10.5: run the IMM (tracking/imm.hpp) instead of the plain filter.
    //
    // OPT-IN rather than the default, and the reason is not caution about the
    // IMM. Every reproducibility fingerprint in this project is a hash of the
    // simulation state, and the tracker's estimate feeds the controller, which
    // moves the mount, which changes every frame that follows. Making a
    // six-state filter the default would churn every one of them, so the switch
    // is a scenario key and the fingerprints stay meaningful on both settings.
    //
    // It is also an honest reflection of what CP 10.5 measures: the IMM earns
    // its place on MANOEUVRING motion (the figure-8, spec row 12) and costs a
    // little on a straight line, which is the ablation, not a defect.
    // -----------------------------------------------------------------------
    bool      imm = false;
    ImmParams imm_params{};

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
    // TRACK QUALITY (§10.2) — a lock that is not following anything must die
    //
    // The lifecycle above has a hole, and it is the kind that only shows up on
    // a long run. Confirmed -> miss -> Coasting -> any hit -> Confirmed, and
    // `misses_` is reset to zero by every hit. So a track that alternates hit,
    // miss, hit, miss can NEVER reach coast_max_misses CONSECUTIVE misses, and
    // it lives for ever. Worse, reach_urad() grows with the miss count, so each
    // miss widens the gate and makes the next spurious hit easier to catch.
    //
    // Measured: on a scenario with the beacon blanked, no clutter and nothing
    // but read noise, the tracker held a "lock" for the whole run. The two
    // cases in tests/tracking/test_reacquire.cpp that assert the lock is LOST
    // when the target is hidden are what caught it — and they had been passing
    // only because one particular noise realisation happened not to line up,
    // which is not the same as the code being right.
    //
    // Two independent quality tests, either of which deletes the track:
    //
    //   HIT RATIO over a long window. Not `confirm_window`, which is five
    //   frames and is about earning confirmation, but a couple of dozen, which
    //   is about whether the thing is still there. A real target blanked for
    //   CP 6.4's eight frames keeps a ratio of 16/24 = 0.67 and survives; a
    //   track being fed by occasional noise does not.
    //
    //   NORMALISED INNOVATION SQUARED. This is the stronger of the two and it
    //   is the standard test. For a correctly-associated track the innovation
    //   is distributed as chi-square with 2 degrees of freedom, so E[NIS] = 2 —
    //   and it STAYS 2 in fog, because CP 6.5's adaptive R inflates the
    //   covariance to match. A track being fed by whatever happens to fall
    //   inside its gate has innovations at the scale of the GATE, which is
    //   9.21. The two separate cleanly and the threshold needs no per-weather
    //   tuning, which is exactly the property a hit-ratio test does not have.
    //
    // Both are evaluated only on Confirmed or Coasting tracks, and only once
    // the track is old enough to have a history: a Tentative track is already
    // governed by M-of-N.
    // -----------------------------------------------------------------------
    int   quality_window    = 24;    ///< frames the hit ratio is measured over
    float quality_min_ratio = 0.40f; ///< below this over the window -> Deleted
    float quality_max_nis   = 6.0f;  ///< EMA of NIS above this -> Deleted
    float quality_nis_tau   = 12.0f; ///< EMA time constant, frames
    /// 0 disables quality management entirely, which is the ablation arm and
    /// what the before/after in docs/RESULTS.md is measured against.
    bool  quality_enabled   = true;

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
    // gate cannot prevent on its own. The trace, from
    // scenarios/hard/clutter_field.toml (the beacon in view, clutter on):
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

    // -----------------------------------------------------------------------
    // MotionNet history ring (SAT-ML §6). Fixed array, INV-4.
    //
    // Runtime input is tracker state including coasts, not FrameTruth. miss()
    // still pushes the predicted [az, el, vaz, vel] so a dropout does not
    // freeze the window the net sees.
    // -----------------------------------------------------------------------
    static constexpr int kHistory = 30;
    void push_history() noexcept;
    void history_tensor(float out[kHistory][4]) const noexcept;
    [[nodiscard]] int history_count() const noexcept { return hist_count_; }

    /// Forward to ImmFilter when the IMM is on. No-op on the plain Kalman path.
    void set_regime_prior(MotionRegime regime, float confidence) noexcept;

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

    // --- CP 10.5 ----------------------------------------------------------
    /// The IMM, valid only when TrackParams::imm is set. Exposed for the mode
    /// probability panel (§12) and for the tests that measure the switch.
    [[nodiscard]] const ImmFilter& imm() const noexcept { return imm_; }
    [[nodiscard]] bool uses_imm() const noexcept { return p_.imm; }

    /// The estimate's uncertainty, from whichever filter is running.
    [[nodiscard]] double position_sigma_urad() const noexcept {
        return p_.imm ? imm_.position_sigma_urad() : kf_.position_sigma_urad();
    }

    [[nodiscard]] Angle2 position() const noexcept {
        return p_.imm ? imm_.position() : kf_.position();
    }
    [[nodiscard]] Rate2  rate()     const noexcept {
        return p_.imm ? imm_.rate() : kf_.rate();
    }
    [[nodiscard]] Angle2 predict_position(double dt) const noexcept {
        return p_.imm ? imm_.predict_position(dt) : kf_.predict_position(dt);
    }

    /// Mean angular speed over the track's whole life: the straight-line
    /// displacement from where it started, divided by how long it has existed.
    ///
    /// NOT the filter's instantaneous rate, and the difference is what makes
    /// this usable. Spec row 23 puts +/- 20 px per FRAME of jitter on the
    /// boresight, and because the tracker converts pixels to angles through the
    /// COMMANDED boresight — it cannot know the true one — that jitter appears
    /// as apparent target motion of up to 65,448 urad/s. The filter's velocity
    /// estimate is noisy at that scale for the first dozen frames, which is
    /// exactly when a hypothesis is being judged.
    ///
    /// A displacement over the whole lifetime suppresses zero-mean jitter as
    /// 1/age^1.5 rather than as the filter's transient. At age 12 a static
    /// source reads about 7,700 urad/s against a 200 px/s beacon's 21,816; by
    /// age 30 it reads 563. The discriminator therefore gets STRONGER the
    /// longer a hypothesis survives, which is the right shape for it.
    [[nodiscard]] double mean_speed_urad_s() const noexcept;

    /// The same displacement as a VECTOR.
    [[nodiscard]] Angle2 mean_velocity_urad_s() const noexcept;

    // -----------------------------------------------------------------------
    // RELATIVE MOTION — the statistic §10.2's policy actually scores
    //
    // How far this track has moved since it started, MINUS how far the static
    // world moved over the same frames. See Tracker::update_common_motion for
    // where the second quantity comes from.
    //
    // Subtracting frame by frame rather than at the end is what makes this
    // usable, and the reason is spec row 23. The engine floors every
    // measurement's R at the pointing uncertainty, which row 23 sets at
    // +/- 20 px per frame — 2,182 urad. A velocity fitted over N frames against
    // that noise has a standard error of sigma * sqrt(12/(N(N^2-1)))/dt, which
    // at N = 20 is 2,542 urad/s: larger than the 2,682 urad/s that separates
    // the beacon from a rock. Waiting it out needs 45 frames, which is 1.5 s of
    // spec row 16's 2 s acquisition budget.
    //
    // But the jitter is COMMON MODE. Every object in a frame is displaced by
    // the same unknown boresight error, so it cancels exactly in the difference
    // between two tracks — and therefore in the difference between one track's
    // frame-to-frame step and the median step of all of them. What survives is
    // the centroiding noise alone, about 12 urad, and the same arithmetic gives
    // a standard error of 30 urad/s after twelve frames.
    //
    // Two orders of magnitude, from choosing where to subtract.
    // -----------------------------------------------------------------------

    /// True if the most recent frame produced an associated measurement.
    ///
    /// The bottom bit of the hit history, which record_hit() shifts in.
    [[nodiscard]] bool last_frame_hit() const noexcept { return (window_ & 1u) != 0u; }

    /// How far the estimate moved since the last call to accumulate_residual().
    [[nodiscard]] Angle2 frame_delta() const noexcept;

    /// Fold this frame into the residual, given the displacement the static
    /// world underwent. Called once per frame by the Tracker, on every live
    /// track, after all updates.
    ///
    /// A frame with no associated measurement is SKIPPED, and that is not an
    /// optimisation. On such a frame the estimate moves by the filter's own
    /// prediction, so folding it in would add the track's current velocity
    /// belief to the evidence for that belief — a positive feedback loop.
    /// Measured before this rule existed: a weak track that picked up one
    /// spurious fast step kept confirming it, reached the top of the priority
    /// score on the motion term alone, and took the mount.
    ///
    /// A frame on which the ego-motion could not be MEASURED is skipped for
    /// the same reason. The tracker needs several live tracks to form the
    /// median that cancels spec rows 23 and 25; with fewer, there is no way to
    /// tell a target's motion from the platform's, and guessing with a stale
    /// estimate is worse than not answering. This matters most immediately
    /// after a lock, when the camera has centred one object and every other
    /// candidate has left the field of view — exactly the moment a bad lock
    /// needs to be detectable.
    void accumulate_residual(Angle2 common_delta, bool common_valid) noexcept;

    /// The residual displacement as a velocity, over a SLIDING WINDOW of the
    /// most recent kResidWindow measured frames.
    ///
    /// A window rather than the whole lifetime, and the reason is spec row 12's
    /// mandatory motion modes. Over one full period of a figure-8 or a circle
    /// the net displacement returns to zero, so a lifetime measure reports a
    /// target on a closed path as stationary — and then the policy that uses it
    /// to tell a beacon from a rock drops the beacon. A window of about a
    /// second sees any of row 12's paths as locally straight.
    [[nodiscard]] Angle2 relative_velocity_urad_s() const noexcept;
    [[nodiscard]] double relative_speed_urad_s() const noexcept {
        return relative_velocity_urad_s().norm();
    }

    /// Frames that actually contributed to the residual: ones with BOTH an
    /// associated measurement and a usable ego-motion estimate. This, not
    /// age_frames(), is how much evidence the motion term rests on, and it is
    /// what the promotion rule is written against.
    [[nodiscard]] int relative_frames() const noexcept { return resid_n_; }

    [[nodiscard]] int  age_frames()          const noexcept { return age_; }
    [[nodiscard]] int  hits()                const noexcept { return hits_; }
    [[nodiscard]] int  consecutive_misses()  const noexcept { return misses_; }
    [[nodiscard]] float mean_snr()           const noexcept { return mean_snr_; }
    /// The running normalised innovation squared. 2 for a healthy track; the
    /// gate's 9.21 for one that is being fed by whatever falls inside it.
    [[nodiscard]] float nis_ema()            const noexcept { return nis_ema_; }
    [[nodiscard]] double last_sigma_urad()   const noexcept { return last_sigma_; }

    /// Hits within the last `quality_window` frames, as a fraction. The
    /// lifetime ratio below is the wrong statistic for quality management: a
    /// track with a thousand good frames behind it would take another thousand
    /// bad ones to fall below any threshold.
    [[nodiscard]] float recent_hit_ratio() const noexcept;

    /// hits / age. §10.2's priority_score reads it as `hit_ratio`, and it is
    /// also the per-track form of §13.1's lock_retention_rate.
    [[nodiscard]] float hit_ratio() const noexcept {
        return age_ > 0 ? static_cast<float>(hits_) / static_cast<float>(age_) : 0.0f;
    }

private:
    /// R for this measurement, honouring adaptive_r and the clamps.
    [[nodiscard]] double sigma_for(const Measurement& m) const noexcept;

    void record_hit(bool hit) noexcept;
    [[nodiscard]] int  hits_in_window() const noexcept;
    [[nodiscard]] bool quality_failed() const noexcept;

    KalmanFilter kf_{};
    ImmFilter    imm_{};
    TrackParams  p_{};
    TrackState   state_ = TrackState::Deleted;

    int      age_    = 0;      ///< frames since start, hits and misses alike
    int      hits_   = 0;      ///< total updates applied
    int      misses_ = 0;      ///< CONSECUTIVE misses; reset by any hit
    uint32_t window_ = 0;      ///< bitmask of the last 32 frames, 1 = hit
    int64_t  start_frame_ = 0;
    Angle2   start_angle_{};   ///< for mean_speed_urad_s(); see there
    Angle2   prev_pos_{};      ///< last frame's estimate, for frame_delta()
    Angle2   resid_{};         ///< accumulated ego-motion-compensated travel
    int      resid_n_ = 0;     ///< MEASURED frames folded into resid_

    /// Ring of the cumulative residual, so a sliding-window difference is a
    /// subtraction rather than a re-scan. 32 frames is a little over a second
    /// at 30 Hz, which is short against every one of spec row 12's periods and
    /// long enough for the noise to average down.
    static constexpr int kResidWindow = 32;
    std::array<Angle2, kResidWindow> resid_hist_{};

    /// 30-sample MotionNet ring: [az, el, vaz, vel] packed, oldest overwritten.
    std::array<float, kHistory * 4> hist_{};
    int hist_count_ = 0;
    int hist_head_  = 0;   ///< next write index

    float  mean_snr_   = 0.0f;
    float  nis_ema_    = 2.0f;   ///< starts at the value a healthy track has
    double last_sigma_ = 0.0;
    double dt_         = 1.0 / 30.0;   ///< frame interval, for reach_urad()
};

// ---------------------------------------------------------------------------
// Tracker — §10.2's TargetPolicy::Priority.
//
// ONE track drives the mount, always. Every graded metric in §3.2 is defined
// against a single lock, the controller aims at one thing, and centroid.csv has
// one row per frame. That has not changed.
//
// What HAS changed is how that one track is chosen. It used to be "the
// strongest candidate in the first frame that had one", which on the
// specification's own scenario picks a clutter source roughly every time — see
// tracking/priority.hpp for the measurement. So alongside the committed track
// the tracker now maintains a small fixed set of HYPOTHESES: ordinary Tracks,
// fed by the detections the committed track did not take, each accumulating its
// own age, hit history and — the term that matters — its own motion.
//
// Nothing is promoted until it has both earned M-of-N confirmation and lived
// long enough for its motion estimate to mean something, and a committed track
// is only displaced under §10.2's stated hysteresis: a challenger must beat it
// by 1.25x, sustained for fifteen frames.
//
// What it does each frame, in §6.2's order:
//
//   B17  predict, then gate every measurement against the prediction
//   B18  associate — nearest neighbour by the likelihood score
//   B19  update, or miss
//   B20  lifecycle transition (inside Track)
//   B20a the same for every hypothesis, on what is left over
//   B20b score, then promote or switch under hysteresis
//
// INV-4: the hypothesis array is a fixed-size member. Nothing here allocates.
// INV-3: every loop is over a fixed index order and every tie is broken by
// slot index, so two identical runs make identical decisions.
// ---------------------------------------------------------------------------
class Tracker {
public:
    /// How many hypotheses are carried at once.
    ///
    /// Eight rather than the 24 candidates perception can return, because a
    /// hypothesis only exists to answer "is this the beacon" and the answer
    /// arrives within a second. Sized from what the detector actually produces
    /// after the SNR gate on the worst configured scenario — 120 clutter
    /// sources, of which about a dozen are in view and perhaps five survive
    /// gating on any given frame.
    static constexpr int kMaxHypotheses = 8;

    void reset(const TrackParams& p) noexcept;

    /// One frame. `meas` is modified: the associated element is flagged, so the
    /// caller can tell which candidates went unexplained — §10.5's search grid
    /// treats those as evidence rather than noise.
    ///
    /// Returns the index of the associated measurement, or -1.
    int step(double dt, std::span<Measurement> meas, int64_t frame) noexcept;

    [[nodiscard]] const Track& track() const noexcept { return track_; }
    [[nodiscard]] Track&       track()       noexcept { return track_; }

    // -----------------------------------------------------------------------
    // What the MODE FSM is told, which is not quite what track() says.
    //
    // §10.2's policy means a candidate now spends its first frames as a
    // hypothesis, invisible in track(). The FSM reads track_state to decide
    // between Search, Acquire and Track, so with hypotheses invisible it stayed
    // in Search for an extra frame after the first detection — and measured on
    // CP 10.2's 375 px slew, engaging one frame later turned 7.50 px of
    // transient overshoot into 11.24 px.
    //
    // A live hypothesis is exactly what the FSM's Acquire state means: a
    // candidate is being followed up and there is no lock yet. So that is what
    // it is told. Capped at Tentative deliberately — a CONFIRMED hypothesis
    // that the policy has not promoted must not make the FSM stop searching,
    // because not promoting it is the policy saying it does not believe it.
    // -----------------------------------------------------------------------
    [[nodiscard]] TrackState fsm_state() const noexcept {
        if (track_.alive()) return track_.state();
        for (const Track& h : hyp_) if (h.alive()) return TrackState::Tentative;
        return TrackState::Deleted;
    }

    [[nodiscard]] bool has_track()  const noexcept {
        if (track_.alive()) return true;
        for (const Track& h : hyp_) if (h.alive()) return true;
        return false;
    }
    [[nodiscard]] bool has_lock()   const noexcept { return track_.drivable(); }

    /// Gated but unassociated this frame. Diagnostic, and the seed of the
    /// multi-target extension.
    [[nodiscard]] int gated_count() const noexcept { return gated_; }

    [[nodiscard]] const TrackParams& params() const noexcept { return p_; }
    [[nodiscard]] TrackParams& params() noexcept { return p_; }

    /// Force the track away — used when the mode FSM decides the lock is bad.
    ///
    /// The hypotheses go too. They were fed by the same frames that produced
    /// the track the FSM has just rejected, and promoting one of them
    /// immediately afterwards would make the drop a no-op.
    void drop() noexcept;

    // --- §10.2's priority policy ------------------------------------------

    /// Context for the score: where the camera is pointing and how fast the
    /// target can move. Set by the engine each frame before step().
    void set_priority_context(const PriorityContext& c) noexcept { ctx_ = c; }
    [[nodiscard]] const PriorityWeights& weights() const noexcept { return w_; }
    [[nodiscard]] PriorityWeights& weights() noexcept { return w_; }

    /// Live hypotheses, for the GUI panel, the trace and the tests.
    [[nodiscard]] int hypothesis_count() const noexcept;
    [[nodiscard]] const Track& hypothesis(int i) const noexcept { return hyp_[static_cast<size_t>(i)]; }
    /// The priority score a given hypothesis currently has, under the same
    /// context the policy used this frame. For the GUI panel and the tests.
    [[nodiscard]] float hypothesis_score(int i) const noexcept;
    [[nodiscard]] const PriorityContext& context() const noexcept { return ctx_; }

    /// The committed track's own priority score, and the best challenger's.
    /// Both are logged, because "why did it switch" is the first question
    /// anyone asks of a policy like this.
    [[nodiscard]] float committed_score() const noexcept { return committed_score_; }
    [[nodiscard]] float best_rival_score() const noexcept { return best_rival_score_; }
    /// Consecutive frames the best challenger has been ahead by switch_ratio.
    [[nodiscard]] int   switch_pressure() const noexcept { return switch_count_; }

    /// The apparent velocity the static world currently has — spec row 25's
    /// platform motion, as the tracker sees it. Exposed because it is a real
    /// physical estimate the GUI can show and a test can check against the
    /// scenario's configured platform rate.
    [[nodiscard]] Angle2 common_velocity() const noexcept { return common_v_; }
    [[nodiscard]] bool   common_velocity_valid() const noexcept { return common_valid_; }
    /// Frames on which the committed track was replaced by a challenger.
    [[nodiscard]] int64_t switches() const noexcept { return switches_; }
    /// Times the committed track was dropped for failing to keep its score.
    [[nodiscard]] int64_t drops() const noexcept { return drops_; }

private:
    /// Associate the best gated, unclaimed measurement to `t`, or miss.
    /// `claimed` is a bitmask of indices already taken this frame. Returns the
    /// index taken, or -1.
    int associate(Track& t, std::span<Measurement> meas, uint64_t claimed,
                  bool count_gated) noexcept;
    /// B20a/B20b.
    void run_hypotheses(double dt, std::span<Measurement> meas, int64_t frame,
                        uint64_t& claimed) noexcept;
    void promote_or_switch() noexcept;
    /// Re-estimate what the static world did this frame, and fold it out of
    /// every live track's residual. See the definition for why it is a median
    /// and why the subtraction happens per frame.
    void update_common_motion(double dt) noexcept;

    TrackParams     p_{};
    Track           track_{};
    int             gated_ = 0;

    PriorityWeights w_{};
    PriorityContext ctx_{};
    std::array<Track, kMaxHypotheses> hyp_{};
    int             switch_count_     = 0;
    int             best_rival_slot_  = -1;
    float           committed_score_  = 0.0f;
    float           best_rival_score_ = 0.0f;
    int64_t         switches_         = 0;
    int             below_count_      = 0;   ///< frames the lock has scored badly
    int64_t         drops_            = 0;

    Angle2          common_v_{};          ///< apparent velocity of the static world
    bool            common_valid_ = false;

    /// A track must be this old before its velocity joins the median. Below it
    /// the estimate is mostly spec row 23's jitter.
    static constexpr int kCommonMinAge    = 8;
    /// And there must be this many of them, or a median means nothing.
    static constexpr int kCommonMinTracks = 3;
    /// EMA time constant, frames. The platform rate is a smooth physical
    /// quantity (row 25's components are all continuous), so the estimate is
    /// smoothed rather than taken raw from each frame's median.
    static constexpr double kCommonTau    = 20.0;
};

}  // namespace sat
