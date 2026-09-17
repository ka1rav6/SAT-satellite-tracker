// tracking/track.cpp

#include "tracking/track.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>

namespace sat {

const char* track_state_name(TrackState s) noexcept {
    switch (s) {
        case TrackState::Tentative: return "Tentative";
        case TrackState::Confirmed: return "Confirmed";
        case TrackState::Coasting:  return "Coasting";
        case TrackState::Deleted:   return "Deleted";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// Track
// ---------------------------------------------------------------------------

double Track::sigma_for(const Measurement& m) const noexcept {
    const double s = p_.adaptive_r ? m.sigma_urad : p_.fixed_r_sigma_urad;
    return std::clamp(s, p_.min_r_sigma_urad, p_.max_r_sigma_urad);
}

void Track::start(const Measurement& m, const TrackParams& p, int64_t frame) noexcept {
    p_           = p;
    state_       = TrackState::Tentative;
    age_         = 1;
    hits_        = 1;
    misses_      = 0;
    window_      = 1u;                 // frame 0 of the window was a hit
    start_frame_ = frame;
    start_angle_ = m.angle;
    prev_pos_    = m.angle;
    resid_       = Angle2{};
    resid_n_     = 0;
    resid_hist_.fill(Angle2{});
    nis_ema_     = 2.0f;
    mean_snr_    = m.snr;
    last_sigma_  = sigma_for(m);

    Measurement seed = m;
    seed.sigma_urad  = last_sigma_;
    if (p_.imm) imm_.init(seed, p_.imm_params);
    else        kf_.init(seed, p_.kf);
}

void Track::predict(double dt) noexcept {
    if (state_ == TrackState::Deleted) return;
    dt_ = dt;
    if (p_.imm) imm_.predict(dt);
    else        kf_.predict(dt);
}

double Track::distance2(const Measurement& m) const noexcept {
    if (state_ == TrackState::Deleted) return std::numeric_limits<double>::max();
    const double sigma = sigma_for(m);
    return p_.imm ? imm_.mahalanobis2(m.angle, sigma)
                  : kf_.mahalanobis2(m.angle, sigma);
}

double Track::reach_urad() const noexcept {
    if (p_.max_target_speed_urad_s <= 0.0) return std::numeric_limits<double>::max();
    // misses_ + 1 frames of travel: the target was last seen one accepted
    // measurement ago, and every miss since then is another frame in which it
    // could have kept moving. dt is the nominal frame interval, which is exact
    // here because the clock is derived (core/time.hpp), never wall-clock.
    const double frames = static_cast<double>(misses_ + 1);
    return p_.max_target_speed_urad_s * frames * dt_ + p_.gate_pad_urad;
}

bool Track::within_reach(const Measurement& m) const noexcept {
    if (p_.max_target_speed_urad_s <= 0.0) return true;
    const Angle2 pos = position();
    const Angle2 d{m.angle.x - pos.x, m.angle.y - pos.y};
    return d.norm() <= reach_urad();
}

double Track::assoc_score(const Measurement& m) const noexcept {
    if (state_ == TrackState::Deleted) return std::numeric_limits<double>::max();
    const double sigma = sigma_for(m);
    const auto   S     = p_.imm ? imm_.innovation_cov(sigma)
                                : kf_.innovation_cov(sigma);
    const double det   = S.determinant();
    // det > 0 for any covariance plus a positive diagonal; the guard is for the
    // pathological case where P has been corrupted, and returning "worst
    // possible score" is the right failure mode — never select this.
    if (!(det > 0.0)) return std::numeric_limits<double>::max();
    return (p_.imm ? imm_.mahalanobis2(m.angle, sigma)
                   : kf_.mahalanobis2(m.angle, sigma)) + std::log(det);
}

Angle2 Track::mean_velocity_urad_s() const noexcept {
    if (age_ <= 1 || dt_ <= 0.0) return Angle2{};
    const Angle2 p = position();
    const double T = static_cast<double>(age_ - 1) * dt_;
    return Angle2{(p.x - start_angle_.x) / T, (p.y - start_angle_.y) / T};
}

double Track::mean_speed_urad_s() const noexcept {
    return mean_velocity_urad_s().norm();
}

Angle2 Track::frame_delta() const noexcept {
    const Angle2 p = position();
    return Angle2{p.x - prev_pos_.x, p.y - prev_pos_.y};
}

void Track::accumulate_residual(Angle2 common_delta, bool common_valid) noexcept {
    // prev_pos_ advances either way, so the next hit measures from HERE and
    // not from across a gap the filter merely predicted through.
    if (last_frame_hit() && common_valid) {
        const Angle2 d = frame_delta();
        resid_.x += d.x - common_delta.x;
        resid_.y += d.y - common_delta.y;
        resid_hist_[static_cast<size_t>(resid_n_ % kResidWindow)] = resid_;
        ++resid_n_;
    }
    prev_pos_ = position();
}

Angle2 Track::relative_velocity_urad_s() const noexcept {
    if (resid_n_ < 2 || dt_ <= 0.0) return Angle2{};
    const int w   = std::min(resid_n_, kResidWindow);
    const int newest = (resid_n_ - 1) % kResidWindow;
    const int oldest = (resid_n_ - w) % kResidWindow;
    const Angle2 a = resid_hist_[static_cast<size_t>(newest)];
    const Angle2 b = resid_hist_[static_cast<size_t>(oldest)];
    const double T = static_cast<double>(w - 1) * dt_;
    return Angle2{(a.x - b.x) / T, (a.y - b.y) / T};
}

float Track::recent_hit_ratio() const noexcept {
    const int n = std::clamp(p_.quality_window, 1, 32);
    // Early in a track's life the window is not full; divide by what has
    // actually happened, or a three-frame-old track would look like a failure.
    const int span = std::min(n, age_);
    if (span <= 0) return 0.0f;
    const uint32_t mask = (span >= 32) ? 0xffffffffu : ((1u << span) - 1u);
    return static_cast<float>(std::popcount(window_ & mask))
         / static_cast<float>(span);
}

// ---------------------------------------------------------------------------
// quality_failed — the two tests described in track.hpp's TrackParams block.
//
// Evaluated for Confirmed and Coasting tracks only, and only once the track has
// lived a full window. A Tentative track is governed by M-of-N and a younger
// track has no history to judge.
// ---------------------------------------------------------------------------
bool Track::quality_failed() const noexcept {
    if (!p_.quality_enabled) return false;
    if (state_ != TrackState::Confirmed && state_ != TrackState::Coasting) return false;
    if (age_ < p_.quality_window) return false;
    if (recent_hit_ratio() < p_.quality_min_ratio) return true;
    if (nis_ema_ > p_.quality_max_nis) return true;
    return false;
}

int Track::hits_in_window() const noexcept {
    // The low `confirm_window` bits of the history, popcounted. A bitmask is
    // used rather than a ring of counters because N is small and fixed, and
    // because it makes "3 hits in the last 5 frames" one instruction instead of
    // a loop that is easy to get off by one.
    const int n = std::clamp(p_.confirm_window, 1, 32);
    const uint32_t mask = (n >= 32) ? 0xffffffffu : ((1u << n) - 1u);
    return std::popcount(window_ & mask);
}

void Track::record_hit(bool hit) noexcept {
    window_ = (window_ << 1) | (hit ? 1u : 0u);
    ++age_;
    if (hit) { ++hits_; misses_ = 0; }
    else     { ++misses_; }
}

void Track::update(const Measurement& m) noexcept {
    if (state_ == TrackState::Deleted) return;

    const double sigma = sigma_for(m);
    if (p_.imm) imm_.update(m.angle, sigma);
    else        kf_.update(m.angle, sigma);
    last_sigma_ = sigma;
    record_hit(true);

    // Track quality: the innovation this update actually saw, smoothed. Read
    // AFTER the filter's update() because that is where the NIS is computed.
    // alpha = 1 - exp(-1/tau), the same EMA form the supervisor uses (§10.6).
    {
        const double nis = p_.imm ? imm_.last_nis() : kf_.last_nis();
        const float  tau = std::max(1.0f, p_.quality_nis_tau);
        const float  a   = 1.0f - std::exp(-1.0f / tau);
        nis_ema_ += a * (static_cast<float>(nis) - nis_ema_);
    }

    // Running mean of SNR, for §10.2's priority score and for the SAT
    // supervisor's Conditions vector at Stage 12.
    mean_snr_ += (m.snr - mean_snr_) / static_cast<float>(hits_);

    // --- transitions ------------------------------------------------------
    switch (state_) {
        case TrackState::Tentative:
            // M of N. Note this is checked on a HIT only: a miss can never
            // confirm a track, so there is no need to re-evaluate it there.
            if (hits_in_window() >= p_.confirm_hits) state_ = TrackState::Confirmed;
            // The timeout is checked on a hit as well as on a miss, because a
            // track that has had hits but never three inside the window is
            // still a failure to confirm. Confirmation is evaluated FIRST, so
            // a track that qualifies on its fifth frame survives — the window
            // and the lifetime are both 5, so a Tentative track gets exactly
            // the five frames §10.2 grants it, no more and no fewer.
            else if (age_ >= p_.confirm_window) state_ = TrackState::Deleted;
            break;

        case TrackState::Coasting:
            // "any hit" -> Confirmed, straight back. Deliberately not "M of N
            // again": the track already earned confirmation once and still has
            // a velocity estimate, and making it re-qualify would cost several
            // frames of the 1 s re-acquisition budget (spec row 19) for no
            // gain. The gate is what protects this transition — a hit that
            // reaches here has already passed a Mahalanobis test against a
            // prediction, which a Tentative track's first detection never did.
            state_ = TrackState::Confirmed;
            break;

        case TrackState::Confirmed:
        case TrackState::Deleted:
            break;
    }

    // A hit does not exempt a track from the quality tests. It is exactly the
    // alternating hit/miss pattern that the consecutive-miss counter cannot
    // see, so the check has to be here as well as in miss().
    if (quality_failed()) state_ = TrackState::Deleted;
}

void Track::miss() noexcept {
    if (state_ == TrackState::Deleted) return;
    record_hit(false);

    switch (state_) {
        case TrackState::Tentative:
            // "5 frames, no confirm -> Deleted". A tentative track gets exactly
            // its window to prove itself. Note the check is on age, not on
            // consecutive misses: three hits and two misses that never line up
            // inside the window is still a failure to confirm.
            if (age_ >= p_.confirm_window) state_ = TrackState::Deleted;
            break;

        case TrackState::Confirmed:
            state_ = TrackState::Coasting;
            break;

        case TrackState::Coasting:
            if (misses_ >= p_.coast_max_misses) state_ = TrackState::Deleted;
            break;

        case TrackState::Deleted:
            break;
    }

    if (quality_failed()) state_ = TrackState::Deleted;
}

// ---------------------------------------------------------------------------
// Tracker
// ---------------------------------------------------------------------------

void Tracker::reset(const TrackParams& p) noexcept {
    p_      = p;
    track_  = Track{};
    gated_  = 0;
    for (Track& h : hyp_) h = Track{};
    switch_count_     = 0;
    best_rival_slot_  = -1;
    committed_score_  = 0.0f;
    best_rival_score_ = 0.0f;
    switches_         = 0;
    below_count_      = 0;
    drops_            = 0;
    common_v_         = Angle2{};
    common_valid_     = false;
}

void Tracker::drop() noexcept {
    track_ = Track{};
    for (Track& h : hyp_) h = Track{};
    switch_count_    = 0;
    below_count_     = 0;
    best_rival_slot_ = -1;
}

float Tracker::hypothesis_score(int i) const noexcept {
    if (i < 0 || i >= kMaxHypotheses) return 0.0f;
    return priority_score(hyp_[static_cast<size_t>(i)], ctx_, w_);
}

int Tracker::hypothesis_count() const noexcept {
    int n = 0;
    for (const Track& h : hyp_) if (h.alive()) ++n;
    return n;
}

// ---------------------------------------------------------------------------
// associate — B18 for one track, over the measurements nobody has taken yet.
//
// Gating on d^2, selection on the likelihood score; see the long note in
// Tracker::step for why those are different metrics.
// ---------------------------------------------------------------------------
int Tracker::associate(Track& t, std::span<Measurement> meas,
                       uint64_t claimed, bool count_gated) noexcept {
    int    best       = -1;
    double best_score = std::numeric_limits<double>::max();
    for (size_t i = 0; i < meas.size(); ++i) {
        if (i < 64 && (claimed & (1ull << i)) != 0ull) continue;
        if (!t.gates(meas[i])) continue;
        // gated_count() is the COMMITTED track's statistic — "how many
        // candidates were plausible this frame" — and it is reported. A
        // hypothesis gating a measurement is internal.
        if (count_gated) ++gated_;
        const double score = t.assoc_score(meas[i]);
        // Strictly less-than, walking the measurements in index order, so a
        // tie always resolves to the lower index. INV-3.
        if (score < best_score) { best_score = score; best = static_cast<int>(i); }
    }
    return best;
}

// ---------------------------------------------------------------------------
// B20a — the hypotheses.
//
// Order matters and is fixed: the committed track has already taken its
// measurement before this runs, then hypotheses take theirs in SLOT order, then
// whatever is left seeds new hypotheses strongest-first. Every step is a
// deterministic function of the measurement array, so INV-3 holds without any
// special care beyond writing the loops this way.
// ---------------------------------------------------------------------------
void Tracker::run_hypotheses(double dt, std::span<Measurement> meas,
                             int64_t frame, uint64_t& claimed) noexcept {
    // -----------------------------------------------------------------------
    // `claimed` is private to this function; Measurement::associated is not.
    //
    // The distinction matters because `associated` is part of the tracker's
    // OUTPUT contract: §10.5's search grid reads it to find the candidates
    // nothing explained, and treats those as evidence about where the target is
    // not. A hypothesis taking a measurement is an internal bookkeeping fact
    // and must not be allowed to tell the search grid that a clutter source has
    // been explained. So hypotheses claim a slot without flagging it, and only
    // the committed track sets `associated`.
    // -----------------------------------------------------------------------
    for (Track& h : hyp_) {
        if (!h.alive()) continue;
        h.predict(dt);
        const int idx = associate(h, meas, claimed, /*count_gated=*/false);
        if (idx >= 0) {
            if (idx < 64) claimed |= (1ull << idx);
            h.update(meas[static_cast<size_t>(idx)]);
        } else {
            h.miss();
        }
    }

    // Seed new ones from what is left, strongest first. A measurement that no
    // track explained is either a new object or a false alarm, and five frames
    // of M-of-N is the cheapest way to find out which.
    for (;;) {
        int slot = -1;
        for (int i = 0; i < kMaxHypotheses; ++i) {
            if (!hyp_[static_cast<size_t>(i)].alive()) { slot = i; break; }
        }
        if (slot < 0) break;                       // all slots busy

        int best = -1;
        for (size_t i = 0; i < meas.size(); ++i) {
            if (i < 64 && (claimed & (1ull << i)) != 0ull) continue;
            if (best < 0 || meas[i].snr > meas[static_cast<size_t>(best)].snr) {
                best = static_cast<int>(i);
            }
        }
        if (best < 0) break;                       // nothing left to seed with

        // -------------------------------------------------------------------
        // Hypotheses run the SAME filter the committed track does, IMM
        // included.
        //
        // The first version did not: it forced `imm = false` on the argument
        // that a hypothesis only has to answer "is this thing moving at all"
        // and that eight six-state, three-mode filters is a lot of covariance
        // to propagate for that question. The second half of that is true and
        // the first half is irrelevant, because promotion MOVES the hypothesis
        // into the committed slot — so the mount ended up being driven by a
        // constant-velocity filter on a run configured for the IMM, and CP
        // 10.5's figure-8 comparison reported the two arms as bit-identical
        // with all three mode probabilities pinned at zero. That is exactly the
        // kind of defect that looks like "the IMM does not help".
        //
        // The cost is real and small: the tracking stage measures 2.4 us, and
        // eight extra IMMs add about 20 us against design §15's 850 us frame.
        // -------------------------------------------------------------------
        if (best < 64) claimed |= (1ull << best);
        hyp_[static_cast<size_t>(slot)].start(meas[static_cast<size_t>(best)], p_, frame);
    }
}

// ---------------------------------------------------------------------------
// update_common_motion — what the static world did this frame, and the
// residual each track has left after it is taken away.
//
// Every object at a fixed world angle is reported by this tracker at
// theta - disturbance(t), because the conversion from pixels to angles goes
// through the COMMANDED boresight and spec rows 23 and 25 both move the true
// one. So every static source takes the SAME step every frame, and in a field
// of 120 clutter sources (design §9.1) most of the live tracks are static
// sources.
//
// MEDIAN, not mean, and that is the whole robustness argument. A mean is
// dragged by the beacon and by any track following noise; a component-wise
// median over the live tracks is unaffected by a minority of outliers however
// extreme — which is exactly the situation here, because the one track that
// matters is the one that disagrees.
//
// When there are too few tracks to form a median, the previous estimate is
// used rather than zero. The platform rate is a smooth physical quantity
// (row 25's components are all continuous) and the last estimate is a far
// better guess than nothing; zeroing it would make the motion term revert to
// raw apparent speed on precisely the frames where the tracker is down to one
// hypothesis and most needs to be right.
// ---------------------------------------------------------------------------
void Tracker::update_common_motion(double dt) noexcept {
    // kMaxHypotheses + 1 for the committed track. Fixed size, on the stack:
    // INV-4.
    double dx[kMaxHypotheses + 1];
    double dy[kMaxHypotheses + 1];
    int    n = 0;

    auto consider = [&](const Track& t) {
        // age >= 2: a track seeded this frame has not stepped yet, and its
        // "delta" would be the difference between the seed and itself.
        if (!t.alive() || t.age_frames() < 2) return;
        // And only frames with a real measurement. A coasting track's step is
        // its own prediction, which says nothing about what the world did.
        if (!t.last_frame_hit()) return;
        const Angle2 d = t.frame_delta();
        dx[n] = d.x;
        dy[n] = d.y;
        ++n;
    };
    consider(track_);
    for (const Track& h : hyp_) consider(h);

    Angle2 step{};
    if (n >= kCommonMinTracks) {
        // Component-wise median. n is at most nine, so an insertion sort is
        // both the fastest thing available and the easiest to be sure is right.
        auto median = [](double* a, int count) {
            for (int i = 1; i < count; ++i) {
                const double key = a[i];
                int j = i - 1;
                while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; --j; }
                a[j + 1] = key;
            }
            return (count % 2) ? a[count / 2]
                               : 0.5 * (a[count / 2 - 1] + a[count / 2]);
        };
        step = Angle2{median(dx, n), median(dy, n)};
        // Reported as a velocity, for the GUI and for the test that checks it
        // against the scenario's configured platform rate. Smoothed, because
        // what it is estimating is smooth.
        const Angle2 v{dt > 0.0 ? step.x / dt : 0.0, dt > 0.0 ? step.y / dt : 0.0};
        if (!common_valid_) {
            common_v_     = v;
            common_valid_ = true;
        } else {
            const double a = 1.0 - std::exp(-1.0 / kCommonTau);
            common_v_.x += a * (v.x - common_v_.x);
            common_v_.y += a * (v.y - common_v_.y);
        }
    } else if (common_valid_) {
        step = Angle2{common_v_.x * dt, common_v_.y * dt};
    }

    // Fold the frame into every live track's residual. Note this uses the
    // frame's OWN median, not the smoothed velocity: the jitter it is
    // cancelling is white and per-frame, so smoothing it first would leave
    // exactly the term that needs to go.
    const bool measured = (n >= kCommonMinTracks);
    if (track_.alive()) track_.accumulate_residual(step, measured);
    for (Track& h : hyp_) if (h.alive()) h.accumulate_residual(step, measured);
}

// ---------------------------------------------------------------------------
// B20b — promotion and switching, under §10.2's hysteresis.
// ---------------------------------------------------------------------------
void Tracker::promote_or_switch() noexcept {

    committed_score_ = track_.alive() ? priority_score(track_, ctx_, w_) : 0.0f;

    // -----------------------------------------------------------------------
    // "Lone candidate": exactly one confirmed hypothesis, and no committed
    // track. The motion term exists to DISCRIMINATE, and with one candidate
    // there is nothing to discriminate against — the choice is "this or
    // nothing". So the evidence requirement and the score threshold are both
    // waived, and the only question left is the one M-of-N already answers:
    // is this a real detection.
    //
    // Two things make waiving them safe rather than a hole:
    //
    //   the drop rule still applies afterwards. If the lone candidate turns
    //   out to be a rock, its score is measurable as soon as anything else
    //   appears, and it is dropped;
    //
    //   the cases the threshold exists for are not lone. A field of 120
    //   clutter sources produces several confirmed hypotheses at once, which
    //   is exactly when the score has both the evidence and the reason to
    //   choose between them.
    //
    // It is also what CP 6.7's re-acquisition budget needs. Spec row 19 allows
    // one second; waiting thirty frames for motion evidence on a beacon that
    // has just reappeared alone in the frame would spend half of it proving
    // something nothing disagreed with.
    // -----------------------------------------------------------------------
    int confirmed_hyps = 0;
    for (const Track& h : hyp_) {
        if (h.alive() && h.state() == TrackState::Confirmed) ++confirmed_hyps;
    }
    const bool lone = (confirmed_hyps == 1) && !track_.alive();

    // The best CONFIRMED, old-enough challenger. Slot order breaks ties.
    int   best_slot  = -1;
    float best_score = 0.0f;
    for (int i = 0; i < kMaxHypotheses; ++i) {
        const Track& h = hyp_[static_cast<size_t>(i)];
        if (!h.alive()) continue;
        if (h.state() != TrackState::Confirmed) continue;
        const float sc = priority_score(h, ctx_, w_);
        if (!lone) {
            // Evidence, not age: promote_min_age counts frames that actually
            // contributed to the motion estimate. See Track::relative_frames().
            if (h.relative_frames() < w_.promote_min_age) continue;
            // A candidate that does not look like a beacon is not promoted.
            // See PriorityWeights::min_commit_score for the two numbers this
            // sits between and for the 30-second run that made it necessary.
            if (sc < w_.min_commit_score) continue;
        }
        if (best_slot < 0 || sc > best_score) { best_slot = i; best_score = sc; }
    }
    best_rival_slot_  = best_slot;
    best_rival_score_ = best_slot >= 0 ? best_score : 0.0f;

    // -----------------------------------------------------------------------
    // The committed track has to keep earning the mount. See
    // PriorityWeights::drop_frames for why a promotion threshold alone is not
    // enough: once anything is committed, nothing is left in the field of view
    // to challenge it.
    // -----------------------------------------------------------------------
    if (track_.alive() && w_.drop_frames > 0) {
        // Only once the score MEANS something. A track whose motion evidence
        // has not accumulated — because nothing else is in the frame to
        // measure the ego-motion against — scores low for a reason that says
        // nothing about the track, and dropping it would be dropping the only
        // thing the system can see.
        const bool judgeable = track_.relative_frames() >= w_.promote_min_age;
        if (judgeable && committed_score_ < w_.min_commit_score) {
            ++below_count_;
            if (below_count_ >= w_.drop_frames) {
                track_       = Track{};
                below_count_ = 0;
                switch_count_ = 0;
                ++drops_;
                committed_score_ = 0.0f;
            }
        } else {
            below_count_ = 0;
        }
    }

    if (best_slot < 0) { switch_count_ = 0; return; }

    // Nothing committed: take the best challenger straight away. There is no
    // hysteresis to apply, because there is nothing to be hysteretic about.
    if (!track_.alive()) {
        track_ = hyp_[static_cast<size_t>(best_slot)];
        hyp_[static_cast<size_t>(best_slot)] = Track{};
        switch_count_ = 0;
        return;
    }

    // -----------------------------------------------------------------------
    // §10.2: "Switch ONLY if score_new > 1.25 x score_current, sustained 15
    // frames." The design calls the hysteresis mandatory and says why: without
    // it two similar targets make the camera oscillate. It is also what stops
    // this policy from being worse than no policy at all — a challenger that
    // beats the incumbent for a single frame because of one bright detection
    // must not be able to take the mount.
    // -----------------------------------------------------------------------
    if (best_rival_score_ > w_.switch_ratio * committed_score_) {
        ++switch_count_;
    } else {
        switch_count_ = 0;
    }

    if (switch_count_ >= w_.switch_frames) {
        // Swap rather than discard: the track being displaced is still a real
        // object with a real filter, and if the new choice turns out to be
        // wrong it is the best challenger the next fifteen frames will have.
        Track displaced = track_;
        track_ = hyp_[static_cast<size_t>(best_slot)];
        hyp_[static_cast<size_t>(best_slot)] = displaced;
        switch_count_ = 0;
        ++switches_;
    }
}

int Tracker::step(double dt, std::span<Measurement> meas, int64_t frame) noexcept {
    gated_ = 0;
    for (Measurement& m : meas) m.associated = false;

    // -----------------------------------------------------------------------
    // B17 — predict first, ALWAYS.
    //
    // The gate compares measurements against where the target is now, not
    // where it was last frame. At 240 px/s and 33 ms that is 8 px of
    // difference, which against a gate of a few pixels is the whole budget:
    // gating on the stale estimate would reject the true detection on every
    // fast target and the tracker would only work on slow ones.
    // -----------------------------------------------------------------------
    // -----------------------------------------------------------------------
    // No committed track. This used to read:
    //
    //     "No track. The strongest candidate seeds a new Tentative one."
    //
    // and that one line is what made the tracker lock onto a clutter source on
    // almost every run of the specification's own default scenario. See
    // tracking/priority.hpp for the measurement and for what replaced it: the
    // candidates all become hypotheses, and the one that earns the mount is
    // chosen by a score that includes whether it is MOVING.
    //
    // Nothing is committed here. run_hypotheses() seeds, promote_or_switch()
    // promotes, and until one of them is promoted the FSM keeps searching —
    // which is the correct behaviour and was not happening before.
    // -----------------------------------------------------------------------
    // -----------------------------------------------------------------------
    // The ablation arm. With the policy off this is the behaviour that existed
    // before §10.2 was implemented: the strongest candidate in the first frame
    // that has one becomes the track, with no hypotheses and no score. It is
    // kept runnable because docs/RESULTS.md measures the policy against it, and
    // a claim about an improvement needs both arms in the same binary.
    // -----------------------------------------------------------------------
    if (!w_.enabled) {
        if (!track_.alive()) {
            if (meas.empty()) return -1;
            int best = 0;
            for (size_t i = 1; i < meas.size(); ++i) {
                if (meas[i].snr > meas[best].snr) best = static_cast<int>(i);
            }
            meas[static_cast<size_t>(best)].associated = true;
            track_.start(meas[static_cast<size_t>(best)], p_, frame);
            return best;
        }
        track_.predict(dt);
        const int idx = associate(track_, meas, 0ull, /*count_gated=*/true);
        if (idx >= 0) {
            meas[static_cast<size_t>(idx)].associated = true;
            track_.update(meas[static_cast<size_t>(idx)]);
        } else {
            track_.miss();
        }
        return idx;
    }

    if (!track_.alive()) {
        uint64_t claimed = 0ull;
        run_hypotheses(dt, meas, frame, claimed);
        update_common_motion(dt);
        promote_or_switch();
        return -1;
    }

    track_.predict(dt);

    // -----------------------------------------------------------------------
    // B18 — gate on d^2, then associate on the likelihood score.
    //
    // A CORRECTION TO AN EARLIER COMMENT IN THIS FILE, recorded because the
    // reasoning was wrong rather than merely incomplete. It said that the
    // Mahalanobis metric earns its place because the prediction is uncertain
    // along the velocity more than across it, so the gate is an ellipse aligned
    // with the motion. That is false for THIS filter: the CWNA model applies
    // the same q to azimuth and elevation and R is sigma^2 * I, so the two axes
    // are exactly independent and the predicted position covariance is
    // ISOTROPIC — P(0,0) == P(1,1) and P(0,1) == 0, at every step. A test
    // asserting the anisotropy passed only on the last bits of floating point,
    // which is how it was caught.
    //
    // The gate really is a circle here. Anisotropy arrives at CP 10.5, when the
    // coordinate-turn model couples the axes; the full 4x4 form is kept for
    // that reason (see kalman.hpp) and this code needs no change when it does.
    //
    // What the statistical metric buys TODAY is different and is still worth
    // having: the gate's radius is not fixed. It tightens as the filter settles
    // and widens on its own while coasting, so a chi-square threshold means the
    // same thing — "1% of true detections rejected" — in both regimes, which a
    // fixed angular radius cannot.
    //
    // Selection then uses assoc_score rather than d^2; see track.hpp for why
    // d^2 alone picks the worst candidate in the frame once R is adaptive.
    // -----------------------------------------------------------------------
    int    best       = -1;
    double best_score = std::numeric_limits<double>::max();
    for (size_t i = 0; i < meas.size(); ++i) {
        if (track_.gates(meas[i])) {
            ++gated_;
            const double score = track_.assoc_score(meas[i]);
            if (score < best_score) { best_score = score; best = static_cast<int>(i); }
        }
    }

    if (best >= 0) {
        meas[static_cast<size_t>(best)].associated = true;
        track_.update(meas[static_cast<size_t>(best)]);
    } else {
        track_.miss();
    }

    // B20a/B20b. The hypotheses see everything the committed track did not
    // take, which is the point: the beacon sitting unexplained two hundred
    // pixels away from a clutter lock is precisely the measurement that has to
    // grow into a challenger.
    // The committed track's pick is already flagged; hypotheses may not take it.
    uint64_t claimed = (best >= 0 && best < 64) ? (1ull << best) : 0ull;
    run_hypotheses(dt, meas, frame, claimed);
    update_common_motion(dt);
    promote_or_switch();
    return best;
}

}  // namespace sat
