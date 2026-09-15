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
}

// ---------------------------------------------------------------------------
// Tracker
// ---------------------------------------------------------------------------

void Tracker::reset(const TrackParams& p) noexcept {
    p_      = p;
    track_  = Track{};
    gated_  = 0;
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
    if (!track_.alive()) {
        // No track. The strongest candidate seeds a new Tentative one; the
        // gate does not apply because there is nothing to gate against, which
        // is exactly why Tentative exists and why a Tentative track is not
        // allowed to drive the mount.
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
    return best;
}

}  // namespace sat
