// control/mode_fsm.cpp

#include "control/mode_fsm.hpp"

#include <cmath>

namespace sat {

void ModeFsm::reset(const ModeFsmParams& p) noexcept {
    p_              = p;
    mode_           = TrackMode::Idle;
    log_            = Log{};
    changed_        = false;
    frames_in_mode_ = 0;
    time_in_mode_   = 0.0;
    last_time_s_    = 0.0;
    last_lock_s_    = 0.0;
    handover_run_   = 0;
    empty_frames_   = 0;
    started_        = false;
}

void ModeFsm::go(TrackMode to, const char* reason, int64_t frame, double time_s) noexcept {
    if (to == mode_) return;
    log_.push(ModeTransition{frame, time_s, mode_, to, reason});
    mode_           = to;
    changed_        = true;
    frames_in_mode_ = 0;
    time_in_mode_   = 0.0;
    handover_run_   = 0;
}

TrackMode ModeFsm::step(const ModeFsmInputs& in, int64_t frame, double time_s) noexcept {
    changed_ = false;

    // Elapsed time is taken from the frame timestamp, never from a wall clock
    // (INV-3). On the first frame there is no previous timestamp, so dt is 0.
    const double dt = started_ ? (time_s - last_time_s_) : 0.0;
    last_time_s_ = time_s;

    if (in.candidate_count > 0) empty_frames_ = 0; else ++empty_frames_;

    const bool drivable = in.have_track
                       && (in.track_state == TrackState::Confirmed
                        || in.track_state == TrackState::Coasting);
    if (drivable) last_lock_s_ = time_s;

    // -----------------------------------------------------------------------
    // Idle -> Search, on run start. Everything below assumes the run is going.
    // -----------------------------------------------------------------------
    if (!started_) {
        if (!in.running) return mode_;
        started_     = true;
        last_lock_s_ = time_s;      // the loss timeout starts counting now
        go(TrackMode::Search, "run start", frame, time_s);
        frames_in_mode_ = 1;
        return mode_;
    }

    // -----------------------------------------------------------------------
    // any -> Safe, on the loss timeout.
    //
    // Checked FIRST and before anything else can fire, because §10.4 writes it
    // as "any -> Safe" and an "any" transition that can be pre-empted by a more
    // specific rule is not an "any" transition. The one exception is Safe
    // itself, which has no exit in the diagram — a run that has reached Safe
    // has declared failure, and quietly resuming would hide it from the
    // acquisition metrics in §13.1.
    // -----------------------------------------------------------------------
    if (mode_ != TrackMode::Safe && mode_ != TrackMode::Idle
        && (time_s - last_lock_s_) > p_.loss_timeout_s) {
        go(TrackMode::Safe, "loss timeout exceeded", frame, time_s);
        frames_in_mode_ = 1;
        time_in_mode_   = dt;
        return mode_;
    }

    switch (mode_) {
        case TrackMode::Search:
            // "Search -> Detect: >= 1 gated detection". The gate meant here is
            // §9.4.7's SHAPE gate, not the Mahalanobis one — there is no track
            // to gate against yet, which is precisely what distinguishes Search
            // from everything after it.
            if (in.candidate_count > 0) go(TrackMode::Detect, ">=1 gated detection", frame, time_s);
            break;

        case TrackMode::Detect:
            // "Detect -> Acquire: track reaches Tentative".
            if (in.have_track && in.track_state != TrackState::Deleted) {
                go(TrackMode::Acquire, "track reached Tentative", frame, time_s);
            } else if (empty_frames_ >= p_.detect_timeout_frames) {
                // Not in §10.4's table, and added deliberately: Detect means
                // "candidates exist but no track yet", and if the candidates
                // stop existing there is nothing left to acquire. Without this
                // the FSM would sit in Detect forever after a one-frame noise
                // blob and the search pattern would never resume, which is a
                // silent failure to acquire — the thing spec row 16 grades.
                go(TrackMode::Search, "no candidates", frame, time_s);
            }
            break;

        case TrackMode::Acquire:
            // "Acquire -> Track: track reaches Confirmed".
            if (in.have_track && in.track_state == TrackState::Confirmed) {
                go(TrackMode::Track, "track reached Confirmed", frame, time_s);
            } else if (!in.have_track || in.track_state == TrackState::Deleted) {
                // The M-of-N window expired without confirming. §10.4 does not
                // draw this arrow because its table stops at the happy path,
                // but TrackState::Deleted is reachable from Tentative by
                // construction (tracking/track.cpp), so the mode has to have
                // somewhere to go.
                go(TrackMode::Search, "tentative track deleted", frame, time_s);
            }
            break;

        case TrackMode::Track:
            // "Track -> Reacquire: track enters Coasting".
            if (!in.have_track || in.track_state == TrackState::Deleted) {
                go(TrackMode::Search, "track deleted", frame, time_s);
            } else if (in.track_state == TrackState::Coasting) {
                go(TrackMode::Reacquire, "track entered Coasting", frame, time_s);
            } else if (p_.handover_enabled) {
                // "Track -> Handover: RMS error < capture_range/3 for 30
                // consecutive frames."
                const double capture_px = (p_.handover_capture_urad / 3.0) / p_.ifov_urad;
                if (in.rms_error_px < capture_px) {
                    if (++handover_run_ >= p_.handover_frames) {
                        go(TrackMode::Handover, "inside capture range", frame, time_s);
                    }
                } else {
                    handover_run_ = 0;
                }
            }
            break;

        case TrackMode::Reacquire:
            // "Reacquire -> Track: track returns to Confirmed".
            // "Reacquire -> Search: track Deleted".
            if (!in.have_track || in.track_state == TrackState::Deleted) {
                go(TrackMode::Search, "track deleted", frame, time_s);
            } else if (in.track_state == TrackState::Confirmed) {
                go(TrackMode::Track, "track returned to Confirmed", frame, time_s);
            }
            break;

        case TrackMode::Handover:
            if (!in.have_track || in.track_state == TrackState::Deleted) {
                go(TrackMode::Search, "track deleted", frame, time_s);
            } else if (in.track_state == TrackState::Coasting) {
                go(TrackMode::Reacquire, "track entered Coasting", frame, time_s);
            }
            break;

        case TrackMode::Safe:
        case TrackMode::Idle:
            break;
    }

    ++frames_in_mode_;
    time_in_mode_ += dt;
    return mode_;
}

void ModeFsm::trace(std::vector<TrackMode>& out) const {
    out.clear();
    // The ring holds transitions, not states, so the trace starts from the
    // `from` of the oldest one — which is Idle on any run short enough not to
    // have overflowed 64 transitions, and honestly reported as whatever it was
    // on one that has.
    bool first = true;
    log_.for_each([&](const ModeTransition& t) {
        if (first) { out.push_back(t.from); first = false; }
        out.push_back(t.to);
    });
}

}  // namespace sat
