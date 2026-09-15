// control/mode_fsm.hpp — CP 6.6. The mode state machine.
//
// Design §10.4:
//
//     Idle ─▶ Search ─▶ Detect ─▶ Acquire ─▶ Track ─▶ Handover
//               ▲                              │
//               └────── Reacquire ◀────────────┘
//                           │
//                           ▼
//                         Safe
//
// ---------------------------------------------------------------------------
// WHAT THIS IS FOR, GIVEN THE TRACK LIFECYCLE ALREADY EXISTS
// ---------------------------------------------------------------------------
// tracking/track.hpp already has a four-state FSM, and at first glance this is
// the same machine with more names. It is not, and keeping them separate is
// deliberate:
//
//   TrackState answers "how much do I believe this estimate?" It is a property
//   of ONE track and it is driven purely by detections.
//
//   TrackMode answers "what is the SYSTEM doing?" It is a property of the whole
//   run, it drives the CONTROLLER and the search pattern, and it has states
//   with no track at all (Search) and states with a perfectly good track that
//   still are not Track (Handover, Safe).
//
// Collapsing them would mean the controller could not distinguish "coasting on
// a prediction, keep slewing along the last velocity" from "coasting and about
// to give up, start widening the look" — which is the whole of CP 6.7.
//
// The mode is also what §13.2's centroid.csv logs per row and what the GUI
// timeline draws, so it has to be a named, stable enum rather than a set of
// booleans. It lives in core/mode.hpp, which has no dependencies of its own —
// see the note there about why it had to move out of engine/snapshot.hpp, and
// how INV-1's configure-time link check caught the original placement.
//
// ---------------------------------------------------------------------------
// THE TRANSITION LOG
// ---------------------------------------------------------------------------
// CP 6.6's acceptance criterion, as amended by §14.0, is "the transition
// sequence matches the expected trace for a scripted scenario". That requires
// the transitions to be a recorded artifact rather than something reconstructed
// from a per-frame mode column, because the interesting property is the ORDER
// and the REASON, and a mode column loses the reason entirely.
//
// The log is a fixed-capacity ring: bounded memory, no allocation in the frame
// loop (INV-4), and it keeps the most recent 64 transitions, which is far more
// than any run produces outside a pathological oscillation — and if one ever
// does oscillate, the last 64 are exactly the ones worth having.

#pragma once

#include "core/mode.hpp"
#include "core/ring.hpp"
#include "tracking/track.hpp"

#include <cstdint>
#include <vector>

namespace sat {

// ---------------------------------------------------------------------------
// ModeTransition — one row of the log.
// ---------------------------------------------------------------------------
struct ModeTransition {
    int64_t   frame  = 0;
    double    time_s = 0.0;
    TrackMode from   = TrackMode::Idle;
    TrackMode to     = TrackMode::Idle;
    /// Why. A string literal, not an allocated string: these are all compile
    /// time constants naming the §10.4 table row that fired, and storing a
    /// pointer keeps ModeTransition trivially copyable so the ring is
    /// allocation-free.
    const char* reason = "";
};

// ---------------------------------------------------------------------------
// ModeFsmInputs — everything the FSM is allowed to look at.
//
// Passed as a struct rather than as eight arguments so that adding an input is
// a compile error at the call site rather than a silently-defaulted bool, and
// so the whole decision is inspectable in a debugger in one place.
// ---------------------------------------------------------------------------
struct ModeFsmInputs {
    bool       have_track      = false;   ///< a track object exists at all
    TrackState track_state     = TrackState::Deleted;
    int        candidate_count = 0;       ///< gated detections from §9.4.7
    double     rms_error_px    = 1e9;     ///< recent tracking error, for Handover
    bool       running         = false;   ///< the run has started
};

// ---------------------------------------------------------------------------
// ModeFsmParams
// ---------------------------------------------------------------------------
struct ModeFsmParams {
    /// §10.4: "Track -> Handover: RMS error < capture_range/3 for 30
    /// consecutive frames." capture_range is the handover sensor's ~1 mrad
    /// (CP 10.7); a third of that at the default 109 urad/px is ~3 px.
    double handover_capture_urad = 1000.0;
    double ifov_urad             = 109.08;
    int    handover_frames       = 30;

    /// CP 10.7. Enabled now that control/handover.hpp's quadrant detector
    /// exists and the pipeline feeds it an OBSERVABLE offset rather than the
    /// truth-derived tracking error. It stays a switch because handover is a
    /// claim about a downstream system that this project does not contain, and
    /// a run that should not make that claim should be able to say so.
    bool   handover_enabled = true;

    /// §10.4: "any -> Safe: loss timeout exceeded". Seconds with no drivable
    /// track. Spec row 19 allows 1 s for re-acquisition, so a timeout below
    /// that would declare failure while the system is still inside budget;
    /// 5 s is comfortably beyond it and still short enough that a run that has
    /// genuinely lost the target says so rather than searching forever.
    double loss_timeout_s = 5.0;

    /// Frames of empty perception before Detect falls back to Search. Short:
    /// Detect means "candidates exist but no track yet", and if they stop
    /// existing there is nothing to acquire.
    int detect_timeout_frames = 5;
};

// ---------------------------------------------------------------------------
// ModeFsm
// ---------------------------------------------------------------------------
class ModeFsm {
public:
    void reset(const ModeFsmParams& p) noexcept;

    /// One frame. Returns the mode AFTER any transition.
    TrackMode step(const ModeFsmInputs& in, int64_t frame, double time_s) noexcept;

    [[nodiscard]] TrackMode mode() const noexcept { return mode_; }

    /// Should the controller drive the mount from the tracker's estimate?
    /// False in Search and Safe, where there is nothing to drive from and the
    /// search pattern owns the aim point instead (§6.2 step B25).
    [[nodiscard]] bool tracking_active() const noexcept {
        return mode_ == TrackMode::Track || mode_ == TrackMode::Reacquire
            || mode_ == TrackMode::Handover;
    }

    /// True on the frame a transition fired. The engine uses it to clear the
    /// controller's integrator when the setpoint is about to jump.
    [[nodiscard]] bool changed_this_frame() const noexcept { return changed_; }

    using Log = Ring<ModeTransition, 64>;
    [[nodiscard]] const Log& log() const noexcept { return log_; }

    /// The transition sequence as a flat list of modes, oldest first, starting
    /// with Idle. This is what CP 6.6's acceptance test compares against an
    /// expected trace.
    void trace(std::vector<TrackMode>& out) const;

    [[nodiscard]] double time_in_mode_s() const noexcept { return time_in_mode_; }
    [[nodiscard]] int    frames_in_mode() const noexcept { return frames_in_mode_; }

private:
    void go(TrackMode to, const char* reason, int64_t frame, double time_s) noexcept;

    ModeFsmParams p_{};
    TrackMode     mode_ = TrackMode::Idle;
    Log           log_{};

    bool   changed_        = false;
    int    frames_in_mode_ = 0;
    double time_in_mode_   = 0.0;
    double last_time_s_    = 0.0;
    double last_lock_s_    = 0.0;   ///< when a drivable track was last present
    int    handover_run_   = 0;     ///< consecutive frames inside capture range
    int    empty_frames_   = 0;     ///< consecutive frames with no candidates
    bool   started_        = false;
};

}  // namespace sat
