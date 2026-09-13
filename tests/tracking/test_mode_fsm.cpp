// tests/tracking/test_mode_fsm.cpp — CP 6.6.
//
// "Mode FSM + transition log. Accept when (amended, §14.0): the transition
//  sequence matches the expected trace for a scripted scenario."
//
// The amendment matters. The original criterion was a GUI observation; a trace
// comparison is strictly better, because it names the expected sequence up
// front and fails loudly on any deviation, including one a person watching a
// timeline would not notice — a mode that flickers for a single frame, or an
// arrow that fires for the wrong reason.
//
// The scripted scenarios below drive the FSM directly with synthetic inputs
// rather than through the engine. That is deliberate: the FSM's contract is
// "given these observations, produce these transitions", and testing it through
// the whole simulator would make a failure ambiguous between the FSM and the
// six stages upstream of it.

#include <doctest/doctest.h>

#include "control/mode_fsm.hpp"

#include <string>
#include <vector>

using namespace sat;

namespace {

constexpr double kDt = 1.0 / 30.0;

/// A scripted frame: what perception and the tracker reported.
struct Beat {
    int        frames;
    int        candidates;
    bool       have_track;
    TrackState state;
};

/// Run a script and return the resulting mode trace.
std::vector<TrackMode> run(ModeFsm& fsm, const std::vector<Beat>& script,
                           int64_t& frame, double& t) {
    for (const Beat& b : script) {
        for (int i = 0; i < b.frames; ++i) {
            ModeFsmInputs in;
            in.running         = true;
            in.candidate_count = b.candidates;
            in.have_track      = b.have_track;
            in.track_state     = b.state;
            fsm.step(in, frame++, t);
            t += kDt;
        }
    }
    std::vector<TrackMode> trace;
    fsm.trace(trace);
    return trace;
}

std::string render(const std::vector<TrackMode>& trace) {
    std::string s;
    for (size_t i = 0; i < trace.size(); ++i) {
        if (i) s += " -> ";
        s += track_mode_name(trace[i]);
    }
    return s;
}

}  // namespace

TEST_CASE("CP 6.6: the nominal acquisition trace") {
    // Idle -> Search -> Detect -> Acquire -> Track, and nothing else.
    ModeFsm fsm;
    ModeFsmParams p;
    fsm.reset(p);

    int64_t frame = 0;
    double  t     = 0.0;
    const auto trace = run(fsm, {
        {5,  0, false, TrackState::Deleted},    // searching, nothing in view
        {1,  1, false, TrackState::Deleted},    // a candidate appears
        {1,  1, true,  TrackState::Tentative},  // the tracker starts a track
        {3,  1, true,  TrackState::Tentative},  // M-of-N in progress
        {10, 1, true,  TrackState::Confirmed},  // locked
    }, frame, t);

    MESSAGE(render(trace));
    CHECK(trace == std::vector<TrackMode>{
        TrackMode::Idle, TrackMode::Search, TrackMode::Detect,
        TrackMode::Acquire, TrackMode::Track});
    CHECK(fsm.mode() == TrackMode::Track);
    CHECK(fsm.tracking_active());
}

TEST_CASE("CP 6.6: the dropout-and-recovery trace") {
    // The sequence the whole of Stage 6 exists to produce:
    // ... Track -> Reacquire -> Track, with no visit to Search.
    ModeFsm fsm;
    fsm.reset(ModeFsmParams{});

    int64_t frame = 0;
    double  t     = 0.0;
    const auto trace = run(fsm, {
        {2,  0, false, TrackState::Deleted},
        {1,  1, false, TrackState::Deleted},
        {2,  1, true,  TrackState::Tentative},
        {10, 1, true,  TrackState::Confirmed},
        {8,  0, true,  TrackState::Coasting},    // the beacon is blanked
        {10, 1, true,  TrackState::Confirmed},   // and comes back
    }, frame, t);

    MESSAGE(render(trace));
    CHECK(trace == std::vector<TrackMode>{
        TrackMode::Idle, TrackMode::Search, TrackMode::Detect, TrackMode::Acquire,
        TrackMode::Track, TrackMode::Reacquire, TrackMode::Track});
}

TEST_CASE("CP 6.6: a track that is deleted sends the FSM back to Search") {
    ModeFsm fsm;
    fsm.reset(ModeFsmParams{});

    int64_t frame = 0;
    double  t     = 0.0;
    const auto trace = run(fsm, {
        {2,  0, false, TrackState::Deleted},
        {1,  1, false, TrackState::Deleted},
        {2,  1, true,  TrackState::Tentative},
        {6,  1, true,  TrackState::Confirmed},
        {4,  0, true,  TrackState::Coasting},
        {2,  0, false, TrackState::Deleted},     // 15 misses; the track is gone
    }, frame, t);

    MESSAGE(render(trace));
    CHECK(trace.back() == TrackMode::Search);
    CHECK(trace == std::vector<TrackMode>{
        TrackMode::Idle, TrackMode::Search, TrackMode::Detect, TrackMode::Acquire,
        TrackMode::Track, TrackMode::Reacquire, TrackMode::Search});
}

TEST_CASE("CP 6.6: a Tentative track that never confirms returns to Search") {
    // §10.4's table stops at the happy path and does not draw this arrow, but
    // TrackState::Deleted is reachable from Tentative by construction, so the
    // mode must have somewhere to go. Without it the FSM sits in Acquire
    // forever after one noise blob and never searches again.
    ModeFsm fsm;
    fsm.reset(ModeFsmParams{});

    int64_t frame = 0;
    double  t     = 0.0;
    const auto trace = run(fsm, {
        {2, 0, false, TrackState::Deleted},
        {1, 1, false, TrackState::Deleted},
        {3, 1, true,  TrackState::Tentative},
        {2, 0, false, TrackState::Deleted},
    }, frame, t);

    MESSAGE(render(trace));
    CHECK(trace == std::vector<TrackMode>{
        TrackMode::Idle, TrackMode::Search, TrackMode::Detect,
        TrackMode::Acquire, TrackMode::Search});
}

TEST_CASE("CP 6.6: a one-frame noise blob does not strand the FSM in Detect") {
    ModeFsm fsm;
    ModeFsmParams p;
    p.detect_timeout_frames = 5;
    fsm.reset(p);

    int64_t frame = 0;
    double  t     = 0.0;
    const auto trace = run(fsm, {
        {2, 0, false, TrackState::Deleted},
        {1, 1, false, TrackState::Deleted},   // a blob, gone next frame
        {4, 0, false, TrackState::Deleted},
    }, frame, t);
    CHECK(fsm.mode() == TrackMode::Detect);   // not yet: 4 empty frames
    run(fsm, {{1, 0, false, TrackState::Deleted}}, frame, t);
    CHECK(fsm.mode() == TrackMode::Search);   // the fifth fires it
    (void)trace;
}

TEST_CASE("CP 6.6: any state goes to Safe on the loss timeout, and stays there") {
    ModeFsm fsm;
    ModeFsmParams p;
    p.loss_timeout_s = 1.0;    // 30 frames
    fsm.reset(p);

    int64_t frame = 0;
    double  t     = 0.0;
    run(fsm, {
        {2,  0, false, TrackState::Deleted},
        {1,  1, false, TrackState::Deleted},
        {2,  1, true,  TrackState::Tentative},
        {10, 1, true,  TrackState::Confirmed},
    }, frame, t);
    REQUIRE(fsm.mode() == TrackMode::Track);

    // Lose it for well over the timeout.
    run(fsm, {{60, 0, false, TrackState::Deleted}}, frame, t);
    CHECK(fsm.mode() == TrackMode::Safe);
    CHECK_FALSE(fsm.tracking_active());

    // Safe has no exit in §10.4's diagram, and that is respected: a run that
    // has declared failure must not quietly resume, or the acquisition metrics
    // in §13.1 would be reported against a run that never admitted it lost.
    run(fsm, {{30, 1, true, TrackState::Confirmed}}, frame, t);
    CHECK(fsm.mode() == TrackMode::Safe);
}

TEST_CASE("CP 6.6: Handover is disabled until CP 10.7 builds the quadrant detector") {
    // Entering Handover would claim a capability the system does not have. The
    // transition is implemented and tested so it cannot rot, but it is off.
    ModeFsmParams p;
    int64_t frame = 0;
    double  t     = 0.0;

    auto drive_to_track = [&](ModeFsm& fsm) {
        run(fsm, {
            {2,  0, false, TrackState::Deleted},
            {1,  1, false, TrackState::Deleted},
            {2,  1, true,  TrackState::Tentative},
            {5,  1, true,  TrackState::Confirmed},
        }, frame, t);
        REQUIRE(fsm.mode() == TrackMode::Track);
    };

    auto hold_accurate = [&](ModeFsm& fsm, int n) {
        for (int i = 0; i < n; ++i) {
            ModeFsmInputs in;
            in.running = true; in.candidate_count = 1;
            in.have_track = true; in.track_state = TrackState::Confirmed;
            in.rms_error_px = 0.5;     // well inside capture_range/3
            fsm.step(in, frame++, t);
            t += kDt;
        }
    };

    ModeFsm off;
    off.reset(p);
    drive_to_track(off);
    hold_accurate(off, 60);
    CHECK(off.mode() == TrackMode::Track);

    p.handover_enabled = true;
    ModeFsm on;
    on.reset(p);
    frame = 0; t = 0.0;
    drive_to_track(on);
    hold_accurate(on, 29);
    CHECK(on.mode() == TrackMode::Track);     // 29 frames is not 30
    hold_accurate(on, 1);
    CHECK(on.mode() == TrackMode::Handover);
}

TEST_CASE("CP 6.6: the log records why, not just what") {
    ModeFsm fsm;
    fsm.reset(ModeFsmParams{});
    int64_t frame = 0;
    double  t     = 0.0;
    run(fsm, {
        {2, 0, false, TrackState::Deleted},
        {1, 1, false, TrackState::Deleted},
        {2, 1, true,  TrackState::Tentative},
        {6, 1, true,  TrackState::Confirmed},
    }, frame, t);

    std::vector<std::string> reasons;
    fsm.log().for_each([&](const ModeTransition& tr) { reasons.emplace_back(tr.reason); });
    REQUIRE(reasons.size() == 4);
    CHECK(reasons[0] == "run start");
    CHECK(reasons[1] == ">=1 gated detection");
    CHECK(reasons[2] == "track reached Tentative");
    CHECK(reasons[3] == "track reached Confirmed");

    // Frame numbers are recorded so a trace can be lined up against the
    // centroid.csv rows and the error plot.
    std::vector<int64_t> frames;
    fsm.log().for_each([&](const ModeTransition& tr) { frames.push_back(tr.frame); });
    CHECK(frames[0] == 0);
    CHECK(frames[1] == 2);
    CHECK(std::is_sorted(frames.begin(), frames.end()));
}
