// tests/robustness/test_alloc.cpp — CP 14.3, INV-4's enforcement.
//
// "Debug operator new trap. Accept when: a full run completes with zero
//  steady-state allocations."
//
// ---------------------------------------------------------------------------
// THIS TEST IS A NO-OP IN RELEASE, AND THAT IS THE POINT
// ---------------------------------------------------------------------------
// The trap replaces the global operator new and aborts if it is called while a
// frame is in flight. It is armed only in Debug, because replacing operator new
// in a shipping binary is a liability — third-party code that allocates during
// a frame for a legitimate reason (a video decoder, an inference runtime) would
// abort the program rather than be slightly slower than we would like.
//
// So the assertion this case makes is not "no allocation happened", which it
// cannot observe. It is "a full run COMPLETES" — and in a Debug build that is
// the same statement, because an allocation would have aborted the process
// before the run finished. `just test-debug` is where this has teeth, and it is
// part of `just ci`.
//
// ---------------------------------------------------------------------------
// WHAT IT CAUGHT THE FIRST TIME IT WAS ARMED
// ---------------------------------------------------------------------------
// Four real violations, none of which had ever shown up in a profile:
//
//   grouping's blob vector          grew to its high-water mark inside the loop
//   std::stable_sort on detections  asks for a temporary buffer (360 bytes)
//   the fingerprint vector          one entry per frame, doubling as it went
//   the detection vector            reserved for max_candidates, but the shape
//                                   gate pushes BEFORE truncation, and Stage
//                                   12's supervisor lowering the CFAR threshold
//                                   pushed it to 48 against a reserve of 24
//
// Every one is the shape INV-4 exists to prevent: nothing goes wrong
// immediately, the frame budget just drifts and the p99 grows a tail nobody can
// attribute.

#include <doctest/doctest.h>

#include "engine/pipeline.hpp"
#include "scenario/scenario.hpp"

using namespace sat;

namespace {

Scenario base(double duration_s) {
    Scenario sc;
    sc.duration_s = duration_s;
    sc.seed       = 42;
    TargetSpec t;
    t.size_px        = 10;
    t.intensity      = 120.0;
    t.random_initial = false;
    t.initial_px[0]  = 1040.0;
    t.initial_px[1]  = 1020.0;
    MotionSpec m;
    m.kind = "linear";
    m.velocity_px_s[0] =  22.0;
    m.velocity_px_s[1] = -11.0;
    t.motion.push_back(m);
    sc.targets.push_back(t);
    return sc;
}

int64_t run(const Scenario& sc) {
    Pipeline p;
    p.build_from_scenario(sc);
    int64_t n = 0;
    while (p.step()) ++n;
    return n;
}

}  // namespace

TEST_CASE("CP 14.3: a full run completes with no steady-state allocation") {
    // The specification's own defaults, clutter and all — the configuration
    // that produces the most blobs and therefore the most pressure on every
    // reserve in the pipeline.
    Scenario sc = base(6.0);
    sc.static_sources = 120;
    sc.decoy_beacons  = 1;
    const int64_t frames = run(sc);
    MESSAGE(frames << " frames with 120 clutter sources, no allocation in any of them"
#if defined(SAT_ALLOC_TRAP)
            << " (trap ARMED)"
#else
            << " (trap not armed — Release; `just test-debug` is where this bites)"
#endif
    );
    CHECK(frames > 150);
}

TEST_CASE("CP 14.3: the supervisor's own switches do not allocate") {
    // The case that found the fourth violation. Lowering the CFAR threshold
    // mid-run lets more blobs through the shape gate, and the detection vector
    // was reserved for the POST-truncation cap rather than the pre-truncation
    // count. It reallocated at 48 detections against a reserve of 24.
    //
    // A run that switches strategies is therefore a different test from a run
    // that does not, and it is the one worth having: the steady state has to
    // survive the system changing its own configuration.
    Scenario sc = base(20.0);
    sc.static_sources = 0;
    sc.decoy_beacons  = 0;
    sc.supervisor_enabled = true;
    sc.targets[0].intensity = 40.0;     // dim enough that the fog rule fires

    EventSpec fog;   fog.t_s = 5.0;  fog.action = "set_atmosphere"; fog.mode = "fog";
    EventSpec clear; clear.t_s = 14.0; clear.action = "set_atmosphere"; clear.mode = "clear";
    sc.events.push_back(fog);
    sc.events.push_back(clear);

    Pipeline p;
    p.build_from_scenario(sc);
    int64_t frames = 0;
    while (p.step()) ++frames;

    MESSAGE(frames << " frames, " << p.supervisor().switch_count()
            << " strategy switches, no allocation in any of them");
    CHECK(frames > 500);
    CHECK(p.supervisor().switch_count() > 0);   // it really did switch
}

TEST_CASE("CP 14.3: the trap is armed where it is supposed to be") {
    // A self-test, in the spirit of gate-inv1-selftest and
    // gate-repro-selftest: an invariant checker nobody has watched fail is an
    // invariant checker you are trusting rather than using.
    //
    // It cannot deliberately allocate inside a frame to prove the trap fires —
    // that would abort the test process, which is what the trap is for. What it
    // CAN assert is that the Debug build is the one carrying it, so a build
    // system change that silently stopped defining SAT_ALLOC_TRAP turns this
    // red instead of quietly disarming every case above.
#if defined(NDEBUG)
    MESSAGE("Release build: the trap is deliberately not armed");
    CHECK(true);
#else
  #if defined(SAT_ALLOC_TRAP)
    MESSAGE("Debug build: the trap is armed");
    CHECK(true);
  #else
    FAIL("Debug build with SAT_ALLOC_TRAP undefined — CP 14.3's trap is "
         "disarmed and every allocation test above is vacuous");
  #endif
#endif
}
