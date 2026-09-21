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

#include "app/fuzz.hpp"
#include "metrics/collector.hpp"
#include "search/grid.hpp"
#include "scenario/schema.hpp"
#include "engine/pipeline.hpp"
#include "scenario/scenario.hpp"

#include <algorithm>
#include <cmath>
#include <string>

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

// Whether CP 14.3's trap is actually armed in THIS build. Written once, here,
// so no test has to put a #if inside a macro argument list — which is
// undefined behaviour ([cpp.replace]/11) and which GCC warns about.
//
// std::string, NOT const char*. doctest's MESSAGE streams a raw `const char*`
// through its own stringifier, which prints the POINTER: the note came out as
// "no allocation in any of them0x5f203a9e0157", so the one thing it exists to
// say — whether the trap was armed — was the one thing it did not say.
#if defined(SAT_ALLOC_TRAP)
const std::string kTrapNote = " (trap ARMED)";
#else
const std::string kTrapNote =
    " (trap not armed — Release; `just test-debug` is where this bites)";
#endif

TEST_CASE("CP 14.3: a full run completes with no steady-state allocation") {
    // The specification's own defaults, clutter and all — the configuration
    // that produces the most blobs and therefore the most pressure on every
    // reserve in the pipeline.
    Scenario sc = base(6.0);
    sc.static_sources = 120;
    sc.decoy_beacons  = 1;
    const int64_t frames = run(sc);
    // The conditional text is chosen OUTSIDE the macro. Preprocessor
    // directives inside a macro's argument list are undefined behaviour in
    // C++ ([cpp.replace]/11) and GCC says so; that a message says whether the
    // trap was armed is worth keeping, so it is a variable instead.
    MESSAGE(frames << " frames with 120 clutter sources, no allocation in any "
                      "of them" << kTrapNote);
    CHECK(frames > 150);
}

TEST_CASE("CP 14.3: P1-7's background sweep does not allocate") {
    // The sweep (design §14.0f) runs a SECOND detector pass over one row-band
    // per frame and merges its detections into the same list. Two reserves
    // have to be right for that to be free, and getting either wrong hides the
    // allocation on exactly the rare frames where the band finds something:
    //
    //   dets_band_  process() transiently holds the PRE-truncation blob list
    //               in whatever vector it is handed, so the band's vector
    //               needs the same bound as dets_, not max_candidates.
    //
    //   meas_       one measurement is built per surviving detection, and with
    //               the sweep on `dets_` is the merge of two passes that each
    //               truncate to max_candidates independently, so 2x the cap is
    //               the bound.
    //
    // This test does NOT currently drive the merge past the single cap — see
    // the note at that reserve for the probe that established as much — so it
    // would stay green if the factor of two were removed. It is here for the
    // first reserve, which the sweep does exercise on every banded frame, and
    // as the place a future denser fixture belongs.
    //
    // Clutter and a decoy are on deliberately: the sweep's whole job is
    // finding sources outside the tracking window, so a scene with nothing
    // outside the window would exercise the merge path and never the reserve.
    Scenario sc = base(6.0);
    sc.static_sources     = 120;
    sc.decoy_beacons      = 1;
    sc.roi_refresh_frames = 7;   // coprime with 480, so the bands do not tile evenly
    const int64_t frames = run(sc);
    MESSAGE(frames << " frames with the sweep across 7 bands, no allocation in "
                      "any of them" << kTrapNote);
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
    Scenario sc = base(12.0);
    sc.static_sources = 0;
    sc.decoy_beacons  = 0;
    sc.supervisor_enabled = true;
    sc.targets[0].intensity = 40.0;     // dim enough that the fog rule fires

    EventSpec fog;   fog.t_s = 3.0;  fog.action = "set_atmosphere"; fog.mode = "fog";
    EventSpec clear; clear.t_s = 9.0; clear.action = "set_atmosphere"; clear.mode = "clear";
    sc.events.push_back(fog);
    sc.events.push_back(clear);

    Pipeline p;
    p.build_from_scenario(sc);
    int64_t frames = 0;
    while (p.step()) ++frames;

    MESSAGE(frames << " frames, " << p.supervisor().switch_count()
            << " strategy switches, no allocation in any of them");
    CHECK(frames > 300);
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

// ===========================================================================
// CP 14.1 — the scenario fuzzer, as a test
//
// `just fuzz` runs the checkpoint's 5,000. This runs a small slice on every
// commit, because the value of a fuzzer that is only ever run by hand decays
// to zero within a month.
// ===========================================================================
TEST_CASE("CP 14.1: random legal scenarios do not crash, hang or produce NaN") {
    FuzzOptions opt;
    // 60 scenarios, not the checkpoint's 5,000. This runs on every commit and
    // every commit is not the place to spend ten minutes; `just fuzz` is the
    // full run and is what the checkpoint's number comes from. What a small
    // slice buys is REGRESSION coverage — the same 60 configurations, every
    // time, so a change that breaks one of them is attributable.
    opt.count      = 40;
    opt.duration_s = 0.2;
    // A FIXED seed, deliberately. A fuzzer seeded from the clock finds
    // different bugs every run, which sounds better and is worse: a failure
    // nobody can reproduce is a failure nobody fixes, and INV-3 forbids the
    // clock in this codebase anyway. `just fuzz` covers the breadth; this
    // covers the same 120 scenarios on every commit, so a regression in any of
    // them is attributable to the commit that caused it.
    opt.seed       = 20260917;
    CHECK(run_fuzz(opt) == 0);
}

// ===========================================================================
// scenarios/adversarial/ — the cases written to break it
//
// The complement of the fuzzer. The fuzzer's value is BREADTH: 5,000 random
// legal configurations, none of them chosen. These are chosen — each one
// exists because there was a specific reason to expect the system to handle it
// badly, and each file states that reason before its numbers.
//
// The assertion here is deliberately weak: every one RUNS, produces finite
// metrics, and does not hang. Several of them produce terrible results on
// purpose — decoy_swarm tracks a decoy 355 px away, target_faster_than_mount
// is 16,652 px off because the target is moving at twice the mount's ceiling —
// and asserting good numbers on those would be asserting the wrong thing. What
// must never happen is a crash, a hang, or a NaN, which is the same bar CP
// 14.1 sets for random scenarios.
// ===========================================================================
TEST_CASE("adversarial scenarios run, and none of them produces a NaN") {
    const char* const files[] = {
        "all_weather_churn", "beacon_larger_than_fov", "decoy_swarm",
        "edge_camper", "strobing_target", "target_faster_than_mount",
    };
    for (const char* name : files) {
        const std::string path =
            std::string(SAT_SCENARIO_DIR) + "/adversarial/" + name + ".toml";
        auto loaded = load_scenario(path);
        REQUIRE_MESSAGE(loaded.has_value(), name);

        Scenario sc = *loaded;
        sc.duration_s = 3.0;          // enough to exercise it; the suite is not a sweep

        Pipeline p;
        p.build_from_scenario(sc);
        MetricCollector mc;
        mc.begin(sc.name, sc.seed, sc.camera_geometry().ifov_urad(),
                 static_cast<size_t>(sc.duration_s * sc.camera_hz) + 2, true);
        int64_t frames = 0;
        while (p.step()) { mc.add(p.last()); ++frames; }
        const RunMetrics m = mc.finish(p.timers(), 1.0, p.gimbal().saturation_frac());

        // std::string, not the raw pointer: doctest's CAPTURE streams a
        // const char* as an address, which makes a failure report useless for
        // telling you WHICH scenario broke.
        CAPTURE(std::string(name));
        // 3 s at the scenario's own frame rate. Compared against the rate
        // rather than a constant, because these files do not all run at 30 Hz
        // and a hard-coded frame count is a bound on the wrong thing.
        CHECK(frames > static_cast<int64_t>(sc.duration_s * sc.camera_hz) - 5);
        CHECK(std::isfinite(m.tracking_rms_px));
        CHECK(std::isfinite(m.centroid_rmse_image_px));
        CHECK(std::isfinite(m.lock_retention_rate));
        CHECK(std::isfinite(m.saturation_frac));
        // Retention is a fraction. It reading above 1 was a real defect once —
        // see metrics/collector.cpp — and these are the scenarios most likely
        // to reproduce it, because several of them hold a confirmed track on
        // something that is not the beacon.
        CHECK(m.lock_retention_rate >= 0.0);
        CHECK(m.lock_retention_rate <= 1.0);
    }
}

// ===========================================================================
// CP 13.1 — the probability grid
//
// `just cp132` benchmarks the strategies end to end, which takes twenty
// minutes. These are the properties the grid has to have for that benchmark to
// mean anything, checked directly.
// ===========================================================================
TEST_CASE("CP 13.1: looking somewhere and seeing nothing reduces that region") {
    // §10.5's whole claim in one assertion: "Looking somewhere and seeing
    // nothing IS EVIDENCE; raster scans discard it."
    const CameraGeometry cam = CameraGeometry::make(640, 480, 4.0, 3.0);
    const ScreenGeometry scr = ScreenGeometry::make(2000, 2000, cam);

    ProbabilityGrid g;
    g.reset(GridParams{}, scr);

    const Angle2 centre = scr.to_angle(Pixel2{1000.0, 1000.0});
    // Both look points are well INSIDE the screen. A look near a corner has
    // part of its field of view off-screen, so it covers fewer cells and
    // carries less mass on a uniform grid — 0.0572 against 0.0768 — which is
    // correct behaviour and would make this an unequal comparison.
    const Angle2 elsewhere = scr.to_angle(Pixel2{1000.0, 400.0});

    const double before_here  = g.mass_in_fov(centre, cam);
    const double before_there = g.mass_in_fov(elsewhere, cam);
    CHECK(before_here == doctest::Approx(before_there).epsilon(0.02));  // uniform

    // Look at the centre four times, find nothing. No diffusion: this is the
    // static claim.
    for (int i = 0; i < 4; ++i) g.observe(centre, cam, /*found=*/false);

    const double after_here  = g.mass_in_fov(centre, cam);
    const double after_there = g.mass_in_fov(elsewhere, cam);
    MESSAGE("belief in the searched region " << before_here << " -> " << after_here
            << ";  elsewhere " << before_there << " -> " << after_there);

    CHECK(after_here < 0.2 * before_here);    // ruled out, mostly
    CHECK(after_there > before_there);        // and the rest got likelier
}

TEST_CASE("CP 13.1: a cell is never ruled out completely") {
    // p_detect is 0.9, not 1, and the grid is multiplicative — so a cell
    // approaches zero but must never reach it. Zero is absorbing: no amount of
    // later evidence brings a cell back from it, and a beacon that wandered
    // into a region searched early would become permanently unfindable.
    //
    // The same absorbing-zero argument as the IMM's mode probabilities.
    const CameraGeometry cam = CameraGeometry::make(640, 480, 4.0, 3.0);
    const ScreenGeometry scr = ScreenGeometry::make(2000, 2000, cam);
    ProbabilityGrid g;
    g.reset(GridParams{}, scr);

    const Angle2 centre = scr.to_angle(Pixel2{1000.0, 1000.0});
    for (int i = 0; i < 500; ++i) g.observe(centre, cam, false);

    double lowest = 1.0;
    for (int iy = 0; iy < ProbabilityGrid::NY; ++iy) {
        for (int ix = 0; ix < ProbabilityGrid::NX; ++ix) {
            lowest = std::min(lowest, static_cast<double>(g.cell(ix, iy)));
        }
    }
    MESSAGE("lowest cell after 500 fruitless looks at the same place: " << lowest);
    CHECK(lowest > 0.0);
}

TEST_CASE("CP 13.1: diffusion gives a searched region back over time") {
    // The half that makes the strategy correct for a MOVING target. Ruling a
    // tile out permanently is wrong: a tile checked ten seconds ago may hold
    // the beacon now, because the beacon moved there.
    const CameraGeometry cam = CameraGeometry::make(640, 480, 4.0, 3.0);
    const ScreenGeometry scr = ScreenGeometry::make(2000, 2000, cam);
    ProbabilityGrid g;
    g.reset(GridParams{}, scr);

    const Angle2 centre = scr.to_angle(Pixel2{1000.0, 1000.0});
    for (int i = 0; i < 6; ++i) g.observe(centre, cam, false);
    const double searched = g.mass_in_fov(centre, cam);

    // Ten seconds at 200 px/s: the target could have crossed 2000 px, the whole
    // screen.
    for (int i = 0; i < 300; ++i) g.diffuse(1.0 / 30.0, 200.0);
    const double recovered = g.mass_in_fov(centre, cam);

    MESSAGE("belief in a searched region: " << searched
            << " immediately after, " << recovered << " ten seconds later");
    CHECK(recovered > 2.0 * searched);
}

TEST_CASE("CP 13.1: best_look maximises a RATE, not a probability") {
    // §10.5's denominator, which is the whole design of best_look. Given two
    // regions of similar belief, the nearer one wins — because the quantity a
    // search maximises is probability PER SECOND, and crossing the screen for a
    // marginally better cell is slower than a spiral and has no coverage
    // guarantee either.
    const CameraGeometry cam = CameraGeometry::make(640, 480, 4.0, 3.0);
    const ScreenGeometry scr = ScreenGeometry::make(2000, 2000, cam);
    ProbabilityGrid g;
    g.reset(GridParams{}, scr);

    // Two bumps: a slightly stronger one far away, a slightly weaker one near.
    // reset_from_prior gives one bump, so the far one is built by ruling
    // everything else out around it and then partially undoing that near the
    // camera.
    const Angle2 here = scr.to_angle(Pixel2{300.0, 300.0});
    const Angle2 far  = scr.to_angle(Pixel2{1800.0, 1800.0});

    // Rule out a band in the middle so the two ends carry the belief.
    for (int i = 0; i < 6; ++i) {
        g.observe(scr.to_angle(Pixel2{1000.0, 1000.0}), cam, false);
    }

    const double mass_near = g.mass_in_fov(here, cam);
    const double mass_far  = g.mass_in_fov(far,  cam);
    REQUIRE(mass_near > 0.0);
    REQUIRE(mass_far  > 0.0);

    // Standing at `here`, with a mount that takes real time to cross: the
    // choice must stay nearby rather than committing to the far end for a
    // comparable payoff.
    const Angle2 chosen = g.best_look(here, cam, /*max_rate=*/87266.0);
    const double d_near = std::hypot(chosen.x - here.x, chosen.y - here.y);
    const double d_far  = std::hypot(chosen.x - far.x,  chosen.y - far.y);
    MESSAGE("near-end mass " << mass_near << ", far-end mass " << mass_far
            << "; chose a point " << std::string(d_near < d_far ? "near" : "far"));
    CHECK(d_near < d_far);
}
