// tests/metrics/test_spec_run.cpp — the whole system, at the SPECIFICATION's
// own parameters, measured with the §13.1 definitions.
//
// ---------------------------------------------------------------------------
// WHY THIS FILE EXISTS
// ---------------------------------------------------------------------------
// Every Stage 6 test passed while the system was, on the specification's own
// default scenario, holding lock on 0.6% of frames. They passed because every
// one of them built its Scenario by hand and set jitter to zero — reasonably,
// since each was testing one mechanism in isolation. The result was a suite
// that could not see a whole class of failure, which is worse than no suite,
// because it gets trusted.
//
// So this file runs scenarios/hard/cold_start_in_clutter.toml — spec row 23's
// jitter, row 25's platform motion, row 21's noise, §9.1's clutter, all at
// their stated values — and asserts the GRADED requirements from §3.2 rather
// than any internal property. If the loop stops working at the specification's
// parameters, this is what says so.
//
// WHY THIS FILE AND NOT scenarios/spec_defaults.toml, which is now the default:
// spec_defaults has the clutter field OFF, for the reasons its header gives.
// This suite's whole purpose is to run at the HARDEST stated parameters, so it
// loads the hard case and switches the clutter off itself for the cases that
// need it — which also means the clutter cases below compare two runs of ONE
// file rather than two different files.
//
// The one deviation from the file as shipped is the beacon's initial position:
// row 11 permits "random", and §10.5 derives that a cold search of the whole
// screen cannot meet row 16 by geometry. A run that spends its length
// searching measures acquisition, which is a different question and has its
// own (honestly labelled) metric. Pinning the beacon in view measures the loop
// — and makes this file's scenario identical to spec_defaults.toml plus
// clutter, i.e. exactly scenarios/hard/clutter_field.toml.

#include <doctest/doctest.h>

#include "engine/pipeline.hpp"
#include "metrics/collector.hpp"
#include "scenario/schema.hpp"

#include <cmath>
#include <cstdio>
#include <map>
#include <string>

using namespace sat;

namespace {

/// Length of every run below.
///
/// 4 s is 120 frames, which is ample for all of it: acquisition happens in two
/// frames, and an RMS or a retention ratio over 120 samples is not short of
/// data. It is deliberately not longer, because each frame here is a full §9.4
/// pipeline pass at ~87 ms and this file is the most expensive in the suite —
/// at 10 s it ran five pipelines and timed out under parallel load.
constexpr double kRunSeconds = 4.0;

Scenario spec_scenario(bool clutter) {
    auto r = load_scenario(std::string(SAT_SCENARIO_DIR) + "/hard/cold_start_in_clutter.toml");
    REQUIRE_MESSAGE(r.has_value(), r.error());
    Scenario sc = *r;
    sc.duration_s = kRunSeconds;

    // Row 11: pin it in view. Everything else is left exactly as the
    // specification's defaults have it — in particular jitter stays at row 23's
    // 20 px/frame, which is the value that broke the tracker.
    REQUIRE(!sc.targets.empty());
    sc.targets[0].random_initial = false;
    sc.targets[0].initial_px[0]  = sc.initial_pos_px[0] + 40.0;
    sc.targets[0].initial_px[1]  = sc.initial_pos_px[1] + 20.0;

    if (!clutter) { sc.static_sources = 0; sc.decoy_beacons = 0; }
    return sc;
}

RunMetrics run_uncached(const Scenario& sc) {
    Pipeline p;
    p.build_from_scenario(sc);
    p.set_publish_snapshots(false);

    MetricCollector m;
    m.begin(sc.name, sc.seed, sc.camera_geometry().ifov_urad(),
            static_cast<size_t>(sc.duration_s * sc.camera_hz) + 2);
    while (p.step()) m.add(p.last());
    return m.finish(p.timers(), 1.0, p.gimbal().saturation_frac());
}

// ---------------------------------------------------------------------------
// run — memoised by configuration.
//
// The three test cases below need three distinct configurations between them,
// but they share them: the clean, jittered run appears in all three. Running it
// once per case meant five full pipelines and a suite that timed out under
// parallel load.
//
// Caching across test cases is normally a bad idea, because it makes one case's
// result depend on another having run. It is safe HERE for a specific reason:
// INV-3 guarantees these runs are bit-exact functions of their configuration,
// so a cached result is indistinguishable from a fresh one. If that ever stops
// being true, the reproducibility gate (CP 2.6) goes red first and says so much
// more clearly than a flaky number here would.
// ---------------------------------------------------------------------------
const RunMetrics& run(const Scenario& sc) {
    static std::map<std::string, RunMetrics> cache;
    char key[128];
    std::snprintf(key, sizeof key, "%d|%.1f|%.2f|%llu",
                  sc.static_sources, sc.jitter_px_per_frame, sc.duration_s,
                  static_cast<unsigned long long>(sc.seed));
    auto it = cache.find(key);
    if (it == cache.end()) it = cache.emplace(key, run_uncached(sc)).first;
    return it->second;
}

}  // namespace

TEST_CASE("the loop holds lock at the specification's own jitter (row 23)") {
    const Scenario sc = spec_scenario(/*clutter=*/false);
    REQUIRE(sc.jitter_px_per_frame == doctest::Approx(20.0));   // row 23, unmodified

    const RunMetrics& m = run(sc);
    MESSAGE("in-FOV " << m.frames_in_fov << " frames, confirmed " << m.frames_confirmed
            << ", retention " << (100.0 * m.lock_retention_rate) << "%");
    MESSAGE("acquisition (in view) " << m.acquisition_in_fov_s << " s");
    MESSAGE("tracking " << m.tracking_rms_px << " px RMS, centroid (image) "
            << m.centroid_rmse_image_px << " px RMSE");

    // The beacon is in view for essentially the whole run: the mount follows it.
    // This is the assertion that fails outright when the gate is mis-sized —
    // the camera slews away and never comes back.
    CHECK(m.frames_in_fov > static_cast<int64_t>(0.95 * m.frames_total));

    // Spec row 18: target loss < 5%.
    CHECK(m.lock_retention_rate > 0.95);
    CHECK(m.target_loss_frac < sc.target_loss_frac);

    // Spec row 16, in the sense §10.5 says is the intended one.
    CHECK(m.acquired);
    CHECK(m.acquisition_in_fov_s < sc.acquisition_s);

    // The DETECTOR is unaffected by any of this — it sees one frame at a time
    // and jitter does not move the beacon within the frame. Sub-pixel, and the
    // number to watch as §10.1.3's bias correction lands at CP 9.3.
    CHECK(m.centroid_rmse_image_px < 1.0);
}

TEST_CASE("tracking error at spec-row-23 jitter sits on its derived floor") {
    // -------------------------------------------------------------------
    // A BOUND, DERIVED AND STATED — the same treatment §10.5 gives acquisition.
    //
    // §13.1 defines tracking_error as |camera boresight - true beacon angle|,
    // and spec row 17 caps it at 10 px. Spec row 23 independently specifies a
    // camera jitter of up to +/-20 px per frame, which degrade/disturbance.cpp
    // draws uniformly and applies to the TRUE boresight.
    //
    // Those two requirements are in tension, and the arithmetic is short. For a
    // uniform distribution on [-A, A] the variance is A^2/3, so with A = 20 px
    // each axis contributes 133.3 px^2 and the magnitude over two independent
    // axes has
    //
    //     RMS = sqrt(2 * 400/3) = 16.33 px
    //
    // That is a FLOOR on row 17's metric which no controller can go below,
    // because the disturbance displaces the boresight after the command has
    // been issued and the encoder never observes it. A system reporting under
    // 10 px here would either not be applying the specified jitter or not be
    // measuring against the true boresight.
    //
    // This is reported rather than engineered around, exactly as §10.5's
    // acquisition bound is. The controllable part — how well the loop tracks
    // the beacon apart from the jitter — is what the no-jitter arm below
    // measures, and it is comfortably inside budget.
    // -------------------------------------------------------------------
    const double kJitterFloorPx = std::sqrt(2.0 * 400.0 / 3.0);

    Scenario sc = spec_scenario(/*clutter=*/false);
    const RunMetrics& with = run(sc);

    sc.jitter_px_per_frame = 0.0;
    const RunMetrics& without = run(sc);

    MESSAGE("tracking RMS: " << with.tracking_rms_px << " px with row-23 jitter, "
            << without.tracking_rms_px << " px without; derived jitter floor "
            << kJitterFloorPx << " px");

    // The jittered figure sits on the floor, not far above it: the loop is not
    // adding error of its own on top of the disturbance.
    CHECK(with.tracking_rms_px > 0.8 * kJitterFloorPx);
    CHECK(with.tracking_rms_px < 1.5 * kJitterFloorPx);

    // And with the disturbance removed, spec row 17 is met with room to spare.
    // This is the number that measures the CONTROLLER.
    CHECK(without.tracking_rms_px < sc.tracking_error_px);
}

TEST_CASE("clutter costs tracking accuracy, and the cost is measured not hidden") {
    // §9.1 makes some of the 120 static sources BRIGHTER than the beacon on
    // purpose. Nothing in Stage 6 can tell a bright point source from another
    // bright point source by appearance, so the tracker sometimes locks onto
    // clutter — which is CP 4.11's finding restated for the classical pipeline,
    // and whose designed answers are §10.2's priority policy (Stage 12) and
    // CandidateNet (Stage 11).
    //
    // The point of this test is not to assert that it works. It is to pin the
    // CURRENT number so that Stage 11 and 12 have a baseline to beat, and so
    // that a regression between here and there is visible.
    // The scenarios are named rather than passed inline: run() returns a
    // reference into a static cache, and GCC's -Wdangling-reference cannot
    // tell that from a reference into the temporary argument. Binding the
    // arguments to locals removes the ambiguity for the reader too.
    const Scenario clean_sc     = spec_scenario(/*clutter=*/false);
    const Scenario cluttered_sc = spec_scenario(/*clutter=*/true);
    const RunMetrics& clean     = run(clean_sc);
    const RunMetrics& cluttered = run(cluttered_sc);

    MESSAGE("tracking RMS: " << clean.tracking_rms_px << " px clean, "
            << cluttered.tracking_rms_px << " px with 120 clutter + 1 decoy");
    MESSAGE("centroid (image) RMSE: " << clean.centroid_rmse_image_px << " px clean, "
            << cluttered.centroid_rmse_image_px << " px cluttered");

    // -----------------------------------------------------------------------
    // WHAT MOVED WHEN §10.2's PRIORITY POLICY LANDED, AND THE TRADE IT MADE
    //
    //                          before      after
    //     tracking RMS         205.20 px   87.88 px
    //     lock retention        0.97        0.875
    //
    // The policy refuses to commit the mount to a candidate that does not look
    // like a beacon, and drops one that stops looking like a beacon. So when it
    // is wrong it is wrong for FEWER FRAMES, and the frames it gives up are
    // frames it was previously spending pointed at a rock. Tracking error more
    // than halved; retention fell by ten points.
    //
    // That trade is the right way round for this project — centroiding error is
    // 60% of the marks and lock retention is one line of §13.1 — but it is a
    // trade and it is recorded as one rather than smoothed over. Row 18's 5%
    // target-loss requirement is NOT met in this configuration and was not met
    // before either; see issues_till_now.md §2.2.
    // -----------------------------------------------------------------------
    CHECK(cluttered.lock_retention_rate > 0.80);
    // And the honest part: it is measurably worse than clean, and by how much
    // is recorded.
    CHECK(cluttered.tracking_rms_px > clean.tracking_rms_px);
    // The baseline Stage 11 has to beat, pinned so a regression is visible.
    CHECK(cluttered.tracking_rms_px < 120.0);
}

// ===========================================================================
// CP 14.2 / spec row 20 — the frame budget
// ===========================================================================

// ---------------------------------------------------------------------------
// A SEPARATE, SERIAL TEST SUITE.
//
// Skipped in the ordinary run and registered as its own CTest entry — see
// tests/CMakeLists.txt. A wall-clock assertion under `ctest --parallel 8`
// measures the scheduler, not the tracker: this bound passed serially and
// failed under contention, which was the third time a timing check had gone
// red for a reason with nothing to do with the code.
//
// It is a doctest TEST_SUITE rather than a name filter because the first
// attempt filtered on `-tc="spec row 20*"`, the spaces did not survive the
// argument round trip, and CTest reported the entry as PASSING WITH ZERO
// ASSERTIONS. A test that silently runs nothing is worse than no test — it
// occupies the place where the check was supposed to be. `-ts=perf` has no
// spaces and cannot fail that way, and the CTest entry now asserts a minimum
// assertion count so an empty run is a failure rather than a pass.
// ---------------------------------------------------------------------------
TEST_SUITE("perf") {

TEST_CASE("spec row 20: the loop sustains at least 20 FPS" * doctest::skip()) {
    // -------------------------------------------------------------------
    // A WALL-CLOCK ASSERTION IN CI, AND WHY IT IS WRITTEN LOOSELY.
    //
    // Row 20 is a graded requirement, so it has to be checked; but CI runners
    // vary by a factor of two or three and a tight bound would be flaky. So
    // this is a CHANGE-OF-KIND detector rather than a benchmark: it catches a
    // stage that regresses by an order of magnitude, which is exactly what the
    // three defects found at CP 14.2 looked like.
    //
    // For reference, the per-stage figures that produced the current number
    // are printed by `sat-tracker --headless --stages` and recorded in
    // docs/METRICS.md. Measured on the development machine: 46.6 ms per frame
    // synthetic (21.5 FPS) and 32.5 ms in video mode (30.8 FPS), from 87.4 ms
    // (11.4 FPS) before.
    // -------------------------------------------------------------------
    Scenario sc = spec_scenario(/*clutter=*/false);
    sc.duration_s = 2.0;

    Pipeline p;
    p.build_from_scenario(sc);
    p.set_publish_snapshots(false);       // a speed run publishes nothing
    while (p.step()) {}

    const LatencyHistogram& total = p.timers()[Stage::FrameTotal];
    REQUIRE(total.count() > 30);
    const double ms = total.p50() * 1e-3;
    MESSAGE("frame time p50 " << ms << " ms  (" << (1000.0 / ms) << " FPS)");

    // -------------------------------------------------------------------
    // THE BOUND DEPENDS ON THE BUILD, because it has to.
    //
    // A Debug build runs this code about eight times slower — measured across
    // the whole suite when the test timeouts were calibrated — so a single
    // number cannot serve both. The first version used 200 ms for both and
    // went red in Debug at 250 ms, which is a test measuring the compiler's
    // optimiser rather than the tracker.
    //
    // Nobody ships a Debug build, so the honest reading is that row 20 applies
    // to the optimised one and Debug gets a bound that still catches an
    // order-of-magnitude regression.
    // -------------------------------------------------------------------
    // Run alone, so this is the machine's honest single-core figure.
#ifdef NDEBUG
    CHECK(ms < 200.0);     // ~4x the measured 46.6 ms
#else
    CHECK(ms < 1500.0);    // ~4x the measured Debug figure
#endif
}

TEST_CASE("CP 14.2: no single stage dominates the frame the way three used to"
          * doctest::skip()) {
    // -------------------------------------------------------------------
    // SKIPPED BY DEFAULT, like the wall-clock case above it, and for the same
    // reason arriving one step later than expected.
    //
    // These are RATIOS between stages, and the note further down records that
    // they are not BUILD-independent. They are not CONTENTION-independent
    // either, and that took a red CI run to notice: `just ci` runs ctest in
    // parallel, and this case is measuring p50 stage times on cores it is
    // sharing. It passed standalone and failed in CI, which is the signature
    // of a timing assertion running in a parallel suite.
    //
    // tests/CMakeLists.txt already registers the "perf" suite as its own
    // RUN_SERIAL entry (`frame_budget`) precisely so that timings are measured
    // on a quiet machine. The wall-clock case carried doctest::skip() so that
    // only that serial entry runs it; this one did not, so it ran in both — and
    // the parallel one is the invocation where the measurement means nothing.
    // -------------------------------------------------------------------
    // The three defects fixed at CP 14.2 were each a stage costing an order of
    // magnitude more than its neighbours. Asserting the SHAPE of the profile
    // catches that class directly, and is far less machine-dependent than any
    // absolute number: a stage that goes back to computing a full-image map it
    // reads at two dozen points shows up as a ratio, not as milliseconds.
    Scenario sc = spec_scenario(/*clutter=*/false);
    sc.duration_s = 2.0;

    Pipeline p;
    p.build_from_scenario(sc);
    p.set_publish_snapshots(false);
    while (p.step()) {}

    const StageTimers& t = p.timers();
    const double mf   = t[Stage::MatchedFilter].p50();
    const double cfar = t[Stage::Cfar].p50();
    const double th   = t[Stage::TopHat].p50();
    const double sat  = t[Stage::SummedArea].p50();
    MESSAGE("matched_filter " << mf << " us, cfar " << cfar << " us, top_hat "
            << th << " us, summed_area " << sat << " us");

    // -------------------------------------------------------------------
    // RELEASE ONLY, and the reason is worth recording because the first
    // version got it wrong.
    //
    // These are RATIOS, which I assumed made them build-independent — the
    // whole point of comparing stages rather than asserting milliseconds. That
    // is false. -O0 does not slow everything equally: the top-hat's inner loop
    // processes 64 columns in lockstep and vectorises well, so it loses far
    // more to a Debug build than the summed-area pass does. Measured, the same
    // ratio is 1.0 in Release and 3.2 in Debug, and the test went red on a
    // build nobody ships.
    //
    // So the shape assertions run on the optimised build, which is the one the
    // claim is about. The numbers are still PRINTED in Debug, because they are
    // useful when something is being investigated there.
    // -------------------------------------------------------------------
#ifdef NDEBUG
    // The matched filter ran six extra full-image passes for a scale map read
    // at the candidates. That made it 9x the CFAR pass; it is now well under.
    CHECK(mf < cfar * 2.0);
    // The top-hat's vertical pass walked columns one at a time, which is a
    // cache miss per pixel for an O(1) algorithm. It was 3x the SAT build.
    CHECK(th < sat * 2.5);
#else
    CHECK(mf > 0.0);
    CHECK(th > 0.0);
#endif
}

}  // TEST_SUITE("perf")

// ===========================================================================
// P0-1 — THE DEFAULT SCENARIO IS A PASSING RUN, ASSERTED IN CI
//
// This is the regression lock the audit asks for, and it exists because the
// defect it guards against was invisible to every other test in the suite.
//
// scenarios/baseline.toml was the default for `--gui` and `--headless`, the
// first entry in the GUI's scenario picker, the source of the README's hero
// screenshot, the `just smoke` sanity scenario, and — per docs/DEMO.md — "the
// known-good fallback (CP 15.3)". It scored 913 px tracking RMS, ZERO
// centroiding frames and 1778 false tracks a minute. A judge typing the
// obvious command saw the worst run in the repository.
//
// Twenty CTest suites were green while that was true. None of them ran the
// DEFAULT — every one built its Scenario by hand or named a file explicitly,
// which is reasonable in isolation and left the one configuration a stranger
// would actually reach completely untested.
//
// So this test asserts the property directly: whatever the default is, it has
// to work. It deliberately resolves the path the same way src/app/main.cpp and
// src/app/headless.cpp do, so that changing the default without changing this
// test is not possible.
// ===========================================================================

TEST_CASE("P0-1: the DEFAULT scenario — what `--gui` and `--headless` load — passes") {
    // The same expression as src/app/headless.cpp:344 and src/app/main.cpp:162.
    // If those change, this must change with them, and a mismatch is a
    // compile-visible edit rather than a silent divergence.
    auto r = load_scenario(std::string(SAT_SCENARIO_DIR) + "/spec_defaults.toml");
    REQUIRE_MESSAGE(r.has_value(), r.error());
    Scenario sc = *r;

    // Shortened so this stays inside the suite's time budget. Everything
    // asserted below is a steady-state property and is reached within the
    // first second; the full 30 s run is what `just headless` produces and its
    // numbers are recorded in docs/RESULTS.md.
    sc.duration_s = kRunSeconds;

    const RunMetrics m = run_uncached(sc);

    MESSAGE("centroiding (image) " << m.centroid_rmse_image_px << " px over "
            << m.centroid_frames << " frames");
    MESSAGE("tracking (steady) " << m.tracking_rms_steady_px << " px RMS");
    MESSAGE("FOV containment " << (100.0 * m.fov_containment_frac) << " %");
    MESSAGE("row 18 (post-acq) " << (100.0 * m.target_loss_post_acq) << " %");

    // -------------------------------------------------------------------
    // THE ASSERTION THAT WOULD HAVE CAUGHT IT.
    //
    // The old default scored ZERO centroiding frames — no frame had both a
    // detection and the beacon in view — and every ratio derived from that
    // denominator was therefore undefined and printed as 0.00, which reads as
    // perfect. A single non-zero check on the denominator is the whole guard.
    // -------------------------------------------------------------------
    REQUIRE(m.centroid_frames > 0);
    CHECK(m.centroid_frames >= static_cast<int64_t>(0.9 * m.frames_total));

    // Row 16: acquisition, in view. The beacon is placed in the field at t=0,
    // so this is the in-view figure and it must beat 2 s comfortably.
    REQUIRE(m.acquired);
    CHECK(m.acquisition_in_fov_s <= 2.0);

    // Row 18, on the post-acquisition denominator (P0-2). This is the row the
    // old default passed at 7.59 % while pointing at empty sky for 95.6 % of
    // the run, so it is asserted on the definition that cannot be flattered.
    REQUIRE(m.post_acq_valid);
    CHECK(m.target_loss_post_acq < 0.05);

    // Row 20: processing speed. Loose, because CI runners vary by a factor of
    // three and this is a change-of-kind detector, not a benchmark — the same
    // reasoning as the perf suite's wall-clock case.
    CHECK(m.fps_mean > 20.0);

    // The PS's own objective. Not a numbered row, and asserted anyway: the
    // whole point of P0-2 is that this is the quantity the problem statement
    // names, and the default scenario should be the one place it is 100 %.
    CHECK(m.fov_containment_frac > 0.95);

    // Row 17 is NOT asserted, and the omission is deliberate rather than
    // convenient. Row 23's 20 px/frame of jitter puts a 16.33 px floor under
    // the pointing error (docs/METRICS.md §2.11), which exceeds row 17's 10 px
    // budget. The compliance matrix reports that as "BOUND DERIVED" rather
    // than FAIL for the same reason. What IS asserted is that the loop sits on
    // that floor rather than somewhere else entirely — a regression that
    // doubled the error would still be caught.
    CHECK(m.tracking_rms_steady_px < 2.0 * 16.33);

    // And the detector is doing its job: sub-pixel in its own frame, at row
    // 21's 10 % impulse noise and row 22's read noise at the cap.
    CHECK(m.centroid_rmse_image_px < 1.0);
}

TEST_CASE("P0-1: the hard cases still fail, and each fails for ONE reason") {
    // The complement of the test above, and just as important: the hard cases
    // must keep failing. If scenarios/hard/clutter_field.toml ever starts
    // passing, either the discrimination problem has been solved — in which
    // case this test should be updated and the news recorded — or the scenario
    // has quietly stopped exercising it, which is the failure mode that turns
    // a demo of a known limitation into a demo of nothing.
    //
    // Splitting the two hard cases is the point. The old baseline.toml
    // combined a random initial position (an acquisition problem) with a
    // 120-source clutter field (a discrimination problem) and produced one
    // 913 px number that could not distinguish them.

    SUBCASE("clutter_field: acquisition SUCCEEDS, discrimination fails") {
        auto r = load_scenario(std::string(SAT_SCENARIO_DIR) + "/hard/clutter_field.toml");
        REQUIRE_MESSAGE(r.has_value(), r.error());
        Scenario sc = *r;
        sc.duration_s = kRunSeconds;
        const RunMetrics m = run_uncached(sc);

        MESSAGE("clutter_field: centroid " << m.centroid_rmse_image_px
                << " px, containment " << (100.0 * m.fov_containment_frac) << " %");

        // The beacon is in view at t=0 and IS found. This is what isolates the
        // failure: it is not an acquisition failure.
        CHECK(m.acquired);
        CHECK(m.acquisition_in_fov_s <= 2.0);

        // And then it is lost to clutter. Asserted as an ORDERING against the
        // clean run rather than as a constant, so the test states the
        // relationship that matters and does not go red on a tuning change
        // that improves both.
        auto clean = load_scenario(std::string(SAT_SCENARIO_DIR) + "/spec_defaults.toml");
        REQUIRE(clean.has_value());
        Scenario cs = *clean;
        cs.duration_s = kRunSeconds;
        const RunMetrics c = run_uncached(cs);

        // Centroiding is where the failure shows first and hardest: the
        // reported centroid is a clutter source, so the error is the distance
        // to it. Measured at this run length, 93.1 px against 0.21 px — a
        // factor of 434. The assertion is set two orders of magnitude below
        // that so it states the KIND of failure rather than the current value.
        CHECK(m.centroid_rmse_image_px > 10.0 * c.centroid_rmse_image_px);

        // Pointing follows, but slowly, and the margin here is deliberately
        // weak: 21.5 px against 16.8 px at 4 s, only 1.28x. The mount is
        // locked onto the wrong object but that object has not yet pulled it
        // far from the beacon. Asserting a 2x ratio here fails — it was tried
        // — and the honest conclusion is that four seconds is too short to see
        // this half of the failure, not that the failure is absent.
        CHECK(m.tracking_rms_steady_px > c.tracking_rms_steady_px);

        // FOV CONTAINMENT IS NOT ASSERTED HERE, and the reason is worth
        // stating because it was an assertion that failed and taught
        // something.
        //
        // At this suite's 4 s run length containment is still 100 % in BOTH
        // runs: the mount is locked onto a clutter source, but over four
        // seconds it has not yet been dragged far enough for the beacon to
        // leave the field. Containment only collapses over a longer run — at
        // 30 s it is 80.78 % against the clean run's 100 %.
        //
        // That is a real and useful fact about the failure, not a reason to
        // lengthen this test: it says the clutter failure is a SLOW divergence
        // rather than an immediate loss, which is exactly why a 4 s sweep cell
        // would under-report it (see docs/SIH_26169_CRITICAL_AUDIT.md §9.5).
        // The centroiding and tracking orderings above detect the divergence
        // from its first frame, which is what this test needs.
    }

    SUBCASE("cold_start_in_clutter: acquisition ALSO fails, by geometry") {
        auto r = load_scenario(std::string(SAT_SCENARIO_DIR)
                               + "/hard/cold_start_in_clutter.toml");
        REQUIRE_MESSAGE(r.has_value(), r.error());
        // Row 11's random initial position is what makes this the harder of
        // the two, and it must still be random — pinning it would silently
        // turn this file into clutter_field.toml.
        REQUIRE(!r->targets.empty());
        CHECK(r->targets[0].random_initial);
    }
}
