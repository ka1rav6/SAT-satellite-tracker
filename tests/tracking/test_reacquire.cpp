// tests/tracking/test_reacquire.cpp — CP 6.7's ★ end-to-end criterion, and the
// Stage 6 integration check.
//
// CP 6.7: "Spiral search + predicted-region reacquisition. Accept when: hiding
//          the beacon 2 s then revealing it -> reacquisition in under 15
//          frames."
//
// This runs the WHOLE engine: world, camera, degradation, ClassicalPerception,
// the angular conversion, the Kalman filter, the gate, the lifecycle, the mode
// FSM, the search pattern, the controller and the plant. Every unit test before
// it proved one piece against a specification; this is the first thing that can
// catch two correct pieces disagreeing about what they mean.
//
// The beacon is hidden by zeroing its intensity, not by moving it off-screen.
// That matters: moving it would let the filter's prediction be wrong in a way
// the recovery could accidentally exploit, whereas zeroing it leaves the target
// exactly where the prediction says it should be — which is the honest test of
// "predicted-region reacquisition", because the prediction has to be right for
// the right reason.

#include <doctest/doctest.h>

#include "engine/pipeline.hpp"
#include "scenario/schema.hpp"

#include <cmath>
#include <string>
#include <vector>

using namespace sat;

namespace {

/// A scenario with the beacon in view at t=0 and moving steadily, so the test
/// measures re-acquisition rather than cold acquisition (which §10.5 shows
/// cannot meet spec row 16 anyway).
Scenario base() {
    Scenario sc;
    sc.duration_s     = 8.0;
    sc.seed           = 4242;
    sc.static_sources = 0;
    sc.decoy_beacons  = 0;
    sc.salt_pepper    = 0.0;
    sc.gaussian_sigma = 4.0;
    sc.noise_poisson  = false;
    sc.hot_pixels     = 0;
    sc.jitter_px_per_frame = 0.0;

    TargetSpec t;
    t.size_px        = 10;
    t.intensity      = 120.0;
    t.random_initial = false;
    t.initial_px[0]  = 999.5 + 40.0;
    t.initial_px[1]  = 999.5 + 20.0;
    MotionSpec m;
    m.kind = "linear";
    m.velocity_px_s[0] =  35.0;
    m.velocity_px_s[1] = -18.0;
    t.motion.push_back(m);
    sc.targets.push_back(t);
    return sc;
}

/// Index of the target emitter, so the test can blank it.
size_t target_index(Pipeline& p) {
    const EmitterSoA& e = p.source().emitters();
    for (size_t i = 0; i < e.n; ++i) {
        if (static_cast<EmitterKind>(e.kind[i]) == EmitterKind::Target) return i;
    }
    REQUIRE(false);
    return 0;
}

}  // namespace

// ===========================================================================
// Stage 6 integration — the loop closes on the FILTER, not on a raw detection
// ===========================================================================

TEST_CASE("Stage 6: the wired pipeline confirms a track and holds it") {
    Scenario sc = base();
    sc.duration_s = 4.0;

    Pipeline p;
    p.build_from_scenario(sc);

    std::vector<FrameRecord> rec;
    p.run(rec);
    REQUIRE(rec.size() > 100);

    // It acquires quickly, because the beacon starts in view.
    int first_lock = -1;
    for (size_t i = 0; i < rec.size(); ++i) {
        if (rec[i].mode == TrackMode::Track) { first_lock = static_cast<int>(i); break; }
    }
    REQUIRE(first_lock >= 0);
    MESSAGE("first Track at frame " << first_lock
            << " (" << rec[static_cast<size_t>(first_lock)].time_s << " s)");
    // §10.5's acquisition_in_fov_s: the beacon is already in view, so this is
    // the M-of-N window plus one detection, and nothing more.
    CHECK(first_lock <= 6);

    // And it HOLDS it. §13.1's lock_retention_rate, over frames after the lock.
    int locked = 0, scored = 0;
    double sum_track = 0.0;
    for (size_t i = static_cast<size_t>(first_lock); i < rec.size(); ++i) {
        if (rec[i].has_lock) ++locked;
        sum_track += rec[i].tracking_error_px;
        ++scored;
    }
    const double retention = static_cast<double>(locked) / scored;
    const double mean_track = sum_track / scored;
    MESSAGE("lock retention " << (100.0 * retention) << "%, mean tracking error "
            << mean_track << " px");
    CHECK(retention > 0.95);          // spec row 18: target loss < 5%
    CHECK(mean_track < 10.0);         // spec row 17
}

TEST_CASE("Stage 6: the filter's velocity estimate matches the scenario's analytic one") {
    // The closing of the loop between Stage 3's motion algebra and Stage 6's
    // filter. The scenario states 35 and -18 px/s analytically (CP 3.5); the
    // tracker never sees those numbers and has to recover them from pixels.
    Scenario sc = base();
    sc.duration_s = 4.0;

    Pipeline p;
    p.build_from_scenario(sc);
    std::vector<FrameRecord> rec;
    p.run(rec);

    const double ifov = sc.camera_geometry().ifov_urad();
    double vx = 0.0, vy = 0.0;
    int n = 0;
    for (size_t i = rec.size() / 2; i < rec.size(); ++i) {   // settled half
        if (!rec[i].has_lock) continue;
        vx += rec[i].estimate_rate.x / ifov;
        vy += rec[i].estimate_rate.y / ifov;
        ++n;
    }
    REQUIRE(n > 30);
    vx /= n; vy /= n;
    MESSAGE("estimated velocity " << vx << ", " << vy << " px/s (truth 35, -18)");
    CHECK(vx == doctest::Approx(35.0).epsilon(0.05));
    CHECK(vy == doctest::Approx(-18.0).epsilon(0.05));
}

// ===========================================================================
// CP 6.7 — the checkpoint
// ===========================================================================

TEST_CASE("CP 6.7: hide the beacon for 2 s, and it is reacquired in under 15 frames") {
    Scenario sc = base();
    Pipeline p;
    p.build_from_scenario(sc);

    const size_t ti = target_index(p);
    const double hz = sc.camera_hz;

    // --- phase 1: acquire ------------------------------------------------
    std::vector<FrameRecord> rec;
    int frame = 0;
    auto pump = [&](int n) {
        for (int i = 0; i < n; ++i) {
            if (!p.step()) break;
            rec.push_back(p.last());
            ++frame;
        }
    };

    pump(60);                                    // 2 s to settle into Track
    REQUIRE(p.last().mode == TrackMode::Track);
    const Angle2 estimate_before = p.last().estimate;
    const Rate2  rate_before     = p.last().estimate_rate;

    // --- phase 2: hide it for 2 s ----------------------------------------
    const float kept = p.source().emitters().intensity[ti];
    p.source().emitters().intensity[ti] = 0.0f;
    const int hide_frames = static_cast<int>(2.0 * hz);
    pump(hide_frames);

    // It must have noticed. With no detection for 60 frames the track is
    // deleted (15 misses) and the FSM is back in Search, working the pattern
    // outward from where the target was last believed to be.
    MESSAGE("after 2 s hidden: mode = " << std::string(track_mode_name(p.last().mode)));
    CHECK_FALSE(p.last().has_lock);

    // --- phase 3: reveal it ----------------------------------------------
    p.source().emitters().intensity[ti] = kept;
    const int reveal_frame = frame;

    int reacquired_at = -1;
    for (int i = 0; i < 120 && reacquired_at < 0; ++i) {
        if (!p.step()) break;
        rec.push_back(p.last());
        ++frame;
        if (p.last().has_lock) reacquired_at = frame;
    }

    REQUIRE(reacquired_at > 0);
    const int frames_to_reacquire = reacquired_at - reveal_frame;
    MESSAGE("reacquired " << frames_to_reacquire << " frames after the beacon "
            << "reappeared (" << (frames_to_reacquire / hz) << " s)");

    // The checkpoint's number.
    CHECK(frames_to_reacquire < 15);
    // And spec row 19's, which is the graded one: 1 s.
    CHECK(frames_to_reacquire / hz < 1.0);

    (void)estimate_before;
    (void)rate_before;
}

TEST_CASE("CP 6.7: a short blink is ridden out on the prediction, with no mode change") {
    // The other end of the same mechanism, and the reason coasting exists: a
    // dropout shorter than the coast budget must not cost a lock at all. This
    // is what §13.1's lock_retention_rate measures and BP-2 grades.
    Scenario sc = base();
    sc.duration_s = 6.0;
    Pipeline p;
    p.build_from_scenario(sc);
    const size_t ti = target_index(p);

    for (int i = 0; i < 60; ++i) REQUIRE(p.step());
    REQUIRE(p.last().mode == TrackMode::Track);

    const float kept = p.source().emitters().intensity[ti];
    p.source().emitters().intensity[ti] = 0.0f;
    for (int i = 0; i < 8; ++i) {                 // 8 frames, CP 6.4's number
        REQUIRE(p.step());
        CHECK(p.last().has_lock);                 // never loses the lock
        CHECK(p.last().mode == TrackMode::Reacquire);
        // Note what is NOT asserted: that perception found nothing. It finds
        // several candidates a frame from noise — CFAR's measured Pfa (CP 5.5)
        // times 307,200 pixels, correlated into blobs by the matched filter's
        // box kernel, is a handful of blobs per frame and always will be.
        // Rejecting them is the gate's job, and that it does so is what the
        // 5 px tracking error below actually measures.
    }
    p.source().emitters().intensity[ti] = kept;

    // Back to Track, and the error must not have blown up: the camera kept
    // moving on the prediction throughout.
    for (int i = 0; i < 10; ++i) REQUIRE(p.step());
    CHECK(p.last().mode == TrackMode::Track);
    MESSAGE("tracking error after an 8-frame blink: "
            << p.last().tracking_error_px << " px");
    CHECK(p.last().tracking_error_px < 10.0);     // spec row 17, never breached
}

TEST_CASE("CP 6.7: with the target absent the camera searches instead of stopping") {
    // Until Stage 6 the pipeline zeroed the commanded rate whenever a frame had
    // no detection, so a missing target meant a motionless camera — which can
    // never find anything. The search pattern replaces that.
    Scenario sc = base();
    sc.duration_s = 6.0;
    Pipeline p;
    p.build_from_scenario(sc);
    const size_t ti = target_index(p);
    p.source().emitters().intensity[ti] = 0.0f;   // never visible

    std::vector<Angle2> boresights;
    for (int i = 0; i < 150; ++i) {
        if (!p.step()) break;
        boresights.push_back(p.last().boresight_true);
    }
    REQUIRE(boresights.size() > 100);
    CHECK(p.last().mode == TrackMode::Search);

    double travelled = 0.0;
    for (size_t i = 1; i < boresights.size(); ++i) {
        travelled += (boresights[i] - boresights[i - 1]).norm();
    }
    const double ifov = sc.camera_geometry().ifov_urad();
    MESSAGE("boresight travel over 5 s of fruitless search: "
            << (travelled / ifov) << " px");
    // It moved a long way — at least one full field of view, which is the
    // minimum that could possibly reveal anything new.
    CHECK(travelled > 2.0 * sc.camera_geometry().half_fov_urad().x);
}
