// tests/perception/test_pipeline.cpp — CP 5.7 and the CP 5.9 ★ checkpoint.
//
// CP 5.7: "With 10% S&P + 120 clutter, candidates drop from thousands to under
//          25."
// CP 5.9: "Fog + max noise + 120 clutter + decoy: the real beacon is among the
//          top candidates in >95% of frames."
//
// These run the WHOLE pipeline against the real simulator, not against
// hand-made images. That matters: every stage before this was tested in
// isolation against an oracle, and this is the first test that can catch the
// stages disagreeing with each other.

#include <doctest/doctest.h>

#include "core/arena.hpp"
#include "engine/pipeline.hpp"
#include "perception/pipeline.hpp"
#include "scenario/schema.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using namespace sat;

namespace {

/// The perception stage plus its arena, wired the way the engine will wire it.
struct Detector {
    Arena arena;
    PerceptionWorkspace ws;
    ClassicalPerception perception;
    std::vector<Detection> detections;

    Detector(int width, int height, const PerceptionParams& p) {
        // Generously sized: this is a test, and the point of the arena here is
        // to prove the pipeline needs no allocation per frame, not to find the
        // minimum.
        arena.reserve(64u << 20);
        const bool ok = ws.allocate(arena, width, height,
                                    structuring_element_size(p.target_size_px));
        REQUIRE(ok);
        perception.configure(p);
    }

    void run(std::span<const uint8_t> px, int w, int h) {
        perception.process(px, w, h, ws, detections);
    }
};

/// Load a scenario and override the bits a test wants to control.
Scenario load(const char* name) {
    auto r = load_scenario(std::string(SAT_SCENARIO_DIR) + "/" + name);
    REQUIRE_MESSAGE(r.has_value(), r.error());
    return *r;
}

/// Place the beacon in view and stop it moving, so a perception test measures
/// perception rather than acquisition.
void pin_beacon(Scenario& sc, double dx = 90.0, double dy = 55.0) {
    REQUIRE(!sc.targets.empty());
    TargetSpec& t = sc.targets[0];
    t.random_initial = false;
    t.initial_px[0] = sc.initial_pos_px[0] + dx;
    t.initial_px[1] = sc.initial_pos_px[1] + dy;
    for (MotionSpec& m : t.motion) {
        if (m.kind == "constant") {
            m.offset_px[0] = t.initial_px[0];
            m.offset_px[1] = t.initial_px[1];
        }
    }
}

/// Rank of the detection closest to the true beacon, or -1 if none is within
/// `tol` pixels. Rank 0 is the strongest candidate.
int beacon_rank(const std::vector<Detection>& dets, Pixel2 truth_image, double tol) {
    for (size_t i = 0; i < dets.size(); ++i) {
        if ((dets[i].centroid_image - truth_image).norm() <= tol) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

}  // namespace

// ===========================================================================
// CP 5.7 — the shape gate
// ===========================================================================

TEST_CASE("CP 5.7: the gate keeps beacon-shaped blobs and rejects the rest") {
    PerceptionParams p;   // §9.4.7's defaults: area 12..800, fill >= 0.35, aspect <= 3

    auto blob = [](int64_t n, int w, int h) {
        BlobAccum b{};
        b.n = n;
        b.x0 = 0; b.x1 = static_cast<int16_t>(w - 1);
        b.y0 = 0; b.y1 = static_cast<int16_t>(h - 1);
        return b;
    };

    // A 10x10 beacon: the thing the gate exists to keep.
    CHECK(passes_gate(blob(100, 10, 10), p));
    // Spec row 10's extremes, both of which must survive.
    CHECK(passes_gate(blob(25, 5, 5), p));
    CHECK(passes_gate(blob(400, 20, 20), p));

    // Too small: a single impulse survivor, or a couple of them.
    CHECK_FALSE(passes_gate(blob(1, 1, 1), p));
    CHECK_FALSE(passes_gate(blob(9, 3, 3), p));
    // Too large: a cloud edge or an illumination gradient the top-hat missed.
    CHECK_FALSE(passes_gate(blob(900, 30, 30), p));

    // A streak — a star trail, a scratch, a cosmic ray. Aspect 10.
    CHECK_FALSE(passes_gate(blob(200, 100, 10), p));

    // A diagonal chain of impulse survivors: square bounding box, almost empty.
    // §9.4.7 names this case specifically, and aspect alone would miss it.
    CHECK_FALSE(passes_gate(blob(12, 12, 12), p));

    // A ring or a horseshoe: right size, right aspect, but hollow.
    CHECK_FALSE(passes_gate(blob(40, 14, 14), p));
}

TEST_CASE("CP 5.7: candidates drop from thousands to under 25 under full damage") {
    // The checkpoint's criterion, on the real simulator at specification-level
    // damage: 10% salt and pepper (row 21) with 120 clutter sources (§9.1).
    Scenario sc = load("spec_defaults.toml");
    sc.duration_s = 1.0;
    sc.static_sources = 120;
    sc.decoy_beacons  = 1;
    sc.salt_pepper    = 0.10;
    pin_beacon(sc);

    Pipeline engine;
    engine.build_from_scenario(sc);
    // The engine is wanted here only as a frame source: these tests run their
    // own ClassicalPerception on the snapshot so they can vary its parameters.
    // Leaving the engine's own copy on would run the whole §9.4 pipeline twice
    // per frame and double the suite's runtime for nothing.
    engine.set_detector(PipelineConfig::Detector::BrightestPixel);
    REQUIRE(engine.step());

    // acquire() BEFORE read_slot(): acquiring swaps the slot, so taking the
    // reference first hands you the previous (empty) buffer.
    (void)engine.snapshots().acquire();
    const SimSnapshot& snap = engine.snapshots().read_slot();

    PerceptionParams p;
    p.target_size_px = sc.targets[0].size_px;
    Detector det(sc.resolution[0], sc.resolution[1], p);

    // How many blobs would a naive threshold produce? Count the raw impulse
    // survivors: that is the "thousands" the checkpoint is comparing against.
    int bright_pixels = 0;
    for (uint8_t v : snap.preview) if (v > 200) ++bright_pixels;

    det.run(snap.preview, sc.resolution[0], sc.resolution[1]);

    MESSAGE("full damage (10% S&P, 120 clutter, 1 decoy):");
    MESSAGE("  raw pixels above 200:     " << bright_pixels);
    MESSAGE("  blobs after median+CFAR:  " << det.perception.last_blob_count());
    MESSAGE("  candidates after the gate: " << det.detections.size());

    // The checkpoint's number.
    CHECK(det.detections.size() < 25);
    // And the reduction is real: the raw frame genuinely has thousands of
    // bright pixels, or the test would be proving nothing.
    CHECK(bright_pixels > 1000);
}

// ===========================================================================
// CP 5.9 — the ★ worst-case checkpoint
// ===========================================================================

TEST_CASE("★ CP 5.9: the beacon is among the top candidates in >95% of frames") {
    // ---------------------------------------------------------------------
    // The Stage 5 checkpoint: "fog + max noise + 120 clutter + decoy: the real
    // beacon is among the top candidates in >95% of frames."
    //
    // Every condition at once, at the specification's limits:
    //   row 21   10% salt and pepper, Poisson shot noise
    //   row 22   read noise sigma 20, the maximum permitted
    //   row 23   20 px/frame jitter, the maximum permitted
    //   row 24   fog: contrast x0.35, floor +60
    //   §9.1     120 clutter sources, some brighter than the beacon, plus a
    //            near-identical decoy
    // ---------------------------------------------------------------------
    Scenario sc = load("fog_figure8.toml");
    sc.duration_s = 4.0;
    sc.static_sources = 120;
    sc.decoy_beacons  = 1;
    sc.salt_pepper    = 0.10;
    sc.gaussian_sigma = 20.0;
    sc.noise_poisson  = true;
    sc.jitter_px_per_frame = 20.0;
    sc.atmosphere = Atmosphere::Fog;
    pin_beacon(sc);
    // A stationary beacon, so every frame is a fair perception test rather than
    // a test of whether the (absent) tracker kept up with a figure-8.
    sc.targets[0].motion.clear();

    Pipeline engine;
    engine.build_from_scenario(sc);
    // The engine is wanted here only as a frame source: these tests run their
    // own ClassicalPerception on the snapshot so they can vary its parameters.
    // Leaving the engine's own copy on would run the whole §9.4 pipeline twice
    // per frame and double the suite's runtime for nothing.
    engine.set_detector(PipelineConfig::Detector::BrightestPixel);
    // Open loop: this measures PERCEPTION, and a control loop chasing clutter
    // would move the beacon out of frame and confound the measurement.
    engine.set_control_enabled(false);

    PerceptionParams p;
    p.target_size_px = sc.targets[0].size_px;
    Detector det(sc.resolution[0], sc.resolution[1], p);

    int frames = 0, found_top1 = 0, found_top5 = 0, found_any = 0;
    int total_candidates = 0;

    while (engine.step()) {
        const FrameRecord& r = engine.last();
        if (!r.truth_valid || !r.truth_in_fov) continue;
        (void)engine.snapshots().acquire();
        const SimSnapshot& snap = engine.snapshots().read_slot();

        det.run(snap.preview, sc.resolution[0], sc.resolution[1]);
        total_candidates += static_cast<int>(det.detections.size());

        // Where the beacon actually is on the sensor this frame.
        const Pixel2 truth_img = screen_to_image(sc.camera_geometry(),
                                                 sc.screen_geometry(),
                                                 r.truth_screen, r.boresight_true);
        // Tolerance of 3 px: this asks "did we find the beacon", not "how
        // accurately" — accuracy is CP 5.8's and Stage 9's question.
        const int rank = beacon_rank(det.detections, truth_img, 3.0);

        ++frames;
        if (rank == 0) ++found_top1;
        if (rank >= 0 && rank < 5) ++found_top5;
        if (rank >= 0) ++found_any;
    }

    REQUIRE(frames > 50);
    const double top1 = 100.0 * found_top1 / frames;
    const double top5 = 100.0 * found_top5 / frames;
    const double any  = 100.0 * found_any  / frames;

    MESSAGE("CP 5.9 worst case (fog + max noise + 120 clutter + decoy), "
            << frames << " frames:");
    MESSAGE("  beacon is the STRONGEST candidate:  " << top1 << "%");
    MESSAGE("  beacon is in the top 5:             " << top5 << "%");
    MESSAGE("  beacon is detected at all:          " << any  << "%");
    MESSAGE("  mean candidates per frame:          "
            << (static_cast<double>(total_candidates) / frames));

    // The checkpoint's criterion: "among the top candidates".
    CHECK(top5 > 95.0);
    // The candidate list stays small, which is what makes the tracker's
    // association problem tractable at Stage 6.
    CHECK(static_cast<double>(total_candidates) / frames < 25.0);
}

TEST_CASE("the pipeline survives every atmosphere with one set of parameters") {
    // CFAR's promise, end to end: no weather-dependent constant anywhere. The
    // same PerceptionParams across all five of spec row 24's modes.
    PerceptionParams p;

    for (Atmosphere mode : {Atmosphere::Clear, Atmosphere::Haze, Atmosphere::Rain,
                            Atmosphere::Fog, Atmosphere::LowLight}) {
        Scenario sc = load("spec_defaults.toml");
        sc.duration_s = 0.5;
        sc.atmosphere = mode;
        sc.static_sources = 120;
        sc.salt_pepper = 0.10;
        pin_beacon(sc);
        sc.targets[0].motion.clear();

        Pipeline engine;
        engine.build_from_scenario(sc);
    // The engine is wanted here only as a frame source: these tests run their
    // own ClassicalPerception on the snapshot so they can vary its parameters.
    // Leaving the engine's own copy on would run the whole §9.4 pipeline twice
    // per frame and double the suite's runtime for nothing.
    engine.set_detector(PipelineConfig::Detector::BrightestPixel);
        engine.set_control_enabled(false);

        p.target_size_px = sc.targets[0].size_px;
        Detector det(sc.resolution[0], sc.resolution[1], p);

        int frames = 0, found = 0;
        while (engine.step()) {
            const FrameRecord& r = engine.last();
            if (!r.truth_valid || !r.truth_in_fov) continue;
            (void)engine.snapshots().acquire();
            det.run(engine.snapshots().read_slot().preview,
                    sc.resolution[0], sc.resolution[1]);
            const Pixel2 truth_img = screen_to_image(sc.camera_geometry(),
                                                     sc.screen_geometry(),
                                                     r.truth_screen, r.boresight_true);
            if (beacon_rank(det.detections, truth_img, 3.0) >= 0) ++found;
            ++frames;
        }
        REQUIRE(frames > 5);
        const double rate = 100.0 * found / frames;
        MESSAGE("  " << std::string(atmosphere_name(mode)) << ": beacon found in " << rate
                << "% of frames");
        CHECK(rate > 90.0);
    }
}

TEST_CASE("INV-4: processing a frame allocates nothing after startup") {
    // The arena's high-water mark must not move between the first frame and the
    // hundredth. If it does, something in the pipeline is allocating per frame.
    Scenario sc = load("spec_defaults.toml");
    sc.duration_s = 3.0;
    sc.static_sources = 120;
    sc.salt_pepper = 0.10;
    pin_beacon(sc);

    Pipeline engine;
    engine.build_from_scenario(sc);
    // The engine is wanted here only as a frame source: these tests run their
    // own ClassicalPerception on the snapshot so they can vary its parameters.
    // Leaving the engine's own copy on would run the whole §9.4 pipeline twice
    // per frame and double the suite's runtime for nothing.
    engine.set_detector(PipelineConfig::Detector::BrightestPixel);
    engine.set_control_enabled(false);

    PerceptionParams p;
    p.target_size_px = sc.targets[0].size_px;
    Detector det(sc.resolution[0], sc.resolution[1], p);

    const size_t after_setup = det.arena.high_water();
    REQUIRE(engine.step());
    (void)engine.snapshots().acquire();
    det.run(engine.snapshots().read_slot().preview, sc.resolution[0], sc.resolution[1]);
    const size_t after_first = det.arena.high_water();

    int frames = 0;
    while (engine.step() && frames < 60) {
        (void)engine.snapshots().acquire();
        det.run(engine.snapshots().read_slot().preview, sc.resolution[0], sc.resolution[1]);
        ++frames;
    }
    const size_t after_many = det.arena.high_water();

    MESSAGE("arena high water: " << after_setup << " after setup, "
            << after_first << " after one frame, " << after_many
            << " after " << (frames + 1) << " frames");
    CHECK(after_first == after_setup);   // the workspace was allocated at setup
    CHECK(after_many  == after_first);   // and nothing grew since
    CHECK(det.arena.exhaustions() == 0);
}

TEST_CASE("the detection pipeline is reproducible") {
    // INV-3 through the whole perception stage, including the stable_sort that
    // orders tied candidates.
    Scenario sc = load("maxnoise_random.toml");
    sc.duration_s = 1.0;
    pin_beacon(sc);

    auto run_once = [&] {
        Pipeline engine;
        engine.build_from_scenario(sc);
    // The engine is wanted here only as a frame source: these tests run their
    // own ClassicalPerception on the snapshot so they can vary its parameters.
    // Leaving the engine's own copy on would run the whole §9.4 pipeline twice
    // per frame and double the suite's runtime for nothing.
    engine.set_detector(PipelineConfig::Detector::BrightestPixel);
        engine.set_control_enabled(false);
        PerceptionParams p;
        p.target_size_px = sc.targets[0].size_px;
        Detector det(sc.resolution[0], sc.resolution[1], p);

        std::vector<Pixel2> all;
        while (engine.step()) {
            (void)engine.snapshots().acquire();
            det.run(engine.snapshots().read_slot().preview,
                    sc.resolution[0], sc.resolution[1]);
            for (const Detection& d : det.detections) all.push_back(d.centroid_image);
        }
        return all;
    };

    const auto a = run_once();
    const auto b = run_once();
    REQUIRE(a.size() == b.size());
    REQUIRE(a.size() > 20);
    for (size_t i = 0; i < a.size(); ++i) {
        INFO("detection " << i);
        REQUIRE(a[i].x == b[i].x);
        REQUIRE(a[i].y == b[i].y);
    }
}

TEST_CASE("centroid_sigma follows the theoretical bound and is floored") {
    // Design §10.1.4, and §10.1.1's bound sigma >= w / (2 * SNR).
    CHECK(centroid_sigma(50.0f, 10) == doctest::Approx(0.1f));    // clear air, §10.1.1
    CHECK(centroid_sigma(17.0f, 10) == doctest::Approx(0.294f).epsilon(0.01));  // fog
    CHECK(centroid_sigma(7.0f, 10)  == doctest::Approx(0.714f).epsilon(0.01));  // worst case

    // The floor stops an over-confident sigma reaching the Kalman R, which is
    // how a filter ends up trusting a decoy more than it should.
    CHECK(centroid_sigma(1e6f, 10) == doctest::Approx(0.03f));
    // And a nonsensical SNR does not produce a nonsensical sigma.
    CHECK(centroid_sigma(0.0f, 10) == doctest::Approx(5.0f));
    CHECK(centroid_sigma(-5.0f, 10) == doctest::Approx(5.0f));
    CHECK(std::isfinite(centroid_sigma(0.0f, 0)));
}
