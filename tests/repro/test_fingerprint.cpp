// tests/repro/test_fingerprint.cpp — CP 2.5.
//
// "snapshot_hash() — FNV-1a over frame, boresight, detection, mode. Same
//  scenario + seed twice in one session gives identical hash sequences."
//
// The fingerprint has to be sensitive in both directions: it must be identical
// for identical runs (or it reports failures that are not there, which is worse
// than not checking at all) and it must change when anything observable changes
// (or it reports success while the system drifts).

#include <doctest/doctest.h>

#include "engine/pipeline.hpp"
#include "engine/snapshot.hpp"

#include <vector>

using namespace sat;

namespace {

SimSnapshot make_snapshot() {
    SimSnapshot s;
    s.reserve_preview(8, 8);
    s.frame            = 42;
    s.time_s           = 1.4;
    s.boresight_cmd    = Angle2{100.0, -50.0};
    s.boresight_true   = Angle2{101.0, -49.0};
    s.cmd_rate         = Rate2{10.0, 20.0};
    s.gimbal_rate      = Rate2{9.5, 19.5};
    s.detected         = true;
    s.detection_img    = Pixel2{320.25, 240.75};
    s.detection_snr    = 38.4f;
    s.candidate_count  = 3;
    s.mode             = TrackMode::Track;
    s.track_state      = 2;
    std::vector<uint8_t> img(64, 7);
    img[13] = 200;
    s.set_preview(img);
    return s;
}

PipelineConfig base_config() {
    PipelineConfig cfg;
    cfg.synthetic.camera     = CameraGeometry::make(320, 240, 4.0, 3.0);
    cfg.synthetic.screen     = ScreenGeometry::make(2000, 2000, cfg.synthetic.camera);
    cfg.synthetic.duration_s = 1.5;
    cfg.synthetic.seed       = 99;
    cfg.pan   = GimbalParams::from_dps(5.0, 50.0);
    cfg.tilt  = GimbalParams::from_dps(5.0, 50.0);
    cfg.gains = ControlGains::proportional(4.0);
    return cfg;
}

EmitterSoA base_emitters(const ScreenGeometry& scr) {
    EmitterSoA e;
    e.add(scr.cx + 90.0, scr.cy + 40.0, 220.0f, 10, ShapeKind::Square, EmitterKind::Target);
    e.vx[0] = 25.0;
    e.vy[0] = -10.0;
    return e;
}

}  // namespace

TEST_CASE("an identical snapshot fingerprints identically") {
    CHECK(fingerprint(make_snapshot()).combined()
          == fingerprint(make_snapshot()).combined());
}

TEST_CASE("the fingerprint notices every observable change") {
    const uint64_t base = fingerprint(make_snapshot()).combined();

    SUBCASE("one pixel") {
        SimSnapshot s = make_snapshot();
        s.preview[20] = 8;
        CHECK(fingerprint(s).combined() != base);
    }
    SUBCASE("the commanded boresight, by one ulp") {
        SimSnapshot s = make_snapshot();
        s.boresight_cmd.x = std::nextafter(s.boresight_cmd.x, 1e9);
        CHECK(fingerprint(s).combined() != base);
    }
    SUBCASE("the true boresight") {
        SimSnapshot s = make_snapshot();
        s.boresight_true.y += 1e-9;
        CHECK(fingerprint(s).combined() != base);
    }
    SUBCASE("the commanded rate") {
        SimSnapshot s = make_snapshot();
        s.cmd_rate.x += 1e-9;
        CHECK(fingerprint(s).combined() != base);
    }
    SUBCASE("whether anything was detected") {
        SimSnapshot s = make_snapshot();
        s.detected = false;
        CHECK(fingerprint(s).combined() != base);
    }
    SUBCASE("the detection position, above the declared tolerance") {
        SimSnapshot s = make_snapshot();
        s.detection_img.x += 1e-3;      // a milli-pixel
        CHECK(fingerprint(s).combined() != base);
    }
    SUBCASE("the FSM mode") {
        SimSnapshot s = make_snapshot();
        s.mode = TrackMode::Reacquire;
        CHECK(fingerprint(s).combined() != base);
    }
    SUBCASE("the candidate count") {
        SimSnapshot s = make_snapshot();
        s.candidate_count = 4;
        CHECK(fingerprint(s).combined() != base);
    }
}

TEST_CASE("the fingerprint ignores what it should ignore") {
    const uint64_t base = fingerprint(make_snapshot()).combined();

    SUBCASE("truth, because it is an input not an output") {
        // Including truth would mask a divergence in the TRACKER behind
        // agreement in the world — the fingerprint would go green while the
        // algorithm drifted.
        SimSnapshot s = make_snapshot();
        s.truth_screen = Pixel2{999.0, 999.0};
        s.truth_valid  = false;
        s.truth_in_fov = false;
        CHECK(fingerprint(s).combined() == base);
    }
    SUBCASE("computed metrics, which are derived from what is already hashed") {
        SimSnapshot s = make_snapshot();
        s.centroid_error_px = 123.0;
        s.tracking_error_px = 456.0;
        CHECK(fingerprint(s).combined() == base);
    }
    SUBCASE("a sub-tolerance detection wobble") {
        // Design §11.4: values that pass through libm or a network are hashed
        // at a declared tolerance rather than claimed bit-exact. A nano-pixel
        // is three orders of magnitude below the declared micro-pixel and six
        // below the graded metric.
        SimSnapshot s = make_snapshot();
        s.detection_img.x += 1e-9;
        CHECK(fingerprint(s).combined() == base);
    }
}

TEST_CASE("the split fingerprint says WHICH component diverged") {
    // This is the entire reason FrameFingerprint is a struct rather than one
    // number: "the image is identical but the detection moved" points at
    // perception, "the image differs" points at the world or the damage chain.
    const FrameFingerprint a = fingerprint(make_snapshot());

    SimSnapshot moved = make_snapshot();
    moved.detection_img.x += 0.5;
    const FrameFingerprint b = fingerprint(moved);

    CHECK(a.image     == b.image);        // the world agreed
    CHECK(a.boresight == b.boresight);    // the control agreed
    CHECK(a.detection != b.detection);    // perception did not
    CHECK(a.combined() != b.combined());
}

TEST_CASE("CP 2.5: the same scenario and seed give identical fingerprint sequences") {
    PipelineConfig cfg = base_config();

    std::vector<FrameFingerprint> a, b;
    { Pipeline p; p.build(cfg, base_emitters(cfg.synthetic.screen));
      while (p.step()) {} a = p.fingerprints(); }
    { Pipeline p; p.build(cfg, base_emitters(cfg.synthetic.screen));
      while (p.step()) {} b = p.fingerprints(); }

    REQUIRE(a.size() > 20);
    REQUIRE(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        INFO("frame " << i);
        REQUIRE(a[i].combined() == b[i].combined());
    }
}

TEST_CASE("changing the configuration changes the fingerprints") {
    // The complement of the test above. If the sequence were insensitive to
    // configuration, "identical" would be meaningless.
    PipelineConfig cfg = base_config();
    std::vector<FrameFingerprint> base;
    { Pipeline p; p.build(cfg, base_emitters(cfg.synthetic.screen));
      while (p.step()) {} base = p.fingerprints(); }

    SUBCASE("a different loop gain") {
        PipelineConfig c2 = cfg;
        c2.gains = ControlGains::proportional(8.0);
        Pipeline p; p.build(c2, base_emitters(cfg.synthetic.screen));
        while (p.step()) {}
        CHECK(p.fingerprints().back().combined() != base.back().combined());
    }
    SUBCASE("control disabled") {
        PipelineConfig c2 = cfg;
        c2.control_enabled = false;
        Pipeline p; p.build(c2, base_emitters(cfg.synthetic.screen));
        while (p.step()) {}
        CHECK(p.fingerprints().back().combined() != base.back().combined());
    }
    SUBCASE("a different target position") {
        EmitterSoA e = base_emitters(cfg.synthetic.screen);
        e.x[0] += 1.0;
        Pipeline p; p.build(cfg, std::move(e));
        while (p.step()) {}
        CHECK(p.fingerprints().back().combined() != base.back().combined());
    }
}

TEST_CASE("CP 2.3: the snapshot seam is exercised, ready for the GUI at CP 15.0") {
    // The §14.0 amendment defers the dashboard on the grounds that the seam
    // exists and is used. A seam nobody publishes through is not a seam.
    PipelineConfig cfg = base_config();
    Pipeline p;
    p.build(cfg, base_emitters(cfg.synthetic.screen));

    REQUIRE(p.step());
    REQUIRE(p.snapshots().acquire());
    const SimSnapshot& s = p.snapshots().read_slot();

    CHECK(s.preview_w == cfg.synthetic.camera.width);
    CHECK(s.preview_h == cfg.synthetic.camera.height);
    CHECK(s.preview.size()
          == static_cast<size_t>(cfg.synthetic.camera.pixel_count()));
    CHECK(s.frame == 0);

    // The preview is the frame the detector saw, not an empty buffer.
    bool any_signal = false;
    for (uint8_t v : s.preview) if (v > 20) { any_signal = true; break; }
    CHECK(any_signal);
}

TEST_CASE("CP 2.3: a slow consumer drops frames instead of stalling the simulation") {
    // The property the triple buffer exists for. The simulation publishes every
    // frame; a consumer that reads only occasionally gets the newest one and
    // misses the rest, rather than backing up a queue or blocking the producer.
    PipelineConfig cfg = base_config();
    cfg.synthetic.duration_s = 2.0;

    Pipeline p;
    p.build(cfg, base_emitters(cfg.synthetic.screen));

    int published = 0, consumed = 0;
    while (p.step()) {
        ++published;
        if (published % 5 == 0 && p.snapshots().acquire()) {
            ++consumed;
            // Whatever we get must be a complete, self-consistent snapshot.
            CHECK(p.snapshots().read_slot().preview.size()
                  == static_cast<size_t>(cfg.synthetic.camera.pixel_count()));
        }
    }
    CHECK(published > 50);
    CHECK(consumed > 0);
    CHECK(consumed < published);        // frames were dropped, as intended
}

TEST_CASE("track mode names cover every enumerator") {
    // These strings go into centroid.csv's `state` column (design §13.2), where
    // an "UNKNOWN" would make a graded artifact unreadable.
    for (uint8_t i = 0; i <= static_cast<uint8_t>(TrackMode::Safe); ++i) {
        INFO("mode " << static_cast<int>(i));
        CHECK(std::string(track_mode_name(static_cast<TrackMode>(i))) != "UNKNOWN");
    }
}
