// tests/degrade/test_ablation.cpp — CP 4.11 and 4.12.
//
// CP 4.11's acceptance criterion is unusual: "With 120 clutter sources, the
// brightest-pixel detector demonstrably locks onto the wrong thing."
//
// It is a checkpoint whose success condition is a FAILURE, and that is the
// point. Design §9.4 builds a median filter, a van Herk top-hat, summed-area
// tables, a multi-scale matched filter and CFAR. Every one of those is work,
// and the report has to justify it. The justification is much stronger as a
// measurement than as an assertion — "the naive detector fails at 92% false
// alarms under spec-row-21 noise" beats "a naive detector would not work".
//
// So this suite adds each damage source in turn and records what it does. The
// numbers become the "before" column of the Stage 5 and Stage 11 ablation
// tables (design §13.3, CP 11.6).

#include <doctest/doctest.h>

#include "engine/pipeline.hpp"
#include "scenario/schema.hpp"

#include <string>
#include <vector>

using namespace sat;

namespace {

struct Outcome {
    double centroid_image_px  = 0.0;   ///< the detector alone
    double centroid_screen_px = 0.0;   ///< + unmeasured pointing error
    double tracking_px        = 0.0;
    double in_fov_frac        = 0.0;
    double false_alarm_frac   = 0.0;
    int    scored_frames      = 0;
};

Scenario clean_base() {
    Scenario sc;
    sc.duration_s     = 4.0;
    sc.seed           = 11;
    sc.static_sources = 0;
    sc.decoy_beacons  = 0;
    sc.noise_poisson  = false;
    sc.gaussian_sigma = 0.0;
    sc.salt_pepper    = 0.0;
    sc.hot_pixels     = 0;
    sc.jitter_px_per_frame = 0.0;

    TargetSpec t;
    t.size_px        = 10;
    t.intensity      = 120.0;
    t.random_initial = false;
    // In view at t = 0, so this measures the DETECTOR rather than acquisition.
    t.initial_px[0] = 999.5 + 120.0;
    t.initial_px[1] = 999.5 + 60.0;
    MotionSpec m;
    m.kind = "linear";
    m.velocity_px_s[0] = 22.0;
    m.velocity_px_s[1] = -11.0;
    t.motion.push_back(m);
    sc.targets.push_back(t);
    return sc;
}

// -------------------------------------------------------------------------
// The ablation runs the STRAW MAN through the real closed loop.
//
// That has to be explicit now. Since Stage 6 the engine defaults to
// ClassicalPerception — §9.4 is blunt that the brightest-pixel detector "must
// never be the default" — so an ablation that did not say which detector it
// wanted would quietly start measuring the wrong one, and the whole point of
// CP 4.11 is to record what the naive detector does. The switch exists for
// exactly this test.
// -------------------------------------------------------------------------
Outcome run(const Scenario& sc,
            PipelineConfig::Detector det = PipelineConfig::Detector::BrightestPixel) {
    Pipeline p;
    p.build_from_scenario(sc);
    p.set_detector(det);
    std::vector<FrameRecord> rec;
    p.run(rec);
    REQUIRE(!rec.empty());

    Outcome r;
    int scored = 0, in_fov = 0, false_alarms = 0, tail_n = 0;
    for (size_t i = 0; i < rec.size(); ++i) {
        const FrameRecord& f = rec[i];
        if (f.centroid_error_valid) {
            r.centroid_image_px  += f.centroid_error_px;
            r.centroid_screen_px += f.centroid_error_screen_px;
            ++scored;
        }
        if (f.truth_in_fov) ++in_fov;
        if (f.false_alarm)  ++false_alarms;
        if (i >= rec.size() * 3 / 4) { r.tracking_px += f.tracking_error_px; ++tail_n; }
    }
    if (scored) { r.centroid_image_px /= scored; r.centroid_screen_px /= scored; }
    if (tail_n) r.tracking_px /= tail_n;
    r.in_fov_frac      = static_cast<double>(in_fov) / rec.size();
    r.false_alarm_frac = static_cast<double>(false_alarms) / rec.size();
    r.scored_frames    = scored;
    return r;
}

}  // namespace

TEST_CASE("the detector is accurate on a clean frame") {
    const Outcome r = run(clean_base());
    MESSAGE("clean: centroid " << r.centroid_image_px << " px (image), tracking "
            << r.tracking_px << " px");
    CHECK(r.in_fov_frac == doctest::Approx(1.0));
    CHECK(r.false_alarm_frac == doctest::Approx(0.0));
    CHECK(r.centroid_image_px < 1.5);
    CHECK(r.tracking_px < 10.0);        // inside spec row 17
}

TEST_CASE("INV-6: jitter and platform drift move the SCREEN centroid, not the IMAGE one") {
    // The clearest demonstration of why design §13.2 carries both coordinate
    // pairs. Jitter (row 23) and platform motion (row 25) perturb the true
    // boresight, and §10.3's encoder does not see them — so converting a
    // detection to screen coordinates through the COMMANDED boresight inherits
    // that error. The detector's own accuracy is untouched.
    Scenario sc = clean_base();
    sc.jitter_px_per_frame = 20.0;      // spec row 23, at the maximum

    const Outcome r = run(sc);
    MESSAGE("with 20 px/frame jitter: centroid image " << r.centroid_image_px
            << " px, screen " << r.centroid_screen_px << " px");

    CHECK(r.in_fov_frac > 0.95);
    // The detector still finds the beacon to well under a pixel...
    CHECK(r.centroid_image_px < 1.5);
    // ...while the screen-frame figure is dominated by the unmeasured pointing
    // error, and cannot be better than the jitter amplitude.
    CHECK(r.centroid_screen_px > 5.0);
    CHECK(r.centroid_screen_px > r.centroid_image_px * 5.0);
}

TEST_CASE("the detector survives shot noise and read noise at the spec maximum") {
    // Spec rows 21 and 22. These are ADDITIVE and roughly symmetric, so a
    // centre-of-mass estimator averages most of them away — which is why they
    // are not what breaks the naive detector.
    Scenario sc = clean_base();
    sc.gaussian_sigma = 20.0;           // row 22, at the cap
    sc.noise_poisson  = true;           // row 21

    const Outcome r = run(sc);
    MESSAGE("with shot + read noise: centroid " << r.centroid_image_px
            << " px, false alarms " << (100.0 * r.false_alarm_frac) << "%");
    CHECK(r.centroid_image_px < 2.0);
    CHECK(r.false_alarm_frac < 0.05);
}

TEST_CASE("CP 4.11: salt and pepper destroys the brightest-pixel detector") {
    // THE checkpoint. Spec row 21 asks for ~10% impulse noise, which is 30,720
    // corrupted pixels against a 100-pixel beacon — design §1.4's 307:1 ratio.
    // Every one of those impulses is a 255, so on PEAK BRIGHTNESS they all beat
    // a 120-level beacon outright.
    //
    // This is the measurement that justifies §9.4.1's median filter: "a median
    // removes nearly all of it while barely touching a solid blob."
    Scenario clean = clean_base();
    Scenario noisy = clean_base();
    noisy.salt_pepper = 0.10;           // row 21

    const Outcome before = run(clean);
    const Outcome after  = run(noisy);

    MESSAGE("brightest-pixel detector, spec row 21 salt & pepper:");
    MESSAGE("  centroid error  " << before.centroid_image_px << " px -> "
            << after.centroid_image_px << " px");
    MESSAGE("  tracking error  " << before.tracking_px << " px -> "
            << after.tracking_px << " px");
    MESSAGE("  false alarms    " << (100.0 * before.false_alarm_frac) << "% -> "
            << (100.0 * after.false_alarm_frac) << "%");
    MESSAGE("  beacon in view  " << (100.0 * before.in_fov_frac) << "% -> "
            << (100.0 * after.in_fov_frac) << "%");

    // It fails, and it fails comprehensively.
    CHECK(after.false_alarm_frac > 0.5);          // mostly locking onto noise
    CHECK(after.tracking_px > 100.0);             // and dragging the camera away
    CHECK(after.in_fov_frac < 0.5);               // until the beacon is lost
    CHECK(after.tracking_px > before.tracking_px * 20.0);
}

TEST_CASE("CP 4.11: clutter gives the naive detector something to lock onto") {
    // Design §9.1 calls 50-500 static sources "mandatory for credibility", and
    // deliberately makes some of them BRIGHTER than the beacon. If every clutter
    // source were dimmer, a brightest-pixel detector would still work and the
    // case for CFAR would be unproven.
    Scenario sc = clean_base();
    sc.static_sources = 120;
    sc.decoy_beacons  = 1;
    // No impulse noise, so this isolates clutter as the cause.
    const Outcome r = run(sc);

    MESSAGE("with 120 clutter sources and 1 decoy: centroid "
            << r.centroid_image_px << " px, tracking " << r.tracking_px
            << " px, false alarms " << (100.0 * r.false_alarm_frac) << "%");

    // Either it locks onto clutter (false alarms, or a large centroid error
    // because the "detection" is a different object entirely) or it loses the
    // beacon. Any of those is CP 4.11's "demonstrably locks onto the wrong
    // thing"; asserting the disjunction rather than one specific symptom keeps
    // the test from being brittle about WHICH way it fails.
    const bool degraded = r.false_alarm_frac > 0.1
                       || r.centroid_image_px > 5.0
                       || r.tracking_px > 20.0;
    CHECK(degraded);
}

TEST_CASE("the classical pipeline survives what destroys the straw man") {
    // The other half of the ablation, and the half that makes CP 4.11 a RESULT
    // rather than a complaint. "The naive detector fails" is only an argument
    // for §9.4 if the thing §9.4 built does not.
    //
    // Same two scenarios, same seed, same closed loop — only the detector
    // changes. These numbers are the "after" column of the Stage 5 row of
    // §13.3's ablation table.
    // Shorter than the straw-man arms above. Four runs of the full §9.4
    // pipeline through the closed loop is the most expensive thing in the test
    // suite, and 2 s (60 frames) is ample for the statistics asserted below —
    // the differences being measured are three orders of magnitude, not
    // marginal. Keeping this suite comfortably inside its timeout matters:
    // the Debug build runs the same code roughly five times slower.
    Scenario noise = clean_base();
    noise.duration_s     = 2.0;
    noise.salt_pepper    = 0.10;          // spec row 21
    noise.gaussian_sigma = 20.0;          // spec row 22, at the cap
    noise.noise_poisson  = true;

    Scenario clutter = clean_base();
    clutter.duration_s     = 2.0;
    clutter.static_sources = 120;         // design §9.1, "mandatory"
    clutter.decoy_beacons  = 1;

    struct Arm { const char* name; Scenario sc; };
    for (const Arm& a : {Arm{"spec-max noise", noise}, Arm{"120 clutter + decoy", clutter}}) {
        const Outcome naive     = run(a.sc, PipelineConfig::Detector::BrightestPixel);
        const Outcome classical = run(a.sc, PipelineConfig::Detector::Classical);

        MESSAGE(std::string(a.name) << ":");
        MESSAGE("  centroid (image)  straw man " << naive.centroid_image_px
                << " px -> classical " << classical.centroid_image_px << " px");
        MESSAGE("  tracking          straw man " << naive.tracking_px
                << " px -> classical " << classical.tracking_px << " px");
        MESSAGE("  false alarms      straw man " << (100.0 * naive.false_alarm_frac)
                << "% -> classical " << (100.0 * classical.false_alarm_frac) << "%");
        MESSAGE("  beacon in view    straw man " << (100.0 * naive.in_fov_frac)
                << "% -> classical " << (100.0 * classical.in_fov_frac) << "%");

        // The classical pipeline keeps the beacon and keeps the lock. The
        // thresholds are spec row 17's 10 px and row 18's 5% target loss, so
        // this is written against the graded requirement rather than against
        // whatever the code happens to do today.
        CHECK(classical.in_fov_frac      > 0.95);
        CHECK(classical.false_alarm_frac < 0.05);
        CHECK(classical.tracking_px      < 10.0);
        CHECK(classical.centroid_image_px < 2.0);
    }
}

TEST_CASE("CP 4.12: every scenario in scenarios/ runs to completion") {
    // The spec-complete checkpoint. Not "tracks well" — that is Stage 5's job —
    // but "runs", which means every row of the parameter table is wired to
    // something that executes without crashing or producing a NaN.
    for (const char* name : {"baseline.toml", "fog_figure8.toml",
                             "maxnoise_random.toml"}) {
        INFO("scenario = " << std::string(name));
        auto r = load_scenario(std::string(SAT_SCENARIO_DIR) + "/" + name);
        REQUIRE_MESSAGE(r.has_value(), r.error());

        Scenario sc = *r;
        sc.duration_s = 2.0;            // keep the suite quick

        Pipeline p;
        p.build_from_scenario(sc);
        std::vector<FrameRecord> rec;
        p.run(rec);

        CHECK(rec.size() == static_cast<size_t>(sc.duration_s * sc.camera_hz));
        // CP 14.1's standard: no NaN in any logged value.
        for (const FrameRecord& f : rec) {
            REQUIRE(std::isfinite(f.boresight_true.x));
            REQUIRE(std::isfinite(f.boresight_true.y));
            REQUIRE(std::isfinite(f.tracking_error_px));
            REQUIRE(std::isfinite(f.centroid_error_px));
            REQUIRE(std::isfinite(f.cmd_rate.x));
        }
        // And the world really was populated: targets, decoys and clutter.
        CHECK(p.source().emitters().n >= static_cast<size_t>(sc.static_sources));
    }
}

TEST_CASE("CP 4.12: a scenario run is reproducible with the full damage chain on") {
    // INV-3 with every stochastic source active — shot noise, read noise,
    // salt and pepper, fixed pattern, jitter, platform motion. This is the
    // configuration where a shared RNG stream or an unordered iteration would
    // actually show up.
    auto r = load_scenario(std::string(SAT_SCENARIO_DIR) + "/maxnoise_random.toml");
    REQUIRE_MESSAGE(r.has_value(), r.error());
    Scenario sc = *r;
    sc.duration_s = 2.0;

    std::vector<FrameFingerprint> a, b;
    { Pipeline p; p.build_from_scenario(sc); while (p.step()) {} a = p.fingerprints(); }
    { Pipeline p; p.build_from_scenario(sc); while (p.step()) {} b = p.fingerprints(); }

    REQUIRE(a.size() == b.size());
    REQUIRE(a.size() > 20);
    for (size_t i = 0; i < a.size(); ++i) {
        INFO("frame " << i);
        REQUIRE(a[i].combined() == b[i].combined());
    }
}

TEST_CASE("changing the seed changes the run, with damage enabled") {
    // The complement: if the seed did nothing, "reproducible" would be vacuous.
    auto r = load_scenario(std::string(SAT_SCENARIO_DIR) + "/maxnoise_random.toml");
    REQUIRE(r.has_value());
    Scenario sc = *r;
    sc.duration_s = 1.0;

    Scenario other = sc;
    other.seed = sc.seed + 1;

    std::vector<FrameFingerprint> a, b;
    { Pipeline p; p.build_from_scenario(sc);    while (p.step()) {} a = p.fingerprints(); }
    { Pipeline p; p.build_from_scenario(other); while (p.step()) {} b = p.fingerprints(); }

    REQUIRE(a.size() == b.size());
    CHECK(a.back().combined() != b.back().combined());
}
