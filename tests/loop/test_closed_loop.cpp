// tests/loop/test_closed_loop.cpp — CP 1.8, the Stage 1 ★ GATE.
//
// The checkpoint's three acceptance criteria, verbatim:
//
//   (a) The camera visibly pulls the square toward centre.
//   (b) Commenting out the line applying the command stops the following;
//       uncommenting restores it.
//   (c) grep -r "truth\|emitter\|world" src/perception src/control returns
//       nothing.
//
// Per the §14.0 amendment, (a)'s visual criterion becomes a numeric one: the
// boresight-to-target error must decrease monotonically and settle. (b) becomes
// a flag rather than an edit-and-rebuild ritual, so it runs on every commit.
// (c) is enforced three ways — by `just gate-inv1`, by the configure-time link
// assertion in cmake/modules.cmake, and by the fact that the detector is handed
// a pixel span rather than a SourceFrame.
//
// ---------------------------------------------------------------------------
// WHY (b) IS THE ONE THAT MATTERS
// ---------------------------------------------------------------------------
// A simulation that renders a nicely centred beacon regardless of the controller
// looks exactly the same on screen as one that genuinely tracks. The open-loop
// comparison is the only thing that distinguishes them, and without it every
// number the project reports could be an artifact of the renderer.

#include <doctest/doctest.h>

#include "engine/pipeline.hpp"
#include "scenario/schema.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace sat;

namespace {

// A scene with one beacon, offset from the boresight and drifting across the
// screen, on a 20 px/s straight line (spec row 12's mandatory "straight line").
struct Scene {
    PipelineConfig cfg;
    EmitterSoA     emitters;
    double         ifov = 0.0;
};

Scene make_scene(double offset_px = 120.0, double vx_px_s = 20.0, double duration_s = 8.0) {
    Scene s;
    s.cfg.synthetic.camera     = CameraGeometry::make(640, 480, 4.0, 3.0);
    s.cfg.synthetic.screen     = ScreenGeometry::make(2000, 2000, s.cfg.synthetic.camera);
    s.cfg.synthetic.duration_s = duration_s;
    s.cfg.synthetic.seed       = 7;
    s.cfg.synthetic.background = 8.0f;
    s.cfg.synthetic.blur_substeps = 4;

    // Spec rows 13-14 defaults: 5 deg/s, and a realistically imperfect mount.
    s.cfg.pan  = GimbalParams::from_dps(5.0, 50.0);
    s.cfg.tilt = GimbalParams::from_dps(5.0, 50.0);

    // Spec row 6: the camera starts at the centre of the screen.
    s.cfg.initial_boresight = Angle2{0.0, 0.0};
    s.cfg.gains             = ControlGains::proportional(4.0);

    // -------------------------------------------------------------------
    // The STRAW-MAN detector, on purpose.
    //
    // These are CP 1.6-1.8: they are about the CONTROL LOOP — does the
    // commanded rate actually move the mount, and does disabling it stop the
    // following — and they were written against the brightest-pixel detector
    // because that is all Stage 1 had. Nothing here depends on detection
    // quality: the scene is one bright beacon on a clean background.
    //
    // Since Stage 6 the engine defaults to ClassicalPerception, and running
    // the full §9.4 pipeline on every frame of these scenarios took the suite
    // from under a second to past its two-minute timeout — for tests that do
    // not measure perception at all. The closed loop IS exercised against the
    // real pipeline, in tests/tracking/test_reacquire.cpp, where that is the
    // thing being measured.
    // -------------------------------------------------------------------
    s.cfg.detector = PipelineConfig::Detector::BrightestPixel;

    // The beacon: spec rows 7, 9, 10 — a 10x10 square beacon, placed off-centre
    // so there is a real error for the loop to remove.
    const double cx = s.cfg.synthetic.screen.cx;
    const double cy = s.cfg.synthetic.screen.cy;
    s.emitters.add(cx + offset_px, cy + offset_px * 0.5, 220.0f, 10,
                   ShapeKind::Square, EmitterKind::Target);
    s.emitters.vx[0] = vx_px_s;

    s.ifov = s.cfg.synthetic.camera.ifov_urad();
    return s;
}

/// Boresight-to-target distance in screen pixels, which is design §13.1's
/// tracking_error.
double tracking_error_px(const FrameRecord& r) { return r.tracking_error_px; }

}  // namespace

TEST_CASE("★ GATE CP 1.8 (a): the closed loop pulls the beacon toward centre") {
    Scene s = make_scene();
    Pipeline p;
    p.build(s.cfg, s.emitters);

    std::vector<FrameRecord> rec;
    p.run(rec);

    REQUIRE(rec.size() > 100);

    const double start = tracking_error_px(rec.front());
    // Average over the last half-second rather than taking a single frame, so
    // the result is not at the mercy of one noisy sample.
    double settled = 0.0;
    const size_t tail = 15;
    for (size_t i = rec.size() - tail; i < rec.size(); ++i) {
        settled += tracking_error_px(rec[i]);
    }
    settled /= static_cast<double>(tail);

    MESSAGE("tracking error: " << start << " px at start -> " << settled << " px settled");

    CHECK(start > 100.0);      // it really did begin badly off-target
    CHECK(settled < start / 20.0);

    // Spec row 17's budget is 10 px. Being inside it with nothing but a P
    // controller and a brightest-pixel detector is the point of this gate: the
    // structure is right before any of the sophistication exists.
    CHECK(settled < 10.0);

    // The residual is not noise, it is the textbook steady-state lag of a
    // proportional controller following a ramp: error = v / kp. With a
    // 20 px/s target and kp = 4 /s that is 5 px, and the measurement lands
    // there. Asserting the PREDICTED value rather than a loose bound means a
    // future regression in the plant or the loop gain shows up here instead of
    // hiding under a generous threshold.
    //
    // Removing this lag is exactly what velocity feedforward does at CP 10.1,
    // and this number is the "before" in that comparison.
    const double predicted_lag_px = 20.0 / 4.0;
    CHECK(settled == doctest::Approx(predicted_lag_px).epsilon(0.25));
}

TEST_CASE("★ GATE CP 1.8 (a): the error decreases, it does not oscillate") {
    Scene s = make_scene();
    Pipeline p;
    p.build(s.cfg, s.emitters);

    std::vector<FrameRecord> rec;
    p.run(rec);
    REQUIRE(rec.size() > 100);

    // During ACQUISITION the error must fall steadily. The window stops at
    // frame 30 because by then the loop has reached its steady-state lag and is
    // simply sitting there; continuing to demand improvement past that point
    // would be demanding the impossible, and an earlier draft of this test did
    // exactly that and failed on a 0.03 px wobble at the noise floor.
    for (size_t i = 10; i <= 30; i += 10) {
        INFO("frame " << i);
        CHECK(tracking_error_px(rec[i]) < tracking_error_px(rec[i - 10]));
    }

    // No overshoot. A loop that rang would cross the settled value and come
    // back; requiring the approach to stay above it rules that out.
    double settled = 0.0;
    for (size_t i = rec.size() - 15; i < rec.size(); ++i) settled += tracking_error_px(rec[i]);
    settled /= 15.0;
    for (size_t i = 0; i <= 25; ++i) {
        INFO("frame " << i << " during approach");
        CHECK(tracking_error_px(rec[i]) > settled * 0.9);
    }

    // And once settled it stays settled — no late divergence as the target
    // continues to drift across the screen, and no slow creep.
    double worst_tail = 0.0, best_tail = 1e9;
    for (size_t i = rec.size() / 2; i < rec.size(); ++i) {
        worst_tail = std::max(worst_tail, tracking_error_px(rec[i]));
        best_tail  = std::min(best_tail,  tracking_error_px(rec[i]));
    }
    CHECK(worst_tail < 10.0);                       // inside spec row 17
    CHECK(worst_tail - best_tail < 1.0);            // and genuinely steady
}

TEST_CASE("★ GATE CP 1.8 (b): with the command not applied, the camera does NOT follow") {
    // THE decisive test. Same scene, same detector, same everything — only the
    // line that applies the controller's output to the plant is disabled.
    Scene s = make_scene();

    std::vector<FrameRecord> closed, open;
    {
        Pipeline p;
        s.cfg.control_enabled = true;
        p.build(s.cfg, s.emitters);
        p.run(closed);
    }
    {
        Pipeline p;
        s.cfg.control_enabled = false;
        p.build(s.cfg, s.emitters);
        p.run(open);
    }

    REQUIRE(closed.size() == open.size());
    REQUIRE(closed.size() > 100);

    const double closed_end = tracking_error_px(closed.back());
    const double open_end   = tracking_error_px(open.back());

    MESSAGE("final tracking error: closed loop = " << closed_end
            << " px, open loop = " << open_end << " px");

    // Open loop: the mount never moves, so the error is at least the initial
    // offset and grows as the target drifts away.
    CHECK(open_end > 100.0);
    CHECK(open_end > tracking_error_px(open.front()));

    // Closed loop: converged to the P-controller's ramp lag.
    CHECK(closed_end < 10.0);

    // And the two must differ by orders of magnitude. If disabling the control
    // barely changed anything, the "tracking" was coming from somewhere else.
    CHECK(open_end > closed_end * 20.0);

    // The boresight itself must literally not have moved with control off.
    CHECK(open.back().boresight_true.x == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(open.back().boresight_true.y == doctest::Approx(0.0).epsilon(1e-9));
    // ...while the closed loop moved it a long way.
    CHECK(std::fabs(closed.back().boresight_true.x) > 1000.0);
}

TEST_CASE("★ GATE CP 1.8: re-enabling control mid-run restores tracking") {
    // The live-demo version of (b): CP 15.2 turns feedforward off on stage and
    // watches the trace blow up. This is the same shape, and it proves the
    // dependence is causal rather than a difference between two separate runs.
    Scene s = make_scene(120.0, 20.0, 12.0);
    Pipeline p;
    s.cfg.control_enabled = true;
    p.build(s.cfg, s.emitters);

    std::vector<double> err;
    int frame = 0;
    while (p.step()) {
        // Converge, then open the loop, then close it again.
        if (frame == 120) p.set_control_enabled(false);
        if (frame == 240) p.set_control_enabled(true);
        err.push_back(p.last().tracking_error_px);
        ++frame;
    }
    REQUIRE(err.size() > 300);

    const double converged = err[119];
    const double drifted   = err[239];
    const double recovered = err.back();

    MESSAGE("converged " << converged << " px -> opened " << drifted
            << " px -> reclosed " << recovered << " px");

    CHECK(converged < 10.0);
    CHECK(drifted > converged * 5.0);    // it visibly fell apart
    CHECK(recovered < 10.0);             // and came back
    // Recovery must return to the same place, not merely somewhere better.
    CHECK(recovered == doctest::Approx(converged).epsilon(0.1));
}

TEST_CASE("CP 1.8: the disturbance is applied to the true boresight, not the pixels") {
    // Design §9.3: disturbances perturb the TRUE boresight, and the tracker is
    // told only the commanded one. That difference is the pointing error the
    // system cannot observe directly — the entire problem. If the source were
    // handed truth, this difference would be zero and the simulation would be
    // measuring nothing.
    Scene s = make_scene();
    Pipeline p;
    p.build(s.cfg, s.emitters);

    // Inject a fixed boresight disturbance of 20 px, spec row 23's magnitude.
    const double ifov = s.cfg.synthetic.camera.ifov_urad();
    p.source().set_boresight_disturbance(Angle2{20.0 * ifov, 0.0});

    REQUIRE(p.step());
    const FrameRecord& r = p.last();

    // The commanded and true boresights must now differ by exactly the
    // disturbance.
    CHECK((r.boresight_true.x - r.boresight_cmd.x)
          == doctest::Approx(20.0 * ifov).epsilon(1e-9));
}

TEST_CASE("INV-6: centroiding error and tracking error are separate quantities") {
    // They are computed, stored and reported separately, and they measure
    // different things: one the detector, the other the control loop. Early in
    // a run the camera is badly pointed (large tracking error) while the beacon
    // is still located accurately in the image it does see (small centroiding
    // error). If the two were conflated that distinction would vanish.
    Scene s = make_scene();
    Pipeline p;
    p.build(s.cfg, s.emitters);

    std::vector<FrameRecord> rec;
    p.run(rec);
    REQUIRE(rec.size() > 50);

    const FrameRecord& early = rec[2];
    REQUIRE(early.truth_valid);
    REQUIRE(early.detected);

    MESSAGE("early frame: tracking error " << early.tracking_error_px
            << " px, centroiding error " << early.centroid_error_px << " px");

    CHECK(early.tracking_error_px > 50.0);    // badly pointed
    CHECK(early.centroid_error_px < 10.0);    // but far better located
    // An order of magnitude apart, which is the whole point of INV-6: they
    // measure different subsystems and must never be added together.
    CHECK(early.centroid_error_px < early.tracking_error_px / 5.0);
}

TEST_CASE("INV-9: no detection means no centroid, never a stale one") {
    // Point the camera at empty sky: there is no beacon in frame, so there must
    // be no reported centroid. Design §13.2 is explicit that the columns are
    // BLANK — "never write the last known value, never write the prediction."
    Scene s = make_scene();
    // Put the beacon far outside the field of view and stop it moving.
    s.emitters.x[0] = s.cfg.synthetic.screen.cx + 900.0;
    s.emitters.vx[0] = 0.0;
    s.cfg.control_enabled = false;   // keep the camera pointed away

    Pipeline p;
    p.build(s.cfg, s.emitters);
    REQUIRE(p.step());

    const FrameRecord& r = p.last();
    // The straw-man detector always returns its brightest pixel, so `detected`
    // is true even on an empty frame — that is exactly the weakness CP 4.11
    // will demonstrate. What must hold now is that the centroid error is only
    // computed where there is truth to compare against, and that the beacon is
    // correctly reported as out of view.
    CHECK(r.truth_valid);
    CHECK_FALSE(r.truth_in_fov);
}

TEST_CASE("the loop is reproducible: same config, same frames, bit for bit") {
    // A preview of INV-3 and CP 2.5. If this fails now, it will be far cheaper
    // to fix here than after another ten subsystems have been layered on.
    Scene s = make_scene();

    std::vector<FrameRecord> a, b;
    { Pipeline p; p.build(s.cfg, s.emitters); p.run(a); }
    { Pipeline p; p.build(s.cfg, s.emitters); p.run(b); }

    REQUIRE(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        INFO("frame " << i);
        REQUIRE(a[i].boresight_true.x == b[i].boresight_true.x);
        REQUIRE(a[i].boresight_true.y == b[i].boresight_true.y);
        REQUIRE(a[i].detection_img.x == b[i].detection_img.x);
        REQUIRE(a[i].detection_img.y == b[i].detection_img.y);
        REQUIRE(a[i].centroid_error_px == b[i].centroid_error_px);
    }
}

// ===========================================================================
// A-4 — the encoder model must reach the MEASUREMENT path, not only control
// ===========================================================================

TEST_CASE("A-4: a coarse encoder degrades the reported screen-frame position") {
    // THE DEFECT. pipeline.cpp reconstructed perception's view of the world
    // through `gimbal_.true_position()` on the synthetic path — the exact mount
    // angle — rather than through `gimbal_.position()`, the quantised encoder
    // reading. plant/gimbal.hpp states the rule directly ("position() is what
    // the controller is allowed to read... Mixing them up is the bug §10.3
    // exists to make impossible") and design §14 CP 4.10's acceptance criterion
    // is "the controller reads only the quantised value".
    //
    // The consequence was not subtle. Sweeping encoder_lsb_urad from 20 to
    // 4000 µrad — 0.18 px to 36.67 px, 3.7x the ENTIRE row-17 budget — moved
    // the reported centroiding error by ZERO, because the measurement path
    // never saw the encoder at all.
    //
    // This test sweeps the LSB and requires the reported screen-frame error to
    // GROW. It is the assertion that was missing: a model whose parameter
    // changes nothing observable is indistinguishable from a model that is not
    // wired up.
    //
    // Disturbances are off so the screen-frame error isolates the encoder.
    // With row 25 platform motion active the column is dominated by the
    // accumulated pointing drift D(t) (see FrameRecord::pointing_drift_px) and
    // a 36 px encoder effect is invisible under 300 px of drift.
    std::vector<double> screen_rmse;
    const double lsbs[] = {20.0, 500.0, 4000.0};

    for (double lsb : lsbs) {
        Scene s = make_scene(/*offset_px=*/40.0, /*vx_px_s=*/10.0, /*duration_s=*/6.0);
        s.cfg.pan.encoder_lsb_urad  = lsb;
        s.cfg.tilt.encoder_lsb_urad = lsb;
        // make_scene builds a PipelineConfig directly, which carries no
        // DisturbanceGenerator: there is no row-23 jitter and no row-25
        // platform motion here at all. That is exactly what this test needs —
        // the screen-frame error then isolates the encoder. With the shipped
        // disturbances active the column is dominated by the accumulated
        // pointing drift D(t) and a 36 px encoder effect is invisible beneath
        // 300 px of drift.

        Pipeline p;
        p.build(s.cfg, s.emitters);
        p.set_publish_snapshots(false);

        double sum = 0.0;
        int    n   = 0;
        while (p.step()) {
            const FrameRecord& r = p.last();
            if (!r.centroid_error_valid) continue;
            sum += r.centroid_error_screen_px * r.centroid_error_screen_px;
            ++n;
        }
        REQUIRE(n > 30);
        screen_rmse.push_back(std::sqrt(sum / n));
        MESSAGE("encoder_lsb " << lsb << " urad -> screen RMSE "
                << screen_rmse.back() << " px");
    }

    // Monotone in the LSB, which is the property a wired-up model has and an
    // ignored one does not.
    CHECK(screen_rmse[1] > screen_rmse[0]);
    CHECK(screen_rmse[2] > screen_rmse[1]);

    // And the magnitude is right, not merely the ordering. A uniform
    // quantiser with step q has error RMS q/sqrt(12) per axis, so over two
    // independent axes the magnitude RMS is q*sqrt(2/12) = 0.408 q. At
    // 4000 urad the step is 4000/109.083 = 36.67 px, predicting ~15.0 px.
    // Generous bounds: this is one term added to the detector's own 0.2 px and
    // to whatever the loop contributes, and the point is the order of
    // magnitude, not a calibration.
    CHECK(screen_rmse[2] > 5.0);
    CHECK(screen_rmse[2] < 40.0);

    // The IMAGE-frame error must NOT move. An encoder cannot change where
    // light lands on the focal plane, and a "fix" that contaminated the
    // detector's own accuracy with a quantisation it has no access to would be
    // a different bug in the opposite direction.
    //
    // Asserted here rather than assumed, because the first attempt at this fix
    // routed the RENDERING through the encoder too, which is exactly that bug:
    // it makes the camera point where the encoder thinks the mount is instead
    // of where the mount is.
    {
        Scene fine   = make_scene(40.0, 10.0, 6.0);
        Scene coarse = make_scene(40.0, 10.0, 6.0);
        fine.cfg.pan.encoder_lsb_urad = fine.cfg.tilt.encoder_lsb_urad = 20.0;
        coarse.cfg.pan.encoder_lsb_urad = coarse.cfg.tilt.encoder_lsb_urad = 4000.0;

        auto image_rmse = [](Scene& sc) {
            Pipeline p;
            p.build(sc.cfg, sc.emitters);
            p.set_publish_snapshots(false);
            double sum = 0.0; int n = 0;
            while (p.step()) {
                const FrameRecord& r = p.last();
                if (!r.centroid_error_valid) continue;
                sum += r.centroid_error_px * r.centroid_error_px;
                ++n;
            }
            return n ? std::sqrt(sum / n) : 0.0;
        };
        const double a = image_rmse(fine), b = image_rmse(coarse);
        MESSAGE("image RMSE: " << a << " px at 20 urad, " << b << " px at 4000 urad");

        // A 200x change in the encoder step moves the image-frame error by
        // under 10 %, against the 19x it moves the screen-frame error by. That
        // gap IS the assertion: the encoder reaches the reconstruction and
        // does not reach the focal plane.
        //
        // Not asserted EQUAL, because it is not: a coarser encoder feeds the
        // controller a slightly different error, the mount follows a slightly
        // different path, and the beacon therefore lands on different
        // sub-pixel phases with a different S-curve bias residual. A tolerance
        // of 25 % covers that second-order coupling without covering a
        // first-order contamination.
        CHECK(std::abs(a - b) < 0.25 * a);

        // Both are around 1 px rather than the 0.2 px the shipped pipeline
        // achieves, because make_scene selects the STRAW-MAN detector — see
        // its comment. These cases measure the control loop, and running the
        // full §9.4 pipeline on every frame took this suite past its timeout.
        CHECK(a < 2.0);
        CHECK(b < 2.0);
    }
}

// ===========================================================================
// A-3 — the exposure smear must sample the platform rate at the REAL frame time
// ===========================================================================

TEST_CASE("A-3: motion blur is correct at a camera rate above row 5's minimum") {
    // THE DEFECT. pipeline.cpp read the platform's analytic rate as
    // `platform_rate(frame_ / 30.0)` — spec row 5's MINIMUM camera rate,
    // hardcoded as if it were fixed. scenario/schema.cpp permits camera_hz from
    // 30 to 1000, and the PS explicitly allows a higher rate.
    //
    // At camera_hz = 60 the blur therefore sampled the platform rate at TWICE
    // the true elapsed time. For a constant-velocity platform that happens to
    // be harmless — the rate is the same at every t — which is why every
    // shipped scenario hid it: they all use linear platform motion at 30 Hz.
    // For any PERIODIC platform motion (row 25 permits circular, figure-8 and
    // sinusoidal, and §7.2 gives all three in closed form) it is wrong in both
    // direction and magnitude, and wrong by more as the run goes on.
    //
    // Built from a real Scenario rather than from a hand-made PipelineConfig,
    // because the platform motion stack only exists on the scenario path.
    auto loaded = load_scenario(std::string(SAT_SCENARIO_DIR) + "/spec_defaults.toml");
    REQUIRE_MESSAGE(loaded.has_value(), loaded.error());
    Scenario sc = *loaded;

    sc.duration_s = 2.0;
    sc.camera_hz  = 60;        // above row 5's minimum, which the PS permits
    sc.truth_hz   = 600;       // must stay an exact multiple (schema.cpp)
    sc.control_hz = 60;

    // A platform that REVERSES inside the run, so sampling its rate at the
    // wrong time gives a genuinely different answer rather than the same
    // constant. A 1 s period at 60 Hz means the buggy sampling (t = frame/30)
    // reaches a full period ahead by the halfway point.
    sc.platform.clear();
    MotionSpec ms;
    ms.kind            = "sinusoid";
    ms.amplitude_px[0] = 30.0;
    ms.period_s        = 1.0;
    ms.axis            = 0;
    sc.platform.push_back(ms);

    Pipeline p;
    p.build_from_scenario(sc);
    p.set_publish_snapshots(false);

    int  checked   = 0;
    int  divergent = 0;
    while (p.step()) {
        const FrameRecord& r = p.last();
        const Rate2 expected = p.source().disturbance().platform_rate(r.time_s);
        // What the old code would have used: the frame index over a hardcoded
        // 30, which at camera_hz = 60 is twice the elapsed time.
        const Rate2 wrong = p.source().disturbance().platform_rate(
                                static_cast<double>(r.frame) / 30.0);

        // The source is told gimbal rate + platform rate; subtract the former
        // to recover the platform component that was actually used. The gimbal
        // is stepped at the top of the frame and not again, so its rate here
        // is the one the blur was built from.
        const Rate2 blur = p.source().blur_rate();
        const Rate2 gr   = p.gimbal().rate();
        const Rate2 used{blur.x - gr.x, blur.y - gr.y};

        CHECK(used.x == doctest::Approx(expected.x).epsilon(1e-9));
        CHECK(used.y == doctest::Approx(expected.y).epsilon(1e-9));

        // Count the frames where the right and wrong answers genuinely differ.
        // Without this the test would pass on the buggy code whenever the two
        // sampling times happened to agree, and prove nothing.
        if (std::abs(expected.x - wrong.x) > 1.0) ++divergent;
        ++checked;
    }

    MESSAGE(checked << " frames at " << sc.camera_hz << " Hz, " << divergent
            << " of them where the hardcoded-30 sampling would have differed");
    REQUIRE(checked > 60);
    // The bug has to be VISIBLE on most frames for this to be a real guard.
    CHECK(divergent > checked / 2);
}
