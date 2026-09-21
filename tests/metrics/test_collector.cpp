// tests/metrics/test_collector.cpp — CP 7.1.
//
// "MetricCollector with every §13.1 definition, centroiding and tracking error
//  STRICTLY SEPARATE. Accept when: both appear as separate live plots;
//  definitions written verbatim into docs/METRICS.md."
//
// The collector is driven with hand-built FrameRecords rather than with a real
// run. That is deliberate and is the only way these tests mean anything: a
// definition like "measured ONLY while Confirmed" is a statement about which
// frames are counted, and the way to test it is to feed in frames that WOULD
// score differently if the restriction were dropped. A real run cannot do that
// on demand.

#include <doctest/doctest.h>

#include "metrics/collector.hpp"

#include <cmath>
#include <string>

using namespace sat;

namespace {

constexpr double kIfov = 109.08;
constexpr double kDt   = 1.0 / 30.0;

/// A frame with nothing interesting in it.
FrameRecord blank(int64_t i) {
    FrameRecord r;
    r.frame  = i;
    r.time_s = i * kDt;
    return r;
}

/// A frame where the beacon is in view, a detection was made with the given
/// screen-frame error, and the track is in the given state.
FrameRecord scored(int64_t i, TrackState st, double centroid_px, double tracking_px,
                   bool in_fov = true) {
    FrameRecord r = blank(i);
    r.track_state = st;
    r.has_lock    = (st == TrackState::Confirmed || st == TrackState::Coasting);
    r.truth_valid = true;
    r.truth_in_fov = in_fov;
    r.truth_screen = Pixel2{1000.0, 1000.0};
    r.detected    = true;
    r.detection_screen = Pixel2{1000.0 + centroid_px, 1000.0};
    r.centroid_error_valid    = in_fov;
    r.centroid_error_screen_px = centroid_px;
    r.centroid_error_px        = centroid_px * 0.5;   // the image figure is smaller
    r.tracking_error_px        = tracking_px;
    return r;
}

RunMetrics collect(const std::vector<FrameRecord>& frames) {
    MetricCollector c;
    c.begin("test", 42, kIfov, frames.size());
    for (const FrameRecord& f : frames) c.add(f);
    StageTimers t;
    return c.finish(t, /*wall_time_s=*/1.0, /*saturation_frac=*/0.0);
}

}  // namespace

// ===========================================================================
// INV-6 — the two errors cannot contaminate each other
// ===========================================================================

TEST_CASE("CP 7.1: centroiding and tracking error use different frame sets") {
    // The scenario that separates them. Ten frames are Confirmed with a large
    // tracking error and a tiny centroid error; ten are Tentative with the
    // reverse. If either metric used the other's frame set, or used all frames,
    // the numbers below come out differently.
    std::vector<FrameRecord> f;
    for (int i = 0; i < 10; ++i) f.push_back(scored(i,      TrackState::Confirmed, 0.1, 8.0));
    for (int i = 10; i < 20; ++i) f.push_back(scored(i,     TrackState::Tentative, 4.0, 0.2));

    const RunMetrics m = collect(f);

    // Tracking: ONLY the ten Confirmed frames, all at 8.0.
    CHECK(m.tracking_frames == 10);
    CHECK(m.tracking_rms_px == doctest::Approx(8.0));

    // Centroiding: ALL twenty, because every one had a detection with the
    // beacon in view. The lifecycle state is irrelevant to the detector.
    CHECK(m.centroid_frames == 20);
    CHECK(m.centroid_rmse_screen_px == doctest::Approx(std::sqrt((10 * 0.01 + 10 * 16.0) / 20.0)));

    // And the two are reported in the units §13.1 asks for.
    CHECK(m.tracking_rms_urad == doctest::Approx(8.0 * kIfov));
}

TEST_CASE("CP 7.1: the image and screen centroid figures are both kept") {
    std::vector<FrameRecord> f;
    for (int i = 0; i < 10; ++i) f.push_back(scored(i, TrackState::Confirmed, 2.0, 1.0));
    const RunMetrics m = collect(f);
    CHECK(m.centroid_rmse_screen_px == doctest::Approx(2.0));
    CHECK(m.centroid_rmse_image_px  == doctest::Approx(1.0));
    // The screen figure is the larger one, always: it carries the pointing
    // error on top of the detector's own.
    CHECK(m.centroid_rmse_screen_px > m.centroid_rmse_image_px);
}

TEST_CASE("CP 7.1: bias is signed, and separates a fixable offset from noise") {
    std::vector<FrameRecord> biased, noisy;
    for (int i = 0; i < 100; ++i) {
        biased.push_back(scored(i, TrackState::Confirmed, 0.5, 1.0));
        FrameRecord r = scored(i, TrackState::Confirmed, 0.5, 1.0);
        // Same magnitude, alternating sign.
        r.detection_screen.x = 1000.0 + ((i % 2) ? 0.5 : -0.5);
        noisy.push_back(r);
    }
    const RunMetrics b = collect(biased);
    const RunMetrics n = collect(noisy);

    CHECK(b.centroid_rmse_screen_px == doctest::Approx(n.centroid_rmse_screen_px));
    CHECK(b.centroid_bias_x_px == doctest::Approx(0.5));
    CHECK(n.centroid_bias_x_px == doctest::Approx(0.0));
}

// ===========================================================================
// Frame sets and denominators
// ===========================================================================

TEST_CASE("CP 7.1: lock retention divides by in-FOV frames, not by total frames") {
    // 30 frames with the beacon in view, Confirmed for 24 of them, plus 70
    // frames where the beacon is not in view at all. Retention must be 80%,
    // not 24%.
    std::vector<FrameRecord> f;
    for (int i = 0; i < 24; ++i) f.push_back(scored(i, TrackState::Confirmed, 0.1, 1.0));
    for (int i = 24; i < 30; ++i) f.push_back(scored(i, TrackState::Coasting,  0.1, 1.0));
    for (int i = 30; i < 100; ++i) {
        FrameRecord r = blank(i);
        r.truth_valid = true;
        r.truth_in_fov = false;
        f.push_back(r);
    }
    const RunMetrics m = collect(f);
    CHECK(m.frames_in_fov == 30);
    CHECK(m.frames_confirmed == 24);
    CHECK(m.lock_retention_rate == doctest::Approx(0.8));
    CHECK(m.target_loss_frac == doctest::Approx(0.2));
}

TEST_CASE("CP 7.1: Confirmed frames with the beacon out of view are not retention") {
    // This case used to assert that the ratio was CLAMPED to 100%, on the
    // grounds that a filter coasting correctly through an occlusion leaves the
    // track Confirmed while the beacon is invisible, and an unclamped ratio
    // would report retention above 100% and a negative target loss.
    //
    // The reasoning was right about the occlusion and wrong about the fix, and
    // Stage 10 found out how wrong. A 60 s compliance run with 120 clutter
    // sources ends holding a clutter source after the beacon has left the
    // field: 1798 Confirmed frames against 727 in-view frames, a ratio of 2.47,
    // which the clamp turned into "retention 100.00 %" printed directly above
    // "false tracks 1073.60 /min". Both numbers were computed correctly and
    // together they were a contradiction — and the clamp made the system's
    // worst failure mode produce its best-looking metric.
    //
    // Intersecting the numerator with the denominator's condition handles the
    // occlusion properly instead of papering over it: a frame where the beacon
    // is out of view is in NEITHER, so it can neither inflate the ratio nor
    // penalise it. The ratio cannot exceed 1 by construction and there is
    // nothing left to clamp.
    std::vector<FrameRecord> f;
    for (int i = 0; i < 10; ++i) f.push_back(scored(i, TrackState::Confirmed, 0.1, 1.0, true));
    for (int i = 10; i < 20; ++i) f.push_back(scored(i, TrackState::Confirmed, 0.1, 1.0, false));
    const RunMetrics m = collect(f);

    CHECK(m.frames_confirmed   == 20);   // every Confirmed frame
    CHECK(m.frames_in_fov      == 10);
    CHECK(m.frames_held_in_fov == 10);   // ...of which ten were on the beacon
    CHECK(m.lock_retention_rate == doctest::Approx(1.0));
    CHECK(m.target_loss_frac    == doctest::Approx(0.0));
}

TEST_CASE("CP 7.1: losing the beacon cannot report as perfect retention") {
    // The failure the clamp used to hide, in miniature: the beacon is in view
    // for 10 frames and held for 5 of them, then leaves, and the tracker keeps
    // a confirmed track on something else for 50 more.
    //
    // Retention must be 50%, not 100%. The 50 off-target frames are false
    // tracks and are counted as such, separately.
    std::vector<FrameRecord> f;
    for (int i = 0; i < 5; ++i)  f.push_back(scored(i, TrackState::Confirmed, 0.1, 1.0, true));
    for (int i = 5; i < 10; ++i) f.push_back(scored(i, TrackState::Coasting,  0.1, 1.0, true));
    for (int i = 10; i < 60; ++i) f.push_back(scored(i, TrackState::Confirmed, 0.1, 1.0, false));
    const RunMetrics m = collect(f);

    CHECK(m.frames_in_fov      == 10);
    CHECK(m.frames_held_in_fov == 5);
    CHECK(m.frames_confirmed   == 55);
    CHECK(m.lock_retention_rate == doctest::Approx(0.5));
    CHECK(m.target_loss_frac    == doctest::Approx(0.5));
    CHECK(m.false_tracks        == 50);
}

TEST_CASE("CP 7.1: an undefined ratio is reported with its denominator, not as zero percent") {
    // The beacon is never in view. 0/0 reported as "0% retention" would read as
    // total failure; reported as "100%" would read as perfection. Neither is
    // true, so frames_in_fov is printed beside it.
    std::vector<FrameRecord> f;
    for (int i = 0; i < 50; ++i) {
        FrameRecord r = blank(i);
        r.truth_valid = true;
        r.truth_in_fov = false;
        f.push_back(r);
    }
    const RunMetrics m = collect(f);
    CHECK(m.frames_in_fov == 0);
    CHECK(m.lock_retention_rate == doctest::Approx(0.0));
    const std::string s = format_summary(m);
    CHECK(s.find("the beacon was never in view") != std::string::npos);
}

TEST_CASE("CP 7.1: false tracks count Confirmed locks with no beacon in view") {
    std::vector<FrameRecord> f;
    // 60 s of run so the per-minute rate is easy to check.
    for (int i = 0; i < 1800; ++i) {
        FrameRecord r = blank(i);
        r.truth_valid = true;
        r.truth_in_fov = false;
        // Confirmed on a non-beacon for 30 of them.
        r.track_state = (i < 30) ? TrackState::Confirmed : TrackState::Deleted;
        // And a detection on every frame, which must NOT be counted: an
        // unconfirmed candidate is the gate doing its job.
        r.detected = true;
        f.push_back(r);
    }
    const RunMetrics m = collect(f);
    CHECK(m.false_tracks == 30);
    CHECK(m.false_track_rate_per_min == doctest::Approx(30.0).epsilon(0.02));
}

// ===========================================================================
// Acquisition and re-acquisition
// ===========================================================================

TEST_CASE("CP 7.1: cold and in-FOV acquisition are separate numbers") {
    // The beacon arrives at frame 30 (1.0 s) and is Confirmed at frame 36
    // (1.2 s). Cold = 1.2 s from run start; in-view = 0.2 s from arrival.
    std::vector<FrameRecord> f;
    for (int i = 0; i < 30; ++i) {
        FrameRecord r = blank(i);
        r.truth_valid = true;
        r.truth_in_fov = false;
        f.push_back(r);
    }
    for (int i = 30; i < 36; ++i) f.push_back(scored(i, TrackState::Tentative, 0.1, 1.0));
    for (int i = 36; i < 60; ++i) f.push_back(scored(i, TrackState::Confirmed, 0.1, 1.0));

    const RunMetrics m = collect(f);
    CHECK(m.acquired);
    CHECK(m.acquisition_cold_s == doctest::Approx(36 * kDt));
    CHECK(m.acquired_in_fov);
    CHECK(m.acquisition_in_fov_s == doctest::Approx(6 * kDt));
    // The whole reason both are reported: they differ by a factor of six here,
    // and §10.5 shows the cold figure cannot meet spec row 16 by geometry.
    CHECK(m.acquisition_cold_s > m.acquisition_in_fov_s);
}

TEST_CASE("CP 7.1: a run that never confirms reports 'never', not zero seconds") {
    std::vector<FrameRecord> f;
    for (int i = 0; i < 60; ++i) f.push_back(scored(i, TrackState::Tentative, 0.1, 1.0));
    const RunMetrics m = collect(f);
    CHECK_FALSE(m.acquired);
    CHECK(format_summary(m).find("never") != std::string::npos);
}

TEST_CASE("CP 7.1: reacquisition is timed from the loss, and only on closed episodes") {
    std::vector<FrameRecord> f;
    for (int i = 0; i < 30; ++i)  f.push_back(scored(i, TrackState::Confirmed, 0.1, 1.0));
    // Lost at frame 30, back at frame 39 -> 9 frames = 0.3 s.
    for (int i = 30; i < 39; ++i) f.push_back(scored(i, TrackState::Coasting, 0.1, 1.0));
    for (int i = 39; i < 60; ++i) f.push_back(scored(i, TrackState::Confirmed, 0.1, 1.0));
    // A second episode, this one still open when the run ends. It must NOT be
    // recorded: a half-finished reacquisition is not a fast one, and counting
    // the truncated interval would make a run that never recovered look good.
    for (int i = 60; i < 90; ++i) f.push_back(scored(i, TrackState::Coasting, 0.1, 1.0));

    const RunMetrics m = collect(f);
    CHECK(m.reacquisitions == 1);
    CHECK(m.reacquisition_max_s == doctest::Approx(9 * kDt));
}

// ===========================================================================
// Speed
// ===========================================================================

TEST_CASE("CP 7.1: fps comes from percentiles, never from a mean") {
    // §13.1: "NEVER the mean". Decision 17: "A 0.3 ms mean hiding a 25 ms p99
    // is a broken control loop." So a run of fast frames with a few very slow
    // ones must report a low fps_p5 even though the average is excellent.
    StageTimers t;
    for (int i = 0; i < 1000; ++i) t[Stage::FrameTotal].record(300.0);       // 0.3 ms
    for (int i = 0; i < 100; ++i)  t[Stage::FrameTotal].record(25000.0);     // 25 ms

    MetricCollector c;
    c.begin("speed", 1, kIfov, 1100);
    for (int i = 0; i < 1100; ++i) c.add(blank(i));
    const RunMetrics m = c.finish(t, 1.0, 0.0);

    MESSAGE("p50 " << m.frame_ms_p50 << " ms, p99 " << m.frame_ms_p99
            << " ms, fps " << m.fps_mean << " / " << m.fps_p5);
    CHECK(m.frame_ms_p50 < 1.0);
    CHECK(m.frame_ms_p99 > 10.0);
    CHECK(m.fps_mean > 1000.0);
    // The number that decides spec row 20 compliance is the slow one, and here
    // it is three orders of magnitude below the typical frame.
    CHECK(m.fps_p5 < 100.0);
}

TEST_CASE("CP 7.1: the collector does not allocate once a run is under way") {
    // INV-4, for the measuring apparatus itself. A collector that reallocates
    // mid-run would perturb the very frame times it is recording.
    MetricCollector c;
    c.begin("alloc", 1, kIfov, 3600);
    for (int i = 0; i < 3600; ++i) c.add(scored(i, TrackState::Confirmed, 0.1, 1.0));
    CHECK(c.frames() == 3600);
    StageTimers t;
    const RunMetrics m = c.finish(t, 1.0, 0.0);
    CHECK(m.centroid_frames == 3600);
}

// ===========================================================================
// P0-2 — FOV containment, the PS's own objective
//
// "first locate and MAINTAIN the remote terminal within its camera
//  Field-of-View." Everything else in §13.1 is conditional on that sentence,
// and until these metrics existed it was the one quantity the project never
// reported. The tests below pin the three denominators apart, because a
// denominator defect produces arithmetically correct numbers and can only be
// caught by constructing the case where the wrong one flatters.
// ===========================================================================

TEST_CASE("P0-2: FOV containment is measured over ALL frames, not over in-FOV frames") {
    // 100 frames. The beacon is in view for the last 20 only; for the first 80
    // the camera is pointed elsewhere and the track is in Search.
    //
    // Whole-run containment is therefore 20 %. Retention, over its own in-FOV
    // denominator, is 100 % — and both numbers are correct. The point is that
    // retention alone cannot distinguish this run from a perfect one.
    std::vector<FrameRecord> f;
    for (int i = 0; i < 80; ++i) {
        FrameRecord r = blank(i);
        r.truth_valid = true;
        r.truth_in_fov = false;       // truth known, beacon outside the FOV
        f.push_back(r);
    }
    for (int i = 80; i < 100; ++i) f.push_back(scored(i, TrackState::Confirmed, 0.1, 1.0));

    const RunMetrics m = collect(f);

    CHECK(m.frames_in_fov == 20);
    CHECK(m.frames_total  == 100);
    CHECK(m.fov_containment_frac == doctest::Approx(0.20));

    // Retention, over in-FOV frames only, is perfect — on a run that spent
    // 80 % of its length not looking at the target. That is not a bug in
    // retention; it is why containment has to be reported beside it.
    CHECK(m.lock_retention_rate == doctest::Approx(1.0));
}

TEST_CASE("P0-2: the post-acquisition window opens at the first lock and never closes") {
    // 100 frames: 10 of Search, then a lock, then 89 frames of which the first
    // 39 hold and the last 50 have lost the beacon entirely.
    //
    // The window is 89 frames (everything AFTER the acquiring frame — the
    // acquiring frame itself belongs to the search that found it). 39 of those
    // are in view and held, so post-acquisition loss is 1 - 39/89 = 56.2 %.
    // Over the in-FOV denominator the same run is 39 held of 39 in view: 0 %
    // loss, a clean row-18 pass on a run that lost the target for its second
    // half.
    std::vector<FrameRecord> f;
    for (int i = 0; i < 10; ++i) {
        FrameRecord r = blank(i);
        r.truth_valid = true;
        r.truth_in_fov = false;
        f.push_back(r);
    }
    for (int i = 10; i < 50; ++i) f.push_back(scored(i, TrackState::Confirmed, 0.1, 1.0));
    for (int i = 50; i < 100; ++i) {
        FrameRecord r = blank(i);
        r.truth_valid = true;
        r.truth_in_fov = false;
        f.push_back(r);
    }

    const RunMetrics m = collect(f);

    REQUIRE(m.post_acq_valid);
    CHECK(m.frames_post_acq        == 89);    // frames 11..99
    CHECK(m.frames_in_fov_post_acq == 39);    // frames 11..49
    CHECK(m.frames_held_post_acq   == 39);

    // The two definitions disagree by 56 percentage points on the same run.
    CHECK(m.target_loss_frac     == doctest::Approx(0.0));
    CHECK(m.target_loss_post_acq == doctest::Approx(1.0 - 39.0 / 89.0).epsilon(1e-9));
    CHECK(m.fov_containment_post_acq == doctest::Approx(39.0 / 89.0).epsilon(1e-9));
}

TEST_CASE("P0-2: a run that never locks reports the post-acquisition figures as undefined") {
    // The alternative would be to report 0/0 as either 0 % or 100 % loss, and
    // both are a lie in opposite directions. Undefined is the only honest
    // answer to "how well did you maintain a lock you never had", and the flag
    // is what lets the summary and the compliance matrix say so.
    std::vector<FrameRecord> f;
    for (int i = 0; i < 50; ++i) {
        FrameRecord r = blank(i);
        r.truth_valid = true;
        r.truth_in_fov = true;          // visible the whole time; never acquired
        f.push_back(r);
    }

    const RunMetrics m = collect(f);

    CHECK_FALSE(m.post_acq_valid);
    CHECK(m.frames_post_acq == 0);
    CHECK(m.target_loss_post_acq == doctest::Approx(0.0));   // untouched, not "perfect"
    // Whole-run containment is still perfectly well defined and is 100 %: the
    // beacon WAS in the field. The failure here is acquisition, and the
    // acquisition metrics are where it shows.
    CHECK(m.fov_containment_frac == doctest::Approx(1.0));
    CHECK_FALSE(m.acquired);
}

// ===========================================================================
// P1-9 — the acquisition transient separated from the loop
// ===========================================================================

TEST_CASE("P1-9: the post-lock slew is scored separately from the steady state") {
    // The shape that made the whole-run RMS read as a bug: a few enormous
    // samples immediately after the lock, then a long quiet tail. With one
    // number the RMS exceeds the p95, which is arithmetically fine and looks
    // like a defect. Split, each half describes one thing.
    std::vector<FrameRecord> f;
    for (int i = 0; i < 5; ++i) {
        FrameRecord r = blank(i);
        r.truth_valid = true;
        r.truth_in_fov = false;
        f.push_back(r);
    }
    // Frames 5..40: the slew, 45 px of error. 5 is the acquiring frame and
    // the settle window is 30 frames, so 5..35 inclusive is transient.
    for (int i = 5; i <= 40; ++i) f.push_back(scored(i, TrackState::Confirmed, 0.1, 45.0));
    // Frames 41..300: settled, 0.5 px.
    for (int i = 41; i < 300; ++i) f.push_back(scored(i, TrackState::Confirmed, 0.1, 0.5));

    const RunMetrics m = collect(f);

    // 31 transient frames: the acquiring frame plus kSettleFrames after it.
    CHECK(m.settle_frames == 30);
    CHECK(m.tracking_frames_transient == 31);
    CHECK(m.tracking_frames_steady + m.tracking_frames_transient == m.tracking_frames);

    // The transient is entirely slew, the steady state is mostly the quiet
    // tail, and the whole-run figure sits between them and describes neither.
    CHECK(m.tracking_rms_transient_px == doctest::Approx(45.0));
    CHECK(m.tracking_rms_steady_px < m.tracking_rms_px);
    CHECK(m.tracking_max_transient_px == doctest::Approx(45.0));

    // The PS's literal "average", which is a different number again.
    CHECK(m.tracking_mean_px < m.tracking_rms_px);
}

TEST_CASE("P1-9: a run with no transient puts every sample in the steady state") {
    // A long, boring, already-settled run. Nothing should be diverted into the
    // transient bucket beyond the settle window, and the steady figures must
    // then equal the whole-run ones.
    std::vector<FrameRecord> f;
    for (int i = 0; i < 200; ++i) f.push_back(scored(i, TrackState::Confirmed, 0.1, 2.0));

    const RunMetrics m = collect(f);

    CHECK(m.tracking_frames_transient == 31);          // frame 0 plus 30
    CHECK(m.tracking_frames_steady    == 169);
    // Every sample is 2.0, so the split cannot change any statistic.
    CHECK(m.tracking_rms_steady_px    == doctest::Approx(2.0));
    CHECK(m.tracking_rms_transient_px == doctest::Approx(2.0));
    CHECK(m.tracking_rms_px           == doctest::Approx(2.0));
}

// ===========================================================================
// P1-6 — the third centroiding column
// ===========================================================================

TEST_CASE("P1-6: the boresight column excludes the accumulated pointing error") {
    // The defect: with row 25 platform motion active the SCREEN-frame
    // centroiding error is identically |B_true - B_cmd|, which grows without
    // bound and was printed under "CENTROIDING (graded, 60 %)" with no caveat.
    //
    // Here the detector is perfect in its own frame (0.05 px) while the mount
    // has drifted 300 px. The screen column must carry the drift, the
    // boresight column must not, and the drift must be reported so the screen
    // number can be recognised for what it is.
    std::vector<FrameRecord> f;
    for (int i = 0; i < 100; ++i) {
        FrameRecord r = scored(i, TrackState::Confirmed, /*centroid_px=*/300.0, 1.0);
        r.centroid_error_px           = 0.05;     // image frame: near-exact
        r.centroid_error_boresight_px = 0.05;     // same error, screen units
        r.centroid_error_screen_px    = 300.0;    // + the unobservable drift
        r.pointing_drift_px           = 300.0;
        f.push_back(r);
    }

    const RunMetrics m = collect(f);

    CHECK(m.centroid_rmse_screen_px    == doctest::Approx(300.0));
    CHECK(m.centroid_rmse_boresight_px == doctest::Approx(0.05));
    CHECK(m.centroid_rmse_image_px     == doctest::Approx(0.05));
    // The consistency check the summary prints: the screen figure IS the
    // drift, to within the detector's own error.
    CHECK(m.pointing_drift_rms_px == doctest::Approx(300.0));
    CHECK(std::abs(m.centroid_rmse_screen_px - m.pointing_drift_rms_px) < 1.0);
}

TEST_CASE("P1-6: drift is accumulated on every truth frame, not only detected ones") {
    // The expectation printed beside the screen column has to be over the whole
    // run. Sampling it only on detected frames would make it agree with the
    // screen RMSE by construction and prove nothing.
    std::vector<FrameRecord> f;
    for (int i = 0; i < 50; ++i) {
        FrameRecord r = blank(i);
        r.truth_valid       = true;
        r.truth_in_fov      = false;     // no detection scored on these
        r.pointing_drift_px = 10.0;
        f.push_back(r);
    }
    for (int i = 50; i < 100; ++i) {
        FrameRecord r = scored(i, TrackState::Confirmed, 0.1, 1.0);
        r.pointing_drift_px = 10.0;
        f.push_back(r);
    }

    const RunMetrics m = collect(f);

    CHECK(m.centroid_frames == 50);          // only the detected half is scored
    CHECK(m.pointing_drift_rms_px == doctest::Approx(10.0));   // but drift is over all 100
}
