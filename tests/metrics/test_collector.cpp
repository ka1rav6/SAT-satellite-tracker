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
