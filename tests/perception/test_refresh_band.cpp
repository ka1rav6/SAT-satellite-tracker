// tests/perception/test_refresh_band.cpp
//
// P1-7 — the background sweep, and the properties that make it safe to enable.
//
// WHAT THE SWEEP IS. A confirmed track gets a detection WINDOW centred on the
// prediction (DetectRoi, design §14.0b), and the rest of the frame is never
// examined. That is the right trade almost always, and it has one failure
// mode: if the track is following a decoy, the true beacon sits outside the
// window forever and nothing can ever recover it. The sweep is the hedge — a
// second detector pass over one horizontal band per frame, rotating, so that
// every row of the sensor is examined at least once every N frames.
//
// WHY THESE TESTS AND NOT OTHERS. The sweep's value is entirely in a coverage
// claim — "every row, once per N frames" — and a coverage claim that is not
// checked is a coverage claim that is wrong at the edges. Three of the four
// tests below are about the edges:
//
//   1. The bands TILE. Exactly, with no gap and no overlap, including when N
//      does not divide the frame height. A naive `height / N` band height
//      leaves the last `height % N` rows never swept, which fails as "the one
//      time we lost it, the target was near the bottom".
//   2. The sweep is ADDITIONAL. It must not replace the tracking window, or
//      M-of-N promotion (3 hits in 5 frames) could never be satisfied when a
//      target is only shown to the tracker once every N frames.
//   3. The overlap does NOT duplicate. Where a band crosses the window, a
//      target is inside both passes, and handing the tracker two copies of one
//      blob would manufacture a rival track — a self-inflicted decoy, in the
//      feature whose whole purpose is surviving decoys.
//   4. INV-3 survives. The sweep must be a pure function of the frame index.

#include <doctest/doctest.h>

#include "engine/pipeline.hpp"
#include "scenario/schema.hpp"

#include <cmath>
#include <map>
#include <string>
#include <vector>

using namespace sat;

namespace {

Scenario load_spec() {
    auto r = load_scenario(std::string(SAT_SCENARIO_DIR) + "/spec_defaults.toml");
    REQUIRE_MESSAGE(r.has_value(), r.error());
    return *r;
}

/// One run, returning every frame's record. Short: these tests are about the
/// geometry of the sweep, not about whether the run scores well.
std::vector<FrameRecord> run_frames(const Scenario& sc, int frames) {
    Pipeline engine;
    engine.build_from_scenario(sc);
    std::vector<FrameRecord> out;
    out.reserve(static_cast<size_t>(frames));
    for (int i = 0; i < frames && engine.step(); ++i) out.push_back(engine.last());
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
TEST_CASE("P1-7: the sweep bands tile the frame exactly") {
    // 7 is deliberately coprime with both 480 and 640. With N=8 the arithmetic
    // divides evenly and a truncating implementation would pass; the whole
    // point of this test is the remainder rows, so N must not divide H.
    constexpr int kBands = 7;

    Scenario sc = load_spec();
    sc.roi_refresh_frames = kBands;
    sc.duration_s         = 8.0;

    const std::vector<FrameRecord> recs = run_frames(sc, 240);
    REQUIRE(recs.size() > 100);

    const int W = sc.resolution[0];
    const int H = sc.resolution[1];

    // Collect, per band index, the band that was actually swept. A band is
    // only emitted on frames where the tracking window was NOT the whole
    // frame, so this map fills in as the track confirms.
    std::map<int, DetectRoi> bands;
    int swept_frames = 0;
    for (const FrameRecord& r : recs) {
        if (!r.roi_band.valid()) continue;
        ++swept_frames;
        const int idx = static_cast<int>(r.frame % kBands);
        auto [it, inserted] = bands.emplace(idx, r.roi_band);
        if (!inserted) {
            // The same band index must produce the same band on every
            // rotation. If it does not, the sweep is carrying state.
            CHECK(it->second.y0     == r.roi_band.y0);
            CHECK(it->second.height == r.roi_band.height);
        }
    }

    REQUIRE_MESSAGE(swept_frames > 0,
                    "no frame swept a band; the track never confirmed, so this "
                    "test measured nothing");
    REQUIRE_MESSAGE(bands.size() == static_cast<size_t>(kBands),
                    "not every band index was observed; the run was too short "
                    "to prove coverage");

    // Every band spans the full width: these are ROW bands.
    for (const auto& [idx, b] : bands) {
        CHECK(b.x0    == 0);
        CHECK(b.width == W);
    }

    // And they tile [0, H) end to end: band 0 starts at row 0, each band
    // starts where the previous one ended, and the last one ends at H. That
    // single walk rules out gaps AND overlaps AND the dropped remainder.
    int expect_y0 = 0;
    for (int i = 0; i < kBands; ++i) {
        const DetectRoi& b = bands.at(i);
        CHECK_MESSAGE(b.y0 == expect_y0,
                      "band " << i << " starts at " << b.y0
                              << ", leaving a gap or an overlap at row "
                              << expect_y0);
        CHECK(b.height > 0);
        expect_y0 = b.y0 + b.height;
    }
    CHECK_MESSAGE(expect_y0 == H,
                  "the bands cover " << expect_y0 << " of " << H
                                     << " rows; the remainder is never swept");
}

// ---------------------------------------------------------------------------
TEST_CASE("P1-7: the sweep is additional and does not disturb the track") {
    // The failure this guards against is the tempting implementation: sweep a
    // band INSTEAD of the window on sweep frames. That costs less still, and
    // it breaks acquisition, because tracking/track.hpp wants 3 hits in 5
    // frames and a target shown once every N frames cannot supply them.
    //
    // Specification row 16 grades acquisition. Nothing grades the perception
    // tail. So the sweep is not allowed to buy the second with the first, and
    // the check is that the graded quantities do not move at all.
    Scenario off = load_spec();
    off.duration_s         = 8.0;
    off.roi_refresh_frames = 0;

    Scenario on = off;
    on.roi_refresh_frames = 7;

    const std::vector<FrameRecord> a = run_frames(off, 240);
    const std::vector<FrameRecord> b = run_frames(on,  240);
    REQUIRE(a.size() == b.size());

    // The tracking window itself must be untouched: the sweep runs beside it,
    // so the same frame must still ask for the same window.
    for (size_t i = 0; i < a.size(); ++i) {
        CHECK_MESSAGE(a[i].roi.x0     == b[i].roi.x0,     "frame " << i);
        CHECK_MESSAGE(a[i].roi.y0     == b[i].roi.y0,     "frame " << i);
        CHECK_MESSAGE(a[i].roi.width  == b[i].roi.width,  "frame " << i);
        CHECK_MESSAGE(a[i].roi.height == b[i].roi.height, "frame " << i);
    }

    // And the lock must arrive on the same frame. On the specification
    // defaults there is one emitter and no decoy, so the sweep has nothing to
    // find and must change nothing whatsoever about the track's life.
    for (size_t i = 0; i < a.size(); ++i) {
        CHECK_MESSAGE(a[i].track_state == b[i].track_state, "frame " << i);
        CHECK_MESSAGE(a[i].has_lock     == b[i].has_lock,    "frame " << i);
    }
}

// ---------------------------------------------------------------------------
TEST_CASE("P1-7: a band crossing the window does not duplicate its detections") {
    // Where the band crosses the tracking window a source is inside BOTH
    // passes. Without the dedupe in Pipeline::step the tracker is handed the
    // same blob twice and can associate one copy while treating the other as a
    // rival — the exact failure the sweep exists to prevent, caused by the
    // sweep.
    //
    // WHAT THIS TEST IS NOT. The first version of it asserted that a frame
    // resolving to one candidate without the sweep still resolves to one with
    // it. That premise was wrong, and the run said so: on the specification
    // defaults the sweep legitimately finds EXTRA sources — saturated hot
    // pixels from the defect stage — in the rows the window never examines.
    // Finding those is not a bug, it is the entire feature. Frame 1 of the
    // same run reports two candidates with the sweep disabled and the window
    // still full-frame, which settles it: candidate COUNT says nothing about
    // duplication.
    //
    // So the property is tested directly. A duplicate is two detections at the
    // same place, and nothing else is.
    Scenario sc = load_spec();
    sc.duration_s         = 8.0;
    sc.roi_refresh_frames = 7;

    Pipeline engine;
    engine.build_from_scenario(sc);

    // Two detections of one source land within a pixel or so of each other —
    // the two passes see the same blob through different clipping. 5 px is
    // comfortably inside that and comfortably outside the separation of two
    // genuinely distinct sources, which the grouping stage will not merge
    // below the structuring element's own width anyway.
    constexpr double kDuplicateSeparationPx = 5.0;

    int frames_with_band = 0, overlap_frames = 0;
    for (int i = 0; i < 240 && engine.step(); ++i) {
        const FrameRecord& f = engine.last();
        if (!f.roi_band.valid()) continue;
        ++frames_with_band;

        // Does this frame's band actually cross the window? If it never does
        // over the whole run, the dedupe was never exercised and a green
        // result here would mean nothing.
        const int wy0 = f.roi.y0, wy1 = f.roi.y0 + f.roi.height;
        const int by0 = f.roi_band.y0, by1 = f.roi_band.y0 + f.roi_band.height;
        if (by0 < wy1 && wy0 < by1) ++overlap_frames;

        const std::vector<Detection>& dets = engine.detections();
        for (size_t a = 0; a < dets.size(); ++a) {
            for (size_t b = a + 1; b < dets.size(); ++b) {
                const double dx = dets[a].centroid_image.x - dets[b].centroid_image.x;
                const double dy = dets[a].centroid_image.y - dets[b].centroid_image.y;
                const double sep = std::sqrt(dx * dx + dy * dy);
                CHECK_MESSAGE(sep > kDuplicateSeparationPx,
                              "frame " << f.frame << ": detections " << a
                                       << " and " << b << " are " << sep
                                       << " px apart — the same source counted twice");
            }
        }
    }

    REQUIRE_MESSAGE(frames_with_band > 0,
                    "no frame swept a band; this test measured nothing");
    REQUIRE_MESSAGE(overlap_frames > 0,
                    "the band never crossed the tracking window over the whole "
                    "run, so the dedupe was never exercised");
}

// ---------------------------------------------------------------------------
TEST_CASE("P1-7: INV-3 — the sweep is a pure function of the frame index") {
    // The band is chosen by `frame_ % N` and nothing else: no state, no
    // randomness, no dependence on what previous frames found. Two runs of the
    // same scenario must therefore sweep the same bands in the same order, and
    // produce the same run.
    Scenario sc = load_spec();
    sc.duration_s         = 5.0;
    sc.roi_refresh_frames = 7;

    const std::vector<FrameRecord> a = run_frames(sc, 150);
    const std::vector<FrameRecord> b = run_frames(sc, 150);
    REQUIRE(a.size() == b.size());
    REQUIRE(a.size() > 50);

    for (size_t i = 0; i < a.size(); ++i) {
        CHECK_MESSAGE(a[i].roi_band.y0     == b[i].roi_band.y0,     "frame " << i);
        CHECK_MESSAGE(a[i].roi_band.height == b[i].roi_band.height, "frame " << i);
        CHECK_MESSAGE(a[i].candidate_count == b[i].candidate_count, "frame " << i);
    }
}
