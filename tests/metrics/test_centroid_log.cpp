// tests/metrics/test_centroid_log.cpp — CP 7.2.
//
// "centroid.csv in its FINAL SUBMISSION FORMAT (§13.2). Accept when: a run
//  produces a valid CSV whose header alone suffices to interpret it; blank rows
//  (not stale values) on no-detection frames."
//
// This is the artifact Benchmark Performance-2 is graded from, so the tests are
// deliberately pedantic about the format. The parser below is written from
// §13.2's specification, not from the writer's source — if the two ever
// disagree, that is the bug this test exists to find.

#include <doctest/doctest.h>

#include "metrics/centroid_log.hpp"
#include "scenario/schema.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace sat;

namespace {

std::string temp_path(const char* stem) {
    // Written beside the build tree rather than in /tmp, so a failing test
    // leaves the artifact somewhere a person will actually look.
    return std::string(SAT_TEST_TMP_DIR) + "/" + stem;
}

std::vector<std::string> read_lines(const std::string& path) {
    std::ifstream in(path);
    std::vector<std::string> out;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        out.push_back(line);
    }
    return out;
}

/// Split a CSV line, KEEPING empty fields. The whole point of §13.2's
/// no-detection rule is that fields can be empty, so a splitter that collapsed
/// them would hide exactly the bug being tested for.
std::vector<std::string> split(const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    std::istringstream in(s);
    while (std::getline(in, cur, ',')) out.push_back(cur);
    // getline drops a trailing empty field; restore it.
    if (!s.empty() && s.back() == ',') out.emplace_back();
    return out;
}

CentroidLogHeader make_header() {
    CentroidLogHeader h;
    h.source      = "scenarios/baseline.toml";
    h.mode        = "synthetic";
    h.build       = "a3f21c9";
    h.utc         = "2026-09-14T10:22:31Z";
    h.screen_w    = 2000; h.screen_h = 2000;
    h.camera_w    = 640;  h.camera_h = 480;
    h.fps         = 30.0;
    h.ifov_urad   = 109.08;
    h.onnxruntime = "none";
    h.ai_enabled  = false;
    return h;
}

FrameRecord frame(int64_t i, bool detected, TrackMode mode) {
    FrameRecord r;
    r.frame  = i;
    r.time_s = i / 30.0;
    r.mode   = mode;
    r.boresight_cmd = Angle2{};
    if (detected) {
        r.detected           = true;
        r.detection_screen   = Pixel2{1423.812, 674.209};
        r.detection_img      = Pixel2{331.812, 180.209};
        r.centroid_sigma_px  = 0.142f;
        r.detection_snr      = 38.4f;
        r.detection_area_px  = 98;
        r.detection_size_est_px = 10;
    }
    return r;
}

}  // namespace

TEST_CASE("CP 7.2: the header alone suffices to interpret the file") {
    // §13.2's fourth rule. The test is the honest form of it: take ONLY the
    // comment block, and check that every quantity needed to make sense of a
    // data row is present in it.
    const std::string h = CentroidLog::header_text(make_header());
    MESSAGE("\n" << h);

    CHECK(h.find("# SAT centroid log v1") == 0);      // versioned, and first

    // Provenance: which run produced this, from which build, when.
    CHECK(h.find("source=scenarios/baseline.toml") != std::string::npos);
    CHECK(h.find("mode=synthetic") != std::string::npos);
    CHECK(h.find("build=a3f21c9") != std::string::npos);
    CHECK(h.find("utc=2026-09-14T10:22:31Z") != std::string::npos);

    // Geometry: enough to convert either coordinate pair into the other, or
    // into angles, with nothing else to hand.
    CHECK(h.find("screen_px=2000x2000") != std::string::npos);
    CHECK(h.find("camera_px=640x480") != std::string::npos);
    CHECK(h.find("fps=30.000") != std::string::npos);
    CHECK(h.find("ifov_urad=109.08") != std::string::npos);

    // INV-7's disclosure: was AI in the loop for these numbers or not?
    CHECK(h.find("ai_enabled=false") != std::string::npos);

    // And the columns, named.
    CHECK(h.find("# columns: ") != std::string::npos);
    CHECK(h.find(CentroidLog::kColumns) != std::string::npos);

    // Every line of the block is a comment, so a naive CSV reader that skips
    // '#' lines gets a clean table.
    std::istringstream in(h);
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty()) CHECK(line[0] == '#');
    }
}

TEST_CASE("CP 7.2: the column list matches design 13.2 exactly, in order") {
    // Pinning the order, because a reader written against the specification
    // will index by position. A silently reordered column is the kind of error
    // that produces plausible-looking wrong answers.
    CHECK(std::string(CentroidLog::kColumns) ==
          "frame,time_s,state,cx_screen,cy_screen,cx_cam,cy_cam,"
          "sigma_px,snr,area_px,size_est_px,bore_x,bore_y");
    CHECK(split(CentroidLog::kColumns).size() == 13);
}

TEST_CASE("CP 7.2: a no-detection frame leaves the measurement columns EMPTY") {
    // §13.2's first rule, and INV-9: "Blank centroid columns when no detection.
    // Never stale, never interpolated."
    const ScreenGeometry screen =
        ScreenGeometry::make(2000, 2000, CameraGeometry::make(640, 480, 4.0, 3.0));
    const std::string path = temp_path("cp72_blank.csv");

    {
        CentroidLog log;
        REQUIRE(log.open(path, make_header()));
        log.write(frame(0, false, TrackMode::Search),  screen);
        log.write(frame(1, true,  TrackMode::Detect),  screen);
        log.write(frame(2, false, TrackMode::Reacquire), screen);   // lost again
        CHECK(log.rows() == 3);
    }

    const std::vector<std::string> lines = read_lines(path);
    std::vector<std::string> data;
    for (const std::string& l : lines) if (!l.empty() && l[0] != '#') data.push_back(l);
    REQUIRE(data.size() == 3);
    MESSAGE("\n" << data[0] << "\n" << data[1] << "\n" << data[2]);

    // Columns 3..10 are the measurements: cx_screen .. size_est_px.
    for (int row : {0, 2}) {
        const std::vector<std::string> f = split(data[static_cast<size_t>(row)]);
        REQUIRE(f.size() == 13);
        for (int c = 3; c <= 10; ++c) {
            CHECK(f[static_cast<size_t>(c)].empty());
        }
        // But frame, time, state and the boresight are still there: a reader
        // must be able to see that the frame EXISTED and where we were looking.
        CHECK_FALSE(f[0].empty());
        CHECK_FALSE(f[1].empty());
        CHECK_FALSE(f[2].empty());
        CHECK_FALSE(f[11].empty());
        CHECK_FALSE(f[12].empty());
    }

    // THE STALE-VALUE TEST. Row 2 follows a row that DID have a detection. If
    // anything remembered the previous value, 1423.812 would appear here.
    CHECK(data[2].find("1423.812") == std::string::npos);
    CHECK(data[2].find("38.4")     == std::string::npos);
    // And zero is not an acceptable stand-in either — it is a position.
    const std::vector<std::string> f2 = split(data[2]);
    CHECK(f2[3] != "0.000");
}

TEST_CASE("CP 7.2: a detection row carries both coordinate frames and the uncertainty") {
    const ScreenGeometry screen =
        ScreenGeometry::make(2000, 2000, CameraGeometry::make(640, 480, 4.0, 3.0));
    const std::string path = temp_path("cp72_detect.csv");
    {
        CentroidLog log;
        REQUIRE(log.open(path, make_header()));
        log.write(frame(7, true, TrackMode::Track), screen);
    }
    std::vector<std::string> data;
    for (const std::string& l : read_lines(path)) if (!l.empty() && l[0] != '#') data.push_back(l);
    REQUIRE(data.size() == 1);

    const std::vector<std::string> f = split(data[0]);
    REQUIRE(f.size() == 13);
    CHECK(f[0] == "7");
    CHECK(f[2] == "TRACK");
    CHECK(std::stod(f[3]) == doctest::Approx(1423.812));   // screen
    CHECK(std::stod(f[4]) == doctest::Approx(674.209));
    CHECK(std::stod(f[5]) == doctest::Approx(331.812));    // camera
    CHECK(std::stod(f[6]) == doctest::Approx(180.209));
    CHECK(std::stod(f[7]) == doctest::Approx(0.142));      // §13.2 rule 2: sigma_px
    CHECK(std::stod(f[8]) == doctest::Approx(38.4));
    CHECK(f[9]  == "98");
    CHECK(f[10] == "10");

    // §13.2 rule 3 wants BOTH frames, and they must genuinely differ — a writer
    // that emitted the same pair twice would satisfy the column count and
    // nothing else.
    CHECK(std::stod(f[3]) != doctest::Approx(std::stod(f[5])));

    // bore_x/bore_y are in SCREEN pixels, so they can be subtracted from
    // cx_screen directly. A zero commanded angle is the screen centre.
    CHECK(std::stod(f[11]) == doctest::Approx(999.5));
    CHECK(std::stod(f[12]) == doctest::Approx(999.5));
}

TEST_CASE("CP 7.2: an aborted run still leaves a valid file") {
    // §13.2: "Write the header at file open (A9), not at close." The abort
    // cases that matter never flush — a sweep worker hitting its timeout,
    // CP 7.5 runs 500 of them — so the header is flushed immediately and this
    // test reads the file while the log is STILL OPEN.
    const ScreenGeometry screen =
        ScreenGeometry::make(2000, 2000, CameraGeometry::make(640, 480, 4.0, 3.0));
    const std::string path = temp_path("cp72_abort.csv");

    CentroidLog log;
    REQUIRE(log.open(path, make_header()));
    // Nothing written, nothing closed. Simulating a process killed at startup.
    const std::vector<std::string> lines = read_lines(path);
    REQUIRE(lines.size() >= 5);
    CHECK(lines[0] == "# SAT centroid log v1");
    CHECK(lines.back().find("# columns: ") == 0);
}

TEST_CASE("CP 7.2: opening an unwritable path fails loudly rather than silently") {
    // A run whose graded artifact went nowhere is worse than one that refused
    // to start, because nobody finds out until the submission.
    CentroidLog log;
    CHECK_FALSE(log.open("/nonexistent-directory-for-sat-test/x.csv", make_header()));
    CHECK_FALSE(log.is_open());
}
