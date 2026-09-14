// tests/video/test_video_source.cpp — Stage 8. Benchmark Performance-2, 30%.
//
// CP 8.3  bicubic crop at a continuous boresight
// CP 8.5  auto-detection from resolution
// CP 8.6  crop error characterisation — "you must know how much of the error
//         is your own resampling"
// CP 8.8  ten nasty clips: all run or fail cleanly, NONE CRASH

#include <doctest/doctest.h>

#include "engine/video_probe.hpp"
#include "engine/truth_csv.hpp"
#include "engine/video_source.hpp"
#include "scenario/schema.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

using namespace sat;

namespace {

std::string clip(const char* name) {
    return std::string(SAT_TEST_CLIP_DIR) + "/" + name;
}

Scenario base_scenario() {
    auto r = load_scenario(std::string(SAT_SCENARIO_DIR) + "/video_screen.toml");
    if (r) return *r;
    Scenario sc;                    // defaults are enough for these tests
    return sc;
}

/// Render a Gaussian beacon into a buffer at a known sub-pixel position.
/// Gaussian rather than a square, because a square's centroid is dominated by
/// its edges and the thing being measured here is sub-pixel behaviour.
void render_beacon(std::vector<uint8_t>& img, int w, int h,
                   double cx, double cy, double sigma, double amp, double bg) {
    img.assign(static_cast<size_t>(w) * h, static_cast<uint8_t>(bg));
    const int r = static_cast<int>(std::ceil(4.0 * sigma));
    for (int y = std::max(0, static_cast<int>(cy) - r);
         y <= std::min(h - 1, static_cast<int>(cy) + r); ++y) {
        for (int x = std::max(0, static_cast<int>(cx) - r);
             x <= std::min(w - 1, static_cast<int>(cx) + r); ++x) {
            const double dx = x - cx, dy = y - cy;
            const double v = bg + amp * std::exp(-(dx * dx + dy * dy) / (2 * sigma * sigma));
            img[static_cast<size_t>(y) * w + x] =
                static_cast<uint8_t>(std::min(255.0, v + 0.5));
        }
    }
}

/// Intensity-weighted centroid over the whole image, above a floor.
Pixel2 centroid(const uint8_t* img, int w, int h, double floor_level) {
    double sw = 0.0, sx = 0.0, sy = 0.0;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const double v = img[static_cast<size_t>(y) * w + x] - floor_level;
            if (v <= 0.0) continue;
            sw += v; sx += v * x; sy += v * y;
        }
    }
    return sw > 0.0 ? Pixel2{sx / sw, sy / sw} : Pixel2{-1, -1};
}

}  // namespace

// ===========================================================================
// CP 8.5 — auto-detection
// ===========================================================================

TEST_CASE("CP 8.5: resolution decides the mode, and the rule tolerates odd sizes") {
    // §8.3: "if video resolution >> camera resolution -> video_screen;
    //        if ~= camera resolution -> video_direct."
    CHECK(detect_video_mode(2000, 2000, 640, 480) == VideoMode::Screen);
    CHECK(detect_video_mode(1920, 1080, 640, 480) == VideoMode::Screen);
    CHECK(detect_video_mode(640,  480,  640, 480) == VideoMode::Direct);

    // CP 8.8 ships a 641x481 clip precisely because a camera feed is not
    // obliged to be exactly the nominal size. An exact-match rule would send it
    // to screen mode and crop a 640x480 window out of a 641x481 file, which
    // would look almost right and be wrong.
    CHECK(detect_video_mode(641, 481, 640, 480) == VideoMode::Direct);
    CHECK(detect_video_mode(720, 576, 640, 480) == VideoMode::Direct);

    // And the boundary is stated rather than implied: 1.5x on either axis.
    CHECK(detect_video_mode(960, 480, 640, 480) == VideoMode::Screen);
    CHECK(detect_video_mode(959, 480, 640, 480) == VideoMode::Direct);
}

// ===========================================================================
// CP 8.3 / 8.6 — the crop
// ===========================================================================

TEST_CASE("CP 8.6: an integer crop offset is an exact identity") {
    // Catmull-Rom is the INTERPOLATING cubic: w(0)=1, w(±1)=w(±2)=0. So a
    // sample taken exactly on a source pixel returns that pixel, and a crop at
    // integer offsets adds no error whatsoever. This is the property that makes
    // the number in the next test a measurement of sub-pixel resampling rather
    // than of the resampler being wrong.
    std::vector<uint8_t> src;
    render_beacon(src, 200, 200, 100.0, 100.0, 2.0, 200.0, 10.0);

    std::vector<uint8_t> dst(64 * 64);
    bicubic_crop(src.data(), 200, 200, 100.0, 100.0, dst.data(), 64, 64);

    // The window centred at (100,100) with an even size spans a half-pixel
    // offset by construction, so compare against a hand-computed window.
    int mismatches = 0;
    const double x0 = 100.0 - 63 * 0.5, y0 = 100.0 - 63 * 0.5;
    for (int y = 0; y < 64; ++y) {
        for (int x = 0; x < 64; ++x) {
            const double sx = x0 + x, sy = y0 + y;
            if (std::fabs(sx - std::round(sx)) > 1e-9) continue;   // not integer
            if (std::fabs(sy - std::round(sy)) > 1e-9) continue;
            const uint8_t want = src[static_cast<size_t>(std::lround(sy)) * 200
                                   + static_cast<size_t>(std::lround(sx))];
            if (dst[static_cast<size_t>(y) * 64 + x] != want) ++mismatches;
        }
    }
    CHECK(mismatches == 0);

    // The same claim at the sample level, which is where it actually lives.
    for (int i = 0; i < 20; ++i) {
        const int x = 90 + i, y = 95;
        CHECK(bicubic_sample(src.data(), 200, 200, x, y)
              == doctest::Approx(src[static_cast<size_t>(y) * 200 + x]));
    }
}

TEST_CASE("CP 8.6: how much centroid error the crop alone contributes") {
    // §8.4: "Render a synthetic beacon at known sub-pixel positions, crop at
    // many fractional offsets, measure how much centroid error the crop alone
    // contributes. Report the number. You are graded on centroid accuracy; you
    // must know how much of the error is your own resampling."
    //
    // The measurement compares the centroid of the CROPPED image against the
    // centroid of the same beacon computed on the source. Any difference is
    // the resampler's, because nothing else changed.
    constexpr int kW = 240, kH = 240;
    constexpr int kOut = 64;
    std::vector<uint8_t> src, dst(static_cast<size_t>(kOut) * kOut);

    double worst = 0.0, sum_sq = 0.0;
    int n = 0;
    // 20 sub-pixel phases in each axis: the resampler's error is periodic in
    // the fractional offset, so sampling the phase densely is the only way to
    // see its amplitude rather than one arbitrary point on the curve.
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 20; ++j) {
            const double fx = i / 20.0, fy = j / 20.0;
            const double bx = 120.0 + fx, by = 120.0 + fy;
            render_beacon(src, kW, kH, bx, by, 2.2, 200.0, 10.0);

            // Crop centred on the beacon's nominal pixel, so the beacon sits at
            // a known sub-pixel position inside the window.
            const double cx = 120.0, cy = 120.0;
            bicubic_crop(src.data(), kW, kH, cx, cy, dst.data(), kOut, kOut);

            const Pixel2 c_src = centroid(src.data(), kW, kH, 12.0);
            const Pixel2 c_dst = centroid(dst.data(), kOut, kOut, 12.0);
            // Map the cropped centroid back to source coordinates: the window's
            // top-left is at cx - (kOut-1)/2.
            const Pixel2 mapped{c_dst.x + cx - (kOut - 1) * 0.5,
                                c_dst.y + cy - (kOut - 1) * 0.5};
            const double err = (mapped - c_src).norm();
            worst = std::max(worst, err);
            sum_sq += err * err;
            ++n;
        }
    }
    const double rms = std::sqrt(sum_sq / n);
    MESSAGE("crop-only centroid error over 400 sub-pixel phases: "
            << rms << " px RMS, " << worst << " px worst");

    // THE NUMBER, recorded so the report can quote it and so a regression in
    // the resampler shows up here rather than as a mysterious rise in the
    // graded metric. The bound is generous relative to the measurement because
    // its job is to catch a change of KIND — someone switching to bilinear —
    // not to pin the third decimal.
    CHECK(rms   < 0.02);
    CHECK(worst < 0.05);
}

TEST_CASE("CP 8.3: bilinear would be measurably worse, which is why it is not used") {
    // §8.3 requirement 4 states the reason for bicubic as a fact. This checks
    // it, because an unchecked justification is just an opinion and this one
    // costs real work.
    constexpr int kW = 200, kH = 200;
    std::vector<uint8_t> src;

    auto bilinear = [](const uint8_t* img, int w, int h, double x, double y) {
        const int ix = static_cast<int>(std::floor(x)), iy = static_cast<int>(std::floor(y));
        const double fx = x - ix, fy = y - iy;
        auto at = [&](int xx, int yy) {
            xx = std::clamp(xx, 0, w - 1); yy = std::clamp(yy, 0, h - 1);
            return static_cast<double>(img[static_cast<size_t>(yy) * w + xx]);
        };
        return (1 - fx) * (1 - fy) * at(ix, iy) + fx * (1 - fy) * at(ix + 1, iy)
             + (1 - fx) * fy * at(ix, iy + 1) + fx * fy * at(ix + 1, iy + 1);
    };

    // Peak preservation at the worst phase: a beacon centred exactly between
    // pixels. Bilinear averages the peak away; Catmull-Rom's negative lobes
    // preserve it.
    render_beacon(src, kW, kH, 100.5, 100.5, 1.6, 220.0, 10.0);
    const double bic = bicubic_sample(src.data(), kW, kH, 100.5, 100.5);
    const double bil = bilinear(src.data(), kW, kH, 100.5, 100.5);
    const double truth = 10.0 + 220.0;   // the analytic peak

    MESSAGE("peak at a half-pixel offset: bicubic " << bic << ", bilinear " << bil
            << ", analytic " << truth);
    CHECK(std::fabs(bic - truth) < std::fabs(bil - truth));
}

// ===========================================================================
// INV-8
// ===========================================================================

TEST_CASE("INV-8: a scenario that asks for noise on a video is told so") {
    // A real video scenario never warns, whatever the noise DEFAULTS are —
    // Scenario::damage_enabled() is false for video input and the degradation
    // chain is never reached. This is the case that matters: the shipped
    // video_screen.toml omits the noise section entirely, so its fields hold
    // the specification's defaults, and warning about those would fire on
    // every correct run.
    {
        auto shipped = load_scenario(std::string(SAT_SCENARIO_DIR) + "/video_screen.toml");
        if (shipped) CHECK(check_inv8(*shipped).empty());
    }

    Scenario sc;                    // defaults: synthetic, with spec-max noise
    sc.gaussian_sigma = 0.0; sc.salt_pepper = 0.0;
    sc.noise_poisson = false; sc.hot_pixels = 0;
    sc.atmosphere = Atmosphere::Clear;
    CHECK(check_inv8(sc).empty());

    sc.salt_pepper = 0.1;
    const std::string w = check_inv8(sc);
    MESSAGE(w);
    CHECK(w.find("INV-8") != std::string::npos);
    CHECK(w.find("salt_pepper") != std::string::npos);
    // It warns; it does not silently clear the flag. A scenario asking for
    // noise on a supplied clip is a mistake worth telling someone about.
    CHECK(sc.salt_pepper == doctest::Approx(0.1));

    sc.salt_pepper = 0.0;
    sc.atmosphere = Atmosphere::Fog;
    CHECK(check_inv8(sc).find("atmosphere") != std::string::npos);
}

// ===========================================================================
// CP 8.8 — the nasty clips. NONE CRASH.
// ===========================================================================

TEST_CASE("CP 8.8: every awkward clip runs or fails cleanly, and none crash") {
    if (!video_support_compiled_in()) {
        MESSAGE("SKIPPED: this build has no video support");
        return;
    }
    const Scenario sc = base_scenario();

    // Every file in the clip directory, whatever it is. Adding a clip to the
    // directory therefore adds it to this test, which is the behaviour that
    // keeps CP 8.8's "ten nasty clips" honest as the set grows.
    std::vector<std::string> names;
    for (const auto& e : std::filesystem::directory_iterator(SAT_TEST_CLIP_DIR)) {
        if (e.path().extension() == ".mp4") names.push_back(e.path().filename().string());
    }
    std::sort(names.begin(), names.end());
    REQUIRE(names.size() >= 10);

    for (const std::string& n : names) {
        auto src = VideoSource::open(clip(n.c_str()), sc);
        if (!src) {
            // A clean refusal is a PASS. truncated_noindex.mp4 has no moov
            // atom and cannot be opened by anything; saying so is correct.
            MESSAGE(n << ": refused cleanly — " << src.error());
            continue;
        }

        int64_t frames = 0;
        SourceFrame f;
        Angle2 bore{};
        // Pan while reading, so screen-mode clips exercise the crop at
        // continuously varying sub-pixel offsets rather than at zero.
        while (frames < 120 && (*src)->next(bore, f)) {
            CHECK(f.width > 0);
            CHECK(f.height > 0);
            CHECK(f.pixels.size() == static_cast<size_t>(f.width) * f.height);
            bore.x += 700.0;
            bore.y += 300.0;
            ++frames;
        }
        (*src)->shutdown();

        MESSAGE(n << ": " << std::string(video_mode_name((*src)->mode()))
                << ", " << (*src)->fps() << " fps"
                << ", " << frames << " frames read"
                << ", " << (*src)->skipped_frames() << " skipped"
                << ", divisor " << (*src)->camera_divisor());
        // "beacon absent at start" and "beacon exits" are about CONTENT, not
        // decoding: they must deliver frames like any other clip.
        if (n.rfind("truncated", 0) != 0) CHECK(frames > 0);
        // A clean clip reports NO skipped frames. This is the assertion that
        // catches the EOF probe being miscounted as data loss — it fails on
        // every file in the suite when that regresses.
        if (n.rfind("truncated", 0) != 0 && n.rfind("corrupt", 0) != 0
            && n.rfind("lowbitrate", 0) != 0) {
            CHECK((*src)->skipped_frames() == 0);
        }
    }
}

TEST_CASE("CP 8.2: the clock divisor is re-derived from the container, never assumed") {
    if (!video_support_compiled_in()) return;
    Scenario sc = base_scenario();
    sc.truth_hz = 300;

    struct Case { const char* file; double fps; int divisor; };
    // 300 / 30 = 10, 300 / 25 = 12, 300 / 60 = 5 — all whole numbers, so all
    // three run. The point of the check is that the divisor comes from the
    // FILE: assuming 30 would advance the world by 10 sub-ticks per frame on
    // the 25 fps clip and put every metric on a 20% wrong time axis.
    for (const Case& c : {Case{"screen_2000x2000_30fps.mp4", 30.0, 10},
                          Case{"rate_25fps.mp4", 25.0, 12},
                          Case{"rate_60fps.mp4", 60.0, 5}}) {
        auto src = VideoSource::open(clip(c.file), sc);
        if (!src) { MESSAGE(c.file << ": " << src.error()); continue; }
        MESSAGE(std::string(c.file) << ": " << (*src)->fps() << " fps -> divisor "
                << (*src)->camera_divisor());
        CHECK((*src)->fps() == doctest::Approx(c.fps).epsilon(0.02));
        CHECK((*src)->camera_divisor() == c.divisor);
        (*src)->shutdown();
    }

    // A rate that does not divide truth_hz cleanly reports divisor 0 rather
    // than rounding — the caller has to raise truth_hz, and silent drift is the
    // one outcome that must not happen.
    sc.truth_hz = 100;                       // 100 / 60 is not a whole number
    auto odd = VideoSource::open(clip("rate_60fps.mp4"), sc);
    if (odd) {
        CHECK((*odd)->camera_divisor() == 0);
        (*odd)->shutdown();
    }
}

TEST_CASE("CP 8.1: the decoder keeps ahead of the consumer") {
    // "the main loop never waits; a 2000x2000 clip decodes at >60 fps."
    if (!video_support_compiled_in()) return;
    const Scenario sc = base_scenario();

    auto src = VideoSource::open(clip("screen_2000x2000_30fps.mp4"), sc);
    if (!src) FAIL(src.error());

    SourceFrame f;
    Angle2 bore{};
    int64_t frames = 0;
    while ((*src)->next(bore, f)) { ++frames; bore.x += 500.0; }
    (*src)->shutdown();

    MESSAGE(frames << " frames, consumer waited " << (*src)->consumer_waits()
            << " times, " << (*src)->skipped_frames() << " skipped");
    CHECK(frames > 0);
    // Not "never waits" as an absolute: the FIRST pop always waits, because the
    // decoder has not started yet. What matters is that it is not waiting
    // frame after frame, which would mean decode is the bottleneck.
    CHECK((*src)->consumer_waits() < frames / 2);
}


// ===========================================================================
// CP 8.7 — self-scoring
// ===========================================================================

TEST_CASE("CP 8.7: a clip with known truth scores at the expected accuracy") {
    // "Feeding a self-generated video with known truth reports the expected
    //  error."
    //
    // This is the only way the video path can be trusted at all. In synthetic
    // mode the simulator knows where the beacon is, so an error is exact by
    // construction; in video mode the clip is somebody else's and the system
    // can report where it thinks the beacon is and no more. Closing the loop on
    // ourselves — render a clip whose truth we already have, feed it back in as
    // if it came from outside, and check — is what converts "it produced
    // numbers" into "the numbers are right".
    //
    // The generator draws a 10 px box whose top-left is at (100 + 4n, 200), so
    // the centre is at (104.5 + 4n, 204.5). between() in ffmpeg's geq is
    // inclusive at both ends, so the box spans [100, 109] — which is 10 px, and
    // was 11 until this test measured a +0.498 px bias and found the fixture
    // wrong rather than the tracker.
    if (!video_support_compiled_in()) return;

    std::string csv = "frame,time_s,cx_screen,cy_screen\n";
    for (int n = 0; n < 60; ++n) {
        char b[128];
        std::snprintf(b, sizeof b, "%d,%.4f,%.3f,%.3f\n",
                      n, n / 30.0, 100.0 + 4.0 * n + 4.5, 204.5);
        csv += b;
    }

    Scenario sc = base_scenario();
    const ScreenGeometry screen = ScreenGeometry::make(640, 480, sc.camera_geometry());
    auto truth = parse_truth_csv(csv, screen, "<test>");
    if (!truth) FAIL(truth.error());
    CHECK(truth->size() == 60);

    auto src = VideoSource::open(clip("direct_640x480_30fps.mp4"), sc);
    if (!src) FAIL(src.error());
    (*src)->set_truth(*truth);

    SourceFrame f;
    Angle2 bore{};
    int scored = 0;
    double sum_sq = 0.0, sum_dx = 0.0;
    while ((*src)->next(bore, f)) {
        if (!f.has_truth) continue;
        const FrameTruth::Target* t = f.truth.primary();
        if (!t || !t->in_fov) continue;
        // A plain intensity-weighted centroid, so this measures the VIDEO PATH
        // — decode, greyscale, coordinate conversion — rather than the
        // perception pipeline, which has its own tests.
        const Pixel2 c = centroid(f.pixels.data(), f.width, f.height, 40.0);
        if (c.x < 0) continue;
        const double dx = c.x - t->image_pos.x;
        const double dy = c.y - t->image_pos.y;
        sum_sq += dx * dx + dy * dy;
        sum_dx += dx;
        ++scored;
    }
    (*src)->shutdown();

    REQUIRE(scored > 40);
    const double rmse = std::sqrt(sum_sq / scored);
    MESSAGE("self-scored over " << scored << " frames: " << rmse
            << " px RMSE, mean dx " << (sum_dx / scored) << " px");

    // Sub-pixel, and by a wide margin. The bound is loose enough to survive a
    // re-encode with a different ffmpeg, and tight enough that a half-pixel
    // coordinate-frame error — which is what this test has already caught
    // twice — fails it.
    CHECK(rmse < 0.25);
    CHECK(std::fabs(sum_dx / scored) < 0.1);
}

TEST_CASE("CP 8.7: a truth CSV leaves gaps rather than inventing a position") {
    // §13.2 writes empty columns on a no-detection row, and a truth file
    // inherits the meaning. Interpolating would silently invent a reference
    // and every error measured against it would be fiction.
    const ScreenGeometry screen =
        ScreenGeometry::make(2000, 2000, CameraGeometry::make(640, 480, 4.0, 3.0));
    auto t = parse_truth_csv(
        "frame,time_s,cx_screen,cy_screen\n"
        "0,0.0,100.0,200.0\n"
        "1,0.033,,\n"                      // no truth this frame
        "2,0.066,108.0,200.0\n", screen);
    if (!t) FAIL(t.error());
    REQUIRE(t->size() == 3);
    CHECK((*t)[0].n == 1);
    CHECK((*t)[1].n == 0);                  // a gap, not an interpolation
    CHECK((*t)[2].n == 1);
    CHECK((*t)[0].targets[0].screen_pos.x == doctest::Approx(100.0));
}

TEST_CASE("CP 8.7: a centroid.csv from a previous run is valid truth input") {
    // The format is §13.2's on purpose: a truth file can be produced by a run
    // of this program, which makes the self-scoring loop one command rather
    // than a script.
    const ScreenGeometry screen =
        ScreenGeometry::make(2000, 2000, CameraGeometry::make(640, 480, 4.0, 3.0));
    auto t = parse_truth_csv(
        "# SAT centroid log v1\n"
        "# source=x  mode=synthetic  build=abc  utc=now\n"
        "# columns: frame,time_s,state,cx_screen,cy_screen,cx_cam,cy_cam,"
        "sigma_px,snr,area_px,size_est_px,bore_x,bore_y\n"
        "0,0.0000,SEARCH,,,,,,,,,999.500,999.500\n"
        "1,0.0333,TRACK,1423.812,674.209,331.812,180.209,0.142,38.4,98,10,999.5,999.5\n",
        screen);
    if (!t) FAIL(t.error());
    REQUIRE(t->size() == 2);
    CHECK((*t)[0].n == 0);
    CHECK((*t)[1].n == 1);
    CHECK((*t)[1].targets[0].screen_pos.x == doctest::Approx(1423.812));
}
