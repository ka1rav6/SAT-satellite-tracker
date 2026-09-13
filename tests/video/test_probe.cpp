// tests/video/test_probe.cpp — CP 0.7, the Stage 0 ★ GATE.
//
// "Windows CI prints 320x240 @ 30.00 fps, 30 frames. If this fails, STOP and
//  solve it — 30% of marks depend on it."
//
// Benchmark Performance-2 is 30% of the total and consists entirely of running
// evaluator-supplied MP4s. If cv::VideoCapture cannot open an H.264 file on the
// target platform, nothing downstream matters. This suite is the tripwire.
//
// It also covers the CP 8.8 clips early. Those are not due until Stage 8, but
// generating them costs nothing now and having them under CI from Stage 0 means
// a decoder regression is caught the day it appears rather than the week the
// video subsystem is being written.

#include <doctest/doctest.h>

#include "engine/video_probe.hpp"

#include <cstdlib>
#include <filesystem>
#include <string>
#if !defined(_WIN32)
#include <sys/wait.h>
#endif

using namespace sat;

namespace {

// The clips live next to this file. SAT_TEST_CLIP_DIR is injected by CMake so
// the tests do not depend on the working directory ctest happens to use.
std::filesystem::path clip(const char* name) {
    return std::filesystem::path(SAT_TEST_CLIP_DIR) / name;
}

bool clips_available() {
    return std::filesystem::exists(clip("gate_320x240_30fps.mp4"));
}

/// Probe one clip in a SEPARATE PROCESS, returning the exit code.
///
/// ---------------------------------------------------------------------------
/// WHY THIS IS NOT IN-PROCESS LIKE THE TESTS ABOVE
/// ---------------------------------------------------------------------------
/// The CP 8.8 clips include a deliberately corrupted bitstream, and feeding a
/// sequence of them to libavcodec in one process intermittently segfaults —
/// reproduced at roughly 4 runs in 30 with the suite pinned to two cores, and
/// never when a clip is probed alone. It is cumulative decoder state, and it is
/// a fault inside FFmpeg's error path: no amount of checking on our side of the
/// API can catch a SIGSEGV raised in a decoder.
///
/// (Single-threaded decode, forced in engine/video_probe.cpp, already removed a
/// companion DEADLOCK with all four threads parked in futex_do_wait. The
/// residual crash needs isolation rather than serialisation.)
///
/// Running each clip through the shipped binary is the mitigation that actually
/// works, and it is the one design §8.3 already points at: decode is meant to
/// live on its own thread, and promoting that to its own PROCESS is the natural
/// extension when the library being wrapped can take the process down.
///
/// It also makes this a better test: it exercises the real `sat-tracker
/// --probe-video` entry point, which is what CP 8.9 says must work bare, rather
/// than a function the shipping path does not call the same way.
int probe_in_subprocess(const char* clip_name) {
    const std::string cmd = std::string("\"") + SAT_TRACKER_BINARY + "\" --probe-video \""
                          + clip(clip_name).string() + "\" > "
#if defined(_WIN32)
                            "NUL 2>&1";
#else
                            "/dev/null 2>&1";
#endif
    const int rc = std::system(cmd.c_str());
#if defined(_WIN32)
    return rc;
#else
    // Distinguish a clean non-zero exit from death by signal: the first is the
    // required behaviour for an unopenable file, the second is never acceptable.
    if (WIFSIGNALED(rc)) return -WTERMSIG(rc);
    return WIFEXITED(rc) ? WEXITSTATUS(rc) : -1;
#endif
}

}  // namespace

TEST_CASE("★ GATE CP 0.7: OpenCV opens a committed MP4 and reports its properties") {
    if (!video_support_compiled_in()) {
        // Not a failure of this build: SAT_WITH_OPENCV=OFF is a supported
        // configuration and synthetic scenarios are unaffected. But it IS a
        // failure of the gate, so say so loudly rather than passing silently.
        MESSAGE("SKIPPED: this build has no OpenCV. The video path (30% of marks) "
                "is NOT covered. Reconfigure with -DSAT_WITH_OPENCV=ON.");
        return;
    }
    REQUIRE_MESSAGE(clips_available(),
                    "test clips missing — run tools/make_test_videos.sh");

    auto r = probe_video(clip("gate_320x240_30fps.mp4"), -1);
    REQUIRE_MESSAGE(r.has_value(), r.error());

    const VideoProbe& p = *r;
    INFO(p.summary());

    CHECK(p.width == 320);
    CHECK(p.height == 240);
    CHECK(p.fps == doctest::Approx(30.0).epsilon(1e-3));
    CHECK(p.frame_count == 30);
    CHECK(p.frames_read == 30);      // the container is not lying
    CHECK(p.plausible());
    CHECK_FALSE(p.backend.empty()); // we know which backend served it
}

TEST_CASE("probing a missing file is an error, not a crash") {
    auto r = probe_video(clip("this_file_does_not_exist.mp4"));
    CHECK_FALSE(r.has_value());
    CHECK(r.error().find("not found") != std::string::npos);
}

TEST_CASE("frame rate is read from the container, never assumed to be 30") {
    // Design §8.3 requirement 3. Assuming 30 would make every derived clock
    // divisor wrong on a 25 or 60 fps file, and the drift would be silent.
    if (!video_support_compiled_in() || !clips_available()) return;

    struct Case { const char* file; double fps; };
    const Case cases[] = {
        {"rate_25fps.mp4", 25.0},
        {"rate_60fps.mp4", 60.0},
    };
    for (const auto& c : cases) {
        INFO("clip = " << c.file);
        auto r = probe_video(clip(c.file));
        REQUIRE_MESSAGE(r.has_value(), r.error());
        CHECK(r->fps == doctest::Approx(c.fps).epsilon(1e-3));
    }
}

TEST_CASE("auto-detection inputs: screen-sized and camera-sized clips differ clearly") {
    // Design §8.3: "if video resolution >> camera resolution -> video_screen;
    // if ~= camera resolution -> video_direct." The detection logic lands in
    // Stage 8; this asserts the two clips it will be given are distinguishable.
    if (!video_support_compiled_in() || !clips_available()) return;

    auto screen = probe_video(clip("screen_2000x2000_30fps.mp4"));
    auto direct = probe_video(clip("direct_640x480_30fps.mp4"));
    REQUIRE(screen.has_value());
    REQUIRE(direct.has_value());

    CHECK(screen->width == 2000);
    CHECK(screen->height == 2000);
    CHECK(direct->width == 640);
    CHECK(direct->height == 480);
    // The ratio is what the heuristic will key on; it must not be marginal.
    CHECK(static_cast<double>(screen->width) / direct->width > 3.0);
}

TEST_CASE("CP 8.8 awkward clips: all open or fail cleanly, none crash") {
    // The full list from design §8.8. Reaching the end of this test at all is
    // most of the point — a crash or a hang here fails the suite, and CP 14.1
    // treats a hang as a first-class failure mode.
    if (!video_support_compiled_in() || !clips_available()) return;

    struct Case {
        const char* file;
        bool        must_open;   ///< false = failing cleanly is the correct answer
        const char* why;
    };
    const Case cases[] = {
        {"odd_641x481.mp4",           true,  "odd, non-even resolution"},
        {"vfr_640x480.mp4",           true,  "variable frame rate"},
        {"colour_640x480.mp4",        true,  "colour, not monochrome (spec row 2)"},
        {"lowbitrate_640x480.mp4",    true,  "heavy compression artifacts"},
        {"beacon_late_640x480.mp4",   true,  "beacon absent at start"},
        {"beacon_exits_640x480.mp4",  true,  "beacon leaves and never returns"},
        {"corrupt_640x480.mp4",       true,  "damaged payload, valid container"},
        {"truncated_faststart.mp4",   true,  "index intact, stream cut short"},
        {"truncated_noindex.mp4",     false, "moov atom removed — unopenable"},
    };

    for (const auto& c : cases) {
        INFO("clip = " << c.file << "  (" << c.why << ")");
        const int rc = probe_in_subprocess(c.file);

        // A negative code means the process died by signal. That is the failure
        // this whole test exists to rule out — "None crash", in §8.8's words —
        // and it is reported distinctly from a clean rejection.
        INFO("exit code " << rc << (rc < 0 ? "  (KILLED BY SIGNAL)" : ""));
        REQUIRE(rc >= 0);

        if (c.must_open) {
            CHECK(rc == 0);
        } else {
            // Failing is correct here, but it must be a reported failure with a
            // usable message, not a crash and not a silent success.
            CHECK(rc != 0);
        }
    }
}

TEST_CASE("the odd-resolution clip really is odd") {
    // This test exists because the first version of the generator did NOT
    // produce an odd clip: libx264 with yuv420p chroma subsampling silently
    // rounds odd dimensions down to even, so 641x481 became 640x480 and the
    // case tested nothing at all. Pinning the dimensions here means that
    // regression cannot come back unnoticed.
    if (!video_support_compiled_in() || !clips_available()) return;

    auto r = probe_video(clip("odd_641x481.mp4"));
    REQUIRE_MESSAGE(r.has_value(), r.error());
    CHECK(r->width == 641);
    CHECK(r->height == 481);
    CHECK(r->width % 2 == 1);
    CHECK(r->height % 2 == 1);
}

TEST_CASE("a truncated stream is detected by the frame-count discrepancy") {
    // The container claims a full frame count it cannot deliver. Reporting that
    // gap is how §8.3 requirement 7's "EOF is a clean termination" stays
    // distinguishable from "we silently lost half the run".
    if (!video_support_compiled_in() || !clips_available()) return;

    auto r = probe_video(clip("truncated_faststart.mp4"), -1);
    REQUIRE_MESSAGE(r.has_value(), r.error());
    CHECK(r->frame_count > 0);
    CHECK(r->frames_read > 0);                  // it did decode something
    CHECK(r->frames_read < r->frame_count);     // but not what it promised
}

TEST_CASE("a corrupt frame is skipped, not fatal") {
    // Design §8.3 requirement 8: "Never crash on a corrupt frame — skip, log,
    // continue." The clip has a valid container and a deliberately damaged
    // payload, so decoding must survive and return most of the frames.
    if (!video_support_compiled_in() || !clips_available()) return;

    auto r = probe_video(clip("corrupt_640x480.mp4"), -1);
    REQUIRE_MESSAGE(r.has_value(), r.error());
    CHECK(r->frames_read > 0);
    CHECK(r->plausible());
}
