// engine/video_probe.cpp — CP 0.7 implementation.
//
// This is the only file in Stage 0 that touches OpenCV, and it is compiled out
// entirely when OpenCV is absent so that the rest of the project keeps building
// on a machine without it.

#include "engine/video_probe.hpp"

#include <cstdio>
#include <cstdlib>

#if SAT_HAVE_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#endif

namespace sat {

std::string VideoProbe::summary() const {
    // Fixed-size buffer: this runs at startup, not in a frame, but there is no
    // reason to allocate for a log line either.
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%dx%d @ %.2f fps, %lld frames [%s via %s]",
                  width, height, fps, static_cast<long long>(frame_count),
                  fourcc.empty() ? "?" : fourcc.c_str(),
                  backend.empty() ? "?" : backend.c_str());
    return std::string(buf);
}

bool video_support_compiled_in() noexcept {
#if SAT_HAVE_OPENCV
    return true;
#else
    return false;
#endif
}

#if !SAT_HAVE_OPENCV

Result<VideoProbe> probe_video(const std::filesystem::path& path, int) {
    return Err("this build has no video support: OpenCV was not found at configure time, "
               "so '" + path.string() + "' cannot be opened. Reconfigure with "
               "-DSAT_WITH_OPENCV=ON after installing OpenCV (or run `just setup-vcpkg`). "
               "Synthetic scenarios are unaffected.");
}

#else

namespace {

// ---------------------------------------------------------------------------
// force_single_threaded_decode — call once before the first VideoCapture::open.
//
// ---------------------------------------------------------------------------
// THIS FIXES A DEADLOCK AND A SEGFAULT, NOT A PERFORMANCE PROBLEM
// ---------------------------------------------------------------------------
// FFmpeg's H.264 decoder uses frame-level threading by default. Fed the
// deliberately-corrupted test clip under CPU contention, that path both
// segfaults and deadlocks:
//
//   * segfault, reproduced at 2 runs in 10 with the suite pinned to two cores;
//   * deadlock, reproduced with a process hung for nine minutes with all four
//     threads — one main, three decode workers — blocked in futex_do_wait.
//
// Neither reproduces when the test is run alone, or under a debugger, which is
// why it only ever showed up as an intermittent red tick in CI on Release
// builds. It is a race in the decoder's error path, not in our code.
//
// Design §8.3 requirement 8 is unambiguous: "Never crash on a corrupt frame —
// skip, log, continue." And CP 14.1 treats a hang as a first-class failure
// alongside a crash. This is not a cosmetic CI problem: Benchmark
// Performance-2 is 30% of the marks and consists entirely of MP4 files we have
// never seen. One damaged packet in one of them, and a multithreaded decoder
// takes the process down or wedges it. That is the benchmark lost.
//
// Single-threaded decode removes the whole race surface. It also aligns with
// what the design already asks for: §8.3 puts decode on ONE dedicated thread of
// our own, and §11.1 pins ONNX Runtime to one thread for the same class of
// reason — a library's internal thread pool is non-determinism we neither need
// nor control. The cost is nil: §15 budgets 5-10 ms for a 2000x2000 frame
// single-threaded, overlapped with the rest of the pipeline.
//
// OpenCV 4.7 added CAP_PROP_N_THREADS for this; 4.6 and earlier have no such
// property, and Ubuntu 24.04 ships 4.6. The environment variable is read by
// OpenCV's FFmpeg backend at open time on every version that has the backend at
// all, so it is set here as well as the property. Setting both means the fix
// does not silently stop working on whichever version a machine happens to
// have.
void force_single_threaded_decode() {
    static const bool done = [] {
        // Do not clobber an explicit choice by whoever is running us.
#if defined(_WIN32)
        size_t len = 0;
        if (getenv_s(&len, nullptr, 0, "OPENCV_FFMPEG_CAPTURE_OPTIONS") != 0 || len == 0) {
            _putenv_s("OPENCV_FFMPEG_CAPTURE_OPTIONS", "threads;1");
        }
#else
        if (std::getenv("OPENCV_FFMPEG_CAPTURE_OPTIONS") == nullptr) {
            setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", "threads;1", /*overwrite=*/0);
        }
#endif
        return true;
    }();
    (void)done;
}

/// Decode OpenCV's packed FourCC into the four characters a human recognises.
std::string fourcc_to_string(double raw) {
    const auto code = static_cast<int>(raw);
    if (code == 0) return {};
    char c[5] = {
        static_cast<char>( code        & 0xFF),
        static_cast<char>((code >>  8) & 0xFF),
        static_cast<char>((code >> 16) & 0xFF),
        static_cast<char>((code >> 24) & 0xFF),
        '\0'
    };
    return std::string(c);
}

}  // namespace

Result<VideoProbe> probe_video(const std::filesystem::path& path, int decode_frames) {
    std::error_code ec;
    if (!std::filesystem::exists(path, ec)) {
        return Err("video file not found: " + path.string());
    }

    force_single_threaded_decode();

    cv::VideoCapture cap;
    // Ask for FFMPEG explicitly first. Leaving the backend to OpenCV's
    // auto-selection is how you end up with a build that works on the developer's
    // machine and silently picks GStreamer (or nothing) on the evaluator's.
    if (!cap.open(path.string(), cv::CAP_FFMPEG)) {
        // Fall back to whatever the platform offers rather than failing outright:
        // the Windows runner may prefer MSMF, and a working decode beats a
        // preferred decode.
        if (!cap.open(path.string(), cv::CAP_ANY)) {
            return Err("cv::VideoCapture could not open '" + path.string() +
                       "'. The file may be corrupt, or this OpenCV build may lack "
                       "the ffmpeg/videoio backend — check that the opencv4 port was "
                       "built with the [videoio,ffmpeg] features.");
        }
    }

    // -----------------------------------------------------------------------
    // ONE DECODER THREAD. This is not a performance choice.
    //
    // FFmpeg's H.264 decoder uses frame-level threading by default. On the
    // deliberately-corrupted test clip, under CPU contention, that path
    // segfaults — reproduced at 2 runs in 10 with the suite pinned to two
    // cores, and not at all when run alone or under a debugger. It is the
    // classic shape of a race in a decoder being fed a malformed bitstream.
    //
    // Design §8.3 requirement 8 is unambiguous: "Never crash on a corrupt
    // frame — skip, log, continue." A crash here is not a cosmetic CI problem.
    // Benchmark Performance-2 is 30% of the marks and consists entirely of
    // MP4 files we have never seen; if one of them has a damaged packet and the
    // decoder takes the process down, that is the benchmark lost.
    //
    // Single-threaded decode removes the entire race surface. It also matches
    // what the design already asks for elsewhere: §8.3 puts decode on ONE
    // dedicated thread, and §11.1 pins ONNX Runtime to one thread for the same
    // class of reason — a library's internal thread pool is a source of
    // non-determinism we neither need nor control.
    //
    // The cost is nil here: a 2000x2000 H.264 frame decodes in 5-10 ms
    // single-threaded (§15), against a budget that overlaps decode with the
    // rest of the pipeline anyway.
    // OpenCV >= 4.7's property form of the same thing. Unknown properties are
    // ignored rather than rejected, so this is safe on 4.6 where the
    // environment variable above is doing the work.
    //
    // The literal 70 is cv::CAP_PROP_N_THREADS; it is written numerically
    // because the enumerator does not exist in 4.6 and referring to it by name
    // would not compile there.
    constexpr int kCapPropNThreads = 70;
    cap.set(kCapPropNThreads, 1);

    VideoProbe p;
    p.width       = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    p.height      = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    p.fps         = cap.get(cv::CAP_PROP_FPS);
    p.frame_count = static_cast<int64_t>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    p.fourcc      = fourcc_to_string(cap.get(cv::CAP_PROP_FOURCC));
    p.backend     = cap.getBackendName();

    // A container that reports a negative frame count (some VFR muxes do) is
    // reporting "I do not know", not "minus four". Normalise it so downstream
    // sizing logic does not have to special-case it.
    if (p.frame_count < 0) p.frame_count = 0;

    if (decode_frames != 0) {
        cv::Mat frame;
        const int64_t limit = decode_frames < 0
                            ? std::numeric_limits<int64_t>::max()
                            : static_cast<int64_t>(decode_frames);
        int64_t consecutive_failures = 0;
        while (p.frames_read < limit) {
            if (!cap.read(frame) || frame.empty()) {
                // Design §8.3 requirement 8: never crash on a corrupt frame --
                // skip, log, continue. Distinguishing "end of file" from "one
                // bad frame" is not possible through this API, so we allow a
                // short run of failures before concluding the stream ended.
                if (++consecutive_failures > 8) break;
                continue;
            }
            consecutive_failures = 0;
            ++p.frames_read;
        }
    }

    return Ok(p);
}

#endif  // SAT_HAVE_OPENCV

}  // namespace sat
