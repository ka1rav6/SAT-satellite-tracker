// engine/video_probe.cpp — CP 0.7 implementation.
//
// This is the only file in Stage 0 that touches OpenCV, and it is compiled out
// entirely when OpenCV is absent so that the rest of the project keeps building
// on a machine without it.

#include "engine/video_probe.hpp"

#include <cstdio>

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
