// engine/video_probe.hpp — CP 0.7, the Stage 0 ★ GATE.
//
// ---------------------------------------------------------------------------
// WHY THIS TINY FILE IS A HARD GATE
// ---------------------------------------------------------------------------
// Benchmark Performance-2 is 30% of the total marks and consists entirely of
// running MP4 files the evaluators hand over (design §3.1). If cv::VideoCapture
// cannot open an H.264 file on the target platform -- because the vcpkg build
// was configured without ffmpeg, because the Windows runner is missing a codec
// DLL, because the opencv4 port's videoio feature was not requested -- then
// nothing downstream of it matters. Design §14 is blunt about it: "If this
// fails, STOP and solve it."
//
// So the probe exists before the video subsystem does, it is exercised by CI on
// every platform, and it is the thing to run first when a machine behaves oddly.
//
// It reports what it found rather than asserting a particular answer, because
// the interesting failures are partial: a file that opens but reports 0 frames,
// or reports fps as 0, or decodes only the first frame. A bare "ok" would hide
// all three.

#pragma once

#include "core/result.hpp"

#include <cstdint>
#include <filesystem>
#include <string>

namespace sat {

/// What the container and the decoder say about a video file.
struct VideoProbe {
    int         width         = 0;
    int         height        = 0;
    double      fps           = 0.0;
    int64_t     frame_count   = 0;    ///< as reported by the container; may be wrong
    int64_t     frames_read   = 0;    ///< how many actually decoded, if requested
    std::string fourcc;               ///< the codec the container claims
    std::string backend;              ///< which OpenCV backend served the file

    /// Whether the numbers are internally consistent enough to run on.
    /// A container can report a frame count it cannot deliver, and an fps of 0
    /// would make every derived clock divisor nonsense.
    [[nodiscard]] bool plausible() const noexcept {
        return width > 0 && height > 0 && fps > 0.0 && fps < 1000.0;
    }

    /// One line, in the format the CP 0.7 acceptance criterion asks for:
    ///     "1920x1080 @ 30.00 fps, 900 frames"
    [[nodiscard]] std::string summary() const;
};

/// Open `path`, read its properties, and optionally decode frames to confirm
/// the container is not lying about them.
///
/// `decode_frames`:
///   0  -- properties only; fast, and enough for the gate's headline numbers.
///   >0 -- decode up to this many frames and report how many really arrived.
///         Design §8.8 needs this: a truncated clip reports a full frame count
///         and then stops early, and the difference is the whole point.
///  -1  -- decode to the end. Use on short clips only.
///
/// Returns an error when OpenCV is not compiled in, when the file does not
/// exist, or when the decoder refuses to open it. A corrupt *frame* is not an
/// error: design §8.3 requirement 8 says skip, log and continue.
[[nodiscard]] Result<VideoProbe> probe_video(const std::filesystem::path& path,
                                             int decode_frames = 0);

/// True when this build can decode video at all. Every video code path checks
/// this and degrades to a clear message rather than a link error or a crash.
[[nodiscard]] bool video_support_compiled_in() noexcept;

}  // namespace sat
