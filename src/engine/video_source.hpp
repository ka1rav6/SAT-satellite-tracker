// engine/video_source.hpp — CP 8.3, 8.4, 8.5. Benchmark Performance-2, 30%.
//
// ---------------------------------------------------------------------------
// THE REQUIREMENT IS AMBIGUOUS, SO BOTH READINGS ARE IMPLEMENTED
// ---------------------------------------------------------------------------
// BP-2 says: "Each team will be given a few video files (.mp4) @30 fps,
// covering a complete screen with noise and moving beacon spot. The software
// needs to bypass its PTZ camera and take this video as an input to the coarse
// pointing system."
//
// "Covering a complete screen" can mean the video IS the 2000x2000 screen, with
// the PTZ cropping a 640x480 viewport out of it and panning — or it can mean
// the video is what the camera already sees, with no cropping at all. §8.3's
// judgement: "Implement both readings — ~200 lines, removes all risk." Thirty
// percent of the marks is not a thing to be clever about.
//
//   video_screen   the video is the screen; crop and pan;  pointing ACTIVE
//   video_direct   the video is the camera feed;           pointing DISABLED
//
// Auto-detected from resolution, logged, and overridable from the config.
//
// ---------------------------------------------------------------------------
// INV-8 — NO DAMAGE IS ADDED IN VIDEO MODES
// ---------------------------------------------------------------------------
// The supplied clip already contains whatever noise it contains. Adding ours on
// top would mean the graded benchmark measures a scene nobody gave us, and the
// number would be unfalsifiable. §8.3 requirement 5 says to ASSERT the noise
// and atmosphere generators are disabled and warn if the config enables them,
// which is what check_inv8() does — it does not quietly clear the flags,
// because a scenario asking for noise on a video is a mistake worth telling
// someone about.
//
// ---------------------------------------------------------------------------
// BICUBIC, NOT BILINEAR
// ---------------------------------------------------------------------------
// Requirement 4, and it is about the graded metric directly: "bilinear smooths
// peaks and biases centroids toward pixel centres". A beacon is a small bright
// blob a few pixels across, which is precisely the signal bilinear damages
// most. CP 8.6's harness measures how much error the resampling contributes on
// its own, so the claim is checked rather than trusted.

#pragma once

#include "engine/decode_thread.hpp"
#include "engine/frame_source.hpp"
#include "scenario/scenario.hpp"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace sat {

/// Which reading of BP-2 a source implements.
enum class VideoMode : uint8_t { Screen, Direct };

[[nodiscard]] const char* video_mode_name(VideoMode m) noexcept;

// ---------------------------------------------------------------------------
// CP 8.5 — auto-detection.
//
// "if video resolution >> camera resolution -> video_screen; if ~= camera
// resolution -> video_direct."
//
// The threshold is a factor of 1.5 on either axis rather than an exact match,
// because a clip is not obliged to be exactly 640x480 to be a camera feed —
// CP 8.8 ships a 641x481 file for exactly this reason. Anything at least half
// again bigger than the camera in either dimension is a screen.
// ---------------------------------------------------------------------------
[[nodiscard]] VideoMode detect_video_mode(int video_w, int video_h,
                                          int camera_w, int camera_h) noexcept;

/// §8.3 requirement 5. Returns a warning string, or empty when the scenario is
/// already clean. The caller prints it; nothing is silently modified.
[[nodiscard]] std::string check_inv8(const Scenario& sc);

// ---------------------------------------------------------------------------
// bicubic_sample — one sample of an 8-bit image at a continuous position.
//
// Catmull-Rom (a = -0.5), which is the interpolating member of the cubic family
// — it passes exactly through the source samples, so an integer offset is an
// identity and the crop adds nothing at all. A non-interpolating cubic
// (B-spline) would blur even at zero offset, which on this metric is free error.
//
// Edge behaviour is clamp. A crop near the border of the screen genuinely has
// no data beyond it, and clamping is the choice that does not invent gradient
// where there is none — a mirrored edge would create a false peak that the
// detector could lock onto.
// ---------------------------------------------------------------------------
[[nodiscard]] float bicubic_sample(const uint8_t* img, int w, int h,
                                   double x, double y) noexcept;

/// Crop a `out_w` x `out_h` window whose CENTRE is at (cx, cy) in source
/// pixels, resampling bicubically. Sub-pixel `cx`/`cy` is the point of the
/// exercise: the pan is continuous, not snapped to a pixel.
void bicubic_crop(const uint8_t* src, int sw, int sh,
                  double cx, double cy,
                  uint8_t* dst, int out_w, int out_h) noexcept;

// ---------------------------------------------------------------------------
// VideoSource — both readings, one class.
//
// They differ in four lines (whether a crop happens and what the geometry
// reports), and §6.2 B4's claim that "this is the only place the modes differ"
// is easier to keep true when the difference is visible in one file than when
// it is spread across two.
// ---------------------------------------------------------------------------
class VideoSource final : public IFrameSource {
public:
    /// Open a clip. `mode_override` of nullptr means auto-detect (CP 8.5).
    [[nodiscard]] static Result<std::unique_ptr<VideoSource>>
    open(const std::filesystem::path& path, const Scenario& sc,
         const VideoMode* mode_override = nullptr);

    [[nodiscard]] bool next(Angle2 commanded_boresight, SourceFrame& out) override;
    [[nodiscard]] FrameGeometry geometry() const override;
    [[nodiscard]] bool supports_pointing() const override {
        return mode_ == VideoMode::Screen;
    }
    [[nodiscard]] const char* name() const override {
        return mode_ == VideoMode::Screen ? "video_screen" : "video_direct";
    }
    void shutdown() override { decoder_.close(); }

    [[nodiscard]] VideoMode mode() const noexcept { return mode_; }
    [[nodiscard]] double    fps()  const noexcept { return fps_; }
    /// The FILE's dimensions, which in direct mode are not the screen's.
    [[nodiscard]] int source_width()  const noexcept { return src_w_; }
    [[nodiscard]] int source_height() const noexcept { return src_h_; }

    /// The crop's centre in source pixels, for the GUI overlay and the log.
    [[nodiscard]] Pixel2 last_crop_centre() const noexcept { return crop_centre_; }

    /// Diagnostics that belong in run.json: how much of the file could not be
    /// read, and whether the main loop ever waited on the decoder.
    [[nodiscard]] int64_t skipped_frames() const noexcept { return decoder_.skipped(); }
    [[nodiscard]] int64_t consumer_waits() const noexcept {
        return decoder_.consumer_waits();
    }

    /// §8.3 requirement 3: the real fps, and the clock divisor derived from it.
    /// Zero means the rate does not divide truth_hz cleanly — the caller must
    /// report that rather than drift silently.
    [[nodiscard]] int camera_divisor() const noexcept { return divisor_; }

    /// The warning from check_inv8(), if any. Empty when the scenario is clean.
    [[nodiscard]] const std::string& inv8_warning() const noexcept { return inv8_; }

    /// Truth from a --truth CSV (CP 8.7), if one was loaded.
    void set_truth(std::vector<FrameTruth> truth) { truth_ = std::move(truth); }

private:
    DecodeThread  decoder_;
    DecodedFrame  frame_;
    VideoMode     mode_ = VideoMode::Direct;

    CameraGeometry cam_{};
    ScreenGeometry screen_{};
    double fps_      = 0.0;
    int    divisor_  = 0;
    int    src_w_    = 0, src_h_ = 0;

    std::vector<uint8_t> cropped_;
    Pixel2               crop_centre_{};
    std::string          inv8_;
    std::vector<FrameTruth> truth_;
    int64_t              emitted_ = 0;
};

}  // namespace sat
