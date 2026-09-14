// engine/video_source.cpp

#include "engine/video_source.hpp"

#include <algorithm>
#include <cmath>

namespace sat {

const char* video_mode_name(VideoMode m) noexcept {
    return m == VideoMode::Screen ? "video_screen" : "video_direct";
}

VideoMode detect_video_mode(int video_w, int video_h,
                            int camera_w, int camera_h) noexcept {
    if (camera_w <= 0 || camera_h <= 0) return VideoMode::Direct;
    const double rx = static_cast<double>(video_w) / camera_w;
    const double ry = static_cast<double>(video_h) / camera_h;
    return (rx >= 1.5 || ry >= 1.5) ? VideoMode::Screen : VideoMode::Direct;
}

std::string check_inv8(const Scenario& sc) {
    // ------------------------------------------------------------------
    // Only a scenario that is itself a VIDEO scenario can violate INV-8 by
    // asking for noise, and Scenario::damage_enabled() already encodes that:
    // it is false for anything but synthetic input, which is how the
    // degradation chain is kept out of the video path regardless of what the
    // file says.
    //
    // The first version of this function inspected the noise fields directly
    // and warned on their STRUCT DEFAULTS, which are the specification's
    // values — 20 grey levels, 10% salt and pepper, 40 hot pixels. So running
    // the shipped video_screen.toml, whose whole point is that it omits the
    // noise section, printed a five-line INV-8 warning about settings the file
    // does not contain. A warning that fires on correct input trains people to
    // ignore warnings.
    //
    // A scenario reaching here with damage_enabled() true is being used as a
    // video scenario while declaring itself synthetic, which IS worth saying.
    // ------------------------------------------------------------------
    if (!sc.damage_enabled()) return {};

    std::string w;
    auto note = [&](const std::string& s) {
        if (w.empty()) {
            w = "INV-8: no damage may be added in video modes, but this "
                "scenario enables:\n";
        }
        w += "  - " + s + "\n";
    };
    if (sc.gaussian_sigma > 0.0) note("noise.gaussian_sigma = " +
                                      std::to_string(sc.gaussian_sigma));
    if (sc.salt_pepper > 0.0)    note("noise.salt_pepper = " +
                                      std::to_string(sc.salt_pepper));
    if (sc.noise_poisson)        note("noise.poisson = true");
    if (sc.hot_pixels > 0)       note("noise.hot_pixels = " +
                                      std::to_string(sc.hot_pixels));
    if (sc.atmosphere != Atmosphere::Clear) {
        note(std::string("atmosphere.mode = ") + atmosphere_name(sc.atmosphere));
    }
    if (!w.empty()) {
        w = "INV-8: this scenario declares input.mode = synthetic but is being\n"
            "  used with a video clip, and it enables:\n" + w.substr(w.find('\n') + 1);
        // Not cleared silently: a scenario asking for noise on a supplied clip
        // is a mistake, and the useful response is to say so. The generators
        // are not RUN in video mode regardless — the source never calls them —
        // so the warning is about the config being wrong, not about the output.
        w += "  These generators are not run in video mode. The supplied clip's\n"
             "  own noise is the only noise, which is what makes the benchmark\n"
             "  a measurement of the clip rather than of our simulator.\n";
    }
    return w;
}

// ---------------------------------------------------------------------------
// bicubic_sample — Catmull-Rom.
//
//   w(t) = 1.5|t|^3 - 2.5|t|^2 + 1            for |t| <= 1
//        = -0.5|t|^3 + 2.5|t|^2 - 4|t| + 2    for 1 < |t| < 2
//
// The a = -0.5 member of the cubic family, which is the interpolating one:
// w(0) = 1 and w(1) = w(2) = 0, so sampling exactly on a source pixel returns
// that pixel unchanged. That property is why an integer crop offset costs
// nothing at all, which CP 8.6's harness checks.
// ---------------------------------------------------------------------------
namespace {

inline double cr_weight(double t) noexcept {
    t = std::fabs(t);
    if (t <= 1.0) return ((1.5 * t - 2.5) * t) * t + 1.0;
    if (t <  2.0) return (((-0.5 * t + 2.5) * t) - 4.0) * t + 2.0;
    return 0.0;
}

inline int clampi(int v, int lo, int hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

float bicubic_sample(const uint8_t* img, int w, int h, double x, double y) noexcept {
    if (w <= 0 || h <= 0) return 0.0f;

    const int ix = static_cast<int>(std::floor(x));
    const int iy = static_cast<int>(std::floor(y));
    const double fx = x - ix;
    const double fy = y - iy;

    double wx[4], wy[4];
    for (int i = 0; i < 4; ++i) {
        wx[i] = cr_weight(fx - (i - 1));
        wy[i] = cr_weight(fy - (i - 1));
    }

    double acc = 0.0;
    for (int j = 0; j < 4; ++j) {
        const int sy = clampi(iy + j - 1, 0, h - 1);   // clamp, never mirror
        const uint8_t* row = img + static_cast<ptrdiff_t>(sy) * w;
        double rowacc = 0.0;
        for (int i = 0; i < 4; ++i) {
            const int sx = clampi(ix + i - 1, 0, w - 1);
            rowacc += wx[i] * row[sx];
        }
        acc += wy[j] * rowacc;
    }
    // Catmull-Rom overshoots at a sharp edge — that is what makes it sharp —
    // so the result is clamped to the 8-bit range rather than wrapped.
    return static_cast<float>(std::clamp(acc, 0.0, 255.0));
}

void bicubic_crop(const uint8_t* src, int sw, int sh,
                  double cx, double cy,
                  uint8_t* dst, int out_w, int out_h) noexcept {
    // The crop window's top-left in source coordinates. The centre convention
    // matches core/frames.hpp's principal point: the centre of an N-pixel span
    // is at (N-1)/2, so a window centred on a source pixel lands on integers
    // and the interpolation is an identity.
    const double x0 = cx - (out_w - 1) * 0.5;
    const double y0 = cy - (out_h - 1) * 0.5;

    for (int y = 0; y < out_h; ++y) {
        uint8_t* row = dst + static_cast<ptrdiff_t>(y) * out_w;
        const double sy = y0 + y;
        for (int x = 0; x < out_w; ++x) {
            // +0.5 then truncate: round-to-nearest, so the 8-bit round trip of
            // an identity sample is exact. Truncation alone would bias every
            // pixel down by half a level, which over a beacon is a systematic
            // intensity error and therefore a centroid error.
            row[x] = static_cast<uint8_t>(bicubic_sample(src, sw, sh, x0 + x, sy) + 0.5f);
        }
    }
}

// ---------------------------------------------------------------------------
// VideoSource
// ---------------------------------------------------------------------------
Result<std::unique_ptr<VideoSource>>
VideoSource::open(const std::filesystem::path& path, const Scenario& sc,
                  const VideoMode* mode_override) {
    auto vs = std::make_unique<VideoSource>();

    if (Status st = vs->decoder_.open(path); !st) {
        return Err(st.error());
    }

    vs->src_w_ = vs->decoder_.width();
    vs->src_h_ = vs->decoder_.height();
    vs->fps_   = vs->decoder_.fps();
    vs->cam_   = sc.camera_geometry();
    vs->screen_ = sc.screen_geometry();

    vs->mode_ = mode_override
        ? *mode_override
        : detect_video_mode(vs->src_w_, vs->src_h_, vs->cam_.width, vs->cam_.height);

    // ------------------------------------------------------------------
    // §8.3 requirement 3: "Probe the real fps from the container; re-derive
    // camera_divisor. Reject or adapt if it does not divide truth_hz cleanly.
    // NEVER ASSUME 30."
    //
    // The sub-tick loop runs the world at truth_hz and the camera at
    // camera_hz, and the divisor between them has to be a whole number or the
    // world advances by a different amount between consecutive frames — a
    // slow, invisible drift in every metric. CP 8.8 ships 25 fps and 60 fps
    // clips precisely to exercise this.
    //
    // Adapting rather than rejecting: truth_hz is ours to choose, so a rate
    // that does not divide it is handled by reporting divisor 0 and letting
    // the caller raise truth_hz to a multiple. Rejecting a perfectly good
    // 25 fps file because our own default was 300 would be our bug presented
    // as the file's.
    // ------------------------------------------------------------------
    if (vs->fps_ > 0.0 && sc.truth_hz > 0) {
        const double ratio = sc.truth_hz / vs->fps_;
        const int    n     = static_cast<int>(std::lround(ratio));
        vs->divisor_ = (n >= 1 && std::fabs(ratio - n) < 1e-6) ? n : 0;
    }

    // ------------------------------------------------------------------
    // THE SCREEN GEOMETRY IS THE FILE'S, IN BOTH MODES.
    //
    // In screen mode that is obvious: the video IS the canvas, so reporting
    // the scenario's nominal 2000x2000 would put every screen-frame centroid
    // on the wrong scale, and screen-frame centroids are exactly what §13.2's
    // graded artifact carries.
    //
    // In direct mode it is less obvious and was wrong at first. The video is
    // the camera feed, so there is no separate canvas at all — the screen
    // frame and the image frame are the same frame. Leaving the scenario's
    // 2000x2000 in place mapped a centroid at image (105, 205) to screen
    // (785, 965) and the self-scoring run reported 1020 px of "centroiding
    // error" that was entirely a change of coordinate system.
    //
    // With the file's own geometry, a zero boresight makes screen == image
    // exactly, which is what a mode with no pointing should mean.
    // ------------------------------------------------------------------
    vs->screen_ = ScreenGeometry::make(vs->src_w_, vs->src_h_, vs->cam_);
    if (vs->mode_ == VideoMode::Screen) {
        vs->cropped_.assign(static_cast<size_t>(vs->cam_.pixel_count()), 0);
    }

    vs->inv8_ = check_inv8(sc);
    return Ok(std::move(vs));
}

bool VideoSource::next(Angle2 commanded_boresight, SourceFrame& out) {
    if (!decoder_.pop(frame_)) return false;   // EOF: a clean termination

    out.frame_index         = emitted_;
    out.timestamp_s         = frame_.timestamp_s;
    out.commanded_boresight = commanded_boresight;
    out.has_truth           = false;

    if (mode_ == VideoMode::Direct) {
        // The frame IS the camera feed. No crop, no pointing.
        out.pixels = std::span<const uint8_t>(frame_.grey.data(), frame_.grey.size());
        out.width  = frame_.width;
        out.height = frame_.height;
        // (N-1)/2, matching core/frames.hpp's principal-point convention, so
        // the implied crop origin is exactly zero and image == screen. Using
        // N/2 left a half-pixel offset in every direct-mode truth comparison.
        crop_centre_ = Pixel2{(frame_.width - 1) * 0.5, (frame_.height - 1) * 0.5};
    } else {
        // The frame is the screen; crop a viewport at the commanded boresight.
        //
        // The centre comes from the boresight through the SCREEN geometry, so
        // an angle of zero is the middle of the file and the pan is continuous
        // in sub-pixel steps. There is no true-vs-commanded distinction here:
        // INV-8 disables the disturbances, so the commanded boresight IS where
        // the camera points, and §8.3 requirement 6's "convert back through the
        // crop offset" is exact rather than approximate. That is why video mode
        // is the honest place to grade centroiding.
        const Pixel2 c = screen_.to_pixel(commanded_boresight);
        crop_centre_ = c;
        bicubic_crop(frame_.grey.data(), frame_.width, frame_.height,
                     c.x, c.y, cropped_.data(), cam_.width, cam_.height);
        out.pixels = std::span<const uint8_t>(cropped_.data(), cropped_.size());
        out.width  = cam_.width;
        out.height = cam_.height;
    }

    // --- truth from a --truth CSV, if one was supplied (CP 8.7) -----------
    if (!truth_.empty() && frame_.index < static_cast<int64_t>(truth_.size())) {
        out.truth     = truth_[static_cast<size_t>(frame_.index)];
        out.has_truth = true;
        // The CSV gives screen coordinates; the image position depends on where
        // we cropped, which the CSV cannot know. Filled in here so the metrics
        // get both frames (INV-6).
        for (uint8_t i = 0; i < out.truth.n; ++i) {
            FrameTruth::Target& t = out.truth.targets[i];
            t.image_pos = Pixel2{t.screen_pos.x - (crop_centre_.x - (out.width - 1) * 0.5),
                                 t.screen_pos.y - (crop_centre_.y - (out.height - 1) * 0.5)};
            t.in_fov = cam_.contains(t.image_pos);
        }
        out.truth.boresight_commanded = commanded_boresight;
        out.truth.boresight_true      = commanded_boresight;   // INV-8: no disturbance
    }

    ++emitted_;
    return true;
}

FrameGeometry VideoSource::geometry() const {
    FrameGeometry g;
    if (mode_ == VideoMode::Screen) {
        g.width  = cam_.width;
        g.height = cam_.height;
    } else {
        g.width  = src_w_;
        g.height = src_h_;
    }
    g.fps      = fps_;
    g.screen_w = screen_.width;
    g.screen_h = screen_.height;
    return g;
}

}  // namespace sat
