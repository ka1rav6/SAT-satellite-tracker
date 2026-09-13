// engine/frame_source.hpp — CP 1.1, the abstraction everything downstream sees.
//
// ---------------------------------------------------------------------------
// WHY THIS EXISTS BEFORE ANYTHING USES IT
// ---------------------------------------------------------------------------
// Design decision 1: "IFrameSource from day one. BP-2 (30%) requires external
// MP4 ingest. Retrofitting the abstraction means editing every module." And CP
// 1.1's note: "Defining this now costs an hour; at Stage 8 it costs a rewrite."
//
// There are three ways a frame can arrive (design §8.3):
//
//   synthetic     the simulator renders the world at the true boresight
//   video_screen  the video IS the 2000x2000 screen; the PTZ crops and pans
//   video_direct  the video IS the camera feed; pointing is disabled
//
// Design §6.2 step B4 is emphatic that this is "THE ONLY PLACE THE MODES
// DIFFER". Everything from B5 onwards — perception, tracking, control, metrics —
// is identical code for all three. That property is worth a lot: it means the
// 30%-weighted video benchmark runs the same tracker that the 30%-weighted
// simulation benchmark does, so a result in one is evidence about the other.
//
// The interface is deliberately narrow to keep it that way. A source is asked
// for the next frame given a commanded boresight, and reports its geometry and
// whether it can be pointed at all. Anything richer would leak mode-specific
// behaviour into the common path.

#pragma once

#include "core/frames.hpp"
#include "core/units.hpp"

#include <array>
#include <cstdint>
#include <span>

namespace sat {

/// Spec row 8: "Number of targets — 1 mandatory, multiple optional." The cap is
/// a compile-time constant so FrameTruth is a fixed-size, trivially-copyable
/// aggregate that costs nothing to pass around and never allocates (INV-4).
inline constexpr int kMaxTargets = 8;

// ---------------------------------------------------------------------------
// FrameTruth — what the simulator knows and the tracker must not.
//
// ---------------------------------------------------------------------------
// THIS STRUCT IS THE MOST DANGEROUS TYPE IN THE PROGRAM
// ---------------------------------------------------------------------------
// It contains the answer. INV-1 exists because a tracker that can read this is
// not a tracker, and every number the project reports would be worthless.
//
// The protection is structural, not conventional:
//   * it lives in sat_engine, which the tracker-side modules cannot link
//     (cmake/modules.cmake asserts this at configure time);
//   * sat_metrics may read it, because computing a centroiding error requires
//     the true position by definition;
//   * perception receives only `pixels`, never the enclosing SourceFrame.
//
// If you find yourself wanting truth in perception, tracking, search or
// control, the answer is no. The thing you actually want is a metric.
// ---------------------------------------------------------------------------
struct FrameTruth {
    /// One target's exact state at the instant the frame was captured.
    struct Target {
        uint32_t id         = 0;
        Pixel2   image_pos{};    ///< sub-pixel, camera coordinates
        Pixel2   screen_pos{};   ///< sub-pixel, screen coordinates — the graded reference
        Angle2   world_ang{};    ///< world-fixed pointing angle
        Rate2    world_rate{};   ///< analytic velocity (design §7.2), not a difference
        bool     in_fov     = false;

        /// Whether this entry is the beacon rather than a decoy. Spec row 8
        /// allows multiple targets; only one is the thing being scored.
        bool     is_primary = false;
    };

    int64_t                            tick = 0;
    Angle2                             boresight_true{};   ///< after jitter and platform motion
    Angle2                             boresight_commanded{};
    std::array<Target, kMaxTargets>    targets{};
    uint8_t                            n = 0;

    /// The primary beacon, or nullptr if it is not in this frame's truth.
    /// Metrics call this; nothing else may.
    [[nodiscard]] const Target* primary() const noexcept {
        for (uint8_t i = 0; i < n; ++i) {
            if (targets[i].is_primary) return &targets[i];
        }
        return nullptr;
    }
};

// ---------------------------------------------------------------------------
// SourceFrame — one frame, plus everything the pipeline needs to interpret it.
// ---------------------------------------------------------------------------
struct SourceFrame {
    /// The image the detector sees: 8-bit greyscale, row-major, width*height.
    /// A span, not a container: the storage belongs to the source (a render
    /// target, or a slot in the decode ring), and copying it every frame would
    /// be 300 KB of pointless memcpy inside a 0.85 ms budget.
    std::span<const uint8_t> pixels;

    int     width       = 0;
    int     height      = 0;
    int64_t frame_index = 0;

    /// Seconds since run start. In video modes this comes from the CONTAINER
    /// timestamp, not a counted index (design §8.3 requirement 9) — a VFR file
    /// would otherwise put every metric on a wrong time axis.
    double  timestamp_s = 0.0;

    /// What the controller asked for. Distinct from truth's boresight_true,
    /// which additionally includes jitter and platform motion the controller
    /// cannot know about. Perception is allowed to use this: it is the system's
    /// own commanded state, not privileged information.
    Angle2  commanded_boresight{};

    /// Truth is absent in video mode unless a --truth CSV was supplied, which
    /// is why every metric path has to handle its absence rather than assume it.
    bool       has_truth = false;
    FrameTruth truth{};

    [[nodiscard]] int pixel_count() const noexcept { return width * height; }
};

// ---------------------------------------------------------------------------
// FrameGeometry — what a source reports about itself at open time.
// ---------------------------------------------------------------------------
struct FrameGeometry {
    int    width     = 0;    ///< frames delivered by this source
    int    height    = 0;
    double fps       = 0.0;  ///< probed from the container in video modes
    int    screen_w  = 0;    ///< the canvas the coordinates are reported in
    int    screen_h  = 0;
};

// ---------------------------------------------------------------------------
// IFrameSource — the interface itself.
//
// Virtual dispatch is fine here and nowhere else in the hot path: this is one
// indirect call per frame, roughly 30 per second, against a 0.85 ms budget.
// The alternative (a template parameter threaded through the whole engine)
// would triple compile times and make the three modes structurally different,
// which is precisely what §6.2 B4 says they must not be.
// ---------------------------------------------------------------------------
class IFrameSource {
public:
    virtual ~IFrameSource() = default;

    /// Produce the next frame for the given commanded boresight.
    ///
    /// Returns false when the run is over: duration reached, or video EOF.
    /// EOF is a CLEAN termination (design §8.3 requirement 7) — the caller
    /// finalises the report and exits 0, it does not treat it as an error.
    ///
    /// `out.pixels` stays valid until the next call to next().
    [[nodiscard]] virtual bool next(Angle2 commanded_boresight, SourceFrame& out) = 0;

    [[nodiscard]] virtual FrameGeometry geometry() const = 0;

    /// False for video_direct, where the frame is the whole camera feed and
    /// there is nothing to point. The controller still runs and still reports
    /// where it *would* aim, but the frame does not follow it — which is why
    /// INV-2's closed-loop requirement carves out an explicit exception for
    /// this mode.
    [[nodiscard]] virtual bool supports_pointing() const = 0;

    /// Short name for logs, the centroid.csv header and run.json.
    [[nodiscard]] virtual const char* name() const = 0;

    /// Called once at shutdown, before arenas are released. Video sources drain
    /// and join their decode thread here; the synthetic source does nothing.
    virtual void shutdown() {}
};

}  // namespace sat
