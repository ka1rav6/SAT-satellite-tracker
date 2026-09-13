// engine/synthetic_source.hpp — the simulator as an IFrameSource.
//
// Design §6.2 step B4: in synthetic mode, "View AABB from true boresight ->
// query_visible -> render background into viewport -> splat each emitter with
// exact coverage across blur_substeps, interpolating boresight start->end ->
// float radiance -> damage chain -> uint8. Emit FrameTruth."
//
// At Stage 1 the background and damage chain are absent (they arrive in Stage 4);
// what is here is the geometry, the truth emission and the closed loop, which is
// what CP 1.8 gates on.
//
// ---------------------------------------------------------------------------
// THE DISTINCTION THAT MAKES THE WHOLE SIMULATION HONEST
// ---------------------------------------------------------------------------
// There are two boresights and they are not the same:
//
//   commanded  what the controller asked for, and what it therefore knows
//   true       where the camera actually points, after jitter and platform
//              motion have perturbed it
//
// The frame is rendered at the TRUE boresight. The tracker is told only the
// COMMANDED one. The difference is the pointing error the system cannot see
// directly and must infer from the image — which is the entire problem.
//
// Design §9.3 is explicit that disturbances "perturb the TRUE BORESIGHT, never
// the pixels", and gives the reason: it is physically correct (motion blur then
// follows automatically) and "it prevents the tracker from implicitly knowing
// its own pointing error".

#pragma once

#include "camera/splat.hpp"
#include "core/arena.hpp"
#include "core/frames.hpp"
#include "core/rng.hpp"
#include "core/time.hpp"
#include "engine/frame_source.hpp"
#include "world/emitters.hpp"

#include <memory>
#include <vector>

namespace sat {

/// Everything SyntheticSource needs that will eventually come from the scenario
/// file. Stage 3 replaces this with the parsed Scenario struct; keeping it
/// separate now means the TOML loader can be written against a settled shape.
struct SyntheticConfig {
    CameraGeometry camera = CameraGeometry::make(640, 480, 4.0, 3.0);
    ScreenGeometry screen{};                 ///< filled from `camera` in build()

    int    truth_hz      = 300;              ///< design §6.2: 10 sub-ticks per frame
    int    camera_hz     = 30;               ///< spec row 5
    int    control_hz    = 30;               ///< spec row 15
    double duration_s    = 10.0;
    uint64_t seed        = 42;

    int    blur_substeps = 8;                ///< design §9.2; 1 disables blur
    double exposure_s    = 0.005;            ///< 5 ms default

    /// Background pedestal, in grey levels. A real sensor never sees zero, and
    /// a zero background would make the straw-man detector look far better than
    /// it is.
    float  background    = 8.0f;
};

// ---------------------------------------------------------------------------
// SyntheticSource
// ---------------------------------------------------------------------------
class SyntheticSource final : public IFrameSource {
public:
    SyntheticSource() = default;

    /// Build the source and its world. This is the one place that allocates
    /// (design §6.1 step A4/A5); after this, frames are produced without
    /// touching the heap.
    void build(const SyntheticConfig& cfg, EmitterSoA emitters);

    // --- IFrameSource ------------------------------------------------------
    [[nodiscard]] bool next(Angle2 commanded_boresight, SourceFrame& out) override;
    [[nodiscard]] FrameGeometry geometry() const override;
    [[nodiscard]] bool supports_pointing() const override { return true; }
    [[nodiscard]] const char* name() const override { return "synthetic"; }

    // --- simulation-side access (engine and metrics only) ------------------

    /// The disturbance to apply to the true boresight this frame. Set by the
    /// engine before calling next(); it is an input to rendering, not something
    /// the source invents, because jitter and platform motion are modelled by
    /// sat_degrade and the source must not duplicate them.
    void set_boresight_disturbance(Angle2 d) noexcept { disturbance_ = d; }

    /// Advance the world by one truth tick. Called `camera_divisor` times per
    /// frame by the engine, so the world moves at 300 Hz while frames come at 30.
    void advance_world(double dt) noexcept;

    [[nodiscard]] EmitterSoA&       emitters()       noexcept { return emitters_; }
    [[nodiscard]] const EmitterSoA& emitters() const noexcept { return emitters_; }
    [[nodiscard]] const Clock&      clock()    const noexcept { return clock_; }
    [[nodiscard]] RngSet&           rng()            noexcept { return rng_; }

    /// The float render, before quantisation. Exposed for tests and for the
    /// centroid-accuracy harness of Stage 9, which needs the undegraded image.
    [[nodiscard]] std::span<const float> radiance() const noexcept { return radiance_; }

private:
    void render_frame(Angle2 true_boresight, double t_s);
    void fill_truth(Angle2 true_bore, Angle2 commanded_bore, FrameTruth& t) const;

    SyntheticConfig cfg_{};
    EmitterSoA      emitters_{};
    Clock           clock_{};
    RngSet          rng_{};

    std::vector<float>    radiance_;   ///< float accumulation target
    std::vector<uint8_t>  frame_;      ///< the 8-bit image the detector sees
    std::vector<uint32_t> visible_;    ///< scratch for query_visible

    Angle2  disturbance_{};            ///< jitter + platform, applied to truth
    Angle2  prev_true_bore_{};         ///< for blur: where the boresight was
    bool    have_prev_bore_ = false;
    int64_t frame_index_    = 0;
    int64_t max_frames_     = 0;
};

}  // namespace sat
