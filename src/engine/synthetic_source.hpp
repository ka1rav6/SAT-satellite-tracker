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
#include "core/profile.hpp"
#include "core/rng.hpp"
#include "core/time.hpp"
#include "degrade/disturbance.hpp"
#include "degrade/sensor.hpp"
#include "engine/frame_source.hpp"
#include "scenario/scenario.hpp"
#include "world/emitters.hpp"
#include "world/world_builder.hpp"

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

    /// Build from a parsed scenario: world, damage chain and disturbances all
    /// configured from one file. This is design §6.1's startup sequence, and
    /// it is what the real entry points use.
    void build_from_scenario(const Scenario& sc);

    // --- IFrameSource ------------------------------------------------------
    [[nodiscard]] bool next(Angle2 commanded_boresight, SourceFrame& out) override;
    [[nodiscard]] FrameGeometry geometry() const override;
    [[nodiscard]] bool supports_pointing() const override { return true; }
    [[nodiscard]] const char* name() const override { return "synthetic"; }

    // --- simulation-side access (engine and metrics only) ------------------

    /// Override the boresight disturbance directly. Used by tests that want a
    /// known, fixed perturbation; normal runs let the DisturbanceGenerator
    /// produce it.
    void set_boresight_disturbance(Angle2 d) noexcept {
        disturbance_ = d;
        manual_disturbance_ = true;
    }

    /// The mount's physical slew rate, used to smear the exposure (design
    /// §9.2). Set by the engine from the gimbal's rate plus the platform's
    /// analytic rate; it deliberately excludes jitter (see render_frame).
    void set_blur_rate(Rate2 r) noexcept { blur_rate_ = r; }

    /// Keep producing frames after `duration_s`. The dashboard uses this so a
    /// demo runs until Pause. Headless and screenshot runs leave it off, so
    /// they still stop on the scenario clock. `next()` already treats
    /// `max_frames_ == 0` as "no cap".
    void set_continuous(bool on) noexcept {
        max_frames_ = on ? 0
                         : static_cast<int64_t>(cfg_.duration_s * cfg_.camera_hz + 0.5);
    }
    /// The rate the last frame's exposure smear was integrated along.
    ///
    /// Read-back for tests and for the GUI's timing panel. A-3 was a defect in
    /// the ARGUMENT to set_blur_rate — the platform rate was sampled at a
    /// hardcoded 30 Hz regardless of camera_hz — and a write-only setter gave
    /// a test no way to see it. A value that cannot be observed cannot be
    /// asserted, and this one was wrong for every camera_hz except 30.
    [[nodiscard]] Rate2 blur_rate() const noexcept { return blur_rate_; }

    /// Attach the engine's stage timers so the render can be broken down the
    /// way design §15's budget table is written — background, splat and damage
    /// chain as three separate lines rather than one `frame_acquire` lump.
    /// Optional: nothing here needs a timer to work, and tests leave it null.
    void set_timers(StageTimers* t) noexcept { timers_ = t; }

    [[nodiscard]] DisturbanceGenerator& disturbance() noexcept { return disturb_; }
    [[nodiscard]] SensorChain&          sensor()      noexcept { return sensor_; }
    [[nodiscard]] const World&          world()  const noexcept { return world_; }

    /// Advance the world by one truth tick. Called `camera_divisor` times per
    /// frame by the engine, so the world moves at 300 Hz while frames come at 30.
    void advance_world(double dt) noexcept;

    /// The emitters. They live inside `world_` so that World::advance writes
    /// into the same arrays the renderer reads — there is exactly one copy.
    [[nodiscard]] EmitterSoA&       emitters()       noexcept { return world_.emitters; }
    [[nodiscard]] const EmitterSoA& emitters() const noexcept { return world_.emitters; }
    [[nodiscard]] const Clock&      clock()    const noexcept { return clock_; }
    [[nodiscard]] RngSet&           rng()            noexcept { return rng_; }

    /// The float render, before the damage chain and before quantisation.
    /// Exposed for tests and for the centroid-accuracy harness of Stage 9,
    /// which needs the undegraded image.
    [[nodiscard]] std::span<const float> radiance() const noexcept { return radiance_; }

private:
    void render_frame(Angle2 true_boresight, double t_s);

    /// Rebuild graded_slot_ if it no longer matches the emitter set. Cheap
    /// (one size compare per frame) and called from render_frame, because
    /// emitters can arrive through build_from_scenario OR be installed by
    /// hand after build() — the benchmarks and several tests do the latter,
    /// and a slot map sized only in one of those paths is an out-of-bounds
    /// read in the other.
    void ensure_scintillation_slots();
    void fill_truth(Angle2 true_bore, Angle2 commanded_bore, FrameTruth& t) const;

    StageTimers*    timers_ = nullptr;   ///< not owned; see set_timers()
    SyntheticConfig cfg_{};
    World           world_{};
    SensorChain     sensor_{};
    DisturbanceGenerator disturb_{};
    Clock           clock_{};
    RngSet          rng_{};
    bool            have_world_ = false;
    bool            manual_disturbance_ = false;

    std::vector<float>    radiance_;   ///< float accumulation target
    std::vector<uint8_t>  frame_;      ///< the 8-bit image the detector sees
    std::vector<uint32_t> visible_;    ///< scratch for query_visible

    /// Audit P2-1. Emitter index -> independent scintillation slot, sized once
    /// at build. Graded emitters (beacon, decoys) get 0..kMaxGraded-1 in
    /// emitter order; clutter and any graded emitter past the cap get a slot
    /// TurbulenceModel::irradiance_gain() answers 1.0 for, so the render loop
    /// needs no second branch and no per-frame lookup.
    std::vector<uint8_t>  graded_slot_;

    Angle2  disturbance_{};            ///< jitter + platform, applied to truth
    Rate2   blur_rate_{};              ///< physical slew, for exposure smear
    double  sim_time_s_ = 0.0;         ///< advanced by advance_world()
    Angle2  prev_true_bore_{};         ///< for blur: where the boresight was
    bool    have_prev_bore_ = false;
    int64_t frame_index_    = 0;
    int64_t max_frames_     = 0;
};

}  // namespace sat
