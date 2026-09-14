// perception/pipeline.hpp — the detection pipeline (design §9.4).
//
// Assembles steps B6 through B13 of §6.2 into one object:
//
//     B6   median 3x3                  remove impulse noise
//     B7   top-hat                     remove background
//     B8   summed-area tables          make everything below O(1)
//     B9   multi-scale matched filter  optimal detection of a known shape
//     B10  CFAR threshold              adapt to whatever the background is
//     B11  run-length grouping         turn a mask into blobs with moments
//     B12  shape gate                  ~5-20 candidates, not thousands
//     B13  centroid per candidate
//
// ---------------------------------------------------------------------------
// THE ONLY INPUT IS PIXELS
// ---------------------------------------------------------------------------
// process() takes an image, not a SourceFrame. That is INV-1 enforced at the
// signature: the truth sitting inside a SourceFrame is unreachable from here,
// on top of the link-level boundary cmake/modules.cmake asserts. If this ever
// needs to take a SourceFrame, the answer is no.

#pragma once

#include "core/arena.hpp"
#include "core/profile.hpp"
#include "core/units.hpp"
#include "perception/centroid/bias.hpp"
#include "perception/centroid/estimators.hpp"
#include "perception/cfar.hpp"
#include "perception/grouping.hpp"
#include "perception/morphology.hpp"
#include "perception/sat.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace sat {

// ---------------------------------------------------------------------------
// Detection — one candidate, as design §9.4 defines it.
// ---------------------------------------------------------------------------
struct Detection {
    Pixel2 centroid_image{};       ///< sub-pixel, camera coordinates
    Pixel2 centroid_screen{};      ///< filled in by the caller, which knows the boresight

    float  peak       = 0.0f;
    float  integrated = 0.0f;
    float  snr        = 0.0f;

    uint16_t area_px  = 0;
    uint16_t bbox_w   = 0;
    uint16_t bbox_h   = 0;
    uint16_t size_est_px = 0;      ///< from the winning matched-filter scale

    float  fill_ratio = 0.0f;
    float  aspect     = 0.0f;
    float  ml_score   = 0.0f;      ///< Stage 11; 0 until then
    float  centroid_sigma_est = 0.0f;
};

// ---------------------------------------------------------------------------
// PerceptionParams — everything tunable, in one place.
//
// The SAT supervisor (§10.6) switches several of these at runtime, so they are
// data rather than constants.
// ---------------------------------------------------------------------------
struct PerceptionParams {
    int  target_size_px = 10;      ///< spec row 10; sizes the structuring element
    bool median_enabled = true;
    CfarParams cfar{};

    // --- the shape gate, design §9.4.7 -----------------------------------
    //
    // The bounds come from spec row 10's 5-20 px range with margin for the
    // blur and the threshold: a 5 px beacon is 25 px of area before any
    // spreading, a 20 px one is 400, and §9.4.7's stated 12..800 brackets both
    // generously enough that a partially-occluded or slightly smeared target is
    // not thrown away.
    int   min_area   = 12;
    int   max_area   = 800;
    float min_fill   = 0.35f;      ///< rejects diagonal chains of noise
    float max_aspect = 3.0f;       ///< rejects streaks

    // -----------------------------------------------------------------------
    // Stage 9 — which estimator, and whether to correct its S-curve.
    //
    // §10.1.2's default: "background-subtracted WindowedCoM with bias
    // correction". The supervisor switches at Stage 12; until then these are
    // the knobs an ablation table varies, which is why they are data rather
    // than constants.
    // -----------------------------------------------------------------------
    CentroidKind centroid_kind = CentroidKind::WindowedCoM;
    bool correct_centroid_bias = true;

    /// Cap on candidates returned, highest SNR first. §9.4 expects 5-20 after
    /// gating; the cap stops a pathological frame from handing the tracker
    /// thousands and blowing the frame budget.
    int max_candidates = 24;
};

/// Scratch for one frame. Sized once at startup; in a run these point into the
/// frame arena, so processing a frame allocates nothing (INV-4).
struct PerceptionWorkspace {
    std::span<uint8_t>  filtered;    ///< median output,    w*h
    std::span<int16_t>  tophat;      ///< background-removed, w*h
    std::span<int64_t>  sat_sum;     ///< (w+1)*(h+1)
    std::span<uint64_t> sat_sumsq;   ///< (w+1)*(h+1)
    std::span<float>    response;    ///< matched-filter response, w*h
    std::span<int16_t>  response_q;  ///< response, quantised for its own SAT
    std::span<int64_t>  resp_sum;    ///< (w+1)*(h+1), SAT over the response
    std::span<uint64_t> resp_sumsq;  ///< (w+1)*(h+1)
    std::span<uint8_t>  scale;       ///< winning scale,     w*h
    std::span<uint8_t>  mask;        ///< CFAR mask,         w*h
    /// The second CFAR pass's mask, OR'd into `mask`. A separate buffer
    /// because cfar_mask writes EVERY pixel — pointing it at `mask` would
    /// erase the first pass rather than add to it.
    std::span<uint8_t>  mask2;       ///< w*h
    MorphWorkspace      morph;
    GroupingWorkspace   grouping;

    /// Allocate every buffer from an arena. Call once, at startup.
    [[nodiscard]] bool allocate(Arena& arena, int width, int height, int se_size);
};

// ---------------------------------------------------------------------------
// ClassicalPerception — the no-ML path (§9.4).
//
// INV-7: "The system must run fully without AI. Every model has a classical
// fallback." This IS that fallback, and it is also the baseline every ML
// ablation is measured against, so it has to be good rather than a placeholder.
// ---------------------------------------------------------------------------
class ClassicalPerception {
public:
    void configure(const PerceptionParams& p) { params_ = p; }
    [[nodiscard]] const PerceptionParams& params() const noexcept { return params_; }
    [[nodiscard]] PerceptionParams& params() noexcept { return params_; }

    /// Run the pipeline. `out` is cleared and filled with gated candidates,
    /// strongest first.
    ///
    /// `timers` is optional. CP 14.4 requires per-stage p50/p95/p99 "from the
    /// SHIPPED binary", not from a special profiling build, so the
    /// instrumentation is always compiled in — it is one rdtsc pair per stage
    /// against a stage that costs milliseconds. Passing nullptr skips it, which
    /// the unit tests do so that a microbenchmark is not timing the timer.
    void process(std::span<const uint8_t> pixels, int width, int height,
                 const PerceptionWorkspace& ws, std::vector<Detection>& out,
                 StageTimers* timers = nullptr);

    /// Blobs before gating, for diagnostics and for CP 5.7's "candidates drop
    /// from thousands to under 25" measurement.
    [[nodiscard]] size_t last_blob_count() const noexcept { return last_blobs_; }

    [[nodiscard]] const char* name() const noexcept { return "classical"; }

private:
    PerceptionParams       params_{};
    std::vector<BlobAccum> blobs_;
    size_t                 last_blobs_ = 0;
};

/// Design §9.4.7's gate, exposed so it can be tested and reasoned about alone.
[[nodiscard]] bool passes_gate(const BlobAccum& b, const PerceptionParams& p) noexcept;

/// Design §10.1.4's uncertainty estimate: sigma ~ size / (2 * SNR), floored.
[[nodiscard]] float centroid_sigma(float snr, int size_est_px) noexcept;

}  // namespace sat
