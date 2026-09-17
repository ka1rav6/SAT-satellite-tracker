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

#include <algorithm>
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
    // THE SNR GATE — the one the shape gate cannot do
    //
    // CFAR's k is a statement about ONE PIXEL: at the specification's operating
    // point k = 3.9, and taking the union of the two passes, the per-pixel
    // false-alarm probability is 2 Q(3.9) = 9.6e-5, so about 29 pixels of a
    // 640 x 480 frame fire on pure noise every frame. That is by design and it
    // is what §9.4.5 promises.
    //
    // What was missing is the step from "this pixel is unusual" to "this is an
    // object". Grouping those 29 pixels does not scatter them harmlessly:
    // CFAR runs on the MATCHED-FILTER RESPONSE, which has been box-summed over
    // the target scale, so a single noise excursion is smeared into a
    // contiguous region of roughly the filter's own footprint. Measured on a
    // frame with no target, no clutter and nothing but read noise at sigma 4,
    // the detector returned SIX candidates of area 12 to 82 px, fill 0.52 to
    // 1.00 and aspect 1.00 to 1.40. Every one of them passes the shape gate,
    // because every one of them is exactly the shape of a beacon.
    //
    // Their SNRs were 4.16, 4.52, 4.56, 4.58, 4.59 and 4.70 — all just over k,
    // which is what a threshold crossing looks like. The beacon on the same
    // run measured 248.92.
    //
    // So the gate is on SNR, expressed as a MULTIPLE of k rather than as an
    // absolute number, for two reasons: it then means the same thing whatever
    // operating point a scenario chooses, and §10.6's supervisor moves k in
    // poor conditions, which must move this with it or the adaptation would
    // quietly stop working in fog.
    //
    // The derivation of the factor, which is what makes 1.5 a number rather
    // than a taste: pixels above 1.5k = 5.85 occur with probability
    // 2 Q(5.85) = 5e-9, so 0.0015 per frame — one every 650 frames instead of
    // 29 every frame, a factor of 19,000. On the other side, the beacon clears
    // it by 40x in clear air and, at the SNR of 12.09 measured after §7.4's fog
    // event, still by 2x.
    //
    // What this costs, stated plainly: a target whose SNR is under 5.85 is
    // rejected outright. That is already the regime where it is under the
    // detector's floor — see the low-light row in issues_till_now.md §2, where
    // target loss reaches 86% for a reason this gate does not change.
    // -----------------------------------------------------------------------
    float min_snr_factor = 1.5f;   ///< multiplier on cfar.k; 0 disables
    float min_snr_abs    = 0.0f;   ///< absolute floor, whichever is larger

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
    /// The cropped window handed to the kernels when a ROI is in force.
    ///
    /// A copy rather than a stride-aware version of every kernel: the crop is
    /// one 25 KB memcpy for a 160x160 window, against threading an origin and a
    /// row pitch through the median, the opening, both summed-area builders,
    /// the matched filter, CFAR and the grouping pass — six kernels, every one
    /// of which has an edge-clamping argument that would then have two
    /// meanings. The copy also leaves each kernel reading a CONTIGUOUS buffer
    /// that fits in L2, which is most of why the window is fast.
    std::span<uint8_t>  crop;        ///< w*h, worst case a full-frame ROI
    MorphWorkspace      morph;
    GroupingWorkspace   grouping;

    /// Allocate every buffer from an arena. Call once, at startup.
    [[nodiscard]] bool allocate(Arena& arena, int width, int height, int se_size);
};

// ---------------------------------------------------------------------------
// DetectRoi — the rectangle of the frame the detector is asked to look at.
//
// ---------------------------------------------------------------------------
// WHY A TRACKER SHOULD NOT SEARCH A FRAME IT HAS ALREADY FOUND THE TARGET IN
// ---------------------------------------------------------------------------
// Every stage from the median filter to CFAR is O(pixels), and at the
// specification's 640 x 480 (row 3) that is 307,200 of them, ten passes deep,
// thirty times a second. Measured, the detector is 11.5 ms of a 16 ms frame
// against design §15's 1.39 ms for the same list of stages.
//
// But on a frame where the track is Confirmed, the tracker already knows where
// the beacon is — to within its own filter's position sigma, which in clear air
// is a couple of pixels. Searching the other 99% of the frame for it is not
// robustness, it is arithmetic performed on the answer to a question nobody
// asked.
//
// So when the track is confirmed the detector is handed a WINDOW: the predicted
// image position, plus a margin sized from the filter's own covariance. When it
// is not — searching, acquiring, coasting past its confidence — it is handed
// the whole frame, because then the question really is "where is it".
//
// Design §14's CP 6.7 already names this idea ("predicted-region
// reacquisition"); this applies it to the steady state as well. See the
// amendment block in docs/SAT-DESIGN.md §14.0b for the full argument and for
// what it costs.
//
// Three properties worth stating because they are what makes it safe:
//
//   INV-1 holds. The window is a rectangle of pixel coordinates derived from
//   the tracker's own estimate. Perception still never sees truth, and nothing
//   here can reach the world model.
//
//   INV-3 holds. The rectangle is a deterministic function of filter state, so
//   two identical runs produce identical windows and identical frames.
//
//   It CANNOT hide a target the tracker had. The window is centred on the
//   prediction and grows with the covariance, so a track that starts to drift
//   widens its own window; one that fails M-of-N drops out of Confirmed and
//   gets the whole frame back on the next frame.
//
// It does change one thing on purpose: clutter outside the window is no longer
// detected, so it can no longer be associated. That is the point — see
// issues_till_now.md §2 for the 205 px clutter measurement this is aimed at.
// ---------------------------------------------------------------------------
struct DetectRoi {
    int x0 = 0, y0 = 0;      ///< inclusive top-left, in full-frame pixels
    int width = 0, height = 0;

    /// The whole frame — what a non-confirmed track gets.
    [[nodiscard]] static constexpr DetectRoi full(int w, int h) noexcept {
        return DetectRoi{0, 0, w, h};
    }
    [[nodiscard]] constexpr bool is_full(int w, int h) const noexcept {
        return x0 == 0 && y0 == 0 && width == w && height == h;
    }
    [[nodiscard]] constexpr bool valid() const noexcept {
        return width > 0 && height > 0;
    }
};

// ---------------------------------------------------------------------------
// RoiParams — how big the window is, exposed so it can be turned off.
// ---------------------------------------------------------------------------
struct RoiParams {
    /// Off means every frame is a full-frame search. Kept switchable because
    /// the windowed and full-frame arms have to be COMPARABLE: the ablation in
    /// docs/RESULTS.md is the evidence that windowing costs no accuracy, and an
    /// ablation needs both arms to be runnable from the same binary.
    bool  enabled = true;

    /// Half-width floor, in pixels. Never smaller than this however confident
    /// the filter is, because the filter's sigma describes where the TARGET is,
    /// not how much context CFAR needs around it: the training annulus is
    /// `train` pixels across (§9.4.5, 61 by default) and a window narrower than
    /// that would estimate the background from almost nothing.
    int   min_half_px = 96;

    /// How many position sigmas of margin beyond the floor.
    double sigma_margin = 6.0;

    /// A full-frame sweep every N frames even while confirmed; 0 disables it.
    ///
    /// The failure this guards against is real but rare: if the tracker has
    /// locked onto a decoy, the window follows the DECOY and the true beacon is
    /// never looked at again. A periodic full sweep gives the association logic
    /// a chance to see both. It is off by default because it puts a full-frame
    /// frame into the p99 and the same failure exists without windowing — a
    /// tracker locked onto a decoy stays locked onto it either way.
    int   refresh_frames = 0;
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
    /// How many components one frame is expected to produce at most. See
    /// configure() for where the number comes from; the engine reserves its
    /// detection vector to the same bound, because in principle every blob can
    /// survive the shape gate.
    static constexpr size_t kBlobReserve = 4096;

    void configure(const PerceptionParams& p) {
        params_ = p;
        // -------------------------------------------------------------------
        // INV-4 (CP 14.3). blobs_ is a MEMBER so its capacity survives between
        // frames, and clear() keeps that capacity — but it still has to reach
        // its high-water mark somehow, and it was doing so by allocating
        // inside the frame loop. The Debug allocation trap caught it on the
        // first frame of the first run it was ever armed for.
        //
        // 4096 is chosen from what the pipeline actually produces rather than
        // from the theoretical worst case. §9.4's own measurement is 15,929
        // bright pixels grouping into 84 blobs on CP 5.9's worst case; the
        // absolute bound is a checkerboard mask, w*h/2 = 153,600 components,
        // which would be 11 MB reserved against a case the median filter
        // (§9.4.1) exists to make impossible.
        //
        // A frame that exceeds 4096 still works — the vector grows, as a
        // vector does — and in a Debug build the trap says so. That is the
        // right failure mode: a pathological frame is reported rather than
        // silently truncated, and truncating would change what the detector
        // found in order to satisfy an invariant about allocation.
        // -------------------------------------------------------------------
        blobs_.reserve(kBlobReserve);
    }
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
                 StageTimers* timers = nullptr) {
        process(pixels, width, height, DetectRoi::full(width, height), ws, out, timers);
    }

    /// The same, restricted to a window. Detections come back in FULL-FRAME
    /// coordinates whatever the window was, so no caller downstream has to know
    /// this happened.
    void process(std::span<const uint8_t> pixels, int width, int height,
                 DetectRoi roi,
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

/// Design §9.4.7's SHAPE gate, exposed so it can be tested and reasoned about
/// alone. It runs before the centroid, on the blob's moments.
[[nodiscard]] bool passes_gate(const BlobAccum& b, const PerceptionParams& p) noexcept;

/// The SNR gate, which necessarily runs AFTER the centroid: the statistic it
/// tests is CFAR evaluated at the candidate's own centroid, which does not
/// exist until the centroid does. See PerceptionParams::min_snr_factor.
[[nodiscard]] inline float min_candidate_snr(const PerceptionParams& p) noexcept {
    return std::max(p.min_snr_abs, p.min_snr_factor * p.cfar.k);
}

/// Design §10.1.4's uncertainty estimate: sigma ~ size / (2 * SNR), floored.
[[nodiscard]] float centroid_sigma(float snr, int size_est_px) noexcept;

}  // namespace sat
