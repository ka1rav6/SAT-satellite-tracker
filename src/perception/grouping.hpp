// perception/grouping.hpp — run-length + union-find connected components
// (design §9.4.6).
//
// ---------------------------------------------------------------------------
// WHY RUN-LENGTH RATHER THAN PER-PIXEL LABELLING
// ---------------------------------------------------------------------------
// The CFAR mask is SPARSE — that is the whole point of a Pfa of 4.8e-5. A
// classic two-pass per-pixel labeller visits all 307,200 pixels twice and does
// union-find work at each; a run-length pass visits each pixel once to find the
// runs, then does union-find work per RUN. On a sparse mask there are a few
// hundred runs, not a few hundred thousand union operations.
//
// §15 budgets 0.05 ms for this stage, and §9.4.6 notes it is "far better than
// full-image labelling".
//
// ---------------------------------------------------------------------------
// DETERMINISM IS A CORRECTNESS REQUIREMENT HERE, NOT A PREFERENCE
// ---------------------------------------------------------------------------
// §9.4.6: "label numbering depends on merge order, deterministic only with a
// fixed scan order. Do not parallelise the row scan."
//
// Two blobs that merge produce a component whose label depends on which run was
// seen first. Under INV-3 the whole run must be reproducible, and the frame
// fingerprint (core/hash.hpp) is deliberately order-sensitive so that a
// reordering shows up as a divergence. A parallel row scan would make the
// labels depend on thread scheduling and break reproducibility without changing
// a single detected position — the worst kind of failure, because the pictures
// would look identical.
//
// So: strict raster order, single-threaded, no exceptions.

#pragma once

#include "core/units.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace sat {

/// One horizontal run of set pixels: row `y`, columns [x0, x1).
///
/// Deliberately has no default member initialisers, which keeps it trivially
/// default-constructible and therefore allocatable from the frame arena
/// (core/arena.hpp hands out raw storage and refuses types that need a
/// constructor). Every field is written before it is read — pass 1 fills the
/// run, pass 3 fills the label — so there is nothing for an initialiser to do
/// except cost a memset of the whole worst-case run array every frame.
struct Run {
    int16_t y;
    int16_t x0;
    int16_t x1;
    int32_t label;
};

// ---------------------------------------------------------------------------
// BlobAccum — a component's accumulated moments.
//
// Design §9.4.6's shape, with the second moments kept because §10.1's
// centroiding and §9.4.7's shape gate both want them and a second pass over the
// pixels to get them would cost more than carrying them.
//
// The weighted sums are double, not float: they accumulate over up to 800
// pixels of values up to 32767, and a float would lose the low bits of the
// first moment — which IS the sub-pixel centroid, the metric worth 60% of the
// marks.
// ---------------------------------------------------------------------------
struct BlobAccum {
    int64_t n = 0;              ///< pixel count (area)

    double  sw   = 0.0;         ///< sum of weights
    double  swx  = 0.0;         ///< sum of w*x
    double  swy  = 0.0;
    double  swxx = 0.0;         ///< sum of w*x^2, for the second moment
    double  swyy = 0.0;

    int16_t x0 = 0, y0 = 0;     ///< bounding box, inclusive
    int16_t x1 = 0, y1 = 0;
    float   peak = 0.0f;        ///< brightest weight in the blob

    [[nodiscard]] int width()  const noexcept { return x1 - x0 + 1; }
    [[nodiscard]] int height() const noexcept { return y1 - y0 + 1; }

    /// Intensity-weighted centre of mass — CP 5.8's estimator.
    ///
    /// Falls back to the bounding-box centre when every weight is zero, which
    /// can happen if the mask fired on pixels the top-hat then flattened. That
    /// is a position with no evidence behind it, and the caller is expected to
    /// treat it accordingly; returning a NaN instead would propagate into the
    /// filter.
    [[nodiscard]] Pixel2 centroid() const noexcept {
        if (sw > 0.0) return Pixel2{swx / sw, swy / sw};
        return Pixel2{0.5 * (x0 + x1), 0.5 * (y0 + y1)};
    }

    /// Fraction of the bounding box that is actually set. Design §9.4.7 uses
    /// this to reject diagonal chains of noise, which have a large box and few
    /// pixels.
    [[nodiscard]] float fill_ratio() const noexcept {
        const int box = width() * height();
        return box > 0 ? static_cast<float>(n) / static_cast<float>(box) : 0.0f;
    }

    /// Longer side over shorter side. §9.4.7 uses it to reject streaks.
    [[nodiscard]] float aspect() const noexcept {
        const int w = width(), h = height();
        const int lo = w < h ? w : h, hi = w < h ? h : w;
        return lo > 0 ? static_cast<float>(hi) / static_cast<float>(lo) : 0.0f;
    }

    /// Second central moment, in pixels. A rough size estimate independent of
    /// the matched filter's, useful as a cross-check.
    [[nodiscard]] double rms_radius() const noexcept;
};

/// Scratch the grouping pass needs. Caller-owned so the stage allocates nothing
/// (INV-4); in a real run these come from the frame arena.
struct GroupingWorkspace {
    std::span<Run>     runs;      ///< at most (width/2 + 1) * height in the worst case
    std::span<int32_t> parent;    ///< union-find, one per run
    std::span<int32_t> rank;      ///< union by rank
};

/// Extract connected components from a binary mask, accumulating moments
/// weighted by `weight` (normally the top-hat image).
///
/// 8-connectivity: runs on adjacent rows merge if their column ranges touch or
/// overlap diagonally. A beacon is a solid blob, so 4- versus 8-connectivity
/// rarely matters for it — but it matters a lot for whether a diagonal chain of
/// impulse survivors becomes one component (which the shape gate then rejects
/// on fill ratio) or many small ones that each pass the area floor.
///
/// Returns the number of components written to `out`.
[[nodiscard]] size_t group_components(std::span<const uint8_t> mask,
                                      std::span<const int16_t> weight,
                                      int width, int height,
                                      const GroupingWorkspace& ws,
                                      std::vector<BlobAccum>& out);

}  // namespace sat
