#include "perception/morphology.hpp"

#include <cassert>

#include <algorithm>

namespace sat {
namespace {

/// The van Herk / Gil-Werman core, parameterised by the operation so the min and
/// max versions cannot drift apart.
///
/// `Op` is std::min or std::max on uint8_t; `kIdentity` is the value a run
/// starts from (255 for min, 0 for max).
template <typename Op>
void running_1d(const uint8_t* src, uint8_t* dst, int n, int k,
                uint8_t* fwd, uint8_t* bwd, uint8_t identity, Op op) noexcept {
    if (n <= 0) return;
    if (k <= 1) {
        std::copy(src, src + n, dst);
        return;
    }

    const int half = k / 2;

    // The window for output i is [i - half, i + half]. With REPLICATE borders
    // the source is conceptually extended, so the algorithm runs over a padded
    // index space of length n + 2*half and reads through a clamping accessor.
    const int padded = n + 2 * half;
    auto at = [&](int i) -> uint8_t {
        const int j = i - half;
        return src[j < 0 ? 0 : (j >= n ? n - 1 : j)];
    };

    // Forward runs: fwd[i] is the reduction from the start of i's block to i.
    for (int block = 0; block < padded; block += k) {
        const int end = std::min(block + k, padded);
        uint8_t acc = identity;
        for (int i = block; i < end; ++i) {
            acc = op(acc, at(i));
            fwd[i] = acc;
        }
    }
    // Backward runs: bwd[i] is the reduction from i to the end of i's block.
    for (int block = 0; block < padded; block += k) {
        const int end = std::min(block + k, padded);
        uint8_t acc = identity;
        for (int i = end - 1; i >= block; --i) {
            acc = op(acc, at(i));
            bwd[i] = acc;
        }
    }

    // Every length-k window straddles exactly one block boundary, so it is the
    // union of one backward run and one forward run — three comparisons per
    // output pixel regardless of k, which is the whole point.
    for (int i = 0; i < n; ++i) {
        dst[i] = op(bwd[i], fwd[i + k - 1]);
    }
}


// ---------------------------------------------------------------------------
// running_1d_strip — van Herk over S columns at once.
//
// ---------------------------------------------------------------------------
// WHY THE VERTICAL PASS NEEDED ITS OWN FUNCTION
// ---------------------------------------------------------------------------
// The horizontal pass walks a row, which is contiguous. The vertical pass walks
// a column, which is `width` bytes apart — so for a 640x480 frame every single
// access lands on a different cache line, and the line is then evicted long
// before the neighbouring column wants it. Gathering each column into a
// contiguous buffer first (which the first version did) does not fix that: the
// GATHER is the strided part.
//
// It showed as 6.4 ms for the top-hat against §15's 0.3 ms, for an algorithm
// that is provably O(1) per pixel. The arithmetic was never the problem.
//
// The fix is to run S columns in lockstep. One cache line of 64 bytes covers 64
// consecutive uint8 columns, so with S = 64 each line that is fetched serves
// every column it contains instead of one. The inner loop over c is contiguous
// and the compiler vectorises the min/max across it.
//
// The result is bit-identical: each column's reduction is the same sequence of
// the same comparisons, just interleaved with its neighbours'.
// ---------------------------------------------------------------------------
template <typename Op>
void running_1d_strip(const uint8_t* src, uint8_t* dst, int n, int stride, int k,
                      int cols, uint8_t* fwd, uint8_t* bwd,
                      uint8_t identity, Op op) noexcept {
    const int half   = k / 2;
    const int padded = n + 2 * half + k;

    auto row_of = [&](int j) noexcept -> const uint8_t* {
        const int c = j - half;
        return src + static_cast<size_t>(c < 0 ? 0 : (c >= n ? n - 1 : c)) * stride;
    };

    for (int block = 0; block < padded; block += k) {
        const int end = std::min(block + k, padded);
        uint8_t acc[64];
        for (int c = 0; c < cols; ++c) acc[c] = identity;
        for (int i = block; i < end; ++i) {
            const uint8_t* r = row_of(i);
            uint8_t* f = fwd + static_cast<size_t>(i) * cols;
            for (int c = 0; c < cols; ++c) { acc[c] = op(acc[c], r[c]); f[c] = acc[c]; }
        }
    }
    for (int block = 0; block < padded; block += k) {
        const int end = std::min(block + k, padded);
        uint8_t acc[64];
        for (int c = 0; c < cols; ++c) acc[c] = identity;
        for (int i = end - 1; i >= block; --i) {
            const uint8_t* r = row_of(i);
            uint8_t* b = bwd + static_cast<size_t>(i) * cols;
            for (int c = 0; c < cols; ++c) { acc[c] = op(acc[c], r[c]); b[c] = acc[c]; }
        }
    }
    for (int i = 0; i < n; ++i) {
        const uint8_t* b = bwd + static_cast<size_t>(i) * cols;
        const uint8_t* f = fwd + static_cast<size_t>(i + k - 1) * cols;
        uint8_t* o = dst + static_cast<size_t>(i) * stride;
        for (int c = 0; c < cols; ++c) o[c] = op(b[c], f[c]);
    }
}

/// Columns processed together in the vertical pass. 64 uint8 columns is exactly
/// one cache line, so every line fetched is fully used.
inline constexpr int kStripCols = 64;

struct MinOp { uint8_t operator()(uint8_t a, uint8_t b) const noexcept { return a < b ? a : b; } };
struct MaxOp { uint8_t operator()(uint8_t a, uint8_t b) const noexcept { return a > b ? a : b; } };

/// Apply a 1-D filter along rows, then along columns. A rectangular structuring
/// element is separable, which is what makes the 2-D case O(1) per pixel too.
template <typename Op>
void separable_2d(const uint8_t* src, uint8_t* dst, int width, int height, int k,
                  uint8_t* tmp, uint8_t* scratch, uint8_t identity, Op op) noexcept {
    const int longest = std::max(width, height);
    const int padded  = longest + 2 * (k / 2) + k;   // room for the padded index space

    // The halves are kStripCols apart, not `padded` apart: the vertical pass
    // writes padded * cols entries into each. Splitting them at `padded` — as
    // the single-column version did — makes bwd overlap fwd the moment the
    // strip is wider than one column, which silently corrupts the reduction.
    // The morphology tests caught it immediately on small random sizes, which
    // is why they run at every SE size against brute force.
    uint8_t* fwd = scratch;
    uint8_t* bwd = scratch + static_cast<size_t>(padded) * kStripCols;

    // Horizontal pass, row by row.
    for (int y = 0; y < height; ++y) {
        running_1d(src + static_cast<size_t>(y) * width,
                   tmp + static_cast<size_t>(y) * width,
                   width, k, fwd, bwd, identity, op);
    }

    // Vertical pass, kStripCols columns at a time. See running_1d_strip for
    // why the obvious column-at-a-time version is cache-bound rather than
    // compute-bound.
    for (int x0 = 0; x0 < width; x0 += kStripCols) {
        const int cols = std::min(kStripCols, width - x0);
        running_1d_strip<Op>(tmp + x0, dst + x0, height, width, k, cols,
                             fwd, bwd, identity, op);
    }
}

/// Scratch needed by separable_2d, in bytes.
[[nodiscard]] size_t scratch_needed(int width, int height, int k) noexcept {
    const int longest = std::max(width, height);
    const int padded  = longest + 2 * (k / 2) + k;
    // The vertical pass needs fwd and bwd for kStripCols columns at once, which
    // dominates: at 480 rows, k = 51 and 64 columns that is about 74 KB, sized
    // to sit in L2. The horizontal pass needs 2 * padded, which this covers.
    return static_cast<size_t>(2 * padded * kStripCols);
}

}  // namespace

void min_filter_1d(std::span<const uint8_t> src, std::span<uint8_t> dst,
                   int n, int k, std::span<uint8_t> scratch) noexcept {
    const int padded = n + 2 * (k / 2) + k;
    if (static_cast<int>(scratch.size()) < 2 * padded) return;
    running_1d(src.data(), dst.data(), n, k,
               scratch.data(), scratch.data() + padded, 255, MinOp{});
}

void max_filter_1d(std::span<const uint8_t> src, std::span<uint8_t> dst,
                   int n, int k, std::span<uint8_t> scratch) noexcept {
    const int padded = n + 2 * (k / 2) + k;
    if (static_cast<int>(scratch.size()) < 2 * padded) return;
    running_1d(src.data(), dst.data(), n, k,
               scratch.data(), scratch.data() + padded, 0, MaxOp{});
}

void erode_rect(std::span<const uint8_t> src, std::span<uint8_t> dst,
                int width, int height, int k,
                std::span<uint8_t> tmp, std::span<uint8_t> scratch) noexcept {
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (width <= 0 || height <= 0 || src.size() < n || dst.size() < n ||
        tmp.size() < n || scratch.size() < scratch_needed(width, height, k)) return;
    separable_2d(src.data(), dst.data(), width, height, k,
                 tmp.data(), scratch.data(), 255, MinOp{});
}

void dilate_rect(std::span<const uint8_t> src, std::span<uint8_t> dst,
                 int width, int height, int k,
                 std::span<uint8_t> tmp, std::span<uint8_t> scratch) noexcept {
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (width <= 0 || height <= 0 || src.size() < n || dst.size() < n ||
        tmp.size() < n || scratch.size() < scratch_needed(width, height, k)) return;
    separable_2d(src.data(), dst.data(), width, height, k,
                 tmp.data(), scratch.data(), 0, MaxOp{});
}

size_t MorphWorkspace::scratch_bytes(int width, int height, int k) noexcept {
    return scratch_needed(width, height, k);
}

bool MorphWorkspace::sufficient(int width, int height, int k) const noexcept {
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    return a.size() >= n && b.size() >= n && c.size() >= n
        && scratch.size() >= scratch_bytes(width, height, k);
}

void open_rect(std::span<const uint8_t> src, std::span<uint8_t> dst,
               int width, int height, int k, const MorphWorkspace& ws) noexcept {
    if (!ws.sufficient(width, height, k)) return;
    // erode into `a`, using `b` as the separable row/column temp; then dilate
    // `a` into dst using `c`. Three distinct buffers, so nothing aliases.
    erode_rect(src, ws.a, width, height, k, ws.b, ws.scratch);
    dilate_rect(ws.a, dst, width, height, k, ws.c, ws.scratch);
}

void top_hat(std::span<const uint8_t> src, std::span<int16_t> dst,
             int width, int height, int k, const MorphWorkspace& ws) noexcept {
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    // An undersized workspace is a STARTUP bug — the caller asked for a frame
    // size it did not allocate for. Returning silently leaves the destination
    // holding whatever was there before, and the failure then surfaces
    // somewhere else entirely as an implausibly bad measurement. The assert
    // puts it where it happened; the release path still returns rather than
    // overrunning.
    assert(dst.size() >= n && ws.sufficient(width, height, k)
           && "top_hat: workspace too small — use MorphWorkspace::scratch_bytes()");
    if (dst.size() < n || !ws.sufficient(width, height, k)) return;

    // The opening lands in `c`; open_rect uses `a` and `b` internally, so
    // nothing aliases.
    open_rect(src, ws.c, width, height, k, ws);
    for (size_t i = 0; i < n; ++i) {
        dst[i] = static_cast<int16_t>(static_cast<int>(src[i]) - static_cast<int>(ws.c[i]));
    }
}

int structuring_element_size(int target_size_px) noexcept {
    int k = target_size_px * 2 + 5;
    k = std::clamp(k, 15, 51);
    // Force odd: an even SE has no centre pixel, so the opening would shift the
    // image by half a pixel and move every centroid — straight into the metric
    // worth 60% of the marks.
    if ((k & 1) == 0) ++k;
    return k;
}

}  // namespace sat
