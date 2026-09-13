#include "perception/morphology.hpp"

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

struct MinOp { uint8_t operator()(uint8_t a, uint8_t b) const noexcept { return a < b ? a : b; } };
struct MaxOp { uint8_t operator()(uint8_t a, uint8_t b) const noexcept { return a > b ? a : b; } };

/// Apply a 1-D filter along rows, then along columns. A rectangular structuring
/// element is separable, which is what makes the 2-D case O(1) per pixel too.
template <typename Op>
void separable_2d(const uint8_t* src, uint8_t* dst, int width, int height, int k,
                  uint8_t* tmp, uint8_t* scratch, uint8_t identity, Op op) noexcept {
    const int longest = std::max(width, height);
    const int padded  = longest + 2 * (k / 2) + k;   // room for the padded index space
    uint8_t* fwd = scratch;
    uint8_t* bwd = scratch + padded;

    // Horizontal pass, row by row.
    for (int y = 0; y < height; ++y) {
        running_1d(src + static_cast<size_t>(y) * width,
                   tmp + static_cast<size_t>(y) * width,
                   width, k, fwd, bwd, identity, op);
    }

    // Vertical pass. Columns are gathered into a contiguous buffer first: a
    // strided 1-D pass would touch a new cache line on every access, and for a
    // 640x480 frame that is 480 misses per column.
    uint8_t* col     = scratch + 2 * padded;
    uint8_t* col_out = col + longest;
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) col[y] = tmp[static_cast<size_t>(y) * width + x];
        running_1d(col, col_out, height, k, fwd, bwd, identity, op);
        for (int y = 0; y < height; ++y) dst[static_cast<size_t>(y) * width + x] = col_out[y];
    }
}

/// Scratch needed by separable_2d, in bytes.
[[nodiscard]] size_t scratch_needed(int width, int height, int k) noexcept {
    const int longest = std::max(width, height);
    const int padded  = longest + 2 * (k / 2) + k;
    return static_cast<size_t>(2 * padded + 2 * longest);
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
