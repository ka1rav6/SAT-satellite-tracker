// perception/morphology.hpp — van Herk / Gil-Werman morphology (design §9.4.2).
//
// ---------------------------------------------------------------------------
// WHAT THIS IS FOR: BACKGROUND REMOVAL
// ---------------------------------------------------------------------------
//     top_hat = image - opening(image),  opening = dilate(erode(image))
//
// An opening removes anything smaller than the structuring element while leaving
// larger structure intact. Subtracting it leaves exactly the small bright things
// — which is what a beacon is, and what a gradient, a cloud edge or a slow
// illumination ramp is not.
//
// Design §9.4.2 insists this is SPATIAL, not temporal, and the reason is
// specific to this problem: the camera slews. Any background model built from
// frame history is invalidated the moment the mount moves, and the mount moves
// constantly — up to 26.7 px/frame at the rate limit (§1.4). A spatial opening
// is slew-invariant by construction, because it looks only at the frame in hand.
//
// ---------------------------------------------------------------------------
// WHY VAN HERK: O(1) PER PIXEL, REGARDLESS OF SE SIZE
// ---------------------------------------------------------------------------
// A naive 1-D min over a window of k costs k-1 comparisons per pixel, so a 51-px
// SE costs ten times a 5-px one. The van Herk / Gil-Werman algorithm costs THREE
// comparisons per pixel whatever k is:
//
//     split the row into blocks of length k
//     forward[i]  = running min from the start of i's block up to i
//     backward[i] = running min from i to the end of i's block
//     min(i .. i+k-1) = min(backward[i], forward[i+k-1])
//
// Every window of length k straddles exactly one block boundary, so it is always
// the union of a backward-run and a forward-run that have already been computed.
//
// This matters because §9.4.2 sizes the SE from the target: `target_size*2 + 5`
// clamped to [15, 51]. Without van Herk, a scenario with a 20 px beacon would
// cost three times one with a 5 px beacon, and the frame budget would depend on
// the scenario. CP 5.2's acceptance is exactly that: "timing flat as SE varies
// 5 -> 51".
//
// Separability does the rest: a 2-D rectangular erosion is a horizontal pass
// followed by a vertical one, so the whole operation is O(1) per pixel in 2-D
// too.

#pragma once

#include <cstdint>
#include <span>

namespace sat {

/// 1-D running minimum over a window of `k`, van Herk / Gil-Werman.
///
/// `scratch` must hold at least 2*n elements; it carries the forward and
/// backward runs. Passing it in rather than allocating keeps the whole stage
/// allocation-free (INV-4) — it comes from the frame arena.
///
/// Border handling is REPLICATE, matching cv::erode's default so the §16 oracle
/// comparison is possible, and matching median_3x3 so the two stages treat the
/// frame edge the same way.
void min_filter_1d(std::span<const uint8_t> src, std::span<uint8_t> dst,
                   int n, int k, std::span<uint8_t> scratch) noexcept;

/// 1-D running maximum, same algorithm.
void max_filter_1d(std::span<const uint8_t> src, std::span<uint8_t> dst,
                   int n, int k, std::span<uint8_t> scratch) noexcept;

/// 2-D erosion (local minimum) with a k x k rectangular structuring element.
///
/// `tmp` needs width*height bytes, `scratch` needs 2*max(width, height).
void erode_rect(std::span<const uint8_t> src, std::span<uint8_t> dst,
                int width, int height, int k,
                std::span<uint8_t> tmp, std::span<uint8_t> scratch) noexcept;

/// 2-D dilation (local maximum) with a k x k rectangular structuring element.
void dilate_rect(std::span<const uint8_t> src, std::span<uint8_t> dst,
                 int width, int height, int k,
                 std::span<uint8_t> tmp, std::span<uint8_t> scratch) noexcept;

// ---------------------------------------------------------------------------
// MorphWorkspace — the scratch these kernels need, sized once.
//
// Passing buffers in rather than allocating keeps the stage allocation-free
// (INV-4); in a real run these spans point into the frame arena. Bundling them
// removes a class of mistake that already bit once: an earlier top_hat() reused
// one caller-supplied buffer as both a destination and a working temp, which
// aliases and silently corrupts the result.
// ---------------------------------------------------------------------------
struct MorphWorkspace {
    std::span<uint8_t> a;        ///< width*height
    std::span<uint8_t> b;        ///< width*height
    std::span<uint8_t> c;        ///< width*height
    std::span<uint8_t> scratch;  ///< see scratch_bytes()

    /// Bytes needed in `scratch` for the given frame and structuring element.
    [[nodiscard]] static size_t scratch_bytes(int width, int height, int k) noexcept;

    [[nodiscard]] bool sufficient(int width, int height, int k) const noexcept;
};

/// Morphological opening: erode then dilate. Removes bright features smaller
/// than the structuring element.
void open_rect(std::span<const uint8_t> src, std::span<uint8_t> dst,
               int width, int height, int k, const MorphWorkspace& ws) noexcept;

/// White top-hat: image - opening(image).
///
/// Output is int16_t, per design §9.4.2. It cannot be uint8_t: although the
/// mathematical top-hat of a uint8 image is non-negative, storing it signed
/// leaves room for the background-subtracted values the centroid stage wants and
/// avoids a clamp that would quietly discard information at the very top of the
/// range. int16 also feeds the summed-area tables directly (§9.4.3).
void top_hat(std::span<const uint8_t> src, std::span<int16_t> dst,
             int width, int height, int k, const MorphWorkspace& ws) noexcept;

/// The structuring-element size design §9.4.2 prescribes for a given target
/// size: `target_size_px * 2 + 5`, clamped to [15, 51].
///
/// Odd by construction, because an even SE has no centre and the opening would
/// shift the image by half a pixel — which would move every centroid, into the
/// metric worth 60% of the marks.
[[nodiscard]] int structuring_element_size(int target_size_px) noexcept;

}  // namespace sat
