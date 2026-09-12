// core/hash.hpp — the reproducibility fingerprint.
//
// INV-3 is enforced, not assumed: CP 2.5/2.6 hash the observable state of every
// frame and CI compares the sequences across optimisation levels and machines.
// This file defines exactly what "observable state" means.
//
// FNV-1a is used because it is:
//   * trivially specified -- an XOR and a multiply, with no table, so there is
//     no chance of two implementations disagreeing;
//   * order-sensitive, which is the point: if the pipeline visits blobs in a
//     different order, the hash must change;
//   * fast enough to run over a 640x480 frame every frame (~0.3 ms) and cheap
//     enough to leave on in release builds.
//
// It is NOT a cryptographic hash and is not used as one. Its job is to make an
// accidental divergence loud, not to resist an adversary.
//
// ---------------------------------------------------------------------------
// THE FLOATING-POINT RULE
// ---------------------------------------------------------------------------
// Hashing the raw bits of a double is the strictest possible check, and it is
// what we do -- but only for values that are genuinely bit-reproducible
// (boresight, filter state under --no-ai). Values that pass through a neural
// network are hashed as *discrete decisions* instead (selected candidate index,
// chosen strategy id, track state), per design §11.4. hash_quantised() exists
// for the middle ground: a float we want in the fingerprint but only to a
// stated tolerance.

#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <span>
#include <string_view>

namespace sat {

// FNV-1a 64-bit. Constants from the reference specification; do not change.
// Written in hex because that is how the specification states them and how the
// published test vectors are quoted -- the decimal forms are 20 digits long and
// easy to mistype, which is exactly what test_hash.cpp caught once already.
inline constexpr uint64_t kFnvOffsetBasis = 0xcbf29ce484222325ULL;   // 14695981039346656037
inline constexpr uint64_t kFnvPrime       = 0x00000100000001b3ULL;   //     1099511628211

/// Fold raw bytes into a running hash.
[[nodiscard]] inline uint64_t fnv1a(const void* data, size_t len,
                                    uint64_t h = kFnvOffsetBasis) noexcept {
    const auto* p = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < len; ++i) {
        h ^= p[i];
        h *= kFnvPrime;
    }
    return h;
}

[[nodiscard]] inline uint64_t fnv1a(std::span<const uint8_t> bytes,
                                    uint64_t h = kFnvOffsetBasis) noexcept {
    return fnv1a(bytes.data(), bytes.size(), h);
}

[[nodiscard]] inline uint64_t fnv1a(std::string_view s,
                                    uint64_t h = kFnvOffsetBasis) noexcept {
    return fnv1a(s.data(), s.size(), h);
}

/// Fold any trivially-copyable value in by its object representation.
///
/// ---------------------------------------------------------------------------
/// PADDING HAZARD -- READ BEFORE USING THIS ON A STRUCT
/// ---------------------------------------------------------------------------
/// This hashes sizeof(T) BYTES, and a struct's padding bytes are part of that.
/// Padding is never initialised by a member-wise constructor, so
///
///     struct Pose { double az, el; int32_t mode; };   // sizeof 24, members 20
///
/// carries four bytes of whatever was previously on the stack. Two runs that
/// are genuinely identical would then produce different fingerprints, and the
/// reproducibility job would report a divergence that does not exist -- the
/// worst possible failure mode for a check whose entire job is to be trusted.
///
/// The static_assert below rejects any type with padding (or with multiple bit
/// patterns for one value, which is why raw floating point is excluded too).
/// For a struct of doubles, hash the members explicitly with hash_double():
///
///     h = hash_double(p.az, h);
///     h = hash_double(p.el, h);
///     h = hash_value (p.mode, h);
///
/// That is what hash_angle() and hash_pixel() below do, and it is what the
/// frame fingerprint uses.
template <typename T>
[[nodiscard]] inline uint64_t hash_value(const T& v, uint64_t h = kFnvOffsetBasis) noexcept {
    static_assert(std::is_trivially_copyable_v<T>, "hash_value needs a trivially copyable type");
    static_assert(std::has_unique_object_representations_v<T>,
                  "hash_value would include padding bytes or ambiguous float encodings; "
                  "hash the members individually (see hash_double / hash_angle)");
    return fnv1a(&v, sizeof(T), h);
}

/// Fold a double in by its bits, normalising the two representations of zero so
/// that -0.0 and +0.0 hash alike. Without this, a sign that is physically
/// meaningless (a rate that settled at exactly zero from below) would show up
/// as a reproducibility failure.
[[nodiscard]] inline uint64_t hash_double(double v, uint64_t h = kFnvOffsetBasis) noexcept {
    if (v == 0.0) v = 0.0;                       // collapses -0.0 to +0.0
    uint64_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    return fnv1a(&bits, sizeof(bits), h);
}

/// Fold a double in at a stated resolution. Use when a value belongs in the
/// fingerprint but is only reproducible to a tolerance -- anything downstream of
/// an ONNX model, or a value that crossed a libm boundary.
///
/// Example: hash_quantised(centroid_x, 1e-6) accepts divergence below a
/// micro-pixel, which is four orders of magnitude below the graded metric.
[[nodiscard]] inline uint64_t hash_quantised(double v, double resolution,
                                             uint64_t h = kFnvOffsetBasis) noexcept {
    const int64_t q = static_cast<int64_t>(std::llround(v / resolution));
    return fnv1a(&q, sizeof(q), h);
}

/// Fold a 2-D vector in component by component. Works for Angle2, Pixel2 and
/// Rate2 -- anything with `.x` and `.y` doubles. Component-wise rather than
/// bytewise so the padding hazard above cannot apply and so -0.0 is normalised
/// on both axes.
template <typename V>
[[nodiscard]] inline uint64_t hash_vec2(const V& v, uint64_t h = kFnvOffsetBasis) noexcept {
    h = hash_double(v.x, h);
    h = hash_double(v.y, h);
    return h;
}

// ---------------------------------------------------------------------------
// FrameFingerprint — the per-frame record the reproducibility job compares.
//
// Kept as a small struct rather than a single number so that when a divergence
// happens, the report can say *which part* diverged: "the image is identical
// but the detection moved" points at perception, while "the image differs"
// points at the world or the degradation chain.
// ---------------------------------------------------------------------------
struct FrameFingerprint {
    int64_t  frame     = 0;
    uint64_t image     = 0;   ///< the 8-bit frame the detector actually sees
    uint64_t boresight = 0;   ///< commanded and true pointing
    uint64_t detection = 0;   ///< candidate count and winning centroid
    uint64_t track     = 0;   ///< filter state and lifecycle status
    uint64_t mode      = 0;   ///< FSM state and active strategy -- integers, always exact

    /// Collapse to one number for a compact log line.
    [[nodiscard]] uint64_t combined() const noexcept {
        uint64_t h = kFnvOffsetBasis;
        h = hash_value(image,     h);
        h = hash_value(boresight, h);
        h = hash_value(detection, h);
        h = hash_value(track,     h);
        h = hash_value(mode,      h);
        return h;
    }
};

}  // namespace sat
