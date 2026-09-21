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

// ---------------------------------------------------------------------------
// fnv1a_bulk — the same primitive, eight independent chains — P1-2.
//
// WHY THIS EXISTS
//
// `snapshot` was measured at 813 us per frame against design §15's 30 us
// budget — 27x over, and 24 % of the entire frame — on a stage that is pure
// provenance and would not exist on real hardware. It is on by default in
// headless, so `just stages`, `just headless` and every FPS figure the project
// has ever quoted included it.
//
// Nearly all of it is THIS hash, not the memcpy beside it. FNV-1a is a serial
// dependency chain: each byte's multiply cannot start until the previous one
// retires, so a 640x480 frame is 307,200 multiplies at ~5 cycles of latency
// each, and the CPU's several multipliers sit idle. The memcpy of the same
// buffer is ~25 us by comparison.
//
// THE FIX, AND WHY NOT THE OBVIOUS ONE
//
// The obvious speed-up is to read eight bytes at a time as a uint64. That is
// ENDIAN-DEPENDENT, and this hash feeds the INV-3 fingerprint, whose entire
// claim is that two different machines produce the same digest. A big-endian
// machine would disagree with a little-endian one on every frame — the exact
// failure the fingerprint exists to detect, manufactured by the fingerprint.
//
// So instead: eight independent FNV-1a chains, byte i going into lane i % 8.
// Each lane still consumes single bytes at INDEXED positions, so there is no
// multi-byte load and no endianness anywhere. The eight multiply chains are
// independent, so they pipeline, and the throughput is bounded by issue width
// rather than by latency. Measured 8.2x faster on this frame size.
//
// The result is NOT equal to fnv1a() over the same bytes, and is not meant to
// be — it is a different function with the same construction. Every digest the
// project has recorded changes once, which is why this landed together with
// the other digest-moving fixes rather than on its own.
//
// The length is folded in at the end so that streams differing only by
// trailing zero bytes cannot collide, which a lane-striped construction would
// otherwise allow.
// ---------------------------------------------------------------------------
[[nodiscard]] inline uint64_t fnv1a_bulk(const uint8_t* p, size_t len,
                                         uint64_t seed = kFnvOffsetBasis) noexcept {
    uint64_t h[8];
    for (int j = 0; j < 8; ++j) {
        // Each lane starts from a distinct state, so a run of identical bytes
        // does not drive all eight lanes through the same sequence.
        h[j] = seed ^ (static_cast<uint64_t>(j) * kFnvPrime);
    }

    size_t i = 0;
    const size_t body = len & ~static_cast<size_t>(7);
    for (; i < body; i += 8) {
        // Unrolled deliberately rather than left as an inner loop: the point of
        // the whole construction is that these eight updates are independent,
        // and writing them out is what makes that visible to both the reader
        // and the scheduler.
        h[0] ^= p[i + 0]; h[0] *= kFnvPrime;
        h[1] ^= p[i + 1]; h[1] *= kFnvPrime;
        h[2] ^= p[i + 2]; h[2] *= kFnvPrime;
        h[3] ^= p[i + 3]; h[3] *= kFnvPrime;
        h[4] ^= p[i + 4]; h[4] *= kFnvPrime;
        h[5] ^= p[i + 5]; h[5] *= kFnvPrime;
        h[6] ^= p[i + 6]; h[6] *= kFnvPrime;
        h[7] ^= p[i + 7]; h[7] *= kFnvPrime;
    }
    for (; i < len; ++i) {                       // tail, same lane assignment
        h[i & 7] ^= p[i];
        h[i & 7] *= kFnvPrime;
    }

    uint64_t out = kFnvOffsetBasis;
    for (int j = 0; j < 8; ++j) {
        // fnv1a over the lane's bytes, in a fixed lane order: byte-wise again,
        // so the combination is endian-independent too.
        for (int b = 0; b < 8; ++b) {
            out ^= static_cast<uint8_t>(h[j] >> (8 * b));
            out *= kFnvPrime;
        }
    }
    for (int b = 0; b < 8; ++b) {
        out ^= static_cast<uint8_t>(static_cast<uint64_t>(len) >> (8 * b));
        out *= kFnvPrime;
    }
    return out;
}

[[nodiscard]] inline uint64_t fnv1a_bulk(std::span<const uint8_t> bytes,
                                         uint64_t seed = kFnvOffsetBasis) noexcept {
    return fnv1a_bulk(bytes.data(), bytes.size(), seed);
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
