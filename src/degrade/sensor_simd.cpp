// degrade/sensor_simd.cpp — see the header for why bit-identical is the hard
// part and for what it costs.

#include "degrade/sensor_simd.hpp"

#include "degrade/fast_normal.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

// ---------------------------------------------------------------------------
// The whole file is x86-64-only and compiles to nothing elsewhere. ARM gets the
// scalar path, which produces the same frames.
// ---------------------------------------------------------------------------
#if (defined(__x86_64__) || defined(_M_X64)) && (defined(__GNUC__) || defined(__clang__))
#define SAT_HAVE_AVX2_PATH 1
#include <immintrin.h>
#else
#define SAT_HAVE_AVX2_PATH 0
#endif

namespace sat {

#if SAT_HAVE_AVX2_PATH

namespace {

constexpr int kLanes = 8;

// ---------------------------------------------------------------------------
// PcgJump — the constants that advance a PCG32 stream by 1..8 steps at once.
//
// PCG's state update is an LCG: state <- state*M + inc. Composing it k times
// gives state*M^k + inc*(M^(k-1) + ... + M + 1), so the k-th state ahead is an
// affine function of the current one with constants that depend only on `inc`.
// Computing them once per call turns "step eight times" into "apply eight
// independent affine maps", which is both vectorisable and free of the serial
// multiply chain that would otherwise set the floor at one 64-bit multiply
// (three to five cycles) per pixel.
//
// mul[k] and add[k] are the constants for k+1 steps, so mul[7]/add[7] advance
// the stream by the eight draws one iteration consumes.
// ---------------------------------------------------------------------------
struct PcgJump {
    uint64_t mul[kLanes];
    uint64_t add[kLanes];
};

PcgJump pcg_jump(uint64_t inc) noexcept {
    PcgJump j{};
    uint64_t m = 1, a = 0;
    for (int k = 0; k < kLanes; ++k) {
        // Compose one more step: (m, a) <- (m*M, a*M + inc).
        m = m * Pcg32::kMultiplier;
        a = a * Pcg32::kMultiplier + inc;
        j.mul[k] = m;
        j.add[k] = a;
    }
    return j;
}

/// 64x64 -> low 64 multiply, four lanes at a time. AVX2 has no vpmullq (that
/// arrives with AVX-512DQ), so it is assembled from three 32x32 -> 64
/// multiplies, which is the standard decomposition and exact for the low half.
/// `b_hi` is b >> 32, passed in because b is loop-invariant here and the shift
/// would otherwise be repeated 38,400 times a frame for a constant.
[[gnu::target("avx2")]]
inline __m256i mul64(__m256i a, __m256i b, __m256i b_hi) noexcept {
    const __m256i a_hi = _mm256_srli_epi64(a, 32);
    const __m256i lo   = _mm256_mul_epu32(a, b);            // a.lo * b.lo
    const __m256i m1   = _mm256_mul_epu32(a_hi, b);         // a.hi * b.lo
    const __m256i m2   = _mm256_mul_epu32(a, b_hi);         // a.lo * b.hi
    const __m256i mid  = _mm256_slli_epi64(_mm256_add_epi64(m1, m2), 32);
    return _mm256_add_epi64(lo, mid);
}

/// Take the low 32 bits of each of eight 64-bit lanes (held in two registers)
/// and pack them into one register of eight 32-bit lanes, in order.
[[gnu::target("avx2")]]
inline __m256i pack_lo32(__m256i a, __m256i b) noexcept {
    // Each input is [x0.lo, x0.hi, x1.lo, x1.hi, x2.lo, x2.hi, x3.lo, x3.hi].
    const __m256i idx = _mm256_setr_epi32(0, 2, 4, 6, 0, 2, 4, 6);
    const __m256i pa  = _mm256_permutevar8x32_epi32(a, idx);  // lows in the low 128
    const __m256i pb  = _mm256_permutevar8x32_epi32(b, idx);
    return _mm256_permute2x128_si256(pa, pb, 0x20);           // [pa.lo | pb.lo]
}

/// PCG-XSH-RR's output function, on eight 64-bit states held in two registers.
/// Returns the eight 32-bit results in lane order.
/// xorshifted = (uint32)(((old >> 18) ^ old) >> 27), four 64-bit lanes at once.
/// A named function rather than a lambda inside pcg_output: a lambda does not
/// inherit the enclosing function's target attribute, so the intrinsics inside
/// it fail to inline.
[[gnu::target("avx2")]]
inline __m256i pcg_xorshift(__m256i s) noexcept {
    const __m256i t = _mm256_xor_si256(_mm256_srli_epi64(s, 18), s);
    return _mm256_srli_epi64(t, 27);
}

[[gnu::target("avx2")]]
inline __m256i pcg_output(__m256i s0, __m256i s1) noexcept {
    const __m256i x = pack_lo32(pcg_xorshift(s0), pcg_xorshift(s1));
    // rot = (uint32)(old >> 59), which is already in the low 32 bits.
    const __m256i r = pack_lo32(_mm256_srli_epi64(s0, 59), _mm256_srli_epi64(s1, 59));

    // rotr32(x, r) = (x >> r) | (x << ((32 - r) & 31)).
    //
    // The scalar spelling is `x << ((~rot + 1u) & 31u)`, which is the same
    // thing: -rot mod 32. At rot = 0 both give a shift of 0, so the result is
    // x | x = x, and _mm256_sllv_epi32 with a count of 0 agrees.
    const __m256i thirty_two = _mm256_set1_epi32(32);
    const __m256i mask31     = _mm256_set1_epi32(31);
    const __m256i left       = _mm256_and_si256(_mm256_sub_epi32(thirty_two, r), mask31);
    return _mm256_or_si256(_mm256_srlv_epi32(x, r), _mm256_sllv_epi32(x, left));
}

// ---------------------------------------------------------------------------
// fast_normal, eight at a time.
//
// The body is the interpolation from degrade/fast_normal.hpp, expressed as a
// gather and two float operations. The tails are NOT vectorised: the mask says
// which lanes need them and those lanes are recomputed with the identical
// scalar call, so they are bit-identical by construction rather than by
// argument. They are 1.56% of draws, and 98.4% of vectors contain none at all.
// ---------------------------------------------------------------------------
[[gnu::target("avx2")]]
inline __m256 fast_normal8(__m256i u, __m256i cell, const float* knots) noexcept {
    constexpr int      kFracBits = FastNormal::kFracBits;   // 20
    constexpr uint32_t kFracMask = (1u << kFracBits) - 1u;

    const __m256i frac = _mm256_and_si256(u, _mm256_set1_epi32(static_cast<int>(kFracMask)));

    // f = frac * 2^-20. frac < 2^20 so the signed convert is exact.
    const __m256 f = _mm256_mul_ps(_mm256_cvtepi32_ps(frac),
                                   _mm256_set1_ps(1.0f / 1048576.0f));

    const __m256 z0 = _mm256_i32gather_ps(knots, cell, 4);
    const __m256 z1 = _mm256_i32gather_ps(knots, _mm256_add_epi32(cell, _mm256_set1_epi32(1)), 4);

    // z0 + f * (z1 - z0) — the same three operations, in the same order, as the
    // scalar body. No FMA: see the header.
    return _mm256_add_ps(z0, _mm256_mul_ps(f, _mm256_sub_ps(z1, z0)));
}

/// Lanes that must take the closed-form tail instead of the table.
[[gnu::target("avx2")]]
inline int tail_mask(__m256i cell) noexcept {
    // The two ends as one shifted range test: t = cell - kExactCells is outside
    // [0, kCells - 2*kExactCells) exactly when cell is in either tail. Values
    // are below 4096 so the signed comparisons are safe, and this is two
    // compares where the obvious spelling is three.
    const __m256i t  = _mm256_sub_epi32(cell, _mm256_set1_epi32(FastNormal::kExactCells));
    const __m256i hi = _mm256_set1_epi32(FastNormal::kCells - 2 * FastNormal::kExactCells - 1);
    const __m256i bad = _mm256_or_si256(_mm256_cmpgt_epi32(t, hi),
                                        _mm256_cmpgt_epi32(_mm256_setzero_si256(), t));
    return _mm256_movemask_ps(_mm256_castsi256_ps(bad));
}

// ---------------------------------------------------------------------------
// The fused loop itself.
// ---------------------------------------------------------------------------
[[gnu::target("avx2")]]
size_t run_avx2(const DamageArgs& a, Pcg32& g) noexcept {
    const size_t vec_n = (a.n / kLanes) * kLanes;
    if (vec_n == 0) return 0;

    const float* knots = kFastNormal.knots();

    const PcgJump jump = pcg_jump(g.raw_inc());
    const __m256i jmul0 = _mm256_setr_epi64x(
        1, static_cast<long long>(jump.mul[0]),
        static_cast<long long>(jump.mul[1]), static_cast<long long>(jump.mul[2]));
    const __m256i jadd0 = _mm256_setr_epi64x(
        0, static_cast<long long>(jump.add[0]),
        static_cast<long long>(jump.add[1]), static_cast<long long>(jump.add[2]));
    const __m256i jmul1 = _mm256_setr_epi64x(
        static_cast<long long>(jump.mul[3]), static_cast<long long>(jump.mul[4]),
        static_cast<long long>(jump.mul[5]), static_cast<long long>(jump.mul[6]));
    const __m256i jadd1 = _mm256_setr_epi64x(
        static_cast<long long>(jump.add[3]), static_cast<long long>(jump.add[4]),
        static_cast<long long>(jump.add[5]), static_cast<long long>(jump.add[6]));
    const __m256i jmul0_hi = _mm256_srli_epi64(jmul0, 32);
    const __m256i jmul1_hi = _mm256_srli_epi64(jmul1, 32);
    const uint64_t step8_mul = jump.mul[7];
    const uint64_t step8_add = jump.add[7];

    const __m256 v_alpha = _mm256_set1_ps(a.alpha);
    const __m256 v_beta  = _mm256_set1_ps(a.beta);
    const __m256 v_invk  = _mm256_set1_ps(a.inv_k);
    const __m256 v_rvar  = _mm256_set1_ps(a.rvar);
    const __m256 v_sigma = _mm256_set1_ps(a.sigma);
    const __m256 v_half  = _mm256_set1_ps(0.5f);
    const __m256 v_zero  = _mm256_setzero_ps();
    const __m256 v_2545  = _mm256_set1_ps(254.5f);
    const __m256i v_255i = _mm256_set1_epi32(255);

    uint64_t state = g.raw_state();
    alignas(32) uint32_t words[kLanes];
    alignas(32) float    zbuf[kLanes];

    for (size_t i = 0; i < vec_n; i += kLanes) {
        // --- eight consecutive PCG states, then eight outputs --------------
        const __m256i base = _mm256_set1_epi64x(static_cast<long long>(state));
        const __m256i s0   = _mm256_add_epi64(mul64(base, jmul0, jmul0_hi), jadd0);
        const __m256i s1   = _mm256_add_epi64(mul64(base, jmul1, jmul1_hi), jadd1);
        const __m256i u    = pcg_output(s0, s1);
        state = state * step8_mul + step8_add;

        // --- the normal deviate -------------------------------------------
        const __m256i cell = _mm256_srli_epi32(u, FastNormal::kFracBits);
        __m256 z = fast_normal8(u, cell, knots);
        const int tails = tail_mask(cell);
        if (tails != 0) [[unlikely]] {
            // 1.56% of draws, 12% of vectors. Fixed up with the identical
            // scalar call so those lanes are the same float, not a near one.
            _mm256_storeu_si256(reinterpret_cast<__m256i*>(words), u);
            _mm256_storeu_ps(zbuf, z);
            for (int L = 0; L < kLanes; ++L) {
                if (tails & (1 << L)) zbuf[L] = fast_normal(words[L]);
            }
            z = _mm256_loadu_ps(zbuf);
        }

        // --- the chain ------------------------------------------------------
        // Atmosphere and black level (spec row 24), pre-summed into beta.
        __m256 v = _mm256_add_ps(_mm256_mul_ps(v_alpha, _mm256_loadu_ps(a.rad + i)), v_beta);

        // Shot and read noise together: variances add (see sensor.cpp).
        if (a.shot) {
            const __m256 var = _mm256_add_ps(_mm256_mul_ps(v, v_invk), v_rvar);
            v = _mm256_add_ps(v, _mm256_mul_ps(_mm256_sqrt_ps(var), z));
        } else {
            v = _mm256_add_ps(v, _mm256_mul_ps(v_sigma, z));
        }

        // Fixed pattern.
        if (a.fp) {
            v = _mm256_add_ps(_mm256_mul_ps(v, _mm256_loadu_ps(a.prnu + i)),
                              _mm256_loadu_ps(a.fpn + i));
        }

        // Clip and quantise, matching core/image.hpp's three cases exactly.
        // The GT/GE comparisons are ordered, so a NaN takes the zero branch —
        // which is what `!(f > 0.0f)` does in the scalar path.
        const __m256  gt0 = _mm256_cmp_ps(v, v_zero, _CMP_GT_OQ);
        const __m256  ge  = _mm256_cmp_ps(v, v_2545, _CMP_GE_OQ);
        const __m256i mid = _mm256_cvttps_epi32(_mm256_add_ps(v, v_half));
        __m256i       q   = _mm256_blendv_epi8(mid, v_255i, _mm256_castps_si256(ge));
        q = _mm256_and_si256(q, _mm256_castps_si256(gt0));       // zero where !(v > 0)

        // 8 x int32 -> 8 x uint8. Values are already in [0, 255].
        const __m256i p16 = _mm256_packs_epi32(q, q);
        const __m256i p8  = _mm256_packus_epi16(p16, p16);
        const int lo = _mm256_extract_epi32(p8, 0);
        const int hi = _mm256_extract_epi32(p8, 4);
        std::memcpy(a.dst + i,     &lo, 4);
        std::memcpy(a.dst + i + 4, &hi, 4);
    }

    // Leave the generator exactly where the scalar path would have left it.
    g.set_raw_state(state);
    return vec_n;
}

bool cpu_has_avx2() noexcept {
    // __builtin_cpu_supports runs the CPUID once and caches it; this wrapper
    // exists so the result is asked for once per process rather than per frame.
    static const bool ok = __builtin_cpu_supports("avx2") != 0;
    return ok;
}

// See the header for why this is a deliberate switch and not an environment
// variable. Default on: the shipped path is the fast one. File-local, so the
// only way to change it is the accessor below.
bool g_simd_enabled = true;

}  // namespace

bool damage_chain_simd_available() noexcept { return cpu_has_avx2(); }

void set_damage_simd_enabled(bool on) noexcept { g_simd_enabled = on; }
bool damage_chain_simd_enabled() noexcept { return g_simd_enabled && cpu_has_avx2(); }

size_t damage_chain_simd(const DamageArgs& a, Pcg32& g) noexcept {
    if (!g_simd_enabled) return 0;
    if (!cpu_has_avx2()) return 0;
    if (!a.read && !a.shot) return 0;      // nothing to vectorise
    if (a.rad == nullptr || a.dst == nullptr) return 0;
    if (a.fp && (a.prnu == nullptr || a.fpn == nullptr)) return 0;
    return run_avx2(a, g);
}

#else   // !SAT_HAVE_AVX2_PATH

bool damage_chain_simd_available() noexcept { return false; }
bool damage_chain_simd_enabled() noexcept { return false; }
// Accepted and ignored: a build with no vector path is already scalar-only, so
// asking for scalar is satisfied. Refusing would make the gate's --no-simd
// arm fail on exactly the machines where it is redundant.
void set_damage_simd_enabled(bool) noexcept {}
size_t damage_chain_simd(const DamageArgs&, Pcg32&) noexcept { return 0; }

#endif

}  // namespace sat
