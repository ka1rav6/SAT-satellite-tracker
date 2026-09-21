// degrade/sensor_simd.hpp — the fused damage chain's vector path.
//
// ---------------------------------------------------------------------------
// WHAT CP 14.2 ASKS FOR, AND WHAT IT MEANS HERE
// ---------------------------------------------------------------------------
// "AVX2 kernels + scalar fallback. Bit-identical to scalar on random inputs."
//
// Bit-identical is the whole difficulty, and it is not decoration. The damage
// chain feeds the rendered frame, which feeds the reproducibility fingerprint
// (INV-3) and the graded centroiding metric. A vector path that agreed to
// within an ulp would make every recorded number depend on which machine
// produced it, which is exactly the property the project spends CP 2.6 on.
//
// Three things follow from it, and each one costs something:
//
//   NO FMA. The scalar path multiplies and then adds, and a fused multiply-add
//   rounds once where that rounds twice. This file therefore targets `avx2`
//   ONLY — not `avx2,fma` — so the compiler has no FMA instruction to contract
//   into and the intrinsics below use separate _mm256_mul_ps/_mm256_add_ps.
//
//   THE SAME RANDOM NUMBERS, IN THE SAME ORDER. Not eight independent streams:
//   the eight lanes must hold the eight CONSECUTIVE outputs of the one PCG32
//   stream the scalar path would have drawn. PCG's state update is an LCG,
//   state <- state*M + inc, so the k-th state ahead is state*M^k + C_k with M^k
//   and C_k constants — computed once per call and then applied in parallel.
//   The stream advances by M^8 per iteration instead of by M eight times, which
//   also removes the serial multiply chain that would otherwise set the floor.
//
//   THE SAME NORMAL DEVIATE. degrade/fast_normal.hpp was written for this: one
//   uniform in, one normal out, no rejection, a pure function of the 32-bit
//   word. The body is a gather and an interpolation; the 1.56% of draws that
//   fall in the exact tails are fixed up lane by lane with the identical scalar
//   call, so those are bit-identical by construction rather than by argument.
//
// The dispatch is runtime, not a build flag, because CP 15.5 ships ONE binary
// that has to run on whatever the evaluators have. A machine without AVX2 takes
// the scalar path and gets the same frames, slower.
// ---------------------------------------------------------------------------

#pragma once

#include "core/rng.hpp"

#include <cstddef>
#include <cstdint>

namespace sat {

// ---------------------------------------------------------------------------
// DamageArgs — everything the fused loop needs, in one struct.
//
// A struct rather than fourteen parameters so the scalar and vector entry
// points cannot drift apart in their argument lists, which is the most likely
// way a "bit-identical" pair of kernels stops being one.
// ---------------------------------------------------------------------------
struct DamageArgs {
    const float* rad  = nullptr;   ///< input radiance, n floats
    uint8_t*     dst  = nullptr;   ///< output frame, n bytes
    const float* prnu = nullptr;   ///< may be null (no fixed pattern)
    const float* fpn  = nullptr;
    size_t       n    = 0;

    float  alpha = 1.0f;      ///< atmosphere gain (spec row 24)
    float  beta  = 0.0f;      ///< atmosphere offset + black level, pre-summed
    float  k     = 8.0f;      ///< photons per grey level
    float  inv_k = 0.125f;
    float  sigma = 20.0f;     ///< read noise, grey levels (rows 21-22)
    float  rvar  = 400.0f;    ///< sigma^2

    bool   shot = true;
    bool   read = true;
    bool   fp   = true;
};

/// True if this build and this CPU can take the vector path.
///
/// Reports the HARDWARE answer and ignores the switch below, so a caller can
/// distinguish "no AVX2 here" from "AVX2 deliberately turned off" — which the
/// reproducibility gate has to do, because only one of those makes the
/// scalar/vector comparison meaningful.
[[nodiscard]] bool damage_chain_simd_available() noexcept;

// ---------------------------------------------------------------------------
// FORCING THE SCALAR PATH — the INV-3 gate's missing half.
//
// The dispatch above is RUNTIME, which is the right call (CP 15.5 ships one
// binary for whatever the evaluators have) and is also precisely the risk
// INV-3 exists to catch: different machines execute different code. The
// reproducibility gate ran on one machine, took the AVX2 path both times, and
// therefore proved nothing at all about the scalar path or about agreement
// between the two.
//
// Proving it needs both paths in ONE process, so the gate can run a scenario
// twice and compare. Hence an explicit switch.
//
// Explicitly a function call and NOT an environment variable, though the
// obvious `SAT_NO_AVX2=1` was the first idea: an env var makes the rendered
// frame — and therefore the fingerprint, and therefore every graded number —
// depend on the ambient environment of whoever ran the binary. That is the
// same class of hazard as reading the wall clock, and INV-3 exists to keep the
// output a pure function of the scenario and the seed. A flag the caller sets
// deliberately, and which run.json records, keeps it a pure function of
// inputs that are written down.
//
// NOT thread-safe by design: it is set once during start-up or between runs,
// never while frames are in flight. Making it an atomic would suggest
// otherwise.
// ---------------------------------------------------------------------------

/// Turn the vector path off (or back on) for this process.
void set_damage_simd_enabled(bool on) noexcept;

/// Whether the vector path will actually be taken: available AND enabled.
[[nodiscard]] bool damage_chain_simd_enabled() noexcept;

/// The vector path. Returns the number of pixels it consumed, which is
/// `n` rounded DOWN to a multiple of the vector width — the caller finishes the
/// remainder with the scalar loop, using the generator state this leaves behind.
///
/// `g` is advanced exactly as many steps as the scalar path would have advanced
/// it, so the two are interchangeable at any pixel boundary. That is what makes
/// the tail handling correct rather than merely close.
///
/// Returns 0 when the vector path is unavailable or does not apply — in
/// particular when any pixel would fall below design §9.3's lambda = 30 cutoff,
/// where shot noise is genuinely discrete and Knuth's method has no vector form.
[[nodiscard]] size_t damage_chain_simd(const DamageArgs& a, Pcg32& g) noexcept;

}  // namespace sat
