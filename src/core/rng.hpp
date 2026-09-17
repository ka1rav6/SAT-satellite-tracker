// core/rng.hpp — deterministic random numbers, one independent stream per
// physical source of randomness.
//
// INV-3 (design §2) says: "No rand(), no unseeded generators. Every draw names
// a Stream." This file is how that rule is kept.
//
// ---------------------------------------------------------------------------
// WHY SEPARATE STREAMS MATTER MORE THAN THE GENERATOR DOES
// ---------------------------------------------------------------------------
// If every subsystem drew from one shared generator, then turning salt-and-
// pepper noise off would change the numbers the *target motion* receives, and
// two runs that differ in one setting would diverge for reasons that have
// nothing to do with that setting. Comparing "fog on" against "fog off" would
// be meaningless, and every ablation in the report depends on exactly that
// kind of comparison.
//
// With one generator per Stream, seeded independently from the master seed,
// each source of randomness is reproducible *and* independent. Disabling a
// noise term removes only its own draws. This is what makes the §13.3
// compliance matrix and the AI ablations honest.
//
// ---------------------------------------------------------------------------
// WHY PCG32
// ---------------------------------------------------------------------------
//   * ~2 ns per draw, a multiply and a shift -- the salt-and-pepper generator
//     alone draws tens of thousands of times per frame (design §9.3).
//   * 64 bits of state, trivially copyable, so a whole RngSet can be snapshot
//     and restored without allocation (INV-4).
//   * Deterministic by construction across platforms: only integer ops, no
//     libm, no compiler-dependent floating point.
//   * std::mt19937 would be 2.5 KB of state per stream and slower; the
//     std::*_distribution types are explicitly NOT portable between standard
//     library implementations, which would break INV-3 outright. That is why
//     the distributions below are hand-written rather than pulled from
//     <random>.
//
// A note on honesty: the uniform and integer paths here are bit-identical on
// any conforming compiler. next_normal() and the Poisson helper in the
// degradation chain use std::log/std::sqrt, and libm is only guaranteed to be
// *nearly* correctly rounded, so a different C library could in principle shift
// the last bit. Design §11.4 asks us to state precisely what is and is not
// bit-reproducible; this is that statement. The reproducibility CI job compares
// hashes for one toolchain across optimisation levels, which is the claim we
// can actually defend.

#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

namespace sat {

// ---------------------------------------------------------------------------
// Pcg32 — PCG-XSH-RR 64/32, the standard minimal variant.
//
// Reference: O'Neill, "PCG: A Family of Simple Fast Space-Efficient
// Statistically Good Algorithms for Random Number Generation" (2014).
// Apache-2.0 / MIT licensed algorithm; this is a clean-room 30-line rewrite.
// ---------------------------------------------------------------------------
class Pcg32 {
public:
    // The multiplier from the reference implementation. Do not change it: the
    // period and statistical quality are properties of this exact constant.
    static constexpr uint64_t kMultiplier = 6364136223846793005ULL;

    constexpr Pcg32() noexcept = default;
    constexpr explicit Pcg32(uint64_t seed, uint64_t sequence = 1u) noexcept {
        this->seed(seed, sequence);
    }

    /// (Re)seed. `sequence` selects one of 2^63 distinct streams that share the
    /// same generator but never overlap -- this is the mechanism that makes
    /// per-Stream independence cheap.
    constexpr void seed(uint64_t seed_value, uint64_t sequence) noexcept {
        state_ = 0u;
        inc_   = (sequence << 1u) | 1u;     // must be odd for a full period
        step();
        state_ += seed_value;
        step();
        has_spare_normal_ = false;
        spare_normal_     = 0.0;
    }

    /// Uniform 32-bit integer. Every other draw is built from this one.
    [[nodiscard]] constexpr uint32_t next_u32() noexcept {
        const uint64_t old = state_;
        step();
        // XSH-RR output function: xorshift down to 32 bits, then rotate by the
        // top 5 bits. The data-dependent rotation is what defeats the lattice
        // structure a plain LCG would have.
        const uint32_t xorshifted = static_cast<uint32_t>(((old >> 18u) ^ old) >> 27u);
        const uint32_t rot        = static_cast<uint32_t>(old >> 59u);
        return (xorshifted >> rot) | (xorshifted << ((~rot + 1u) & 31u));
    }

    [[nodiscard]] constexpr uint64_t next_u64() noexcept {
        // High word first so the sequence is defined independently of endianness.
        const uint64_t hi = next_u32();
        const uint64_t lo = next_u32();
        return (hi << 32) | lo;
    }

    /// Uniform in [0, 1). Built by scaling a 32-bit integer by 2^-32, which is
    /// exact in double and therefore identical on every platform. (The usual
    /// `/ UINT32_MAX` is subtly wrong: it can return exactly 1.0.)
    [[nodiscard]] constexpr double next_double() noexcept {
        return static_cast<double>(next_u32()) * 2.3283064365386963e-10;  // 2^-32
    }

    /// Uniform in (0, 1] -- never zero. Needed wherever a log() follows, e.g.
    /// the geometric skip sampler for salt-and-pepper noise (design §9.3),
    /// where log(0) would be -inf and the skip would jump past the image.
    [[nodiscard]] constexpr double next_double_open() noexcept {
        return (static_cast<double>(next_u32()) + 1.0) * 2.3283064365386963e-10;
    }

    /// Uniform in [lo, hi).
    [[nodiscard]] constexpr double next_range(double lo, double hi) noexcept {
        return lo + (hi - lo) * next_double();
    }

    /// Uniform integer in [0, bound). Uses Lemire's multiply-shift with
    /// rejection: unbiased, and the rejection branch is taken with probability
    /// under 2^-32 for the small bounds we use, so it is effectively one
    /// multiply. `% bound` would be biased, which matters for the fixed-pattern
    /// and hot-pixel maps where the bias would show up as a visible pattern.
    [[nodiscard]] constexpr uint32_t next_below(uint32_t bound) noexcept {
        if (bound == 0) return 0;
        uint64_t m = static_cast<uint64_t>(next_u32()) * static_cast<uint64_t>(bound);
        uint32_t l = static_cast<uint32_t>(m);
        if (l < bound) {
            const uint32_t threshold = (~bound + 1u) % bound;   // (2^32 - bound) % bound
            while (l < threshold) {
                m = static_cast<uint64_t>(next_u32()) * static_cast<uint64_t>(bound);
                l = static_cast<uint32_t>(m);
            }
        }
        return static_cast<uint32_t>(m >> 32);
    }

    /// A fair coin. One bit, no floating point -- used by salt-and-pepper to
    /// choose between 0 and 255.
    [[nodiscard]] constexpr bool next_bool() noexcept { return (next_u32() & 1u) != 0u; }

    // -----------------------------------------------------------------------
    // next_normal -- standard normal, Marsaglia polar method.
    //
    // The polar method is chosen over the textbook Box-Muller trig form
    // deliberately: it needs only log() and sqrt() and no sin/cos. sqrt is
    // correctly rounded by IEEE-754 mandate, and avoiding trigonometry removes
    // the least portable part of libm from the simulation path.
    //
    // It produces two independent normals per accepted pair, so the second is
    // cached. The cache is part of the generator state and is cleared on seed(),
    // so a given sequence of calls always yields the same values.
    // -----------------------------------------------------------------------
    [[nodiscard]] double next_normal() noexcept;

    /// Normal with the given mean and standard deviation.
    [[nodiscard]] double next_normal(double mean, double sigma) noexcept {
        return mean + sigma * next_normal();
    }

    /// Raw state access, for hashing a run's RNG state into a snapshot and for
    /// tests that need to prove two generators are in step.
    [[nodiscard]] constexpr uint64_t raw_state() const noexcept { return state_; }
    [[nodiscard]] constexpr uint64_t raw_inc()   const noexcept { return inc_; }

    /// Put the state back. The ONLY caller is the vectorised damage chain
    /// (degrade/sensor_simd.cpp), which advances eight draws at a time by
    /// applying M^8 directly instead of stepping eight times, and then has to
    /// tell the generator where it ended up.
    ///
    /// Deliberately narrow: it sets the state and not the increment, so a
    /// stream cannot be silently switched to a different sequence, and it
    /// clears the cached spare normal because that spare belongs to the state
    /// being replaced. Anything that wants a different sequence calls seed().
    constexpr void set_raw_state(uint64_t s) noexcept {
        state_            = s;
        has_spare_normal_ = false;
        spare_normal_     = 0.0;
    }

private:
    constexpr void step() noexcept { state_ = state_ * kMultiplier + inc_; }

    uint64_t state_ = 0x853c49e6748fea9bULL;
    uint64_t inc_   = 0xda3e39cb94b95bdbULL;

    // Cached second value from the polar method. Mutable state, but part of the
    // deterministic sequence.
    double   spare_normal_     = 0.0;
    bool     has_spare_normal_ = false;
};

// ---------------------------------------------------------------------------
// Stream — every independent source of randomness in the program.
//
// Adding a new stochastic feature means adding a name here, NOT reusing an
// existing stream. Reuse couples two features together and silently breaks the
// comparability of ablations.
//
// The numeric values are the PCG `sequence` selector, so they must stay stable
// across versions: changing one changes every historical run's numbers.
// ---------------------------------------------------------------------------
enum class Stream : uint32_t {
    // --- world construction (design §6.1 step A5) --------------------------
    TargetInit      = 0,   ///< spec row 11: "initial target location = random"
    ClutterLayout   = 1,   ///< positions/intensities of the 50-500 static sources
    DecoyLayout     = 2,   ///< the near-identical second beacon
    BackgroundSeed  = 3,   ///< procedural background field

    // --- per-tick world motion ---------------------------------------------
    TargetMotion    = 4,   ///< stochastic motion components (ou_noise) on targets
    PlatformMotion  = 5,   ///< spec row 25, stochastic platform components
    Jitter          = 6,   ///< spec row 23, +/-20 px/frame boresight jitter

    // --- sensor degradation (design §9.3) ----------------------------------
    ShotNoise       = 7,   ///< spec row 21, Poisson
    ReadNoise       = 8,   ///< spec rows 21-22, Gaussian
    SaltPepper      = 9,   ///< spec row 21, ~10% impulse noise
    FixedPattern    = 10,  ///< PRNU/FPN maps, drawn once at load
    DefectPixels    = 11,  ///< hot/dead pixels, drawn once at load
    Atmosphere      = 12,  ///< spec row 24, scintillation within a weather mode

    // --- algorithm-side randomness -----------------------------------------
    SearchPattern   = 13,  ///< tie-breaking in probabilistic search (design §10.5)
    Misc            = 14,  ///< anything genuinely one-off; keep this nearly unused

    // --- offline tooling (never used inside a graded run) ------------------
    DatasetSampling = 15,  ///< --gen-dataset scenario sampling (SAT-ML.md)
    Fuzz            = 16,  ///< --fuzz-scenarios parameter sampling (CP 14.1)

    kCount          = 17
};

/// Name of a stream, for logs and for the `run.json` provenance block.
[[nodiscard]] const char* stream_name(Stream s) noexcept;

// ---------------------------------------------------------------------------
// RngSet — one Pcg32 per Stream, all derived from a single master seed.
//
// Fixed-size array, no allocation, trivially copyable: the whole set can be
// saved and restored inside the frame loop without violating INV-4.
// ---------------------------------------------------------------------------
class RngSet {
public:
    RngSet() { seed_all(0); }
    explicit RngSet(uint64_t master_seed) { seed_all(master_seed); }

    /// Seed every stream, including ones this scenario will never use.
    /// Design §6.1 step A3 is explicit about this: "All streams seeded even if
    /// unused." Seeding lazily would make a stream's output depend on whether
    /// an unrelated feature was enabled.
    void seed_all(uint64_t master_seed) {
        master_seed_ = master_seed;
        for (uint32_t i = 0; i < static_cast<uint32_t>(Stream::kCount); ++i) {
            // SplitMix64 finaliser: decorrelates the per-stream seeds so that
            // master seeds 1 and 2 do not produce near-identical streams.
            uint64_t z = master_seed + 0x9e3779b97f4a7c15ULL * (static_cast<uint64_t>(i) + 1u);
            z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
            z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
            z =  z ^ (z >> 31);
            gens_[i].seed(z, i + 1u);
        }
    }

    /// The generator for a stream. Returned by reference: callers draw from it
    /// directly, and the reference stays valid for the life of the RngSet.
    [[nodiscard]] Pcg32& operator[](Stream s) noexcept {
        return gens_[static_cast<uint32_t>(s)];
    }
    [[nodiscard]] const Pcg32& operator[](Stream s) const noexcept {
        return gens_[static_cast<uint32_t>(s)];
    }

    [[nodiscard]] uint64_t master_seed() const noexcept { return master_seed_; }

private:
    std::array<Pcg32, static_cast<size_t>(Stream::kCount)> gens_{};
    uint64_t master_seed_ = 0;
};

}  // namespace sat
