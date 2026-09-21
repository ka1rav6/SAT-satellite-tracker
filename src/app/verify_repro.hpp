// app/verify_repro.hpp — CP 2.6, the Stage 2 ★ GATE.
//
// "--verify-reproducibility; CI job running 5 scenarios x 3 seeds at -O0 and
//  -O2, comparing hashes. CI green. Then deliberately inject a std::chrono call
//  into the sim path and confirm the job goes red."
//
// ---------------------------------------------------------------------------
// WHY THIS IS A GATE RATHER THAN A NICE PROPERTY
// ---------------------------------------------------------------------------
// INV-3 is what makes every other number in the project trustworthy. A
// compliance matrix from 1500 runs (design §13.3) is evidence only if rerunning
// those 1500 runs gives the same answer; an ablation showing that feedforward
// helps is evidence only if the two arms differ by feedforward and nothing else.
// Without reproducibility, every measurement is an anecdote.
//
// The verification runs a scenario twice IN ONE PROCESS and compares
// fingerprints frame by frame. That catches the failure modes that actually
// happen: a wall-clock read, an unseeded generator, an unordered_map iteration,
// accumulated time. It does NOT catch cross-machine or cross-optimisation
// divergence on its own — that is what the CI matrix is for, which runs this
// same command at -O0 and -O2 and compares the printed digest.
//
// ---------------------------------------------------------------------------
// WHAT THIS CHECK CAN AND CANNOT SEE — stated precisely, per design §11.4
// ---------------------------------------------------------------------------
// The fingerprint hashes the 8-bit frame, not the float radiance behind it. So
// a floating-point divergence smaller than one grey level on every pixel is
// invisible here until it grows large enough to change a quantised value.
//
// That is a deliberate choice, not an oversight. INV-3's purpose is that
// RESULTS are reproducible, and every result the project reports is derived
// from the 8-bit frames the detector sees and from the control state (which IS
// hashed at full double precision, since it involves no libm). A float
// difference invisible in both cannot affect any reported number; if it later
// grows enough to matter, it becomes visible and the check fires then.
//
// tools/verify_repro_guard.sh demonstrates both halves of this. Injecting a
// wall-clock-derived timestep makes three of the five built-in scenarios
// diverge within a few frames — but not the two slowest-moving ones, where a
// parts-per-million perturbation stays below the quantisation floor for the
// length of the run. Three failures is still a failed gate, which is the
// behaviour that matters; the two that pass are the honest limit of the method,
// and they are the reason the scenario set includes a "fast" case at all.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sat {

/// Outcome of comparing two runs of the same configuration.
struct ReproResult {
    std::string scenario;
    uint64_t    seed        = 0;
    int64_t     frames      = 0;
    bool        identical   = true;

    /// First frame where the two runs disagreed, or -1. Reported because
    /// "diverged at frame 0" and "diverged at frame 2847" are completely
    /// different bugs — the first is configuration, the second is accumulation.
    int64_t     first_divergence = -1;

    /// Which component diverged first. FrameFingerprint is split precisely so
    /// this question has an answer (design §2, core/hash.hpp).
    std::string diverged_field;

    /// Digest over every frame's combined hash. This is the number the CI
    /// matrix compares across optimisation levels and machines.
    uint64_t    digest = 0;

    // -----------------------------------------------------------------------
    // The scalar/vector arm — P1-1.
    //
    // The damage chain dispatches to AVX2 at RUNTIME
    // (degrade/sensor_simd.cpp), which is right for a single shipped binary
    // and is also the exact hazard INV-3 exists to catch: different machines
    // execute different code. Running this gate twice on one machine took the
    // same path both times and said nothing about the other one.
    //
    // So every pair is also run once with the vector path forced off and the
    // digests compared. On a machine with AVX2 that is a genuine two-path
    // comparison. On a machine without, `simd_compared` is false and the row
    // says so rather than claiming a check it could not perform — a gate that
    // silently does nothing is worse than no gate, because it reads as a green
    // tick.
    // -----------------------------------------------------------------------
    bool     simd_compared = false;   ///< was there a vector path to compare?
    bool     simd_matches  = true;    ///< scalar digest == vector digest
    uint64_t scalar_digest = 0;
};

/// Per-scenario checks that need the whole seed sweep to evaluate.
struct SeedSensitivity {
    std::string scenario;
    /// Distinct digests across the seeds swept. Anything less than the number
    /// of seeds means two different seeds produced identical output, i.e. the
    /// run has no live stochastic component for the seed to reach.
    size_t distinct = 0;
    size_t seeds    = 0;
    [[nodiscard]] bool ok() const noexcept { return seeds < 2 || distinct == seeds; }
};

/// Run the built-in scenario set twice each and compare.
///
/// `seeds` are the master seeds to sweep. Returns one result per
/// (scenario, seed) pair.
[[nodiscard]] std::vector<ReproResult> verify_reproducibility(
    const std::vector<uint64_t>& seeds, double duration_s = 2.0);

/// Group the results by scenario and count distinct digests per scenario.
///
/// THE CHECK THAT WOULD HAVE CAUGHT THE HOLLOW GATE. For months this tool
/// printed, on every run:
///
///     static   1  IDENTICAL  6e5ea6926c621cbf
///     static   2  IDENTICAL  6e5ea6926c621cbf     <- same digest, other seed
///
/// Four of the five scenarios were seed-invariant, because the sensor model
/// was never enabled and there was nothing stochastic left for the seed to
/// reach. Every row said IDENTICAL, which is what the tool was looking for,
/// and the evidence that it was checking nothing was printed in the adjacent
/// column with nobody comparing it.
///
/// Determinism and sensitivity are opposite failures and a gate needs both:
/// repeats must agree, and different seeds must NOT.
[[nodiscard]] std::vector<SeedSensitivity> seed_sensitivity(
    const std::vector<ReproResult>& results);

/// Print the results as a table and return a process exit code:
/// 0 if every check passed, 1 otherwise.
[[nodiscard]] int report_reproducibility(const std::vector<ReproResult>& results);

}  // namespace sat
