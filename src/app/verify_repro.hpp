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
};

/// Run the built-in scenario set twice each and compare.
///
/// `seeds` are the master seeds to sweep. Returns one result per
/// (scenario, seed) pair.
[[nodiscard]] std::vector<ReproResult> verify_reproducibility(
    const std::vector<uint64_t>& seeds, double duration_s = 2.0);

/// Print the results as a table and return a process exit code:
/// 0 if every run was identical, 1 otherwise.
[[nodiscard]] int report_reproducibility(const std::vector<ReproResult>& results);

}  // namespace sat
