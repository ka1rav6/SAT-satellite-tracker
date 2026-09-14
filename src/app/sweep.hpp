// app/sweep.hpp — CP 7.5 and the ★ CP 7.7 gate.
//
// `sat-tracker --sweep sweep.toml --jobs 8 --out results/`
//
// ---------------------------------------------------------------------------
// WHY SEPARATE PROCESSES RATHER THAN THREADS
// ---------------------------------------------------------------------------
// §13.4 says "forking N worker processes", and the reason is not parallelism —
// threads would give that more cheaply. It is ISOLATION, and three properties
// follow from it that a thread pool cannot offer:
//
//   A run that crashes, hangs or exhausts memory takes down one worker. The
//   sweep reports that configuration as failed and carries on. In a thread
//   pool it takes the whole sweep with it, which on a 500-run job means losing
//   everything to the one configuration that was most worth knowing about.
//
//   Every run starts from a genuinely clean process: fresh arenas, fresh
//   statics, no chance of one run's state leaking into the next. INV-3 claims
//   bit-exact reproducibility per run, and a sweep in which run 400 sees
//   anything left by run 399 quietly weakens that claim.
//
//   It is the same mechanism CP 8.8 already uses to probe awkward video clips,
//   for the same reason — a corrupt clip that deadlocks FFmpeg's decoder has
//   to be survivable.
//
// Workers are spawned by re-executing this binary with `--headless`, rather
// than by fork(), so the same code path works on the Windows MSVC build that
// CI covers. A thread pool in the parent owns at most `jobs` live children.

#pragma once

#include "core/result.hpp"
#include "metrics/collector.hpp"
#include "scenario/overlay.hpp"
#include "scenario/sweep_spec.hpp"

#include <string>
#include <vector>

namespace sat {

// ---------------------------------------------------------------------------
// One point of the sweep, and its outcome.
// ---------------------------------------------------------------------------
struct SweepRun {
    std::vector<Override> overrides;  ///< what makes this run different
    uint64_t    seed = 0;
    std::string label;                ///< human-readable, for the matrix rows
    std::string dir;                  ///< where its artifacts went

    bool       ok = false;            ///< did the worker finish and write run.json
    int        exit_code = 0;         ///< the worker's exit status, for diagnosis
    std::string failure;              ///< why it was counted as failed
    RunMetrics metrics{};
};

struct SweepOptions {
    std::string spec_path;
    std::string out_dir = "results";
    int         jobs    = 0;          ///< 0 = hardware concurrency
    bool        quiet   = false;
    /// Keep each run's centroid.csv. A 500-run sweep writes 500 of them
    /// (~2 MB each), which is usually not what anyone wants from a sweep; the
    /// metrics in run.json are. Off by default, and the flag says so.
    bool        keep_csv = false;
};

/// Run the whole sweep. Returns a process exit code.
[[nodiscard]] int run_sweep(const SweepOptions& opt);

/// argv parsing for `--sweep`.
[[nodiscard]] int sweep_command(int argc, char* argv[], int& i);

/// Expand a spec into its run list. Separated so it can be tested without
/// spawning anything.
[[nodiscard]] std::vector<SweepRun> expand(const SweepSpec& spec,
                                           const std::string& out_dir);

}  // namespace sat
