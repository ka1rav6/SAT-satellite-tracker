// app/headless.hpp — CP 7.3 and the entry point every later stage builds on.
//
// `sat-tracker --headless --scenario s.toml --out logs/`
//
// This is the command the sweep at CP 7.5 forks, the command the compliance
// matrix at CP 7.7 aggregates, and — via --video — the Benchmark
// Performance-2 entry point of §13.4 that "must work bare". Everything Stage 7
// produces comes out of this one function, which is why it is a library
// function taking a struct rather than argv-parsing inline in main.

#pragma once

#include "scenario/scenario.hpp"

#include <cstdint>
#include <string>

namespace sat {

struct HeadlessOptions {
    std::string scenario_path;
    std::string out_dir = "logs";

    /// Override the scenario's own seed. -1 means "use the scenario's".
    long long seed_override = -1;
    /// Override the scenario's duration, seconds. <= 0 means "use the scenario's".
    double duration_override = 0.0;

    /// INV-7's switch. Reported in every artifact, because a number produced
    /// with a model in the loop and one produced without are different claims.
    bool no_ai = false;

    /// Write centroid.csv and run.json. CP 7.5's sweep wants them; the `--bench`
    /// path does not, and measuring the simulation with a per-frame fprintf in
    /// the loop would be measuring the fprintf.
    bool write_artifacts = true;

    /// Publish snapshots, which is what produces the INV-3 fingerprint. Costs
    /// a frame copy (~300 KB) per frame, so it is off for a pure speed run and
    /// on when provenance matters.
    bool fingerprint = true;

    /// Print the §13.1 summary block to stdout.
    bool quiet = false;
};

/// Run it. Returns a process exit code: 0 on success, non-zero on a scenario
/// that will not load or an artifact that cannot be written.
[[nodiscard]] int run_headless(const HeadlessOptions& opt);

/// argv parsing for `--headless`. Separated so the options struct can be
/// exercised by a test without building a char* array.
[[nodiscard]] int headless_command(int argc, char* argv[], int& i);

}  // namespace sat
