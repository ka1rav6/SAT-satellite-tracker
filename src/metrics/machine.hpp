// metrics/machine.hpp — what hardware produced a number (audit P3-5).
//
// ---------------------------------------------------------------------------
// WHY THIS EXISTS
// ---------------------------------------------------------------------------
// The critical audit's P3-5 is one line — "performance numbers quoted without
// a machine specification" — and it is the kind of gap that costs credibility
// rather than marks. Every throughput figure this project publishes ("21.5 FPS
// synthetic", "30.8 FPS on video", "0.85 ms frame budget") is a statement
// about a CPU, and until now `run.json` recorded only `hardware_threads`.
//
// Four is not a machine specification. A 4-thread figure could be a laptop
// throttling on battery or a server slice, and the difference is a factor of
// three on exactly the rows Benchmark Performance-2 is scored on. A reader
// months later — or an evaluator asking "on what?" — needs the CPU, the build
// type and whether the vector damage chain was actually taken, because a run
// that silently fell back to the scalar path is 4x slower and looks like a
// regression in the tracker.
//
// This is deliberately NOT a benchmark or a scoring input. Nothing here feeds
// the simulation, nothing enters the reproducibility fingerprint, and the
// fields are free-form strings: their job is to let a human reconstruct the
// conditions, not to be compared programmatically.
//
// ---------------------------------------------------------------------------
// THE ONE THING IT MUST NOT DO
// ---------------------------------------------------------------------------
// It must not fail, and it must not block. Every lookup here degrades to
// "unknown" rather than throwing or returning an error: a provenance record
// that refuses to be written because /proc was not mounted would trade a
// missing string for a lost run.

#pragma once

#include <string>

namespace sat {

// ---------------------------------------------------------------------------
// MachineSpec — read once per run, cheap, and never in the frame loop.
// ---------------------------------------------------------------------------
struct MachineSpec {
    std::string cpu;            ///< model name, e.g. "Intel Core Ultra 7 256V"
    std::string os;             ///< "Linux", "Windows", "macOS"
    std::string compiler;       ///< "gcc 13.2.0", "clang 17.0.6", "MSVC 19.38"
    std::string build_type;     ///< "Release", "Debug", "RelWithDebInfo"
    unsigned    hardware_threads = 0;
    bool        avx2_damage_chain = false;  ///< the PATH TAKEN, not the CPU's
                                            ///< capability: --no-simd makes
                                            ///< these differ, and the timing
                                            ///< follows the path

    /// One line for a terminal summary, e.g.
    ///   "Intel Core Ultra 7 256V, 8 threads, AVX2, Linux, gcc 13.2.0, Release"
    [[nodiscard]] std::string one_line() const;
};

/// Probe the host. Every field degrades to "unknown" rather than failing.
[[nodiscard]] MachineSpec probe_machine();

}  // namespace sat
