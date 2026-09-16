// app/fuzz.hpp — CP 14.1's scenario fuzzer.
//
// `sat-tracker --fuzz-scenarios 5000`
//
// ---------------------------------------------------------------------------
// WHAT IT IS FOR
// ---------------------------------------------------------------------------
// §14's criterion: "5,000 random scenarios sampling every parameter across its
// legal range, INCLUDING CORNERS: no crash, no hang, no NaN in any logged
// value."
//
// Every scenario in scenarios/ was written by someone who knew what the system
// does. That is exactly why they do not find this class of bug. A zero-duration
// run, a beacon larger than the field of view, a camera whose field of view is
// a thousandth of a degree, one truth tick per camera frame, a target starting
// exactly on a boundary — all of them are LEGAL by the schema, and none of them
// is a configuration anyone would think to write.
//
// The bar is deliberately low and absolute. Not "produces a good result" —
// most of these configurations have no good result — but "does not crash, does
// not hang, and does not put a NaN in a number that gets reported". A NaN is
// singled out because it is the failure that propagates: it survives every
// comparison, poisons every average it enters, and shows up three stages later
// as a metric nobody can explain.
//
// ---------------------------------------------------------------------------
// WHY IT SAMPLES CORNERS EXPLICITLY
// ---------------------------------------------------------------------------
// Uniform sampling over a range almost never hits its endpoints, and the
// endpoints are where the bugs are: a division by a value that is legally zero,
// a loop bound that is legally one, an array indexed by a size that is legally
// its own maximum. So a fixed fraction of draws take a parameter's minimum or
// maximum rather than a uniform sample, which is what "including corners" in
// the checkpoint means.

#pragma once

#include <cstdint>

namespace sat {

struct FuzzOptions {
    int      count   = 5000;
    uint64_t seed    = 1;
    /// Simulated seconds per scenario. Short on purpose: this is looking for
    /// crashes and NaNs, both of which appear in the first frames if they
    /// appear at all, and 5,000 runs of any length is the binding constraint.
    double   duration_s = 0.5;
    /// Print every scenario as it runs rather than only the failures.
    bool     verbose = false;
};

/// Returns a process exit code: 0 if every scenario survived.
[[nodiscard]] int run_fuzz(const FuzzOptions& opt);

/// argv parsing for `--fuzz-scenarios`.
[[nodiscard]] int fuzz_command(int argc, char* argv[], int& i);

}  // namespace sat
