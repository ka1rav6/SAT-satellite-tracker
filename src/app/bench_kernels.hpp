// app/bench_kernels.hpp — `--bench-kernels`, the per-kernel microbenchmark.
//
// CP 14.2 asks for AVX2 kernels that are "bit-identical to scalar on random
// inputs" with "total frame time under 1 ms". Neither half of that can be
// worked on with `--headless --stages` alone:
//
//   * a whole-run stage table measures ONE call per frame, against a clock
//     whose resolution is the frame itself, on a machine that is also running
//     the rest of the frame. Run-to-run spread on this laptop is 1.5x, which is
//     larger than most of the optimisations being attempted;
//   * and it measures the kernel in situ, with whatever cache state the
//     previous stage happened to leave, which is the right number for the
//     budget and the wrong number for deciding whether a change helped.
//
// So this runs each kernel on its own, on a fixed synthetic frame, K times,
// and reports the MINIMUM — the run least disturbed by the operating system,
// which is the closest estimate of the kernel's own cost. The p50 of the whole
// pipeline is still the number design §15 is scored against; this is the
// instrument used to move it.

#pragma once

namespace sat {

struct BenchKernelsOptions {
    int  width       = 640;   ///< spec row 3's default sensor
    int  height      = 480;
    int  repeats     = 50;    ///< timed repetitions per kernel
    int  warmup      = 5;
    bool verify_simd = true;  ///< CP 14.2: compare every SIMD kernel to scalar
};

/// Run the benchmark and print the table. Returns a process exit code: nonzero
/// only if `verify_simd` found a kernel whose vector path disagrees with its
/// scalar one, which is a correctness failure and not a slow result.
[[nodiscard]] int run_bench_kernels(const BenchKernelsOptions& opt);

/// argv parsing for `--bench-kernels`.
[[nodiscard]] int bench_kernels_main(int argc, char** argv);

}  // namespace sat
