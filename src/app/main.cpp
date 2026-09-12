// Entry point for sat-tracker.
//
// Checkpoint 0.1 only needs a runnable binary that prints a version string.
// Later checkpoints grow this into CLI parsing, scenario load, and the loop.

#include "sat/version.hpp"

#include <cstdio>

int main(int /*argc*/, char* /*argv*/[]) {
    // Acceptance for CP 0.1: print the version so `cmake --build` yields a
    // binary you can run and verify immediately.
    std::printf("%s\n", SAT_PRODUCT_NAME);
    std::printf("version %s\n", SAT_VERSION);
    return 0;
}