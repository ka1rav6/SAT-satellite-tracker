// app/calibrate.cpp — CP 9.2 and CP 9.3's measurement run.
//
// `sat-tracker --calibrate-centroid [--out DIR] [--offsets N]`
//
// Runs §10.1.3's procedure end to end: 200 sub-pixel offsets x 6 sizes x 8 SNR
// bins x each estimator, fits the S-curve per cell, and writes the table as a
// C++ fragment for compiling in.
//
// ---------------------------------------------------------------------------
// WHY THE GENERATOR LIVES IN THE SHIPPING BINARY
// ---------------------------------------------------------------------------
// It would be a natural thing to do in a script. Doing it here means the
// numbers cannot be produced by a different code path from the one that uses
// them: the same estimators, the same median filter, the same top-hat, the same
// exact-coverage splat. A table measured against a re-implementation of the
// pipeline would be a correction for a system that does not exist, and the
// error it introduced would look exactly like the error it was meant to remove.

#include "app/calibrate.hpp"

#include "metrics/centroid_harness.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>

namespace sat {

int calibrate_centroid_command(int argc, char* argv[], int& i) {
    HarnessParams p;
    p.offsets = 200;                  // §10.1.3 step 1
    std::string out_dir = "src/perception/centroid";
    bool quiet = false;

    for (int k = i + 1; k < argc; ++k) {
        const char* a = argv[k];
        if      (std::strcmp(a, "--out") == 0 && k + 1 < argc)     out_dir = argv[++k];
        else if (std::strcmp(a, "--offsets") == 0 && k + 1 < argc) p.offsets = std::atoi(argv[++k]);
        else if (std::strcmp(a, "--quiet") == 0)                   quiet = true;
        else {
            std::fprintf(stderr,
                         "sat-tracker: --calibrate-centroid: unrecognised option '%s'\n", a);
            return 2;
        }
        i = k;
    }

    if (!quiet) {
        std::printf("Measuring the centroid S-curve: %d offsets x %d sizes x %d SNR bins\n",
                    p.offsets, kBiasSizes, kBiasSnr);
        std::printf("(design section 10.1.3 — this takes a minute)\n\n");
    }

    const std::vector<CentroidAccuracy> before = measure_grid(p);
    const BiasTable table = fit_table(before);

    HarnessParams q = p;
    q.correct_bias = true;
    q.table = &table;
    const std::vector<CentroidAccuracy> after = measure_grid(q);

    // ------------------------------------------------------------------
    // The before/after table §10.1.3 asks to be put in the report, printed
    // here so a calibration run is self-documenting.
    // ------------------------------------------------------------------
    if (!quiet) {
        std::printf("%-14s %4s %6s   %8s %8s %7s   %8s %8s\n",
                    "estimator", "size", "snr", "rmse", "rmse'", "gain", "bias", "bias'");
        std::printf("%s\n", std::string(78, '-').c_str());
        for (size_t k = 0; k < before.size() && k < after.size(); ++k) {
            const CentroidAccuracy& b = before[k];
            const CentroidAccuracy& a = after[k];
            std::printf("%-14s %4d %6.1f   %8.4f %8.4f %6.2fx   %+8.4f %+8.4f\n",
                        centroid_kind_name(b.kind), b.size_px,
                        static_cast<double>(b.snr),
                        b.rmse_px, a.rmse_px,
                        a.rmse_px > 0.0 ? b.rmse_px / a.rmse_px : 0.0,
                        b.bias_px, a.bias_px);
        }
    }

    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    const std::string path = out_dir + "/bias_table_generated.inc";
    std::ofstream f(path, std::ios::binary);
    if (!f) {
        std::fprintf(stderr, "sat-tracker: cannot write '%s'\n", path.c_str());
        return 1;
    }
    f << table.to_source();
    if (!quiet) {
        std::printf("\nWrote %s\n", path.c_str());
        std::printf("Rebuild to compile the table in, then re-run to confirm the gain.\n");
    }
    return 0;
}

}  // namespace sat
