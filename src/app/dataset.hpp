// app/dataset.hpp — tracks-only --gen-dataset (SAT-ML.md §2, SAT-DESIGN 13.4).
//
// This is the data factory for MotionNet. It lives in sat_app so perception,
// tracking and sat_ai never include world/ or consume FrameTruth (INV-1).
// Centroid/candidate capture is still listed as remaining work; this branch
// ships tracks first because the assignment is MotionNet.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sat {

struct DatasetOptions {
    std::string scenario_path;          ///< single-scenario capture
    std::string sweep_path;             ///< existing SweepSpec: one base + seeds
    std::string out_dir = "data/motion_v1";
    std::string task = "tracks";        ///< only "tracks" is implemented
    double duration_s = 0.0;            ///< 0 keeps the scenario's duration
    std::vector<uint64_t> seeds;        ///< empty + no sweep => {1}
};

/// Run the capture. Returns a process exit code.
[[nodiscard]] int run_gen_dataset(const DatasetOptions& opt);

/// argv parsing for `--gen-dataset`.
[[nodiscard]] int gen_dataset_command(int argc, char* argv[], int& i);

/// Official row 12 -> MotionNet regime id. First stacked motion wins.
[[nodiscard]] int regime_from_kind(const std::string& kind) noexcept;

}  // namespace sat
