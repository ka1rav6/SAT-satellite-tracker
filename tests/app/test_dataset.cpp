// tests/app/test_dataset.cpp — tracks-only --gen-dataset (SAT-ML.md §2 / §6.5).
//
// Labels live in sat_app. tracking/ and ai/ never see FrameTruth (INV-1).

#include <doctest/doctest.h>

#include "app/dataset.hpp"
#include "scenario/schema.hpp"

#include <filesystem>
#include <fstream>
#include <string>

using namespace sat;

namespace {

std::string tmp_dir() {
#ifdef SAT_TEST_TMP_DIR
    return std::string(SAT_TEST_TMP_DIR) + "/motion_tracks";
#else
    return "motion_tracks_test";
#endif
}

}  // namespace

TEST_CASE("tracks-only gen-dataset writes a CSV of tracker state plus truth") {
    DatasetOptions opt;
    opt.scenario_path = std::string(SAT_SCENARIO_DIR) + "/control/figure8.toml";
    opt.out_dir = tmp_dir();
    opt.task = "tracks";
    opt.duration_s = 2.0;   // 60 frames — enough for a 30+15 window later
    opt.seeds = {1};

    const int rc = run_gen_dataset(opt);
    REQUIRE(rc == 0);

    const std::filesystem::path csv = std::filesystem::path(opt.out_dir) / "raw" / "figure8_1.csv";
    REQUIRE(std::filesystem::exists(csv));

    std::ifstream in(csv);
    REQUIRE(in);
    std::string header;
    REQUIRE(std::getline(in, header));
    CHECK(header.find("track_az_urad") != std::string::npos);
    CHECK(header.find("truth_az_urad") != std::string::npos);
    CHECK(header.find("regime") != std::string::npos);

    int rows = 0;
    for (std::string line; std::getline(in, line); ) {
        if (!line.empty()) ++rows;
    }
    CHECK(rows >= 60);
}

TEST_CASE("tracks-only gen-dataset rejects an unknown task") {
    DatasetOptions opt;
    opt.scenario_path = std::string(SAT_SCENARIO_DIR) + "/control/figure8.toml";
    opt.out_dir = tmp_dir() + "_bad";
    opt.task = "centroid_patches";
    opt.duration_s = 1.0;
    opt.seeds = {1};
    CHECK(run_gen_dataset(opt) != 0);
}
