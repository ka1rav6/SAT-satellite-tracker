// tests/ai/test_motion_in_loop.cpp — Pipeline B19, INV-7.

#include <doctest/doctest.h>

#include "engine/pipeline.hpp"
#include "scenario/schema.hpp"

#include <string>

using namespace sat;

TEST_CASE("--no-ai still completes a headless figure-8 (INV-7)") {
    auto loaded = load_scenario(std::string(SAT_SCENARIO_DIR) + "/control/figure8.toml");
    REQUIRE(loaded);
    Scenario sc = *loaded;
    sc.ai_enabled = false;
    sc.motion_net.clear();
    sc.duration_s = 2.0;
    sc.seed = 1;

    Pipeline pipe;
    pipe.build_from_scenario(sc);
    pipe.set_publish_snapshots(false);
    CHECK_FALSE(pipe.motion_net_loaded());
    int frames = 0;
    while (pipe.step()) ++frames;
    CHECK(frames >= 60);
}

TEST_CASE("a missing ai.motion_net file still completes (INV-7)") {
    auto loaded = load_scenario(std::string(SAT_SCENARIO_DIR) + "/control/figure8.toml");
    REQUIRE(loaded);
    Scenario sc = *loaded;
    sc.ai_enabled = true;
    sc.motion_net = "models/does_not_exist_motionnet.onnx";
    sc.duration_s = 2.0;
    sc.seed = 1;

    Pipeline pipe;
    pipe.build_from_scenario(sc);
    pipe.set_publish_snapshots(false);
    CHECK_FALSE(pipe.motion_net_loaded());
    int frames = 0;
    while (pipe.step()) ++frames;
    CHECK(frames >= 60);
    CHECK(pipe.motion_prior_applies() == 0);
}

#ifdef SAT_HAVE_ONNX
TEST_CASE("a loaded MotionNet reaches set_regime_prior on a locked figure-8") {
    auto loaded = load_scenario(std::string(SAT_SCENARIO_DIR) + "/ml/motion_figure8.toml");
    REQUIRE(loaded);
    Scenario sc = *loaded;
    sc.ai_enabled = true;
    sc.motion_net = "models/motionnet_v1.onnx";
    sc.duration_s = 4.0;
    sc.seed = 1;
    Pipeline pipe;
    pipe.build_from_scenario(sc);
    if (!pipe.motion_net_loaded()) return;
    pipe.set_publish_snapshots(false);
    while (pipe.step()) {}
    CHECK(pipe.motion_prior_applies() > 0);
}

TEST_CASE("an occluded figure-8 actually consults the coast forecast") {
    auto loaded = load_scenario(std::string(SAT_SCENARIO_DIR) + "/ml/motion_figure8.toml");
    REQUIRE(loaded);
    Scenario sc = *loaded;
    sc.ai_enabled = true;
    sc.motion_net = "models/motionnet_v1.onnx";
    sc.duration_s = 8.0;
    sc.seed = 1;
    Pipeline pipe;
    pipe.build_from_scenario(sc);
    if (!pipe.motion_net_loaded()) return;
    pipe.set_publish_snapshots(false);
    while (pipe.step()) {}
    CHECK(pipe.motion_coast_checks() > 0);
    MESSAGE("coast checks ", pipe.motion_coast_checks(),
            " uses ", pipe.motion_coast_uses());
}
#endif
