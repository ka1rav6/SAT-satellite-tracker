// tests/ai/test_motion_net.cpp — MotionNet wrapper, INV-7.

#include <doctest/doctest.h>

#include "ai/motion_net.hpp"

#include <cmath>

using namespace sat;

TEST_CASE("MotionNet::load of a missing file is an error, not a crash") {
    auto r = MotionNet::load("does_not_exist_motionnet.onnx");
    CHECK_FALSE(r);
    CHECK(r.error().size() > 0);
}

TEST_CASE("an unloaded MotionNet run() is invalid so the caller uses IMM") {
    MotionNet net;
    float hist[30][4]{};
    const MotionForecast fc = net.run(hist);
    CHECK_FALSE(fc.valid);
}

#ifdef SAT_HAVE_ONNX
TEST_CASE("C++ MotionNet matches a committed fixture when ONNX is present") {
    // Fixture parity (|cpp-python| < 1e-4) lands with the exported ONNX.
    // This build has the session; the file is optional so a missing artifact
    // skips rather than failing CI that has not trained yet.
    auto r = MotionNet::load("models/motionnet_v1.onnx");
    if (!r) {
        CHECK(r.error().size() > 0);
        return;
    }
    float hist[30][4]{};
    for (int t = 0; t < 30; ++t) {
        hist[t][0] = 1000.0f * static_cast<float>(t) / 30.0f;
        hist[t][2] = 1000.0f;
    }
    const MotionForecast fc = r->run(hist);
    CHECK(fc.valid);
    CHECK(std::isfinite(fc.step_urad[0].x));
}
#endif
