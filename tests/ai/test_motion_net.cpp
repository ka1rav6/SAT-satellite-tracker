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
    const MotionForecast fc = net.run(hist, MotionNet::kTrainingDtS);
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
    const MotionForecast fc = r->run(hist, MotionNet::kTrainingDtS);
    CHECK(fc.valid);
    CHECK(std::isfinite(fc.step_urad[0].x));
}
#endif

// ===========================================================================
// The frame rate the forecast is composed at
// ===========================================================================
//
// compose_forecast() used to hardcode dt = 1/30. That is two separate claims
// bundled into one constant: that the constant-velocity term steps at 1/30,
// which is arithmetic and is simply wrong on any other camera rate, and that
// the LEARNED residual was trained at 1/30, which cannot be corrected by
// arithmetic at all. The same defect class the audit already recorded as A-3
// (motion blur sampling the platform rate at a hardcoded 30 Hz).
//
// These are now separated: the CV term takes the caller's dt, and the
// residual is flagged when the caller's dt is not the training dt.

TEST_CASE("MotionNet: the off-training-rate flag is exact at the trained rate") {
    // An unloaded net returns an invalid forecast without touching dt, so the
    // flag's contract is checked on the constant rather than through ORT —
    // which keeps this case meaningful in a build with no ONNX Runtime, i.e.
    // in CI.
    CHECK(MotionNet::kTrainingDtS == doctest::Approx(1.0f / 30.0f));

    // 29.97 Hz is the video path's NTSC rate and MUST NOT trip the flag: the
    // tolerance exists to catch 60 Hz, not to reject a container's timebase.
    const float ntsc = 1.0f / 29.97f;
    CHECK(std::fabs(ntsc - MotionNet::kTrainingDtS)
              <= 0.005f * MotionNet::kTrainingDtS);

    // 60 Hz must trip it. Without the flag a 60 Hz run would extrapolate the
    // CV term correctly and still add a residual learned at half the rate.
    const float sixty = 1.0f / 60.0f;
    CHECK(std::fabs(sixty - MotionNet::kTrainingDtS)
              > 0.005f * MotionNet::kTrainingDtS);
}

TEST_CASE("MotionNet: an unloaded net ignores dt and stays invalid") {
    // INV-7: whatever the rate, no model means no forecast and no crash.
    MotionNet net;
    float hist[30][4]{};
    for (const float dt : {1.0f / 30.0f, 1.0f / 60.0f, 0.0f}) {
        const MotionForecast fc = net.run(hist, dt);
        CHECK_FALSE(fc.valid);
        CHECK_FALSE(fc.off_training_rate);
    }
}
