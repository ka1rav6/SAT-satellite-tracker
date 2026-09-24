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

// ===========================================================================
// SAT-ML §4.2's mandatory parity test — against a fixture that is committed
// ===========================================================================
//
// The §4.2 test above (`C++ MotionNet matches a committed fixture when ONNX is
// present`) loads models/motionnet_v1.onnx, which is a training artifact and
// is gitignored. In CI its early return is ALWAYS taken, so every line between
// normalise_hist and argmax_regime had no test that ran anywhere.
//
// tests/ai/motion_fixture.onnx closes that. It is NOT a trained model and must
// never be quoted as one: it has MotionNet's exact signature and a closed form
// chosen so the expected values below are arithmetic done in this file rather
// than numbers read back from a previous run of the code under test.
//
//     forecast[k] = (k+1, -(k+1))          a fixed residual, ignoring the input
//     logits[c]   = sum_t hist_norm[t][c]  per-channel sums of the NORMALISED input
//
// Regenerate with tools/make_motion_fixture.py.

#ifdef SAT_HAVE_ONNX
namespace {

/// Mirrors src/ai/motion_net.cpp's normalise_hist. Written out here on
/// purpose: a test that called the implementation would agree with it however
/// wrong it was.
void expected_norm(const float hist[30][4], float out[30][4]) {
    constexpr float kPosScale = 1.0e5f, kRateScale = 1.0e5f;
    const float last_az = hist[29][0], last_el = hist[29][1];
    for (int t = 0; t < 30; ++t) {
        out[t][0] = (hist[t][0] - last_az) / kPosScale;
        out[t][1] = (hist[t][1] - last_el) / kPosScale;
        out[t][2] = hist[t][2] / kRateScale;
        out[t][3] = hist[t][3] / kRateScale;
    }
}

}  // namespace

TEST_CASE("SAT-ML 4.2: the wrapper's compose, normalise and argmax are exact") {
    auto r = MotionNet::load(SAT_TEST_FIXTURE_DIR "/motion_fixture.onnx");
    REQUIRE_MESSAGE(r, r.error());

    // A ramp in azimuth at a constant rate, and a slower one in elevation, so
    // the two axes are never interchangeable.
    float hist[30][4]{};
    for (int t = 0; t < 30; ++t) {
        hist[t][0] = 3000.0f * static_cast<float>(t);   // az, urad
        hist[t][1] = -1000.0f * static_cast<float>(t);  // el, urad
        hist[t][2] = 90000.0f;                          // vaz, urad/s
        hist[t][3] = -30000.0f;                         // vel, urad/s
    }

    const float dt = MotionNet::kTrainingDtS;
    const MotionForecast fc = r->run(hist, dt);
    REQUIRE(fc.valid);
    CHECK_FALSE(fc.off_training_rate);

    // --- composition ------------------------------------------------------
    // step[k] must be the fixture's residual PLUS the constant-velocity term
    // at the caller's dt. This is the arithmetic the hardcoded 1/30 used to
    // get wrong on any other camera rate.
    for (int k = 0; k < 15; ++k) {
        const float step = static_cast<float>(k + 1) * dt;
        const float want_x = static_cast<float>(k + 1) + 90000.0f * step;
        const float want_y = -static_cast<float>(k + 1) - 30000.0f * step;
        CHECK(fc.step_urad[k].x == doctest::Approx(want_x).epsilon(1e-5));
        CHECK(fc.step_urad[k].y == doctest::Approx(want_y).epsilon(1e-5));
    }

    // --- normalisation ----------------------------------------------------
    // The fixture's logits ARE the per-channel sums of what the wrapper fed
    // the graph, so comparing them to the sums of an independently computed
    // normalisation pins normalise_hist end to end.
    float norm[30][4];
    expected_norm(hist, norm);
    for (int c = 0; c < 4; ++c) {
        double want = 0.0;
        for (int t = 0; t < 30; ++t) want += norm[t][c];
        CHECK(fc.regime_logit[c] == doctest::Approx(want).epsilon(1e-4));
    }

    // --- argmax and softmax ----------------------------------------------
    // Channel 2 (vaz/1e5 = 0.9, summed over 30 steps = 27) is the largest by a
    // wide margin, so the regime is index 2 and the softmax is saturated.
    CHECK(fc.regime == MotionRegime::Figure8);   // enum index 2
    CHECK(fc.confidence > 0.99f);
    CHECK(fc.confidence <= 1.0f);
}

TEST_CASE("SAT-ML 4.2: a different dt changes the composition and raises the flag") {
    auto r = MotionNet::load(SAT_TEST_FIXTURE_DIR "/motion_fixture.onnx");
    REQUIRE_MESSAGE(r, r.error());

    float hist[30][4]{};
    for (int t = 0; t < 30; ++t) {
        hist[t][0] = 3000.0f * static_cast<float>(t);
        hist[t][2] = 90000.0f;
    }

    const float dt60 = 1.0f / 60.0f;
    const MotionForecast fc = r->run(hist, dt60);
    REQUIRE(fc.valid);

    // The flag is the honest half: the CV term below is now right, but the
    // residual was learned at 30 Hz and cannot be rescaled, so Pipeline
    // declines to steer on this forecast.
    CHECK(fc.off_training_rate);

    // And the CV term really did follow dt rather than a baked-in 1/30.
    for (int k = 0; k < 15; ++k) {
        const float want =
            static_cast<float>(k + 1) + 90000.0f * static_cast<float>(k + 1) * dt60;
        CHECK(fc.step_urad[k].x == doctest::Approx(want).epsilon(1e-5));
    }

    // Same input at the trained rate must give a DIFFERENT answer, or the
    // check above would pass on a wrapper that ignored dt in both calls.
    const MotionForecast at30 = r->run(hist, MotionNet::kTrainingDtS);
    CHECK(at30.step_urad[14].x != doctest::Approx(fc.step_urad[14].x));
}
#endif  // SAT_HAVE_ONNX
