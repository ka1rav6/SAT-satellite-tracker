// ai/motion_net.hpp — SAT-ML §6 MotionNet ONNX wrapper (INV-3, INV-5, INV-7).
//
// The net forecasts 15 angular steps from 30 tracker states. It never sees
// pixels (INV-5) and never sees FrameTruth (INV-1). A missing or corrupt
// file is an error string; Pipeline leaves the optional empty and IMM runs.

#pragma once

#include "core/result.hpp"
#include "core/units.hpp"
#include "tracking/imm.hpp"

#include <array>
#include <memory>
#include <string>

namespace sat {

struct MotionForecast {
    bool   valid = false;          ///< false => caller uses IMM
    Angle2 step_urad[15]{};        ///< residual+CV, delta from current az/el
    float  regime_logit[4]{};
    MotionRegime regime = MotionRegime::Line;
    float  confidence = 0.0f;

    /// The camera was not running at the rate the net was trained at, so the
    /// learned residual is out of distribution. The forecast is still
    /// composed and returned — a caller may want the regime head, which is a
    /// classification and degrades far more gracefully than the residual —
    /// but Pipeline refuses to steer the search with it. See kTrainingDtS.
    bool   off_training_rate = false;
};

class MotionNet {
public:
    MotionNet();
    MotionNet(MotionNet&&) noexcept;
    MotionNet& operator=(MotionNet&&) noexcept;
    ~MotionNet();

    /// Missing / corrupt ONNX, or a build without SAT_HAVE_ONNX, returns Err.
    static Result<MotionNet> load(const std::string& path);

    /// The camera rate the network was trained at, in seconds per frame.
    ///
    /// ml/models/motion.py's DT_S. It is BOTH the step the constant-velocity
    /// term is composed with AND the sampling rate the GRU's history was
    /// drawn at, so a scenario at another rate is out of distribution twice
    /// over. The two are separated below: the CV term is recomposed at the
    /// caller's real dt, which is exact arithmetic and always worth doing,
    /// and the residual is flagged, which is a judgement the caller makes.
    static constexpr float kTrainingDtS = 1.0f / 30.0f;

    /// hist is oldest→newest raw [az, el, vaz, vel] in µrad; `dt_s` is the
    /// caller's real seconds-per-frame.
    ///
    /// NOT allocation-free: ONNX Runtime allocates inside Run() (it returns
    /// an owning std::vector<Ort::Value>) and MemoryInfo::CreateCpu allocates
    /// too. An earlier version of this header claimed otherwise. The audit
    /// flags this in §22.4 ("INV-4 vs ORT") and the fix is IO binding, which
    /// is not done yet — so the honest statement is here rather than a
    /// comfortable one, and INV-4's Debug allocation trap deliberately does
    /// NOT cover the inference call.
    [[nodiscard]] MotionForecast run(const float hist[30][4], float dt_s) noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sat
