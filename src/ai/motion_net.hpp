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
};

class MotionNet {
public:
    MotionNet();
    MotionNet(MotionNet&&) noexcept;
    MotionNet& operator=(MotionNet&&) noexcept;
    ~MotionNet();

    /// Missing / corrupt ONNX, or a build without SAT_HAVE_ONNX, returns Err.
    static Result<MotionNet> load(const std::string& path);

    /// hist is oldest→newest raw [az, el, vaz, vel] in µrad. Allocation-free.
    [[nodiscard]] MotionForecast run(const float hist[30][4]) noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sat
