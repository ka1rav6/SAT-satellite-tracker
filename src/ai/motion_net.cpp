// ai/motion_net.cpp — 1-thread sequential BASIC session (SAT-DESIGN §11.1).

#include "ai/motion_net.hpp"

#include <cmath>
#include <cstring>
#include <vector>

#ifdef SAT_HAVE_ONNX
#include <onnxruntime_cxx_api.h>
#endif

namespace sat {
namespace {

#ifdef SAT_HAVE_ONNX
// Baked from models/motion_norm.json (train-split). Do not "improve" these
// without regenerating that file from the real train shards.
constexpr float kPosScale = 1.0e5f;
constexpr float kRateScale = 1.0e5f;
constexpr float kDt = 1.0f / 30.0f;
constexpr int kHist = 30;
constexpr int kHorizon = 15;

void normalise_hist(const float hist[30][4], float* out) noexcept {
    const float last_az = hist[kHist - 1][0];
    const float last_el = hist[kHist - 1][1];
    for (int t = 0; t < kHist; ++t) {
        out[t * 4 + 0] = (hist[t][0] - last_az) / kPosScale;
        out[t * 4 + 1] = (hist[t][1] - last_el) / kPosScale;
        out[t * 4 + 2] = hist[t][2] / kRateScale;
        out[t * 4 + 3] = hist[t][3] / kRateScale;
    }
}

void compose_forecast(const float hist[30][4], const float* residual,
                      MotionForecast& fc) noexcept {
    const float vaz = hist[kHist - 1][2];
    const float vel = hist[kHist - 1][3];
    for (int k = 0; k < kHorizon; ++k) {
        const float step = static_cast<float>(k + 1) * kDt;
        fc.step_urad[k] = Angle2{residual[k * 2 + 0] + vaz * step,
                                 residual[k * 2 + 1] + vel * step};
    }
}

MotionRegime argmax_regime(const float* logits, float& confidence) noexcept {
    float maxv = logits[0];
    int arg = 0;
    for (int i = 1; i < 4; ++i) {
        if (logits[i] > maxv) {
            maxv = logits[i];
            arg = i;
        }
    }
    float sum = 0.0f;
    float shifted[4];
    for (int i = 0; i < 4; ++i) {
        shifted[i] = std::exp(logits[i] - maxv);
        sum += shifted[i];
    }
    confidence = (sum > 0.0f) ? shifted[arg] / sum : 0.25f;
    return static_cast<MotionRegime>(arg);
}
#endif

}  // namespace

struct MotionNet::Impl {
#ifdef SAT_HAVE_ONNX
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "sat_motion"};
    Ort::SessionOptions opts;
    std::unique_ptr<Ort::Session> session;
#endif
    bool ready = false;
};

MotionNet::MotionNet() : impl_(std::make_unique<Impl>()) {}
MotionNet::MotionNet(MotionNet&&) noexcept = default;
MotionNet& MotionNet::operator=(MotionNet&&) noexcept = default;
MotionNet::~MotionNet() = default;

Result<MotionNet> MotionNet::load(const std::string& path) {
#ifndef SAT_HAVE_ONNX
    (void)path;
    return Err("ONNX Runtime was not compiled in; MotionNet stays empty (INV-7)");
#else
    if (path.empty()) return Err("MotionNet path is empty");
    MotionNet net;
    net.impl_->opts.SetIntraOpNumThreads(1);
    net.impl_->opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);
    net.impl_->opts.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    try {
#ifdef _WIN32
        const std::wstring wpath(path.begin(), path.end());
        net.impl_->session = std::make_unique<Ort::Session>(
            net.impl_->env, wpath.c_str(), net.impl_->opts);
#else
        net.impl_->session = std::make_unique<Ort::Session>(
            net.impl_->env, path.c_str(), net.impl_->opts);
#endif
        net.impl_->ready = true;
    } catch (const std::exception& e) {
        return Err(std::string("MotionNet failed to load: ") + e.what());
    }
    return net;
#endif
}

MotionForecast MotionNet::run(const float hist[30][4]) noexcept {
    MotionForecast fc{};
#ifndef SAT_HAVE_ONNX
    (void)hist;
    return fc;
#else
    if (!impl_ || !impl_->ready || !impl_->session) return fc;
    float norm[kHist * 4];
    normalise_hist(hist, norm);
    try {
        Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        const int64_t ishape[3] = {1, kHist, 4};
        Ort::Value input = Ort::Value::CreateTensor<float>(
            mem, norm, static_cast<size_t>(kHist * 4), ishape, 3);
        const char* in_names[] = {"hist"};
        const char* out_names[] = {"forecast", "logits"};
        auto outs = impl_->session->Run(Ort::RunOptions{nullptr},
                                        in_names, &input, 1, out_names, 2);
        const float* residual = outs[0].GetTensorData<float>();
        const float* logits = outs[1].GetTensorData<float>();
        compose_forecast(hist, residual, fc);
        for (int i = 0; i < 4; ++i) fc.regime_logit[i] = logits[i];
        fc.regime = argmax_regime(logits, fc.confidence);
        fc.valid = true;
    } catch (...) {
        fc.valid = false;
    }
    return fc;
#endif
}

}  // namespace sat
