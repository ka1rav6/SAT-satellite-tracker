// core/profile.cpp — out-of-line parts of the timing helpers.

#include "core/profile.hpp"

#include <cmath>

namespace sat {

namespace {
// Precomputed so bucket_of() is one log and one multiply.
const double kLogMin   = std::log(LatencyHistogram::kMinUs);
const double kLogRange = std::log(LatencyHistogram::kMaxUs) - kLogMin;
}  // namespace

int LatencyHistogram::bucket_of(double us) noexcept {
    if (!(us > kMinUs)) return 0;                    // also catches NaN
    if (us >= kMaxUs)   return kBuckets - 1;
    const double f = (std::log(us) - kLogMin) / kLogRange;
    const int    i = static_cast<int>(f * kBuckets);
    return i < 0 ? 0 : (i >= kBuckets ? kBuckets - 1 : i);
}

double LatencyHistogram::bucket_centre(int i) noexcept {
    const double f = (static_cast<double>(i) + 0.5) / kBuckets;
    return std::exp(kLogMin + f * kLogRange);
}

const char* stage_name(Stage s) noexcept {
    switch (s) {
        case Stage::WorldAdvance:  return "world_advance";
        case Stage::Disturbance:   return "disturbance";
        case Stage::GimbalStep:    return "gimbal_step";
        case Stage::FrameAcquire:  return "frame_acquire";
        case Stage::BackgroundRender: return "background";
        case Stage::EmitterSplat:  return "splat";
        case Stage::DamageChain:   return "damage_chain";
        case Stage::Perception:    return "perception";
        case Stage::Median:        return "median_3x3";
        case Stage::TopHat:        return "top_hat";
        case Stage::SummedArea:    return "summed_area";
        case Stage::MatchedFilter: return "matched_filter";
        case Stage::Cfar:          return "cfar";
        case Stage::Grouping:      return "grouping";
        case Stage::Centroid:      return "centroid";
        case Stage::AiCandidate:   return "ai_candidate";
        case Stage::AiCentroid:    return "ai_centroid";
        case Stage::AiRecovery:    return "ai_recovery";
        case Stage::Tracking:      return "tracking";
        case Stage::Supervisor:    return "supervisor";
        case Stage::Control:       return "control";
        case Stage::Metrics:       return "metrics";
        case Stage::Snapshot:      return "snapshot";
        case Stage::FrameTotal:    return "frame_total";
        case Stage::kCount:        break;
    }
    return "unknown";
}

}  // namespace sat
