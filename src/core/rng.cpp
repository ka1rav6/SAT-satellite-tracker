// core/rng.cpp — the parts of the RNG that are not worth inlining.

#include "core/rng.hpp"

#include <cmath>

namespace sat {

// ---------------------------------------------------------------------------
// Marsaglia polar method.
//
// Draw a point uniformly in the square [-1,1]^2, reject it unless it falls
// inside the unit disc, then map the accepted point to a pair of independent
// standard normals. The rejection rate is 1 - pi/4 ~= 21.5%, so the expected
// cost is ~2.55 uniforms per accepted pair, i.e. ~1.27 uniforms per normal.
//
// Determinism note: the rejection loop consumes a variable number of uniforms,
// but that number is fully determined by the generator state, so two runs with
// the same seed consume exactly the same draws. This is why the loop is safe
// under INV-3 while something like "keep drawing until the image looks right"
// would not be.
// ---------------------------------------------------------------------------
double Pcg32::next_normal() noexcept {
    if (has_spare_normal_) {
        has_spare_normal_ = false;
        return spare_normal_;
    }

    double u, v, s;
    do {
        u = 2.0 * next_double() - 1.0;
        v = 2.0 * next_double() - 1.0;
        s = u * u + v * v;
        // s == 0 would divide by zero below; s >= 1 is outside the disc.
    } while (s >= 1.0 || s == 0.0);

    const double f = std::sqrt(-2.0 * std::log(s) / s);
    spare_normal_     = v * f;
    has_spare_normal_ = true;
    return u * f;
}

const char* stream_name(Stream s) noexcept {
    switch (s) {
        case Stream::TargetInit:      return "TargetInit";
        case Stream::ClutterLayout:   return "ClutterLayout";
        case Stream::DecoyLayout:     return "DecoyLayout";
        case Stream::BackgroundSeed:  return "BackgroundSeed";
        case Stream::TargetMotion:    return "TargetMotion";
        case Stream::PlatformMotion:  return "PlatformMotion";
        case Stream::Jitter:          return "Jitter";
        case Stream::ShotNoise:       return "ShotNoise";
        case Stream::ReadNoise:       return "ReadNoise";
        case Stream::SaltPepper:      return "SaltPepper";
        case Stream::FixedPattern:    return "FixedPattern";
        case Stream::DefectPixels:    return "DefectPixels";
        case Stream::Atmosphere:      return "Atmosphere";
        case Stream::SearchPattern:   return "SearchPattern";
        case Stream::Misc:            return "Misc";
        case Stream::DatasetSampling: return "DatasetSampling";
        case Stream::Fuzz:            return "Fuzz";
        case Stream::kCount:          break;
    }
    return "Unknown";
}

}  // namespace sat
