#include "degrade/noise.hpp"

#include <algorithm>
#include <cmath>

namespace sat {

// ---------------------------------------------------------------------------
// Poisson — design §9.3's branch, reproduced exactly.
// ---------------------------------------------------------------------------
double poisson(double lambda, Pcg32& rng) noexcept {
    if (!(lambda > 0.0)) return 0.0;          // also catches NaN

    if (lambda < 30.0) {
        // Knuth's method: multiply uniforms until the product drops below
        // exp(-lambda). The count of draws is the Poisson variate.
        const double L = std::exp(-lambda);
        double p = 1.0;
        int    k = 0;
        do {
            ++k;
            p *= rng.next_double();
            // Guard against a pathological run of near-1 uniforms. At
            // lambda < 30 the expected count is under 31, so 10000 is
            // unreachable in practice; it exists so a fuzzed input can never
            // hang, which CP 14.1 treats as a first-class failure.
            if (k > 10000) break;
        } while (p > L);
        return static_cast<double>(k - 1);
    }

    // Normal approximation. At lambda = 30 the skewness is 1/sqrt(30) = 0.18,
    // which is well below what 8-bit quantisation can express.
    return lambda + std::sqrt(lambda) * rng.next_normal();
}

// ---------------------------------------------------------------------------
// Salt and pepper — geometric skip sampling.
// ---------------------------------------------------------------------------
void salt_pepper(std::span<uint8_t> img, double p, Pcg32& rng) noexcept {
    if (p <= 0.0 || img.empty()) return;
    if (p >= 1.0) {
        for (auto& v : img) v = rng.next_bool() ? 255 : 0;
        return;
    }

    // log(1 - p) is negative, so dividing a negative log by it gives a positive
    // gap. Hoisted out of the loop: it is the same for every draw.
    const double log1mp = std::log(1.0 - p);
    size_t i = 0;
    while (true) {
        // next_double_open() is in (0, 1], so log(1 - u) is never log(0).
        // A uniform of exactly 0 would give a gap of 0 and loop forever.
        const double gap = std::log(1.0 - rng.next_double_open()) / log1mp;
        if (!(gap >= 0.0)) break;                       // NaN guard
        i += static_cast<size_t>(gap);
        if (i >= img.size()) break;
        img[i] = rng.next_bool() ? 255 : 0;
        ++i;
    }
}

void salt_pepper_naive(std::span<uint8_t> img, double p, Pcg32& rng) noexcept {
    if (p <= 0.0) return;
    for (auto& v : img) {
        if (rng.next_double() < p) v = rng.next_bool() ? 255 : 0;
    }
}

// ---------------------------------------------------------------------------
// FixedPattern
// ---------------------------------------------------------------------------
void FixedPattern::build(int width, int height, double prnu_sigma, double fpn_sigma,
                         int hot_pixels, int dead_pixels, RngSet& rng) {
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    prnu_.assign(n, 1.0f);
    fpn_.assign(n, 0.0f);
    hot_.clear();
    dead_.clear();
    if (n == 0) return;

    // Drawn from Stream::FixedPattern, so enabling or disabling PRNU cannot
    // shift the shot-noise or read-noise sequences.
    Pcg32& g = rng[Stream::FixedPattern];
    for (size_t i = 0; i < n; ++i) {
        // Gain is clamped positive: a negative gain would invert the pixel,
        // which no detector does, and at 1% spread it would take a 100-sigma
        // draw anyway.
        prnu_[i] = static_cast<float>(std::max(0.0, 1.0 + prnu_sigma * g.next_normal()));
        fpn_[i]  = static_cast<float>(fpn_sigma * g.next_normal());
    }

    // Defect positions come from their own stream, so changing the hot-pixel
    // count does not move the PRNU map.
    Pcg32& d = rng[Stream::DefectPixels];
    const uint32_t limit = static_cast<uint32_t>(n);
    hot_.reserve(static_cast<size_t>(std::max(0, hot_pixels)));
    for (int i = 0; i < hot_pixels; ++i)  hot_.push_back(d.next_below(limit));
    dead_.reserve(static_cast<size_t>(std::max(0, dead_pixels)));
    for (int i = 0; i < dead_pixels; ++i) dead_.push_back(d.next_below(limit));

    // Sorted so the apply loop walks memory forward, and so two defect lists
    // with the same contents compare equal regardless of draw order. Duplicates
    // are kept rather than deduplicated: removing them would change the number
    // of draws consumed, and a pixel drawn twice is simply stuck twice.
    std::sort(hot_.begin(), hot_.end());
    std::sort(dead_.begin(), dead_.end());
}

void FixedPattern::apply_gain_offset(std::span<float> img) const noexcept {
    if (prnu_.empty()) return;
    const size_t n = std::min(img.size(), prnu_.size());
    for (size_t i = 0; i < n; ++i) {
        img[i] = img[i] * prnu_[i] + fpn_[i];
    }
}

void FixedPattern::apply_defects(std::span<uint8_t> img) const noexcept {
    for (const uint32_t i : dead_) if (i < img.size()) img[i] = 0;
    for (const uint32_t i : hot_)  if (i < img.size()) img[i] = 255;
}

}  // namespace sat
