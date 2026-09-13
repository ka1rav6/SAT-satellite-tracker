#include "perception/cfar.hpp"

#include <algorithm>
#include <cmath>

namespace sat {

double CfarParams::pfa() const noexcept {
    // Q(k) = 0.5 * erfc(k / sqrt(2)), the upper tail of a standard normal.
    return 0.5 * std::erfc(static_cast<double>(k) / std::sqrt(2.0));
}

float CfarParams::k_for_pfa(double target) noexcept {
    // No closed form for the inverse, and none is needed: Q is monotone, so a
    // bisection over a bounded range converges in a few dozen iterations and is
    // impossible to get subtly wrong. This runs once at scenario load.
    if (!(target > 0.0) || target >= 0.5) return 0.0f;
    double lo = 0.0, hi = 40.0;
    for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (lo + hi);
        const double q = 0.5 * std::erfc(mid / std::sqrt(2.0));
        if (q > target) lo = mid; else hi = mid;
    }
    return static_cast<float>(0.5 * (lo + hi));
}

CfarResult cfar_at(const SummedArea& sa, int x, int y, const CfarParams& p) noexcept {
    CfarResult r{};

    const int t = p.train / 2;
    const int g = p.guard / 2;

    // Training statistics come from the ANNULUS: the outer window minus the
    // guard region. Both are box sums, so this is 16 lookups whatever the
    // window size.
    int n_outer = 0, n_guard = 0;
    const int64_t  sum_outer = sa.box_sum_clipped(x - t, y - t, x + t + 1, y + t + 1, n_outer);
    const int64_t  sum_guard = sa.box_sum_clipped(x - g, y - g, x + g + 1, y + g + 1, n_guard);
    int n_outer2 = 0, n_guard2 = 0;
    const uint64_t sq_outer  = sa.box_sum_sq_clipped(x - t, y - t, x + t + 1, y + t + 1, n_outer2);
    const uint64_t sq_guard  = sa.box_sum_sq_clipped(x - g, y - g, x + g + 1, y + g + 1, n_guard2);

    const int n = n_outer - n_guard;
    // Near a corner the annulus can be small. Below a floor the variance
    // estimate is too noisy to threshold against and firing on it would produce
    // false alarms exactly at the frame edge — so the cell is reported with the
    // statistics it has, and simply not flagged.
    if (n < 16) return r;

    const double mean = static_cast<double>(sum_outer - sum_guard) / n;
    const double e2   = static_cast<double>(sq_outer - sq_guard) / n;

    // Var = E[x^2] - E[x]^2, floored because catastrophic cancellation on a
    // near-uniform region can drive it very slightly negative even with exact
    // integer inputs — the two terms are both large and nearly equal.
    const double var = std::max(e2 - mean * mean, 1e-6);
    const double sd  = std::sqrt(var);

    // The cell under test: a single pixel, compared against per-pixel
    // statistics.
    int n_cell = 0;
    const double cell = static_cast<double>(sa.box_sum_clipped(x, y, x + 1, y + 1, n_cell));
    if (n_cell == 0) return r;

    r.local_mean = static_cast<float>(mean);
    r.local_sd   = static_cast<float>(sd);
    r.snr        = static_cast<float>((cell - mean) / sd);
    r.detected   = r.snr > p.k;
    return r;
}

void cfar_mask(const SummedArea& sa, int width, int height, const CfarParams& p,
               std::span<uint8_t> mask, std::span<float> snr) noexcept {
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (width <= 0 || height <= 0 || mask.size() < n || snr.size() < n) return;

    for (int y = 0; y < height; ++y) {
        const size_t row = static_cast<size_t>(y) * static_cast<size_t>(width);
        for (int x = 0; x < width; ++x) {
            const CfarResult r = cfar_at(sa, x, y, p);
            mask[row + static_cast<size_t>(x)] = r.detected ? 1u : 0u;
            snr[row + static_cast<size_t>(x)]  = r.snr;
        }
    }
}

}  // namespace sat
