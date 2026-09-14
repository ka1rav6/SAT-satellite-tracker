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

// ---------------------------------------------------------------------------
// cfar_mask — the whole image, row by row.
//
// ---------------------------------------------------------------------------
// WHY THIS IS NOT JUST A LOOP CALLING cfar_at
// ---------------------------------------------------------------------------
// It was, and at 14.5 ms per pass it was the single most expensive stage in the
// tracker — two passes per frame, 29 of perception's 42 ms.
//
// Two things were wrong, and the first one was the smaller.
//
// CACHE. Every pixel's statistics come from four summed-area rows: y-30 and
// y+31 for the training window, y-15 and y+16 for the guard. Those rows are
// ~150 KB apart in a table whose rows are 641 * 8 bytes, and calling cfar_at
// per pixel re-derived all four offsets and re-clamped all four bounds,
// 307,200 times. For a FIXED y they are all fixed, so hoisting them turns the
// access pattern into four sequential streams. Worth 1.3 ms of the 14.5.
//
// ARITHMETIC, which was the rest of it. Per pixel: two divisions to form the
// mean and E[x^2], a square root for the standard deviation, and a third
// division for the SNR. Four long-latency operations at roughly 15 cycles
// each, 307,200 times, twice a frame — about 12 ms at 3 GHz, which is what was
// measured.
//
// All four are avoidable:
//
//   * cnt is CONSTANT across the interior of a row, so its reciprocal is
//     computed once per run of equal counts and multiplied rather than divided;
//
//   * the detection test `cell - mean > k * sd` is equivalent to
//     `cell - mean > 0 && (cell - mean)^2 > k^2 * var` — no square root, and
//     exactly the same decision because both sides are non-negative;
//
//   * the SNR itself is not needed here at all. It was written for all 307,200
//     pixels and read at the candidate centroids — at most two dozen. The
//     caller now evaluates cfar_at() at those points instead, which returns
//     the identical value. Same lesson as the matched filter's scale map: this
//     was not a slow computation, it was a fast one applied to ten thousand
//     times more data than anything read.
//
// `snr` may be empty, which is the fast path. cfar_at stays unchanged as the
// definition this has to agree with, and the tests compare the two.
// ---------------------------------------------------------------------------
void cfar_mask(const SummedArea& sa, int width, int height, const CfarParams& p,
               std::span<uint8_t> mask, std::span<float> snr) noexcept {
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (width <= 0 || height <= 0 || mask.size() < n) return;
    const bool want_snr = snr.size() >= n;

    const int    t   = p.train / 2;
    const int    g   = p.guard / 2;
    const int    W1  = width + 1;
    const double kk  = static_cast<double>(p.k) * p.k;

    for (int y = 0; y < height; ++y) {
        const int oy0 = std::clamp(y - t,     0, height);
        const int oy1 = std::clamp(y + t + 1, 0, height);
        const int gy0 = std::clamp(y - g,     0, height);
        const int gy1 = std::clamp(y + g + 1, 0, height);

        const int64_t*  so0 = sa.s.data()  + static_cast<size_t>(oy0) * W1;
        const int64_t*  so1 = sa.s.data()  + static_cast<size_t>(oy1) * W1;
        const int64_t*  sg0 = sa.s.data()  + static_cast<size_t>(gy0) * W1;
        const int64_t*  sg1 = sa.s.data()  + static_cast<size_t>(gy1) * W1;
        const uint64_t* qo0 = sa.s2.data() + static_cast<size_t>(oy0) * W1;
        const uint64_t* qo1 = sa.s2.data() + static_cast<size_t>(oy1) * W1;
        const uint64_t* qg0 = sa.s2.data() + static_cast<size_t>(gy0) * W1;
        const uint64_t* qg1 = sa.s2.data() + static_cast<size_t>(gy1) * W1;
        const int64_t*  sc0 = sa.s.data()  + static_cast<size_t>(y)     * W1;
        const int64_t*  sc1 = sa.s.data()  + static_cast<size_t>(y + 1) * W1;

        const int oh = oy1 - oy0;
        const int gh = gy1 - gy0;

        const size_t row = static_cast<size_t>(y) * static_cast<size_t>(width);
        int    last_cnt = -1;
        double inv_cnt  = 0.0;

        for (int x = 0; x < width; ++x) {
            const int ox0 = std::clamp(x - t,     0, width);
            const int ox1 = std::clamp(x + t + 1, 0, width);
            const int gx0 = std::clamp(x - g,     0, width);
            const int gx1 = std::clamp(x + g + 1, 0, width);

            const int cnt = (ox1 - ox0) * oh - (gx1 - gx0) * gh;
            if (cnt < 16) {           // §9.4.5's floor; see cfar_at
                mask[row + static_cast<size_t>(x)] = 0u;
                if (want_snr) snr[row + static_cast<size_t>(x)] = 0.0f;
                continue;
            }
            // Constant across the interior, so this divides a handful of times
            // per row rather than once per pixel.
            if (cnt != last_cnt) { last_cnt = cnt; inv_cnt = 1.0 / cnt; }

            const int64_t  sum_outer = so1[ox1] - so0[ox1] - so1[ox0] + so0[ox0];
            const int64_t  sum_guard = sg1[gx1] - sg0[gx1] - sg1[gx0] + sg0[gx0];
            const uint64_t sq_outer  = qo1[ox1] - qo0[ox1] - qo1[ox0] + qo0[ox0];
            const uint64_t sq_guard  = qg1[gx1] - qg0[gx1] - qg1[gx0] + qg0[gx0];

            const double mean = static_cast<double>(sum_outer - sum_guard) * inv_cnt;
            const double e2   = static_cast<double>(sq_outer - sq_guard) * inv_cnt;
            const double var  = std::max(e2 - mean * mean, 1e-6);

            const double cell = static_cast<double>(sc1[x + 1] - sc0[x + 1]
                                                  - sc1[x]     + sc0[x]);
            const double d = cell - mean;

            // No square root: both sides non-negative, so squaring preserves
            // the comparison exactly.
            mask[row + static_cast<size_t>(x)] = (d > 0.0 && d * d > kk * var) ? 1u : 0u;
            if (want_snr) {
                snr[row + static_cast<size_t>(x)] =
                    static_cast<float>(d / std::sqrt(var));
            }
        }
    }
}

}  // namespace sat
