// metrics/series.hpp — a sample series with the statistics §13.1 asks for.
//
// ---------------------------------------------------------------------------
// WHY THIS KEEPS EVERY SAMPLE
// ---------------------------------------------------------------------------
// RMSE, bias and max are all computable from running accumulators in O(1)
// space. p95 is not — not exactly — and §13.1 asks for p95 on the two graded
// metrics. There are streaming quantile estimators (P-square, t-digest) that
// get close, and they are the right answer when the sample count is unbounded.
//
// Here it is bounded and small: a 120 s run at 30 Hz is 3,600 samples, and
// CP 7.5's sweep runs each of its 500 configurations in a separate process. A
// std::vector<double> of 3,600 doubles is 29 KB. Approximating a number that
// can simply be computed, to save 29 KB, would be a strange trade — and an
// approximate p95 in a compliance matrix is exactly the kind of soft number
// this project is trying not to produce.
//
// INV-4 is not violated: the vector is reserved at construction from the known
// frame count, and push() then never allocates. That is asserted in the tests
// rather than assumed.
//
// ---------------------------------------------------------------------------
// BIAS IS SIGNED, EVERYTHING ELSE IS NOT
// ---------------------------------------------------------------------------
// §13.1 asks centroiding error for "RMSE, bias (mean signed), p95, max". The
// distinction matters and is the reason this class takes both a magnitude and a
// signed component: a centroid estimator with 0.5 px RMSE that is unbiased is a
// noisy estimator, and one with 0.5 px RMSE and 0.5 px bias is a BROKEN one
// that can be fixed by subtracting a constant. §10.1.3's S-curve correction is
// precisely that fix, so the metric that reveals it has to exist first.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace sat {

class Series {
public:
    void reserve(size_t n) { v_.reserve(n); }
    void clear() noexcept { v_.clear(); sorted_ = false; }

    void push(double x) {
        v_.push_back(x);
        sorted_ = false;
    }

    [[nodiscard]] size_t count() const noexcept { return v_.size(); }
    [[nodiscard]] bool   empty() const noexcept { return v_.empty(); }

    /// Root mean square. For a magnitude series this is the RMSE §13.1 names.
    [[nodiscard]] double rms() const noexcept {
        if (v_.empty()) return 0.0;
        double s = 0.0;
        for (double x : v_) s += x * x;
        return std::sqrt(s / static_cast<double>(v_.size()));
    }

    /// Mean. On a SIGNED series this is §13.1's "bias (mean signed)".
    [[nodiscard]] double mean() const noexcept {
        if (v_.empty()) return 0.0;
        double s = 0.0;
        for (double x : v_) s += x;
        return s / static_cast<double>(v_.size());
    }

    [[nodiscard]] double stddev() const noexcept {
        if (v_.size() < 2) return 0.0;
        const double m = mean();
        double s = 0.0;
        for (double x : v_) s += (x - m) * (x - m);
        return std::sqrt(s / static_cast<double>(v_.size() - 1));
    }

    [[nodiscard]] double max() const noexcept {
        return v_.empty() ? 0.0 : *std::max_element(v_.begin(), v_.end());
    }
    [[nodiscard]] double min() const noexcept {
        return v_.empty() ? 0.0 : *std::min_element(v_.begin(), v_.end());
    }

    // -----------------------------------------------------------------------
    // quantile — the exact one, by the "nearest rank" definition.
    //
    // There are nine common conventions and they disagree on small samples.
    // This one is stated rather than inherited from whatever a library happened
    // to do: the p-th quantile is the smallest sample x such that at least p of
    // the samples are <= x, i.e. element ceil(p*n) - 1 of the sorted series.
    //
    // Nearest-rank rather than an interpolating variant because every value
    // reported is then a value that ACTUALLY OCCURRED in the run. A compliance
    // matrix saying "p95 = 7.1 px" should mean some frame really had 7.1 px of
    // error, not that two neighbouring frames were averaged into a figure
    // nothing ever measured.
    // -----------------------------------------------------------------------
    [[nodiscard]] double quantile(double p) const {
        if (v_.empty()) return 0.0;
        sort_if_needed();
        const double n = static_cast<double>(v_.size());
        const long   k = static_cast<long>(std::ceil(std::clamp(p, 0.0, 1.0) * n));
        const size_t i = static_cast<size_t>(std::clamp(k - 1, 0L, static_cast<long>(v_.size()) - 1));
        return v_[i];
    }

    [[nodiscard]] double p50() const { return quantile(0.50); }
    [[nodiscard]] double p95() const { return quantile(0.95); }
    [[nodiscard]] double p99() const { return quantile(0.99); }
    /// The low tail, for metrics where SMALL is the failure — fps, chiefly.
    [[nodiscard]] double p5()  const { return quantile(0.05); }

    /// The samples, in UNSPECIFIED order. Calling any quantile sorts in place,
    /// so this is chronological only until the first quantile() call. That is
    /// stated rather than hidden because a silent reorder is a trap: anything
    /// wanting the time series (a plot, a CSV column) must keep its own copy,
    /// and nothing in this project does — the report plots come from the
    /// per-frame log, not from here.
    [[nodiscard]] const std::vector<double>& samples() const noexcept { return v_; }

    /// Capacity, so a test can assert that a run never reallocated (INV-4).
    [[nodiscard]] size_t capacity() const noexcept { return v_.capacity(); }

private:
    void sort_if_needed() const {
        if (!sorted_) {
            std::sort(v_.begin(), v_.end());
            sorted_ = true;
        }
    }

    // Mutable because sorting does not change the multiset of samples, and
    // every accessor above is logically const. The alternative — sorting a copy
    // on every quantile call — would turn the compliance matrix from one sort
    // into a dozen.
    mutable std::vector<double> v_;
    mutable bool                sorted_ = false;
};

}  // namespace sat
