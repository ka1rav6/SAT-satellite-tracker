// tests/metrics/test_series.cpp — the statistics under the compliance matrix.
//
// Every number Stage 7 reports goes through Series, so a quiet error here would
// be invisible and would contaminate everything downstream. The tests are
// written against closed-form answers rather than against the implementation:
// a hand-checked quantile on a known series is a fact about the definition, and
// that is exactly what §13.1 asks to be pinned down.

#include <doctest/doctest.h>

#include "metrics/series.hpp"

#include <cmath>
#include <numeric>
#include <vector>

using namespace sat;

TEST_CASE("CP 7.1: RMS, mean and stddev on a hand-checked series") {
    Series s;
    for (double x : {3.0, 4.0}) s.push(x);
    // sqrt((9+16)/2) = sqrt(12.5)
    CHECK(s.rms() == doctest::Approx(std::sqrt(12.5)));
    CHECK(s.mean() == doctest::Approx(3.5));
    // Sample stddev (n-1): sqrt(((3-3.5)^2 + (4-3.5)^2) / 1) = sqrt(0.5)
    CHECK(s.stddev() == doctest::Approx(std::sqrt(0.5)));
    CHECK(s.max() == doctest::Approx(4.0));
    CHECK(s.min() == doctest::Approx(3.0));
    CHECK(s.count() == 2);
}

TEST_CASE("CP 7.1: bias is the mean of a SIGNED series and RMSE is not") {
    // The distinction §13.1 draws, and the reason both are reported. A perfectly
    // biased estimator and a perfectly noisy one can have identical RMSE.
    Series biased, noisy;
    for (int i = 0; i < 100; ++i) {
        biased.push(0.5);                        // always half a pixel to the right
        noisy.push((i % 2) ? 0.5 : -0.5);        // never right, never systematically wrong
    }
    CHECK(biased.rms() == doctest::Approx(0.5));
    CHECK(noisy.rms()  == doctest::Approx(0.5));   // identical RMSE
    CHECK(biased.mean() == doctest::Approx(0.5));  // ...and completely different bias
    CHECK(noisy.mean()  == doctest::Approx(0.0));
}

TEST_CASE("CP 7.1: the quantile is nearest-rank, so every value really occurred") {
    // 1..100. Nearest rank: p95 -> element ceil(0.95*100)-1 = index 94 = 95.
    Series s;
    for (int i = 1; i <= 100; ++i) s.push(i);
    CHECK(s.p50() == doctest::Approx(50.0));
    CHECK(s.p95() == doctest::Approx(95.0));
    CHECK(s.p99() == doctest::Approx(99.0));
    CHECK(s.p5()  == doctest::Approx(5.0));
    CHECK(s.quantile(1.0)  == doctest::Approx(100.0));
    CHECK(s.quantile(0.0)  == doctest::Approx(1.0));

    // The property that motivated nearest-rank: the reported value is always a
    // member of the sample set, for every p.
    const std::vector<double>& v = s.samples();
    for (int pct = 0; pct <= 100; ++pct) {
        const double q = s.quantile(pct / 100.0);
        CHECK(std::find(v.begin(), v.end(), q) != v.end());
    }
}

TEST_CASE("CP 7.1: quantiles on awkward sizes") {
    SUBCASE("empty series reports zero rather than reading past the end") {
        Series s;
        CHECK(s.p95() == doctest::Approx(0.0));
        CHECK(s.rms() == doctest::Approx(0.0));
        CHECK(s.mean() == doctest::Approx(0.0));
        CHECK(s.max() == doctest::Approx(0.0));
        CHECK(s.stddev() == doctest::Approx(0.0));
    }
    SUBCASE("one sample is its own every quantile") {
        Series s;
        s.push(7.0);
        CHECK(s.p50() == doctest::Approx(7.0));
        CHECK(s.p95() == doctest::Approx(7.0));
        CHECK(s.stddev() == doctest::Approx(0.0));   // n-1 = 0, not a division by zero
    }
    SUBCASE("out-of-range p is clamped, not undefined behaviour") {
        Series s;
        for (int i = 1; i <= 10; ++i) s.push(i);
        CHECK(s.quantile(-5.0) == doctest::Approx(1.0));
        CHECK(s.quantile(99.0) == doctest::Approx(10.0));
    }
}

TEST_CASE("CP 7.1: pushing after a quantile keeps the results correct") {
    // quantile() sorts in place, so the class has to notice that a later push
    // invalidated that order. Getting this wrong would silently return stale
    // percentiles for the rest of a run.
    Series s;
    for (int i = 1; i <= 10; ++i) s.push(i);
    CHECK(s.p50() == doctest::Approx(5.0));          // forces a sort
    for (int i = 11; i <= 100; ++i) s.push(i);
    CHECK(s.p50() == doctest::Approx(50.0));
    CHECK(s.max() == doctest::Approx(100.0));
    CHECK(s.count() == 100);
}

TEST_CASE("CP 7.1: a reserved series does not allocate while a run is in flight") {
    // INV-4. The reservation happens once in MetricCollector::begin(); after
    // that a 120 s run must add 3,600 samples without touching the allocator.
    Series s;
    s.reserve(3600);
    const size_t cap = s.capacity();
    REQUIRE(cap >= 3600);
    for (int i = 0; i < 3600; ++i) s.push(i * 0.001);
    CHECK(s.capacity() == cap);       // never grew
    CHECK(s.count() == 3600);
}
