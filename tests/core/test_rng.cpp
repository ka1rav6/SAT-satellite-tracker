// tests/core/test_rng.cpp — CP 0.3, the RNG half.
//
// "two identically-seeded RngSets produce identical sequences on every stream"
//
// The independence tests matter as much as the determinism ones: design §13.3
// and every ablation in the report depend on being able to change one setting
// without perturbing the randomness every other setting sees.

#include <doctest/doctest.h>

#include "core/rng.hpp"

#include <cmath>
#include <set>
#include <string>
#include <vector>

using namespace sat;

TEST_CASE("two identically-seeded RngSets agree on every stream") {
    RngSet a(12345);
    RngSet b(12345);

    for (uint32_t i = 0; i < static_cast<uint32_t>(Stream::kCount); ++i) {
        const auto s = static_cast<Stream>(i);
        INFO("stream = " << stream_name(s));
        for (int k = 0; k < 256; ++k) {
            CHECK(a[s].next_u32() == b[s].next_u32());
        }
        for (int k = 0; k < 256; ++k) {
            CHECK(a[s].next_double() == b[s].next_double());
        }
        for (int k = 0; k < 256; ++k) {
            CHECK(a[s].next_normal() == b[s].next_normal());
        }
    }
}

TEST_CASE("different master seeds diverge immediately") {
    RngSet a(1);
    RngSet b(2);
    // The SplitMix64 finaliser exists precisely so that adjacent master seeds
    // do not produce near-identical streams.
    int same = 0;
    for (int k = 0; k < 64; ++k) {
        if (a[Stream::TargetMotion].next_u32() == b[Stream::TargetMotion].next_u32()) ++same;
    }
    CHECK(same == 0);
}

TEST_CASE("streams within one set are independent") {
    // This is the property that makes ablations honest. Drawing from Jitter
    // must not shift what SaltPepper produces.
    RngSet a(777);
    RngSet b(777);

    // Burn a large number of draws on one stream in `a` only.
    for (int i = 0; i < 100000; ++i) (void)a[Stream::Jitter].next_u32();

    // Every other stream must be untouched.
    for (uint32_t i = 0; i < static_cast<uint32_t>(Stream::kCount); ++i) {
        const auto s = static_cast<Stream>(i);
        if (s == Stream::Jitter) continue;
        INFO("stream = " << stream_name(s));
        CHECK(a[s].next_u32() == b[s].next_u32());
    }
}

TEST_CASE("stream sequences do not overlap each other") {
    // A weak but meaningful check: the first 1000 outputs of every stream,
    // pooled, should contain no duplicates. With 17 streams x 1000 draws from a
    // 32-bit output the birthday-paradox collision chance is ~3%, so we allow a
    // handful but not the wholesale overlap that a shared sequence would show.
    RngSet r(42);
    std::multiset<uint32_t> pool;
    for (uint32_t i = 0; i < static_cast<uint32_t>(Stream::kCount); ++i) {
        auto& g = r[static_cast<Stream>(i)];
        for (int k = 0; k < 1000; ++k) pool.insert(g.next_u32());
    }
    size_t dupes = 0;
    for (auto it = pool.begin(); it != pool.end(); ) {
        const size_t n = pool.count(*it);
        if (n > 1) dupes += n - 1;
        it = pool.upper_bound(*it);
    }
    CHECK(dupes < 20);
}

TEST_CASE("seeding is reproducible after re-seeding") {
    Pcg32 g(99, 3);
    std::vector<uint32_t> first;
    for (int i = 0; i < 100; ++i) first.push_back(g.next_u32());

    g.seed(99, 3);
    for (int i = 0; i < 100; ++i) CHECK(g.next_u32() == first[static_cast<size_t>(i)]);
}

TEST_CASE("next_double stays inside [0, 1) and next_double_open inside (0, 1]") {
    Pcg32 g(2024);
    double lo = 2.0, hi = -1.0;
    for (int i = 0; i < 200000; ++i) {
        const double d = g.next_double();
        REQUIRE(d >= 0.0);
        REQUIRE(d < 1.0);
        lo = std::min(lo, d);
        hi = std::max(hi, d);
    }
    // Over 200k draws the extremes should be close to the bounds.
    CHECK(lo < 1e-4);
    CHECK(hi > 1.0 - 1e-4);

    for (int i = 0; i < 200000; ++i) {
        const double d = g.next_double_open();
        REQUIRE(d > 0.0);          // log() of this must never be -inf
        REQUIRE(d <= 1.0);
    }
}

TEST_CASE("next_below is unbiased over small bounds") {
    // A `% bound` implementation would skew the low buckets. The fixed-pattern
    // and hot-pixel maps use this, and a visible bias would look like a real
    // sensor defect pattern that is actually an RNG bug.
    Pcg32 g(7);
    constexpr uint32_t kBound = 7;          // deliberately not a power of two
    constexpr int      kDraws = 700000;
    std::vector<int> hist(kBound, 0);
    for (int i = 0; i < kDraws; ++i) {
        const uint32_t v = g.next_below(kBound);
        REQUIRE(v < kBound);
        ++hist[v];
    }
    const double expected = static_cast<double>(kDraws) / kBound;
    for (uint32_t i = 0; i < kBound; ++i) {
        INFO("bucket " << i << " = " << hist[i] << " expected " << expected);
        CHECK(std::abs(hist[i] - expected) < 0.02 * expected);
    }
    CHECK(g.next_below(0) == 0u);           // degenerate bound is defined
}

TEST_CASE("next_normal has unit mean and variance") {
    Pcg32 g(31337);
    constexpr int kN = 500000;
    double sum = 0.0, sum_sq = 0.0;
    for (int i = 0; i < kN; ++i) {
        const double z = g.next_normal();
        sum += z;
        sum_sq += z * z;
    }
    const double mean = sum / kN;
    const double var  = sum_sq / kN - mean * mean;
    // Standard error of the mean is 1/sqrt(N) ~= 0.0014, so 0.01 is ~7 sigma.
    CHECK(std::abs(mean) < 0.01);
    CHECK(std::abs(var - 1.0) < 0.01);
}

TEST_CASE("next_normal(mean, sigma) scales correctly") {
    Pcg32 g(555);
    constexpr int kN = 200000;
    double sum = 0.0, sum_sq = 0.0;
    for (int i = 0; i < kN; ++i) {
        const double z = g.next_normal(100.0, 20.0);   // spec row 22: sigma = 20 grey levels
        sum += z;
        sum_sq += z * z;
    }
    const double mean = sum / kN;
    const double sd   = std::sqrt(sum_sq / kN - mean * mean);
    CHECK(mean == doctest::Approx(100.0).epsilon(0.01));
    CHECK(sd == doctest::Approx(20.0).epsilon(0.01));
}

TEST_CASE("the cached normal is cleared by seeding") {
    // The polar method produces two normals per accepted pair and caches the
    // second. If seed() forgot to clear that cache, a freshly seeded generator
    // would leak a value from its previous life -- a subtle reproducibility bug
    // that would only show on an odd number of draws.
    Pcg32 g(5);
    const double a0 = g.next_normal();      // fills the cache
    (void)a0;
    g.seed(5, 1);
    const double b0 = g.next_normal();

    Pcg32 h(5, 1);
    const double c0 = h.next_normal();
    CHECK(b0 == c0);
}

TEST_CASE("stream_name covers every enumerator") {
    for (uint32_t i = 0; i < static_cast<uint32_t>(Stream::kCount); ++i) {
        const char* n = stream_name(static_cast<Stream>(i));
        INFO("stream index " << i);
        CHECK(std::string(n) != "Unknown");
    }
}

TEST_CASE("Pcg32 is trivially copyable so a whole RngSet can be snapshot") {
    // Needed by INV-4: saving and restoring RNG state inside a frame must not
    // allocate.
    static_assert(std::is_trivially_copyable_v<Pcg32>);
    Pcg32 a(1234);
    for (int i = 0; i < 10; ++i) (void)a.next_u32();
    Pcg32 b = a;                            // byte copy
    for (int i = 0; i < 100; ++i) CHECK(a.next_u32() == b.next_u32());
}
