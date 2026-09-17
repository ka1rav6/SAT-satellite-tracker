// tests/degrade/test_fast_normal.cpp — the sensor's Gaussian source.
//
// This function is called once per pixel per frame — 307,200 times at the
// specification's sensor size — so it is the most-executed line in the project
// and it decides the statistics that every detection threshold rests on. CP 4.5
// asks for the measured noise variance to match theory within 2%, and CP 5.5
// asks for the CFAR false-alarm count on pure noise to match Pfa = Q(3.9)
// within 20%. Both are properties of THIS function before they are properties
// of anything else, so they are tested here first, where a failure is one line
// rather than a whole pipeline.

#include <doctest/doctest.h>

#include "degrade/fast_normal.hpp"
#include "core/rng.hpp"

#include <cmath>
#include <limits>
#include <vector>

using namespace sat;

namespace {

/// The upper tail of a standard normal, Q(k) = 0.5 * erfc(k / sqrt(2)).
double q_tail(double k) {
    return 0.5 * std::erfc(k / std::sqrt(2.0));
}

}  // namespace

TEST_CASE("acklam_inverse_normal inverts the normal CDF") {
    // Φ(Φ^-1(p)) == p, checked by round-tripping through erfc.
    for (double p : {1e-9, 1e-6, 1e-4, 0.001, 0.02424, 0.02426, 0.1, 0.25,
                     0.5, 0.75, 0.9, 0.97574, 0.97576, 0.999, 1.0 - 1e-6}) {
        const double z   = acklam_inverse_normal(p);
        const double phi = 0.5 * std::erfc(-z / std::sqrt(2.0));
        // Acklam's stated relative error is 1.15e-9; the round trip through
        // erfc adds its own, so 1e-7 relative is a fair bar and is still five
        // orders of magnitude finer than an 8-bit sensor can express.
        CHECK(std::abs(phi - p) < 1e-7 * std::max(p, 1e-3));
    }
    CHECK(acklam_inverse_normal(0.5) == doctest::Approx(0.0).epsilon(1e-12));
    // Symmetry, exactly the property the table's knots rely on.
    CHECK(acklam_inverse_normal(0.2) == doctest::Approx(-acklam_inverse_normal(0.8)).epsilon(1e-9));
}

TEST_CASE("fast_normal agrees with the closed form it is built from") {
    // The table path and the exact path must be the same function. Sampled
    // across the whole 32-bit input range, including the ends where the exact
    // path takes over.
    double worst = 0.0;
    uint32_t worst_u = 0;
    for (uint64_t step = 0; step < 200000; ++step) {
        // A stride that is coprime with 2^32 walks the whole range without
        // repeating and without favouring any region.
        const uint32_t u = static_cast<uint32_t>(step * 21467ull * 1000ull + 12345ull);
        const double p     = (static_cast<double>(u) + 0.5) * 2.3283064365386963e-10;
        const double exact = acklam_inverse_normal(p);
        const double got   = static_cast<double>(fast_normal(u));
        const double err   = std::abs(got - exact);
        if (err > worst) { worst = err; worst_u = u; }
    }
    INFO("worst absolute error ", worst, " at u = ", worst_u);
    // The header's claim: under 1.5e-4 sigma in the interpolated body. The
    // float storage of the knots contributes its own ~1e-7.
    CHECK(worst < 1.5e-4);
}

TEST_CASE("fast_normal has the moments of a standard normal") {
    Pcg32 g{20260917u};
    const int N = 4'000'000;
    double s1 = 0, s2 = 0, s4 = 0;
    for (int i = 0; i < N; ++i) {
        const double z = static_cast<double>(fast_normal(g.next_u32()));
        s1 += z;
        s2 += z * z;
        s4 += z * z * z * z;
    }
    const double mean = s1 / N;
    const double var  = s2 / N - mean * mean;
    const double kurt = s4 / N;          // E[z^4] = 3 for a standard normal

    INFO("mean ", mean, " var ", var, " E[z^4] ", kurt);
    // Standard error of the mean is 1/sqrt(N) = 5e-4; three of those.
    CHECK(std::abs(mean) < 1.5e-3);
    // CP 4.5's bar is 2% on the variance. This is the source of that variance,
    // so it is held to a tenth of it.
    CHECK(std::abs(var - 1.0) < 0.002);
    // The fourth moment is what a truncated or distorted tail would destroy,
    // and it is the moment CFAR's threshold is sensitive to.
    CHECK(std::abs(kurt - 3.0) < 0.05);
}

TEST_CASE("fast_normal's tail matches Q(k), which is what CFAR's Pfa rests on") {
    // CP 5.5: "with k = 3.9 on pure noise, false alarm count matches
    // Pfa = Q(3.9) within 20%". That can only hold if the generator's own tail
    // does, and z = 3.9 sits inside the exact path by construction — the table
    // never interpolates past |z| = 2.66.
    Pcg32 g{777u};
    const int N = 20'000'000;
    long long over39 = 0, over30 = 0, over45 = 0;
    for (int i = 0; i < N; ++i) {
        const double z = static_cast<double>(fast_normal(g.next_u32()));
        if (z > 3.0) ++over30;
        if (z > 3.9) ++over39;
        if (z > 4.5) ++over45;
    }
    const double e30 = q_tail(3.0) * N;
    const double e39 = q_tail(3.9) * N;
    const double e45 = q_tail(4.5) * N;
    INFO("k=3.0 got ", over30, " expected ", e30);
    INFO("k=3.9 got ", over39, " expected ", e39);
    INFO("k=4.5 got ", over45, " expected ", e45);
    // Poisson counting error is sqrt(expected); the bars are five sigma of
    // that, so a real distortion of the tail fails and a run of luck does not.
    CHECK(std::abs(over30 - e30) < 5.0 * std::sqrt(e30));
    CHECK(std::abs(over39 - e39) < 5.0 * std::sqrt(e39));
    CHECK(std::abs(over45 - e45) < 5.0 * std::sqrt(e45));
}

TEST_CASE("fast_normal is a pure function of its input") {
    // INV-3. No state, no cache, no rejection: the same word always gives the
    // same deviate, which is what lets a vectorised implementation be compared
    // against this one bit for bit (CP 14.2).
    for (uint32_t u : {0u, 1u, 12345u, 0x7FFFFFFFu, 0x80000000u, 0xFFFFFFFFu,
                       0x000FFFFFu, 0x00100000u, 0xFFF00000u}) {
        CHECK(fast_normal(u) == fast_normal(u));
    }
    // The two ends are finite: p is (u + 0.5) / 2^32, never 0 or 1.
    CHECK(std::isfinite(fast_normal(0u)));
    CHECK(std::isfinite(fast_normal(0xFFFFFFFFu)));
    CHECK(fast_normal(0u) < -5.0f);
    CHECK(fast_normal(0xFFFFFFFFu) > 5.0f);
}

// ---------------------------------------------------------------------------
// CP 14.2's criterion, stated for the damage chain: "bit-identical to scalar on
// random inputs".
//
// Not "agrees to a tolerance". The chain feeds the rendered frame, which feeds
// the reproducibility fingerprint (INV-3) and the graded centroiding metric, so
// a vector path that differed by an ulp would make every recorded number depend
// on which machine produced it — which is the property CP 2.6 exists to
// establish.
//
// The comparison is run against the scalar loop with the vector path forced
// off, on the same generator state, over inputs chosen to hit every branch:
// negative radiance, zero, values that clip at both ends, and NaN.
// ---------------------------------------------------------------------------

#include "degrade/sensor.hpp"
#include "degrade/sensor_simd.hpp"

#include <cstring>

TEST_CASE("CP 14.2: the vectorised damage chain is bit-identical to the scalar one") {
    if (!damage_chain_simd_available()) {
        MESSAGE("no AVX2 on this host — the scalar path is the only path");
        return;
    }

    constexpr size_t kN = 4096 + 5;        // deliberately not a multiple of 8
    Pcg32 g0{20260917u};

    std::vector<float> rad(kN), prnu(kN), fpn(kN);
    for (size_t i = 0; i < kN; ++i) {
        // Every branch: the ordinary range, both clipping ends, and the values
        // the comparisons are written to be careful about.
        switch (i % 64) {
            case 0:  rad[i] = 0.0f; break;
            case 1:  rad[i] = -50.0f; break;
            case 2:  rad[i] = 1.0e6f; break;
            case 3:  rad[i] = std::numeric_limits<float>::quiet_NaN(); break;
            case 4:  rad[i] = 238.5f; break;      // lands near the 254.5 clip
            default: rad[i] = static_cast<float>(g0.next_range(0.0, 200.0));
        }
        prnu[i] = static_cast<float>(1.0 + 0.01 * g0.next_normal());
        fpn[i]  = static_cast<float>(1.5 * g0.next_normal());
    }

    DamageArgs base;
    base.rad   = rad.data();
    base.prnu  = prnu.data();
    base.fpn   = fpn.data();
    base.n     = kN;
    base.alpha = 0.83f;                 // a weather mode that is not the identity
    base.beta  = 16.0f + 4.0f;          // black level plus an atmosphere offset
    base.k     = 8.0f;
    base.inv_k = 1.0f / 8.0f;
    base.sigma = 20.0f;
    base.rvar  = 400.0f;
    base.shot  = true;
    base.read  = true;
    base.fp    = true;

    // The scalar reference, written out here so the comparison does not depend
    // on SensorChain's plumbing. It is the same arithmetic in the same order as
    // the loop in degrade/sensor.cpp.
    auto scalar = [&](const DamageArgs& a, Pcg32 g, std::vector<uint8_t>& out) {
        out.assign(a.n, 0);
        for (size_t i = 0; i < a.n; ++i) {
            float v = a.alpha * a.rad[i] + a.beta;
            if (a.shot) {
                const float var = v * a.inv_k + a.rvar;
                v += std::sqrt(var) * fast_normal(g.next_u32());
            } else if (a.read) {
                v += a.sigma * fast_normal(g.next_u32());
            }
            if (a.fp) v = v * a.prnu[i] + a.fpn[i];
            if (!(v > 0.0f))      out[i] = 0;
            else if (v >= 254.5f) out[i] = 255;
            else                  out[i] = static_cast<uint8_t>(v + 0.5f);
        }
        return g;
    };

    for (int cfg = 0; cfg < 4; ++cfg) {
        DamageArgs a = base;
        a.shot = (cfg & 1) != 0;
        a.read = true;
        a.fp   = (cfg & 2) != 0;
        if (!a.fp) { a.prnu = nullptr; a.fpn = nullptr; }

        Pcg32 gs{4242u, 7u};
        std::vector<uint8_t> ref;
        const Pcg32 gs_end = scalar(a, gs, ref);

        std::vector<uint8_t> got(kN, 0);
        DamageArgs av = a;
        av.dst = got.data();
        Pcg32 gv{4242u, 7u};
        const size_t done = damage_chain_simd(av, gv);
        REQUIRE(done > 0);
        REQUIRE(done % 8 == 0);
        REQUIRE(done <= kN);

        // Finish the remainder the way SensorChain does, with the state the
        // vector path left behind.
        for (size_t i = done; i < kN; ++i) {
            float v = a.alpha * a.rad[i] + a.beta;
            if (a.shot) {
                const float var = v * a.inv_k + a.rvar;
                v += std::sqrt(var) * fast_normal(gv.next_u32());
            } else if (a.read) {
                v += a.sigma * fast_normal(gv.next_u32());
            }
            if (a.fp) v = v * a.prnu[i] + a.fpn[i];
            if (!(v > 0.0f))      got[i] = 0;
            else if (v >= 254.5f) got[i] = 255;
            else                  got[i] = static_cast<uint8_t>(v + 0.5f);
        }

        size_t mismatches = 0, first = 0;
        for (size_t i = 0; i < kN; ++i) {
            if (got[i] != ref[i]) { if (!mismatches) first = i; ++mismatches; }
        }
        INFO("cfg shot=", a.shot, " fp=", a.fp,
             " mismatches ", mismatches, " first at ", first,
             " (scalar ", int(ref[first]), " vector ", int(got[first]), ")");
        CHECK(mismatches == 0);

        // And the generator ends in the same place, which is what makes the
        // vector and scalar paths interchangeable at any pixel boundary.
        CHECK(gv.raw_state() == gs_end.raw_state());
        CHECK(gv.raw_inc()   == gs_end.raw_inc());
    }
}
