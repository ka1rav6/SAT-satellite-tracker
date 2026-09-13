// tests/render/test_coverage.cpp — the coverage primitives underneath CP 1.3.
//
// These are tested separately from the splatter because when a rendered
// centroid comes out wrong, the first question is always "is the geometry wrong
// or is the accumulation wrong", and two suites answer it without debugging.

#include <doctest/doctest.h>

#include "camera/coverage.hpp"

#include <cmath>
#include <initializer_list>

using namespace sat;

TEST_CASE("overlap_1d computes interval intersection length") {
    CHECK(overlap_1d(0.0, 1.0, 0.0, 1.0) == doctest::Approx(1.0));   // identical
    CHECK(overlap_1d(0.0, 1.0, 0.5, 1.5) == doctest::Approx(0.5));   // half
    CHECK(overlap_1d(0.0, 1.0, 2.0, 3.0) == doctest::Approx(0.0));   // disjoint
    CHECK(overlap_1d(0.0, 1.0, 1.0, 2.0) == doctest::Approx(0.0));   // touching
    CHECK(overlap_1d(0.0, 10.0, 3.0, 4.0) == doctest::Approx(1.0));  // contained
    CHECK(overlap_1d(3.0, 4.0, 0.0, 10.0) == doctest::Approx(1.0));  // containing

    // Never negative, whatever the ordering of the arguments.
    CHECK(overlap_1d(5.0, 6.0, 0.0, 1.0) >= 0.0);
}

TEST_CASE("gauss_1d integrates a normalised Gaussian exactly") {
    // Over the whole line the integral is 1.
    CHECK(gauss_1d(-40.0, 40.0, 0.0, 1.0) == doctest::Approx(1.0).epsilon(1e-12));
    // Symmetric halves.
    CHECK(gauss_1d(-40.0, 0.0, 0.0, 1.0) == doctest::Approx(0.5).epsilon(1e-12));
    // The textbook 1-sigma figure: 68.27% within +/- one sigma.
    CHECK(gauss_1d(-1.0, 1.0, 0.0, 1.0) == doctest::Approx(0.682689492).epsilon(1e-8));
    // Shifting the mean shifts the window identically.
    CHECK(gauss_1d(9.0, 11.0, 10.0, 1.0) == doctest::Approx(0.682689492).epsilon(1e-8));
}

TEST_CASE("a pixel coordinate names the pixel centre") {
    // The convention the whole project depends on (core/frames.hpp). Pixel 0
    // covers [-0.5, 0.5), so its centre is 0.0 and it has unit width.
    CHECK(pixel_lo(0) == doctest::Approx(-0.5));
    CHECK(pixel_hi(0) == doctest::Approx(0.5));
    CHECK(pixel_hi(7) - pixel_lo(7) == doctest::Approx(1.0));
}

TEST_CASE("square coverage sums to exactly the emitter's area") {
    // The cheapest possible check that the splatter neither invents nor loses
    // light. Coverage is the fraction of each PIXEL covered, so the sum over
    // every pixel is the emitter's area in pixels — size^2 for a square.
    for (double frac : {0.0, 0.1, 0.37, 0.5, 0.73, 0.99}) {
        for (double size : {5.0, 10.0, 13.0, 20.0}) {
            const double cx = 50.0 + frac, cy = 40.0 + frac;
            double sum = 0.0;
            for (int j = 20; j < 60; ++j) {
                for (int i = 30; i < 70; ++i) {
                    sum += square_coverage(i, j, cx, cy, size);
                }
            }
            INFO("frac=" << frac << " size=" << size);
            CHECK(sum == doctest::Approx(size * size).epsilon(1e-12));
        }
    }
}

TEST_CASE("a pixel wholly inside an emitter is fully covered") {
    // This is what makes `intensity` mean "grey levels above background", the
    // same for every shape and independent of size (spec row 10 lets size vary
    // 5-20 px, and changing it must not change brightness).
    CHECK(square_coverage(40, 40, 40.0, 40.0, 11.0) == doctest::Approx(1.0).epsilon(1e-12));
    CHECK(circle_coverage(40, 40, 40.0, 40.0, 11.0) == doctest::Approx(1.0).epsilon(1e-12));
    // The Gaussian has no flat top, so its peak pixel is ~1 by construction of
    // the normalisation rather than exactly 1 at an off-centre position.
    CHECK(gaussian_coverage(40, 40, 40.0, 40.0, 11.0) == doctest::Approx(1.0).epsilon(1e-12));
}

TEST_CASE("a square aligned to pixel edges covers whole pixels exactly") {
    // Pixel i spans [i-0.5, i+0.5], so a square's edges land on pixel
    // BOUNDARIES when its centre sits at a half-integer. A 4x4 square centred at
    // (10.5, 10.5) spans [8.5, 12.5], covering pixels 9..12 completely and no
    // others. Each of the 16 gets exactly 1/16.
    const double size = 4.0;
    for (int j = 9; j <= 12; ++j) {
        for (int i = 9; i <= 12; ++i) {
            INFO("pixel (" << i << ", " << j << ")");
            CHECK(square_coverage(i, j, 10.5, 10.5, size)
                  == doctest::Approx(1.0).epsilon(1e-12));
        }
    }
    CHECK(square_coverage(8,  10, 10.5, 10.5, size) == doctest::Approx(0.0));
    CHECK(square_coverage(13, 10, 10.5, 10.5, size) == doctest::Approx(0.0));
}

TEST_CASE("a square centred on a pixel splits its edge pixels evenly") {
    // The most direct demonstration that coverage is continuous rather than
    // sampled. A 4x4 square centred at (10.0, 10.0) spans [8, 12], so its edges
    // fall on the CENTRES of pixels 8 and 12 — each of which is therefore
    // exactly half covered, and by symmetry equally so.
    //
    // A point-sampling renderer would give one of them everything and the other
    // nothing, which is the origin of pixel-locking bias.
    const double size = 4.0;
    const double left  = square_coverage(8,  10, 10.0, 10.0, size);
    const double right = square_coverage(12, 10, 10.0, 10.0, size);
    CHECK(left == doctest::Approx(right).epsilon(1e-15));
    CHECK(left > 0.0);
    // Half covered in x, fully in y.
    CHECK(left == doctest::Approx(0.5).epsilon(1e-12));
}

TEST_CASE("circle coverage sums to exactly pi r squared") {
    for (double size : {5.0, 10.0, 20.0}) {
        const double r = 0.5 * size;
        double sum = 0.0;
        for (int j = 20; j < 60; ++j) {
            for (int i = 20; i < 60; ++i) {
                sum += circle_coverage(i, j, 40.0, 40.0, size);
            }
        }
        INFO("size=" << size);
        // Exact, not approximate: the closed-form intersection area replaced
        // the 4x4 supersampler, which could only manage ~1%.
        CHECK(sum == doctest::Approx(kPi * r * r).epsilon(1e-12));
    }
}

TEST_CASE("gaussian coverage sums to its equivalent area and matches its FWHM") {
    // With peak-relative normalisation the sum is the Gaussian's "equivalent
    // area" 2*pi*sigma^2 divided by the peak pixel's own integral.
    constexpr double kFwhmToSigma = 1.0 / 2.3548200450309493;
    const double size = 10.0;
    const double sigma = size * kFwhmToSigma;

    double sum = 0.0;
    for (int j = 0; j < 80; ++j) {
        for (int i = 0; i < 80; ++i) {
            sum += gaussian_coverage(i, j, 40.0, 40.0, size);
        }
    }
    const double peak_1d = gauss_1d(-0.5, 0.5, 0.0, sigma);
    CHECK(sum == doctest::Approx(1.0 / (peak_1d * peak_1d)).epsilon(1e-6));

    // `size` is the full width at half maximum, so the value 5 px from centre
    // must be half the peak value.
    const double peak = gaussian_coverage(40, 40, 40.0, 40.0, size);
    const double half = gaussian_coverage(45, 40, 40.0, 40.0, size);
    CHECK(half / peak == doctest::Approx(0.5).epsilon(0.02));
    CHECK(peak == doctest::Approx(1.0).epsilon(1e-12));
}


// ===========================================================================
// The exact disk-rectangle intersection (see the long comment in coverage.hpp
// on why design §9.2's 4x4 supersampling was replaced).
// ===========================================================================

namespace {

/// Brute-force area of a disk-rectangle intersection, by midpoint sampling.
/// Slow and only approximate, which is exactly what makes it a useful oracle:
/// it shares no code or reasoning with the closed form it checks.
double numeric_disk_rect(double x0, double x1, double y0, double y1,
                         double cx, double cy, double r, int n = 2000) {
    const double dx = (x1 - x0) / n, dy = (y1 - y0) / n;
    const double cell = dx * dy;
    double acc = 0.0;
    for (int j = 0; j < n; ++j) {
        const double y = y0 + (static_cast<double>(j) + 0.5) * dy - cy;
        for (int i = 0; i < n; ++i) {
            const double x = x0 + (static_cast<double>(i) + 0.5) * dx - cx;
            if (x * x + y * y <= r * r) acc += cell;
        }
    }
    return acc;
}

}  // namespace

TEST_CASE("arc_integral is the antiderivative it claims to be") {
    const double r = 3.0;
    // g(r) is a quarter of the disk; g is odd and flat outside [-r, r].
    CHECK(arc_integral(r, r) == doctest::Approx(0.25 * kPi * r * r).epsilon(1e-12));
    CHECK(arc_integral(-r, r) == doctest::Approx(-0.25 * kPi * r * r).epsilon(1e-12));
    CHECK(arc_integral(0.0, r) == doctest::Approx(0.0));
    CHECK(arc_integral(10.0, r) == doctest::Approx(arc_integral(r, r)));
    CHECK(arc_integral(-10.0, r) == doctest::Approx(arc_integral(-r, r)));

    // Differentiating numerically must give back sqrt(r^2 - t^2).
    for (double t : {-2.0, -0.5, 0.0, 1.0, 2.5}) {
        const double h = 1e-6;
        const double d = (arc_integral(t + h, r) - arc_integral(t - h, r)) / (2 * h);
        INFO("t = " << t);
        CHECK(d == doctest::Approx(std::sqrt(r * r - t * t)).epsilon(1e-6));
    }
}

TEST_CASE("disk_rect_area over a large rectangle is exactly pi r squared") {
    for (double r : {0.5, 2.5, 5.5, 10.0}) {
        INFO("r = " << r);
        CHECK(disk_rect_area(-100, 100, -100, 100, 0, 0, r)
              == doctest::Approx(kPi * r * r).epsilon(1e-14));
        // And the same with the disk parked somewhere awkward.
        CHECK(disk_rect_area(-100, 100, -100, 100, 3.7, -2.1, r)
              == doctest::Approx(kPi * r * r).epsilon(1e-14));
    }
}

TEST_CASE("disk_rect_area matches brute-force integration on partial cells") {
    // Hand-picked cells that each exercise a different branch: the disk cutting
    // a corner, a full edge, a thin sliver, and a cell that contains the centre.
    struct Case { double x0, y0, cx, cy, r; const char* what; };
    const Case cases[] = {
        {-0.5, -0.5,  0.30,  0.20, 2.50, "cell contains the centre"},
        { 1.5, -0.5,  0.37,  0.11, 2.50, "cell fully inside"},
        { 2.5,  2.5,  0.00,  0.00, 4.00, "corner cut diagonally"},
        {-3.5,  1.5,  0.13, -0.44, 3.30, "thin sliver at the rim"},
        {-1.5, -0.5, -0.68,  0.10, 1.23, "small disk, cell on the edge"},
        { 0.5,  0.5,  0.00,  0.00, 0.70, "disk smaller than a pixel"},
    };
    for (const auto& c : cases) {
        INFO(c.what);
        const double x1 = c.x0 + 1.0, y1 = c.y0 + 1.0;
        const double exact = disk_rect_area(c.x0, x1, c.y0, y1, c.cx, c.cy, c.r);
        const double num   = numeric_disk_rect(c.x0, x1, c.y0, y1, c.cx, c.cy, c.r);
        // The oracle's own midpoint-sampling error dominates this comparison.
        CHECK(exact == doctest::Approx(num).epsilon(1e-4));
        CHECK(exact >= 0.0);
        CHECK(exact <= 1.0 + 1e-12);
    }
}

TEST_CASE("circle coverage is exactly symmetric about a pixel-centred disk") {
    // Any asymmetry here would bias the rendered centroid directionally, which
    // is far worse than an equal error on both sides.
    for (double size : {5.0, 11.0, 20.0}) {
        for (int d = 1; d <= 6; ++d) {
            const double l = circle_coverage(40 - d, 40, 40.0, 40.0, size);
            const double r = circle_coverage(40 + d, 40, 40.0, 40.0, size);
            const double u = circle_coverage(40, 40 - d, 40.0, 40.0, size);
            const double b = circle_coverage(40, 40 + d, 40.0, 40.0, size);
            INFO("size=" << size << " d=" << d);
            // The tolerance scales with the disk area because that is what the
            // inclusion-exclusion actually cancels: four quadrant areas of
            // order pi*r^2 are added and subtracted to leave a number of order
            // 1, so the residue is ~area * epsilon. For a 20 px disk that is
            // ~7e-14, which is exact to within a few ulp of the intermediate
            // values, not a looser claim about the result.
            const double tol = 1e-15 * kPi * 0.25 * size * size;
            CHECK(std::abs(l - r) < tol);
            CHECK(std::abs(u - b) < tol);
            CHECK(std::abs(l - u) < tol);   // and across the axes
        }
    }
}

TEST_CASE("circle coverage is area-exact at every sub-pixel offset") {
    // The supersampled version this replaced could only manage ~1e-2 here.
    for (double size : {5.0, 11.0, 20.0}) {
        const double r = 0.5 * size;
        for (double frac : {0.0, 0.13, 0.37, 0.5, 0.86}) {
            double sum = 0.0;
            for (int j = 10; j < 70; ++j) {
                for (int i = 10; i < 70; ++i) {
                    sum += circle_coverage(i, j, 40.0 + frac, 40.0 + frac, size);
                }
            }
            INFO("size=" << size << " frac=" << frac);
            CHECK(sum == doctest::Approx(kPi * r * r).epsilon(1e-12));
        }
    }
}
