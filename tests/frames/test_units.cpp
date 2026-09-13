// tests/frames/test_units.cpp — CP 0.2, the units half.
//
// These look trivial, and that is the point: every number in the project is
// scaled by one of these conversions, so a wrong constant here would be a
// silent 1.7% error in every reported angle.

#include <doctest/doctest.h>

#include "core/units.hpp"

using namespace sat;

TEST_CASE("angle conversions round-trip exactly") {
    // Degrees -> microradians -> degrees, across the range the gimbal uses.
    for (double deg = -180.0; deg <= 180.0; deg += 0.25) {
        CHECK(urad_to_deg(deg_to_urad(deg)) == doctest::Approx(deg).epsilon(1e-15));
    }
    // Radians likewise.
    CHECK(rad_to_urad(1.0) == doctest::Approx(1.0e6));
    CHECK(urad_to_rad(1.0e6) == doctest::Approx(1.0));
}

TEST_CASE("one degree is 17453.29 microradians") {
    // Sanity anchor: pi/180 * 1e6.
    CHECK(deg_to_urad(1.0) == doctest::Approx(17453.292519943295).epsilon(1e-12));
}

TEST_CASE("Vec2 arithmetic behaves component-wise") {
    const Pixel2 a{3.0, 4.0};
    const Pixel2 b{1.0, 2.0};

    CHECK((a + b) == Pixel2{4.0, 6.0});
    CHECK((a - b) == Pixel2{2.0, 2.0});
    CHECK((a * 2.0) == Pixel2{6.0, 8.0});
    CHECK((2.0 * a) == Pixel2{6.0, 8.0});
    CHECK((a / 2.0) == Pixel2{1.5, 2.0});
    CHECK((-a) == Pixel2{-3.0, -4.0});

    CHECK(a.norm_sq() == doctest::Approx(25.0));
    CHECK(a.norm() == doctest::Approx(5.0));
    CHECK(a.norm_inf() == doctest::Approx(4.0));

    Pixel2 c = a;
    c += b;  CHECK(c == Pixel2{4.0, 6.0});
    c -= b;  CHECK(c == a);
    c *= 3.0; CHECK(c == Pixel2{9.0, 12.0});
    c /= 3.0; CHECK(c == a);
}

TEST_CASE("Angle2, Pixel2 and Rate2 are distinct types") {
    // This is the whole reason for the tag parameter. If these ever become
    // interchangeable, a pixel offset could silently be added to an angular
    // state and nothing would complain.
    static_assert(!std::is_same_v<Angle2, Pixel2>);
    static_assert(!std::is_same_v<Angle2, Rate2>);
    static_assert(!std::is_same_v<Pixel2, Rate2>);
    // All three must stay trivially copyable: they are passed by value in the
    // hot loop and stored in arena-backed arrays.
    static_assert(std::is_trivially_copyable_v<Angle2>);
    static_assert(std::is_trivially_copyable_v<Pixel2>);
    static_assert(std::is_trivially_copyable_v<Rate2>);
}

TEST_CASE("pixel <-> microradian scaling is its own inverse") {
    const double ifov = 109.08307824964559;   // the default, see frames test
    const Pixel2 p{12.5, -7.25};
    const Angle2 a = px_to_urad(p, ifov);
    const Pixel2 back = urad_to_px(a, ifov);
    CHECK(back.x == doctest::Approx(p.x).epsilon(1e-15));
    CHECK(back.y == doctest::Approx(p.y).epsilon(1e-15));
}

TEST_CASE("px/frame -> urad/s reproduces the design doc's startup line") {
    // Design §9.3 insists this exact number is logged at startup:
    //   20 px/frame * 109.0830... urad/px * 30 Hz = 65449.8 urad/s = 3.75 deg/s
    // The doc rounds the IFOV to 109.08 and quotes 65,448; recomputing from the
    // unrounded IFOV gives 65449.85. Both round to 3.75 deg/s, which is the
    // figure that matters: 75% of a 5 deg/s motor's authority.
    const double ifov = deg_to_urad(4.0) / 640.0;
    const double rate = px_per_frame_to_urad_s(20.0, ifov, 30.0);
    CHECK(rate == doctest::Approx(65449.85).epsilon(1e-4));
    CHECK(urad_to_deg(rate) == doctest::Approx(3.75).epsilon(1e-6));
}

TEST_CASE("clamp helpers saturate symmetrically") {
    CHECK(clamp(5.0, 0.0, 3.0) == 3.0);
    CHECK(clamp(-5.0, 0.0, 3.0) == 0.0);
    CHECK(clamp(1.5, 0.0, 3.0) == 1.5);

    CHECK(clamp_abs(7.0, 2.0) == 2.0);
    CHECK(clamp_abs(-7.0, 2.0) == -2.0);
    CHECK(clamp_abs(1.0, 2.0) == 1.0);

    // Pan and tilt limit independently (spec rows 13 and 14 are separate rows,
    // and the motors are separate motors) -- this is a box, not a disc.
    const Rate2 r = clamp_rate(Rate2{100.0, -100.0}, 10.0, 20.0);
    CHECK(r.x == 10.0);
    CHECK(r.y == -20.0);
}

TEST_CASE("quantise models an encoder LSB") {
    // The gimbal's controller only ever sees a quantised position (design §10.3).
    CHECK(quantise(103.0, 20.0) == doctest::Approx(100.0));
    CHECK(quantise(111.0, 20.0) == doctest::Approx(120.0));
    CHECK(quantise(-103.0, 20.0) == doctest::Approx(-100.0));
    // A zero or negative step means "no encoder quantisation".
    CHECK(quantise(103.456, 0.0) == doctest::Approx(103.456));
}
