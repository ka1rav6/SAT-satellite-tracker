// tests/frames/test_frames.cpp — CP 0.2 acceptance.
//
// "Test: project(unproject(p)) == p to 1e-9 for 1000 points across the FOV;
//  ifov_urad() returns 109.08 at defaults."
//
// Plus the compound image<->screen conversions, because those are what the
// graded centroid log (design §13.2) is actually written in.

#include <doctest/doctest.h>

#include "core/frames.hpp"

using namespace sat;

namespace {
// The spec defaults: rows 1, 3, 4.
CameraGeometry default_camera() { return CameraGeometry::make(640, 480, 4.0, 3.0); }
ScreenGeometry default_screen() { return ScreenGeometry::make(2000, 2000, default_camera()); }
}  // namespace

TEST_CASE("ifov_urad() is 109.08 at the spec defaults") {
    const auto cam = default_camera();
    // Design §1.4: IFOV = 4 deg / 640 = 0.00625 deg/px = 109.08 urad/px.
    CHECK(cam.ifov_urad() == doctest::Approx(109.08).epsilon(1e-4));
    CHECK(urad_to_deg(cam.ifov_urad()) == doctest::Approx(0.00625).epsilon(1e-12));

    // The default FOV is square in angular terms: 4/640 == 3/480. If a future
    // change breaks that, the per-axis values below are what keeps the maths
    // right, and this check will say so loudly.
    CHECK(cam.ifov_x_urad == doctest::Approx(cam.ifov_y_urad).epsilon(1e-15));
}

TEST_CASE("project(unproject(p)) == p across the whole field of view") {
    const auto cam = default_camera();

    // 1000+ points on a grid spanning the sensor, deliberately at non-integer
    // positions: integer pixels would hide a principal-point off-by-half.
    int checked = 0;
    for (int iy = 0; iy < 32; ++iy) {
        for (int ix = 0; ix < 32; ++ix) {
            const Pixel2 p{ix * (639.0 / 31.0) + 0.37,
                           iy * (479.0 / 31.0) - 0.11};
            const Pixel2 back = cam.project(cam.unproject(p));
            CHECK(std::abs(back.x - p.x) < 1e-9);
            CHECK(std::abs(back.y - p.y) < 1e-9);
            ++checked;
        }
    }
    CHECK(checked >= 1000);
}

TEST_CASE("unproject(project(a)) == a for angular offsets") {
    const auto cam = default_camera();
    const Angle2 half = cam.half_fov_urad();

    for (int i = 0; i < 1000; ++i) {
        const double f = -1.0 + 2.0 * (static_cast<double>(i) / 999.0);
        const Angle2 a{half.x * f, half.y * -f};
        const Angle2 back = cam.unproject(cam.project(a));
        CHECK(std::abs(back.x - a.x) < 1e-9);
        CHECK(std::abs(back.y - a.y) < 1e-9);
    }
}

TEST_CASE("the boresight maps to the principal point and to zero offset") {
    const auto cam = default_camera();
    CHECK(cam.cx == doctest::Approx(319.5));
    CHECK(cam.cy == doctest::Approx(239.5));

    const Angle2 zero = cam.unproject(Pixel2{cam.cx, cam.cy});
    CHECK(zero.x == doctest::Approx(0.0).epsilon(1e-15));
    CHECK(zero.y == doctest::Approx(0.0).epsilon(1e-15));
}

TEST_CASE("the screen spans 12.5 degrees at the defaults") {
    const auto scr = default_screen();
    const Angle2 e = scr.extent_urad();
    // Design §1.4: 2000 px x 0.00625 deg/px = 12.5 deg on each axis.
    CHECK(urad_to_deg(e.x) == doctest::Approx(12.5).epsilon(1e-9));
    CHECK(urad_to_deg(e.y) == doctest::Approx(12.5).epsilon(1e-9));
}

TEST_CASE("screen pixel <-> angle round-trips exactly") {
    const auto scr = default_screen();
    for (int i = 0; i < 1000; ++i) {
        const double t = static_cast<double>(i) / 999.0;
        const Pixel2 p{t * 1999.0 + 0.234, (1.0 - t) * 1999.0 - 0.567};
        const Pixel2 back = scr.to_pixel(scr.to_angle(p));
        CHECK(std::abs(back.x - p.x) < 1e-9);
        CHECK(std::abs(back.y - p.y) < 1e-9);
    }
}

TEST_CASE("the canvas centre is the angular origin (spec row 6)") {
    const auto scr = default_screen();
    // Spec row 6: "Initial camera position = centre of screen". With a 2000 px
    // canvas the centre of the pixel grid is 999.5, and that is zero angle.
    const Angle2 a = scr.to_angle(Pixel2{999.5, 999.5});
    CHECK(a.x == doctest::Approx(0.0).epsilon(1e-15));
    CHECK(a.y == doctest::Approx(0.0).epsilon(1e-15));
}

TEST_CASE("image <-> screen round-trips at an arbitrary boresight") {
    const auto cam = default_camera();
    const auto scr = default_screen();

    // A boresight well away from centre, at a deliberately awkward angle.
    const Angle2 bore = scr.to_angle(Pixel2{1423.8, 674.2});

    for (int iy = 0; iy < 16; ++iy) {
        for (int ix = 0; ix < 16; ++ix) {
            const Pixel2 img{ix * 42.0 + 0.31, iy * 31.0 + 0.77};
            const Pixel2 screen = image_to_screen(cam, scr, img, bore);
            const Pixel2 back   = screen_to_image(cam, scr, screen, bore);
            CHECK(std::abs(back.x - img.x) < 1e-9);
            CHECK(std::abs(back.y - img.y) < 1e-9);
        }
    }
}

TEST_CASE("a centred boresight puts the sensor centre at the canvas centre") {
    const auto cam = default_camera();
    const auto scr = default_screen();
    const Angle2 bore{0.0, 0.0};

    const Pixel2 s = image_to_screen(cam, scr, Pixel2{cam.cx, cam.cy}, bore);
    CHECK(s.x == doctest::Approx(scr.cx));
    CHECK(s.y == doctest::Approx(scr.cy));
}

TEST_CASE("the view AABB is 640x480 screen pixels and follows the boresight") {
    const auto cam = default_camera();
    const auto scr = default_screen();

    // Because the screen and the sensor share an IFOV, one sensor pixel is
    // exactly one screen pixel: the camera window is literally 640x480 of the
    // 2000x2000 canvas. That is what makes design §1.4's "FOV fraction of
    // screen = 7.68%" true.
    const Aabb box = view_aabb(cam, scr, Angle2{0.0, 0.0});
    CHECK((box.x1 - box.x0) == doctest::Approx(640.0).epsilon(1e-9));
    CHECK((box.y1 - box.y0) == doctest::Approx(480.0).epsilon(1e-9));

    const double frac = (640.0 * 480.0) / (2000.0 * 2000.0);
    CHECK(frac == doctest::Approx(0.0768));

    // Slew right by 100 screen pixels; the window must move with it.
    const Angle2 bore = scr.to_angle(Pixel2{999.5 + 100.0, 999.5});
    const Aabb moved = view_aabb(cam, scr, bore);
    CHECK(moved.x0 == doctest::Approx(box.x0 + 100.0).epsilon(1e-9));
    CHECK(moved.y0 == doctest::Approx(box.y0).epsilon(1e-9));
}

TEST_CASE("contains() uses half-open pixel bounds") {
    const auto cam = default_camera();
    CHECK(cam.contains(Pixel2{0.0, 0.0}));
    CHECK(cam.contains(Pixel2{-0.5, -0.5}));
    CHECK(cam.contains(Pixel2{639.0, 479.0}));
    CHECK_FALSE(cam.contains(Pixel2{639.5, 479.0}));
    CHECK_FALSE(cam.contains(Pixel2{-0.51, 0.0}));
}

TEST_CASE("Aabb intersection and expansion") {
    const Aabb a{0.0, 0.0, 10.0, 10.0};
    CHECK(a.contains(5.0, 5.0));
    CHECK_FALSE(a.contains(11.0, 5.0));

    CHECK(a.intersects(Aabb{9.0, 9.0, 20.0, 20.0}));
    CHECK_FALSE(a.intersects(Aabb{10.1, 0.0, 20.0, 20.0}));

    // The splat loop expands the view box by the largest emitter radius so a
    // beacon whose centre is just outside still contributes its visible edge.
    CHECK(a.expanded(5.0).intersects(Aabb{12.0, 5.0, 13.0, 6.0}));
}
