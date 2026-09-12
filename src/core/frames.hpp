// core/frames.hpp — coordinate frames and the only sanctioned conversions
// between them.
//
// There are exactly three frames in this program. Getting them confused is the
// single richest source of bugs in a pointing system, so they are spelled out
// here once and everything else refers back to this file.
//
//   1. IMAGE pixels    — where a detection is found. Origin at the CENTRE of
//                        the top-left pixel, x right, y down. 640x480 default.
//   2. ANGLE (urad)    — the world-fixed pointing frame. This is where all
//                        tracker state lives (INV-5). Origin at the centre of
//                        the simulated screen.
//   3. SCREEN pixels   — the 2000x2000 canvas the spec describes, and the frame
//                        the graded centroid log reports in (design §13.2).
//                        Origin at the centre of the top-left screen pixel.
//
// The conversions are:
//
//        image px  --unproject-->  angular OFFSET from boresight
//                                        + boresight
//                                        = world angle
//        world angle --angle_to_screen--> screen px
//
// and their exact inverses. Round-tripping must be exact to ~1e-9 (CP 0.2), and
// it is, because every step is a multiply and an add.
//
// ---------------------------------------------------------------------------
// TWO CONVENTIONS WORTH READING BEFORE YOU TOUCH ANYTHING
// ---------------------------------------------------------------------------
//
// (a) The projection is LINEAR (equidistant), not gnomonic/pinhole:
//
//         offset_urad = (pixel - principal_point) * ifov_urad
//
//     This is not a simplification we are sneaking past you — it is the model
//     the specification itself uses. Design §1.4 derives the screen's angular
//     extent as "2000 px x 0.00625 deg/px = 12.5 deg", i.e. a constant angular
//     size per pixel across the whole canvas. A tangent-plane (pinhole) model
//     would make the screen 12.53 deg across and would make IFOV vary by 0.1%
//     from centre to corner. Over a 4 deg field the two models differ by about
//     0.13 px at the very corner. Choosing the linear model means:
//       * the screen<->angle map is exact and invertible everywhere;
//       * the measurement equation is linear, so §10.2's plain Kalman filter
//         (not an EKF) is exactly right rather than approximately right;
//       * the derived constants in §1.4 hold literally.
//     If a future revision of the design doc calls for a real camera model,
//     this is the one file to change.
//
// (b) The elevation axis points DOWN, the same way image rows and screen rows
//     do. "Azimuth/elevation" here means "the two gimbal axes", not a
//     celestial convention. Keeping every y axis pointing the same way removes
//     every sign flip from the pipeline, and nothing physical in this problem
//     cares: the gimbal's tilt limits are symmetric (spec row 14), and the
//     scenario describes target motion in screen pixels (spec rows 11, 12).

#pragma once

#include "core/units.hpp"

#include <cstdint>

namespace sat {

// ---------------------------------------------------------------------------
// CameraGeometry — the sensor's fixed optical description.
//
// Built once at scenario load from spec rows 3 (resolution) and 4 (FOV), then
// treated as immutable for the run. Trivially copyable so it can be passed by
// value into hot functions.
// ---------------------------------------------------------------------------
struct CameraGeometry {
  int width = 640; // spec row 3, user-defined
  int height = 480;
  double fov_x_deg = 4.0; // spec row 4, user-defined, default 4 x 3
  double fov_y_deg = 3.0;

  // Derived once in make(); stored so the hot path never divides.
  double ifov_x_urad = 0.0; // microradians per pixel, horizontal
  double ifov_y_urad = 0.0;
  double cx = 0.0; // principal point, image pixels
  double cy = 0.0;

  // -----------------------------------------------------------------------
  // Factory. Use this rather than aggregate-initialising, so the derived
  // members can never go stale.
  //
  // IFOV uses `width`, not `width - 1`: the field of view spans the full
  // angular extent of the sensor, and there are `width` pixels across it.
  // With the defaults this yields 4 deg / 640 = 0.00625 deg/px = 109.0831
  // urad/px, which is the number design §1.4 asks us to log at startup.
  //
  // The principal point is (width - 1) / 2 because pixel coordinates name
  // pixel CENTRES: a 640-wide sensor has centres at 0 .. 639, so the optical
  // axis falls at 319.5.
  // -----------------------------------------------------------------------
  [[nodiscard]] static constexpr CameraGeometry
  make(int w, int h, double fov_x_deg_, double fov_y_deg_) noexcept {
    CameraGeometry g{};
    g.width = w;
    g.height = h;
    g.fov_x_deg = fov_x_deg_;
    g.fov_y_deg = fov_y_deg_;
    g.ifov_x_urad = deg_to_urad(fov_x_deg_) / static_cast<double>(w);
    g.ifov_y_urad = deg_to_urad(fov_y_deg_) / static_cast<double>(h);
    g.cx = static_cast<double>(w - 1) * 0.5;
    g.cy = static_cast<double>(h - 1) * 0.5;
    return g;
  }

  /// The scalar IFOV quoted in reports and log headers. Averaging the two
  /// axes is honest because the defaults are square (4/640 == 3/480) and a
  /// scenario that makes them differ is asking for anisotropic pixels, which
  /// the per-axis values above already handle correctly.
  [[nodiscard]] constexpr double ifov_urad() const noexcept {
    return 0.5 * (ifov_x_urad + ifov_y_urad);
  }

  /// Number of pixels in one frame — the size every image buffer must be.
  [[nodiscard]] constexpr int pixel_count() const noexcept {
    return width * height;
  }

  /// Half-extents of the field of view in microradians, measured from the
  /// boresight. Used for the visibility AABB in design §9.1.
  [[nodiscard]] constexpr Angle2 half_fov_urad() const noexcept {
    return {0.5 * deg_to_urad(fov_x_deg), 0.5 * deg_to_urad(fov_y_deg)};
  }

  // -----------------------------------------------------------------------
  // unproject: image pixel -> angular offset from the boresight.
  //
  // The result is an OFFSET, not a world angle. Add the boresight to get a
  // world angle; that addition is design §6.2 step B16 and it is deliberately
  // left to the caller, because the caller is the only one who knows *which*
  // boresight applies (commanded, true, or estimated).
  // -----------------------------------------------------------------------
  [[nodiscard]] constexpr Angle2 unproject(Pixel2 p) const noexcept {
    return {(p.x - cx) * ifov_x_urad, (p.y - cy) * ifov_y_urad};
  }

  /// project: angular offset from the boresight -> image pixel.
  /// Exact inverse of unproject (CP 0.2 acceptance test).
  [[nodiscard]] constexpr Pixel2 project(Angle2 a) const noexcept {
    return {a.x / ifov_x_urad + cx, a.y / ifov_y_urad + cy};
  }

  /// Is this pixel inside the sensor? Uses the half-open [-0.5, n-0.5) range
  /// so that "the pixel whose centre is at 0" is inside and a coordinate of
  /// exactly `width - 0.5` is outside. Sub-pixel positions are the norm here,
  /// so an integer-index test would be wrong.
  [[nodiscard]] constexpr bool contains(Pixel2 p) const noexcept {
    return p.x >= -0.5 && p.x < static_cast<double>(width) - 0.5 &&
           p.y >= -0.5 && p.y < static_cast<double>(height) - 0.5;
  }
};

// ---------------------------------------------------------------------------
// ScreenGeometry — the 2000x2000 canvas of spec row 1.
//
// The screen never exists as a raster in synthetic mode (design §9.1, decision
// 10); it is only a coordinate system. Its angular scale is the camera's IFOV,
// which is what makes "the camera sees 7.68% of the screen" true.
// ---------------------------------------------------------------------------
struct ScreenGeometry {
  int width = 2000; // spec row 1, minimum 2000 x 2000
  int height = 2000;
  double ifov_x_urad = 0.0; // inherited from the camera
  double ifov_y_urad = 0.0;
  double cx = 0.0; // canvas centre, screen pixels
  double cy = 0.0;

  [[nodiscard]] static constexpr ScreenGeometry
  make(int w, int h, const CameraGeometry &cam) noexcept {
    ScreenGeometry s{};
    s.width = w;
    s.height = h;
    s.ifov_x_urad = cam.ifov_x_urad;
    s.ifov_y_urad = cam.ifov_y_urad;
    s.cx = static_cast<double>(w - 1) * 0.5;
    s.cy = static_cast<double>(h - 1) * 0.5;
    return s;
  }

  /// Screen pixel -> world angle. The canvas centre is the angular origin,
  /// which is also spec row 6's default initial camera position.
  [[nodiscard]] constexpr Angle2 to_angle(Pixel2 p) const noexcept {
    return {(p.x - cx) * ifov_x_urad, (p.y - cy) * ifov_y_urad};
  }

  /// World angle -> screen pixel. Exact inverse of to_angle.
  [[nodiscard]] constexpr Pixel2 to_pixel(Angle2 a) const noexcept {
    return {a.x / ifov_x_urad + cx, a.y / ifov_y_urad + cy};
  }

  /// Angular extent of the whole canvas, in microradians (12.5 deg square at
  /// the defaults — design §1.4).
  [[nodiscard]] constexpr Angle2 extent_urad() const noexcept {
    return {static_cast<double>(width) * ifov_x_urad,
            static_cast<double>(height) * ifov_y_urad};
  }

  [[nodiscard]] constexpr bool contains(Pixel2 p) const noexcept {
    return p.x >= -0.5 && p.x < static_cast<double>(width) - 0.5 &&
           p.y >= -0.5 && p.y < static_cast<double>(height) - 0.5;
  }
};

// ---------------------------------------------------------------------------
// The two compound conversions, written once so nobody re-derives them.
//
// These are the functions design §6.2 steps B16 and §13.2 actually need. Note
// they take the boresight explicitly: there is no ambient "current pointing"
// hiding in a global, because perception must never be able to reach one.
// ---------------------------------------------------------------------------

/// Image pixel + boresight -> screen pixel. This is how a detection becomes the
/// `cx_screen,cy_screen` columns of the graded centroid log.
[[nodiscard]] constexpr Pixel2 image_to_screen(const CameraGeometry &cam,
                                               const ScreenGeometry &scr,
                                               Pixel2 image_px,
                                               Angle2 boresight) noexcept {
  return scr.to_pixel(boresight + cam.unproject(image_px));
}

/// Screen pixel + boresight -> image pixel. The inverse; used by the simulator
/// to place an emitter on the sensor, and by the video sources to work out
/// which part of a decoded frame the camera is looking at.
[[nodiscard]] constexpr Pixel2 screen_to_image(const CameraGeometry &cam,
                                               const ScreenGeometry &scr,
                                               Pixel2 screen_px,
                                               Angle2 boresight) noexcept {
  return cam.project(scr.to_angle(screen_px) - boresight);
}

// ---------------------------------------------------------------------------
// Aabb — an axis-aligned box in screen pixels.
//
// Used for the visible-emitter query (design §9.1): a linear AABB scan over
// fewer than a thousand emitters beats any spatial index, so this stays a
// plain struct with no acceleration structure behind it.
// ---------------------------------------------------------------------------
struct Aabb {
  double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;

  [[nodiscard]] constexpr bool contains(double x, double y) const noexcept {
    return x >= x0 && x <= x1 && y >= y0 && y <= y1;
  }
  [[nodiscard]] constexpr bool intersects(const Aabb &o) const noexcept {
    return !(o.x1 < x0 || o.x0 > x1 || o.y1 < y0 || o.y0 > y1);
  }
  /// Grow the box by `m` on every side. The splat loop uses this to include
  /// emitters whose centre is outside the viewport but whose body is not.
  [[nodiscard]] constexpr Aabb expanded(double m) const noexcept {
    return {x0 - m, y0 - m, x1 + m, y1 + m};
  }
};

/// The screen-space box the camera currently sees. Built from the two opposite
/// corners of the sensor so it stays correct if a future change makes the
/// projection non-trivial.
[[nodiscard]] constexpr Aabb view_aabb(const CameraGeometry &cam,
                                       const ScreenGeometry &scr,
                                       Angle2 boresight) noexcept {
  const Pixel2 tl = image_to_screen(cam, scr, Pixel2{-0.5, -0.5}, boresight);
  const Pixel2 br =
      image_to_screen(cam, scr,
                      Pixel2{static_cast<double>(cam.width) - 0.5,
                             static_cast<double>(cam.height) - 0.5},
                      boresight);
  return {tl.x, tl.y, br.x, br.y};
}

} // namespace sat
