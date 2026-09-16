// search/pattern.hpp — CP 6.7. Where to look when you cannot see it.
//
// ---------------------------------------------------------------------------
// THE GEOMETRIC BOUND, STATED OPENLY
// ---------------------------------------------------------------------------
// Design §10.5 is blunt about this and it is worth repeating at the top of the
// file that implements it:
//
//     Screen 12.5° x 12.5°, camera 4° x 3° -> 20 tiles -> ~74.5° of travel
//     At 5 °/s  ~ 15 s        At 10 °/s ~ 7.5 s
//     P(beacon visible at t=0) = 7.68%
//     SPEC SAYS <= 2 s
//
// A cold search of the whole screen CANNOT meet spec row 16, by geometry, and
// no amount of cleverness in this file changes that. §10.5's instruction is to
// report both metrics clearly labelled — acquisition_cold_s and
// acquisition_in_fov_s — and to note that "deriving the bound and engineering
// around it is worth more under Understanding of the problem than a suspicious
// 1.8 s". full_sweep_time_s() below computes the bound from the actual
// configuration so the number in the report is never stale.
//
// ---------------------------------------------------------------------------
// WHAT THIS MODULE ACTUALLY BUYS, THEN
// ---------------------------------------------------------------------------
// Re-acquisition, which is spec row 19 (<= 1 s) and is achievable, because a
// re-acquiring search is not a cold one: the filter's prediction says where the
// target should be and its covariance says how wrong that could be. Searching
// outward from the prediction rather than from the screen centre is the whole
// of CP 6.7, and the difference is not marginal — it is the difference between
// 20 tiles and, typically, one.
//
// The spiral is centred on the prediction; the SIZE of the spiral's step is
// still the camera's field of view, so the first look almost always contains
// the target and the pattern only expands when it does not.
//
// SearchStrategy::Probabilistic and the negative-information grid of §10.5 are
// a Stage 13 concern (CP 13.1). This file implements Raster and Spiral, which
// are what CP 6.7 names, and leaves the enum room for the rest.

#pragma once

#include "core/frames.hpp"
#include "core/units.hpp"
#include "search/grid.hpp"

#include <cstdint>

namespace sat {

// ---------------------------------------------------------------------------
// spiral_cell — the k-th cell of a square spiral, in integer tile units.
//
// k = 0 is the centre, then right, up, left, left, down, down, ... the standard
// outward square spiral. Returned as tile INDICES so the caller can scale by
// whatever step it wants and clamp to whatever bounds it has.
//
// A closed form rather than an iterator, for two reasons: it makes "where will
// the pattern be 30 frames from now" answerable without simulating, which the
// GUI overlay uses; and a pure function of k is trivially testable against the
// property that every cell inside a ring is visited exactly once.
// ---------------------------------------------------------------------------
void spiral_cell(int k, int& ix, int& iy) noexcept;

/// The ring index (Chebyshev radius) a spiral step lands on. Ring r contains
/// the (2r+1)^2 - (2r-1)^2 = 8r cells at Chebyshev distance exactly r.
[[nodiscard]] int spiral_ring(int k) noexcept;

// ---------------------------------------------------------------------------
// SearchStrategy — §10.5's enum. Only two are implemented at Stage 6.
// ---------------------------------------------------------------------------
enum class SearchStrategy : uint8_t {
    Spiral = 0,      ///< outward from the centre, or from a prediction
    Raster,          ///< left-to-right, top-to-bottom; the naive baseline
    Probabilistic,   ///< CP 13.1
    CampAndWait,     ///< CP 13.2
    Adaptive         ///< CP 12.x, chosen by the SAT supervisor
};

[[nodiscard]] const char* search_strategy_name(SearchStrategy s) noexcept;

// ---------------------------------------------------------------------------
// SearchParams
// ---------------------------------------------------------------------------
struct SearchParams {
    SearchStrategy strategy = SearchStrategy::Spiral;

    /// Half the screen's angular extent, microradians. Looks outside this are
    /// skipped: there is nothing there to see.
    Angle2 half_extent{6.0e5, 6.0e5};

    /// Step between looks. Set from the camera's field of view by
    /// from_camera(), with an overlap so that a target sitting exactly on a
    /// tile boundary is not missed by both neighbours — which, with no overlap,
    /// is a real possibility rather than a theoretical one, because the gate in
    /// §9.4.7 needs the whole beacon present to pass the area test.
    Angle2 step{};

    /// Fraction of the field of view shared between adjacent looks.
    double overlap = 0.15;

    /// How long to sit on a look before moving to the next one. The camera has
    /// to actually arrive and expose at least one frame; moving on sooner
    /// searches the space at the speed of the slew and sees none of it.
    double dwell_s = 0.10;

    /// How close the boresight must be to the look point to count as arrived,
    /// as a fraction of the step. Waiting for exact arrival would stall
    /// forever, because the mount has a finite time constant and the
    /// controller has finite gain.
    double arrive_frac = 0.25;

    [[nodiscard]] static SearchParams from_camera(const CameraGeometry& cam,
                                                  const ScreenGeometry& screen,
                                                  double overlap_ = 0.15) noexcept;
};

/// §10.5's bound: seconds to sweep the whole screen at `max_rate_urad_s`,
/// counting travel only. Computed from the configuration rather than quoted, so
/// the report cannot drift from the code.
[[nodiscard]] double full_sweep_time_s(const SearchParams& p,
                                       double max_rate_urad_s) noexcept;

/// How many looks the pattern needs to cover the screen.
[[nodiscard]] int tile_count(const SearchParams& p) noexcept;

// ---------------------------------------------------------------------------
// SearchPattern — the stateful driver.
//
// It owns one thing: which look point is current. The controller still does the
// pointing, and the mode FSM still decides whether the search owns the aim
// point at all (§6.2 step B25).
// ---------------------------------------------------------------------------
class SearchPattern {
public:
    /// `screen` is needed only by the Probabilistic strategy's grid; the
    /// open-loop patterns ignore it. Passed always rather than conditionally,
    /// because a strategy switched on at runtime (the supervisor can do that)
    /// must not find its grid unconfigured.
    void reset(const SearchParams& p, Angle2 centre,
               const ScreenGeometry& screen = ScreenGeometry{}) noexcept;

    /// Re-centre without restarting the pattern's phase. Used when the target
    /// is lost and the last known position is a far better prior than the
    /// screen centre.
    void recentre(Angle2 centre) noexcept;

    /// Restart from the centre, index 0. Used on entering Search.
    void restart() noexcept;

    // -----------------------------------------------------------------------
    // step — one frame.
    //
    //   dt          seconds since the last call
    //   boresight   where the mount actually is, so arrival can be judged
    //
    // Returns the angle the controller should aim at this frame.
    // -----------------------------------------------------------------------
    Angle2 step(double dt, Angle2 boresight) noexcept;

    // -----------------------------------------------------------------------
    // CP 13.1/13.2: the two closed-loop strategies.
    //
    // Spiral and Raster are OPEN LOOP — their next look does not depend on what
    // the previous ones found — so they need nothing from the caller but dt.
    // Probabilistic and CampAndWait do, which is why they get their own entry
    // point rather than a flag inside step():
    //
    //   Probabilistic  needs the grid updated with what this look saw, and the
    //                  camera geometry to know what "this look" covered.
    //   CampAndWait    needs to know whether anything has been seen at all.
    //
    // A caller that uses step() for a closed-loop strategy gets the open-loop
    // behaviour rather than silently-wrong closed-loop behaviour, which is the
    // failure mode worth designing out.
    // -----------------------------------------------------------------------
    Angle2 step_informed(double dt, Angle2 boresight, const CameraGeometry& cam,
                         bool detected_this_frame, double target_speed_px_s,
                         double max_rate_urad_s) noexcept;

    [[nodiscard]] ProbabilityGrid&       grid()       noexcept { return grid_; }
    [[nodiscard]] const ProbabilityGrid& grid() const noexcept { return grid_; }

    /// The current look point, without advancing.
    [[nodiscard]] Angle2 look() const noexcept { return look_; }

    [[nodiscard]] int index()          const noexcept { return index_; }
    [[nodiscard]] int looks_visited()  const noexcept { return visited_; }
    [[nodiscard]] bool swept_once()    const noexcept { return swept_; }
    [[nodiscard]] Angle2 centre()      const noexcept { return centre_; }

private:
    /// Advance to the next in-bounds look, wrapping when the screen is covered.
    void advance() noexcept;
    [[nodiscard]] Angle2 point_for(int k) const noexcept;
    [[nodiscard]] bool   in_bounds(Angle2 a) const noexcept;
    [[nodiscard]] int    max_ring() const noexcept;

    SearchParams    p_{};
    ProbabilityGrid grid_{};
    double          regrid_accum_ = 0.0;
    Angle2 centre_{};
    Angle2 look_{};
    int    index_    = 0;
    int    visited_  = 0;
    double dwell_    = 0.0;
    bool   arrived_  = false;
    bool   swept_    = false;
};

}  // namespace sat
