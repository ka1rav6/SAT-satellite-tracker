// search/grid.hpp — CP 13.1's probability grid.
//
// ---------------------------------------------------------------------------
// THE IDEA IN ONE SENTENCE
// ---------------------------------------------------------------------------
// §10.5: "Looking somewhere and seeing nothing IS EVIDENCE; raster scans
// discard it."
//
// A raster or spiral pattern is open-loop: it visits tiles in a fixed order and
// its next look does not depend on what the previous ones found. That is fine
// when nothing is known, and it throws away the only information a fruitless
// search produces. Every tile looked at and found empty makes the remaining
// tiles more likely, and a search that tracks that belief finishes sooner —
// not because it moves faster, but because it stops revisiting places it has
// already ruled out.
//
// ---------------------------------------------------------------------------
// WHY IT NEEDS DIFFUSION, AND WHY THAT IS THE HARD HALF
// ---------------------------------------------------------------------------
// The naive version rules a tile out permanently, which is wrong for a MOVING
// target: a tile checked ten seconds ago and found empty may hold the beacon
// now, because the beacon moved there. So the belief has to spread over time at
// a rate set by how fast the target can travel.
//
// That is what makes the probabilistic strategy interesting rather than
// obvious. Too little diffusion and the search convinces itself the target is
// nowhere; too much and the grid returns to uniform between looks and the
// strategy degenerates to "look wherever is closest", which is worse than a
// spiral because it has no coverage guarantee at all.
//
// ---------------------------------------------------------------------------
// WHY 32x32, AND WHY THAT IS NOT ARBITRARY
// ---------------------------------------------------------------------------
// §10.5 specifies it, and the number matches the geometry. The screen is
// 12.5 deg and the camera sees 4 x 3 deg, so a field of view is about a third
// of the screen's width — roughly 10 cells across at 32. A grid much coarser
// than the field of view cannot represent "I have looked at half of this
// cell"; one much finer spends its resolution on a distinction no single look
// can make.
//
// INV-4: the grid is a fixed-size array, 4 KB, and nothing here allocates.

#pragma once

#include "core/frames.hpp"
#include "core/units.hpp"

#include <array>
#include <cstdint>

namespace sat {

struct GridParams {
    /// Probability that ONE look at a cell fully inside the field of view
    /// detects a beacon that is there. Not 1: §9.4's detector misses, fog
    /// happens, and a grid that assumed perfect detection would rule out a
    /// tile the beacon was sitting in.
    float p_detect = 0.9f;

    /// How fast the belief spreads, as a multiple of the target's own top
    /// speed. 1.0 means "the target could be anywhere it could have reached";
    /// the default is slightly generous because the speed bound is itself an
    /// estimate.
    float diffusion_scale = 1.2f;

    /// Seconds a look is assumed to take, used by best_look's denominator to
    /// stop it thrashing across the screen for a marginally better cell.
    double dwell_s = 0.10;

    // -----------------------------------------------------------------------
    // Floor on a single cell, as a fraction of uniform.
    //
    // Not belt-and-braces: without it the grid REACHES zero, and zero is
    // absorbing. observe() and diffuse() are both multiplicative, so a cell at
    // exactly zero can never be brought back by any amount of later evidence,
    // and a beacon that wandered into a region searched early would be
    // permanently unfindable.
    //
    // It is not a theoretical worry. Each fruitless look at a fully-covered
    // cell multiplies it by (1 - 0.9) = 0.1, so 45 looks underflow a float to
    // exactly 0.0 — and a search that stares at one region while the target is
    // elsewhere does far more than 45 looks over a long run. Measured: after
    // 500 looks the lowest cell was 0.
    //
    // 1e-6 of uniform is nine looks' worth of evidence below the starting
    // point, which is far enough down to be ignored by best_look and far
    // enough above zero to recover.
    // -----------------------------------------------------------------------
    float min_cell_frac = 1e-6f;
};

// ---------------------------------------------------------------------------
// ProbabilityGrid — where the beacon probably is.
// ---------------------------------------------------------------------------
class ProbabilityGrid {
public:
    static constexpr int NX = 32;
    static constexpr int NY = 32;
    static constexpr int kCells = NX * NY;

    void reset(const GridParams& p, const ScreenGeometry& screen) noexcept;

    /// Spec row 11's "random" initial position: everywhere is equally likely.
    void reset_uniform() noexcept;

    /// A track was just lost HERE. §10.5's reacquisition prior, and CP 6.7
    /// measures what it is worth: reacquisition in 3 frames rather than a
    /// cold sweep's 18.7 s.
    void reset_from_prior(Pixel2 mean_px, double sigma_px) noexcept;

    // -----------------------------------------------------------------------
    // observe — NEGATIVE INFORMATION.
    //
    // The camera looked at `boresight` and reported `found`. Cells inside the
    // field of view are multiplied by (1 - p_detect) when nothing was found,
    // and the whole grid is renormalised.
    //
    // Partial coverage matters and is handled: a cell half inside the field of
    // view gets half the update, because ruling out a cell the camera only
    // half saw is exactly the error that makes a probabilistic search skip the
    // tile the beacon is in.
    // -----------------------------------------------------------------------
    void observe(Angle2 boresight, const CameraGeometry& cam, bool found) noexcept;

    /// Spread the belief to account for a target that may have moved.
    void diffuse(double dt_s, double target_speed_px_s) noexcept;

    // -----------------------------------------------------------------------
    // best_look — where to point next.
    //
    // §10.5: maximise `probability_mass_in_fov(cell) / (travel_time + dwell)`.
    // The denominator is the whole design. Without it the strategy picks the
    // single most likely cell every time and spends the run crossing the
    // screen, which is slower than a spiral AND has no coverage guarantee.
    // With it the choice is a RATE — probability per second — which is the
    // quantity a search is actually trying to maximise.
    // -----------------------------------------------------------------------
    [[nodiscard]] Angle2 best_look(Angle2 current, const CameraGeometry& cam,
                                   double max_rate_urad_s) const noexcept;

    [[nodiscard]] float cell(int ix, int iy) const noexcept {
        return p_[static_cast<size_t>(iy * NX + ix)];
    }
    /// Total belief inside one field of view centred on `boresight`. The GUI's
    /// heatmap and CP 13.1's acceptance both read this.
    [[nodiscard]] double mass_in_fov(Angle2 boresight,
                                     const CameraGeometry& cam) const noexcept;

    [[nodiscard]] const std::array<float, kCells>& cells() const noexcept { return p_; }

private:
    void  normalise() noexcept;
    /// Fractional overlap of cell (ix, iy) with the field of view, in [0, 1].
    [[nodiscard]] float overlap(int ix, int iy, Angle2 boresight,
                                const CameraGeometry& cam) const noexcept;
    [[nodiscard]] Pixel2 cell_centre_px(int ix, int iy) const noexcept;

    GridParams     p_params_{};
    ScreenGeometry screen_{};
    double         cell_w_px_ = 1.0;
    double         cell_h_px_ = 1.0;
    std::array<float, kCells> p_{};
    std::array<float, kCells> scratch_{};   ///< diffusion needs a second buffer
};

}  // namespace sat
