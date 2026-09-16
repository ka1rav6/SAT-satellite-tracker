// search/grid.cpp — CP 13.1.

#include "search/grid.hpp"

#include <algorithm>
#include <cmath>

namespace sat {

void ProbabilityGrid::reset(const GridParams& p, const ScreenGeometry& screen) noexcept {
    p_params_ = p;
    screen_   = screen;
    cell_w_px_ = static_cast<double>(screen.width)  / NX;
    cell_h_px_ = static_cast<double>(screen.height) / NY;
    reset_uniform();
}

void ProbabilityGrid::reset_uniform() noexcept {
    p_.fill(1.0f / static_cast<float>(kCells));
}

void ProbabilityGrid::reset_from_prior(Pixel2 mean_px, double sigma_px) noexcept {
    // A Gaussian bump, floored. The floor matters more than the bump: a cell at
    // exactly zero can never recover, because observe() and diffuse() are both
    // multiplicative, so a prior that zeroed the far side of the screen would
    // make a target that jumped there permanently unfindable. The same
    // absorbing-zero argument as the IMM's mode probabilities (tracking/imm.cpp).
    const double s2 = std::max(sigma_px * sigma_px, 1.0);
    const float floor_p = 0.05f / static_cast<float>(kCells);
    for (int iy = 0; iy < NY; ++iy) {
        for (int ix = 0; ix < NX; ++ix) {
            const Pixel2 c = cell_centre_px(ix, iy);
            const double dx = c.x - mean_px.x, dy = c.y - mean_px.y;
            const double g = std::exp(-0.5 * (dx * dx + dy * dy) / s2);
            p_[static_cast<size_t>(iy * NX + ix)] =
                floor_p + static_cast<float>(g);
        }
    }
    normalise();
}

Pixel2 ProbabilityGrid::cell_centre_px(int ix, int iy) const noexcept {
    return Pixel2{(static_cast<double>(ix) + 0.5) * cell_w_px_,
                  (static_cast<double>(iy) + 0.5) * cell_h_px_};
}

// ---------------------------------------------------------------------------
// overlap — the fraction of a cell covered by the field of view.
//
// Computed as a rectangle intersection in SCREEN pixels, not as a test of
// whether the cell's centre is inside. A centre test rules out a cell the
// camera only half saw, which is precisely the error that makes a
// probabilistic search skip the tile the beacon is in — and it is a silent
// error, because the search still looks busy.
// ---------------------------------------------------------------------------
float ProbabilityGrid::overlap(int ix, int iy, Angle2 boresight,
                               const CameraGeometry& cam) const noexcept {
    const Pixel2 bore_px = screen_.to_pixel(boresight);
    const Angle2 half    = cam.half_fov_urad();
    const double hw = half.x / std::max(1e-9, screen_.ifov_x_urad);
    const double hh = half.y / std::max(1e-9, screen_.ifov_y_urad);

    const double cx0 = static_cast<double>(ix) * cell_w_px_;
    const double cy0 = static_cast<double>(iy) * cell_h_px_;

    const double ox = std::min(cx0 + cell_w_px_, bore_px.x + hw)
                    - std::max(cx0, bore_px.x - hw);
    const double oy = std::min(cy0 + cell_h_px_, bore_px.y + hh)
                    - std::max(cy0, bore_px.y - hh);
    if (ox <= 0.0 || oy <= 0.0) return 0.0f;
    return static_cast<float>((ox * oy) / (cell_w_px_ * cell_h_px_));
}

void ProbabilityGrid::observe(Angle2 boresight, const CameraGeometry& cam,
                              bool found) noexcept {
    if (found) {
        // A detection is not this class's business: the tracker owns the
        // target once it is seen, and re-weighting the grid toward a cell the
        // tracker is already locked on would only matter if the lock were then
        // lost — at which point reset_from_prior is the right call and carries
        // the filter's own covariance rather than a grid cell's.
        return;
    }
    for (int iy = 0; iy < NY; ++iy) {
        for (int ix = 0; ix < NX; ++ix) {
            const float f = overlap(ix, iy, boresight, cam);
            if (f <= 0.0f) continue;
            // Partial coverage gets a partial update: seeing nothing in half a
            // cell is half the evidence that seeing nothing in all of it would
            // be.
            p_[static_cast<size_t>(iy * NX + ix)] *= (1.0f - p_params_.p_detect * f);
        }
    }
    normalise();
}

// ---------------------------------------------------------------------------
// diffuse — the target may have moved.
//
// A separable 3-tap blur whose weight is set by how far the target could have
// travelled in dt, in CELLS. This is the half that makes the strategy work at
// all: without it the grid rules tiles out permanently, and for a moving target
// that is simply wrong — a tile checked ten seconds ago may hold the beacon
// now, because the beacon moved there.
//
// The weight is clamped at the value that makes the kernel uniform. Past that
// the target could have crossed more than a cell per step, the grid cannot
// represent where it went, and the honest answer is "anywhere nearby" rather
// than an oscillating kernel that would make the belief ring.
// ---------------------------------------------------------------------------
void ProbabilityGrid::diffuse(double dt_s, double target_speed_px_s) noexcept {
    if (dt_s <= 0.0 || target_speed_px_s <= 0.0) return;

    const double reach_px = target_speed_px_s * dt_s * p_params_.diffusion_scale;
    const double cells_x  = reach_px / std::max(1e-9, cell_w_px_);
    const double cells_y  = reach_px / std::max(1e-9, cell_h_px_);
    const float wx = static_cast<float>(std::min(0.5, cells_x * 0.5));
    const float wy = static_cast<float>(std::min(0.5, cells_y * 0.5));
    if (wx <= 0.0f && wy <= 0.0f) return;

    // Horizontal pass. Edges reflect rather than wrap: the screen has edges,
    // and a target at x = 0 cannot arrive from x = width.
    for (int iy = 0; iy < NY; ++iy) {
        for (int ix = 0; ix < NX; ++ix) {
            const size_t i = static_cast<size_t>(iy * NX + ix);
            const float l = p_[static_cast<size_t>(iy * NX + std::max(0, ix - 1))];
            const float r = p_[static_cast<size_t>(iy * NX + std::min(NX - 1, ix + 1))];
            scratch_[i] = (1.0f - 2.0f * wx) * p_[i] + wx * (l + r);
        }
    }
    // Vertical pass.
    for (int iy = 0; iy < NY; ++iy) {
        for (int ix = 0; ix < NX; ++ix) {
            const size_t i = static_cast<size_t>(iy * NX + ix);
            const float u = scratch_[static_cast<size_t>(std::max(0, iy - 1) * NX + ix)];
            const float d = scratch_[static_cast<size_t>(std::min(NY - 1, iy + 1) * NX + ix)];
            p_[i] = (1.0f - 2.0f * wy) * scratch_[i] + wy * (u + d);
        }
    }
    normalise();
}

double ProbabilityGrid::mass_in_fov(Angle2 boresight,
                                    const CameraGeometry& cam) const noexcept {
    double m = 0.0;
    for (int iy = 0; iy < NY; ++iy) {
        for (int ix = 0; ix < NX; ++ix) {
            const float f = overlap(ix, iy, boresight, cam);
            if (f > 0.0f) m += static_cast<double>(f) * p_[static_cast<size_t>(iy * NX + ix)];
        }
    }
    return m;
}

Angle2 ProbabilityGrid::best_look(Angle2 current, const CameraGeometry& cam,
                                  double max_rate_urad_s) const noexcept {
    double best_rate = -1.0;
    Angle2 best = current;

    // Candidate look points are cell centres. Finer than that would be
    // pretending to a precision the grid does not have; coarser would leave
    // mass uncollectable between looks.
    for (int iy = 0; iy < NY; ++iy) {
        for (int ix = 0; ix < NX; ++ix) {
            const Pixel2 c   = cell_centre_px(ix, iy);
            const Angle2 aim = screen_.to_angle(c);
            const double mass = mass_in_fov(aim, cam);
            if (mass <= 0.0) continue;

            const double dist = std::hypot(aim.x - current.x, aim.y - current.y);
            const double travel_s = (max_rate_urad_s > 0.0)
                                  ? dist / max_rate_urad_s : 0.0;
            // §10.5's rate: probability per second, not probability. The
            // denominator is what stops the strategy crossing the screen for a
            // marginally better cell — which is slower than a spiral and has
            // no coverage guarantee either.
            const double rate = mass / (travel_s + p_params_.dwell_s);
            if (rate > best_rate) { best_rate = rate; best = aim; }
        }
    }
    return best;
}

void ProbabilityGrid::normalise() noexcept {
    double total = 0.0;
    for (const float v : p_) total += static_cast<double>(v);
    if (total <= 0.0) { reset_uniform(); return; }
    const float inv = static_cast<float>(1.0 / total);
    // Floored BEFORE the sum is re-taken, so the result is still a normalised
    // distribution to within the floor's own magnitude — which is 1e-6 of
    // uniform, nine orders below anything best_look would choose.
    //
    // The floor is what stops a cell reaching zero, and zero is absorbing:
    // both observe() and diffuse() are multiplicative, so a cell that gets
    // there can never come back and a beacon that wandered into a
    // searched-early region would be permanently unfindable. Measured without
    // it: 500 fruitless looks at one place drove the lowest cell to exactly 0.
    const float floor_v = p_params_.min_cell_frac / static_cast<float>(kCells);
    double after = 0.0;
    for (float& v : p_) { v = std::max(v * inv, floor_v); after += v; }
    if (after > 0.0) {
        const float inv2 = static_cast<float>(1.0 / after);
        for (float& v : p_) v *= inv2;
    }
}

}  // namespace sat
