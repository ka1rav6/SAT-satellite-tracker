// search/pattern.cpp

#include "search/pattern.hpp"

#include <algorithm>
#include <cmath>

namespace sat {

const char* search_strategy_name(SearchStrategy s) noexcept {
    switch (s) {
        case SearchStrategy::Spiral:        return "spiral";
        case SearchStrategy::Raster:        return "raster";
        case SearchStrategy::Probabilistic: return "probabilistic";
        case SearchStrategy::CampAndWait:   return "camp_and_wait";
        case SearchStrategy::Adaptive:      return "adaptive";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// spiral_cell
//
// The square spiral, derived rather than tabulated.
//
// Ring r (Chebyshev radius r) starts at index (2r-1)^2 and contains 8r cells.
// Within a ring the walk is: up the right edge, left along the top, down the
// left edge, right along the bottom. Each edge is 2r long, so the offset within
// the ring divided by 2r selects the edge and the remainder walks it.
//
// The alternative — stepping a direction/leg-length state machine — is fewer
// lines but is not a function of k, and being a function of k is what lets the
// GUI draw the next twenty looks without mutating the pattern.
// ---------------------------------------------------------------------------
void spiral_cell(int k, int& ix, int& iy) noexcept {
    if (k <= 0) { ix = 0; iy = 0; return; }

    const int r = spiral_ring(k);
    const int first = (2 * r - 1) * (2 * r - 1);   // index of this ring's first cell
    const int off   = k - first;                   // 0 .. 8r-1
    const int side  = 2 * r;
    const int edge  = off / side;                  // 0 right, 1 top, 2 left, 3 bottom
    const int t     = off % side;                  // 0 .. 2r-1 along that edge

    switch (edge) {
        case 0:  ix =  r;         iy = -r + 1 + t; break;   // right edge, going up
        case 1:  ix =  r - 1 - t; iy =  r;         break;   // top edge, going left
        case 2:  ix = -r;         iy =  r - 1 - t; break;   // left edge, going down
        default: ix = -r + 1 + t; iy = -r;         break;   // bottom edge, going right
    }
}

int spiral_ring(int k) noexcept {
    if (k <= 0) return 0;
    // The largest r with (2r-1)^2 <= k. Solving: r = floor((sqrt(k)+1)/2).
    const int s = static_cast<int>(std::sqrt(static_cast<double>(k)));
    int r = (s + 1) / 2;
    // sqrt is not exact on every input, so nudge rather than trust it. At most
    // one step either way.
    while ((2 * r - 1) * (2 * r - 1) > k) --r;
    while ((2 * (r + 1) - 1) * (2 * (r + 1) - 1) <= k) ++r;
    return r;
}

// ---------------------------------------------------------------------------
// SearchParams
// ---------------------------------------------------------------------------
SearchParams SearchParams::from_camera(const CameraGeometry& cam,
                                       const ScreenGeometry& screen,
                                       double overlap_) noexcept {
    SearchParams p;
    p.overlap = std::clamp(overlap_, 0.0, 0.9);
    const Angle2 fov = cam.half_fov_urad();
    // half_fov * 2 is the full field; the step is that, shrunk by the overlap.
    p.step = Angle2{2.0 * fov.x * (1.0 - p.overlap),
                    2.0 * fov.y * (1.0 - p.overlap)};
    const Angle2 ext = screen.extent_urad();
    p.half_extent = Angle2{0.5 * ext.x, 0.5 * ext.y};
    return p;
}

// ---------------------------------------------------------------------------
// tile_count — how many looks cover the screen.
//
// The obvious formula, 1 + 2*ceil(half_extent / step), is wrong, and wrong in
// the expensive direction. It counts looks needed to REACH the edge, but a look
// does not have to reach the edge to see it: the look at index n sees out to
// n*step + step/2, because the camera's field extends half a step past its own
// centre. Asking for a look centred ON the far edge adds a whole ring of tiles
// that see nothing but empty space beyond the screen.
//
// Measured on the default configuration: the reach formula gives 5 x 7 = 35
// tiles, and the correct one gives 5 x 5 = 25 — a 29% longer cold sweep, spent
// staring off the edge of the world. §10.5's bound is already the worst number
// in the project; inflating it by a third for no reason is not acceptable.
//
//     n = 1 + 2*floor(half/step + 1/2)
//
// and the coverage condition (n/2)*step >= half follows for every half/step,
// since floor(h + 1/2) >= h - 1/2.
// ---------------------------------------------------------------------------
int tile_count(const SearchParams& p) noexcept {
    if (p.step.x <= 0.0 || p.step.y <= 0.0) return 0;
    const int nx = 1 + 2 * static_cast<int>(std::floor(p.half_extent.x / p.step.x + 0.5));
    const int ny = 1 + 2 * static_cast<int>(std::floor(p.half_extent.y / p.step.y + 0.5));
    return nx * ny;
}

double full_sweep_time_s(const SearchParams& p, double max_rate_urad_s) noexcept {
    if (max_rate_urad_s <= 0.0) return 0.0;
    const int n = tile_count(p);
    if (n <= 1) return 0.0;
    // A boustrophedon estimate: one step of travel per tile after the first,
    // taking the larger axis step as the typical hop. This is the honest form
    // of §10.5's "~74.5 degrees of travel" — it is a lower bound on the real
    // thing, because it ignores acceleration limits and the settling time at
    // each look, and it is labelled as such wherever it is reported.
    const double hop = std::max(p.step.x, p.step.y);
    return (n - 1) * (hop / max_rate_urad_s + p.dwell_s);
}

// ---------------------------------------------------------------------------
// SearchPattern
// ---------------------------------------------------------------------------
void SearchPattern::reset(const SearchParams& p, Angle2 centre) noexcept {
    p_       = p;
    centre_  = centre;
    index_   = 0;
    visited_ = 0;
    dwell_   = 0.0;
    arrived_ = false;
    swept_   = false;
    look_    = point_for(0);
}

void SearchPattern::recentre(Angle2 centre) noexcept {
    centre_ = centre;
    look_   = point_for(index_);
}

void SearchPattern::restart() noexcept {
    index_   = 0;
    visited_ = 0;
    dwell_   = 0.0;
    arrived_ = false;
    swept_   = false;
    look_    = point_for(0);
}

Angle2 SearchPattern::point_for(int k) const noexcept {
    int ix = 0, iy = 0;
    if (p_.strategy == SearchStrategy::Raster) {
        // The naive baseline, kept because §10.5 wants the strategies
        // benchmarked against each other and "spiral is better" is not a claim
        // without something to be better than. Raster covers the same grid in
        // row order, starting at the top-left of the screen rather than at the
        // current position — which is exactly why it is worse for
        // re-acquisition: it throws away the prior.
        const int nx = std::max(1, 1 + 2 * static_cast<int>(
                          std::floor(p_.half_extent.x / std::max(p_.step.x, 1e-9) + 0.5)));
        const int ny = std::max(1, 1 + 2 * static_cast<int>(
                          std::floor(p_.half_extent.y / std::max(p_.step.y, 1e-9) + 0.5)));
        const int n = nx * ny;
        const int j = (n > 0) ? (k % n) : 0;
        ix = (j % nx) - nx / 2;
        iy = (j / nx) - ny / 2;
    } else {
        spiral_cell(k, ix, iy);
    }
    return Angle2{centre_.x + ix * p_.step.x, centre_.y + iy * p_.step.y};
}

// A look is worth making if the camera's FIELD overlaps the screen, not if its
// CENTRE is on the screen. The two differ by half a step in each axis, and
// using the stricter one leaves an uncovered border exactly one half-field
// wide all the way round — which is where a target entering the screen appears,
// so it is the worst possible place to be blind. This is the same off-by-half
// as tile_count's, and the two must agree or the sweep either stops early or
// never terminates.
bool SearchPattern::in_bounds(Angle2 a) const noexcept {
    return std::fabs(a.x) <= p_.half_extent.x + 0.5 * p_.step.x
        && std::fabs(a.y) <= p_.half_extent.y + 0.5 * p_.step.y;
}

// ---------------------------------------------------------------------------
// max_ring — the outermost spiral ring that can still contain a useful look.
//
// Needed because "an entire ring was out of bounds, so stop" is NOT a valid
// termination rule once the pattern can be recentred: a prediction near the
// edge of the screen — or briefly outside it — puts the inner rings off-screen
// while larger rings reach back onto it. Deriving the bound from the geometry
// handles that case; the ring-gap heuristic silently sweeps nothing.
// ---------------------------------------------------------------------------
int SearchPattern::max_ring() const noexcept {
    if (p_.step.x <= 0.0 || p_.step.y <= 0.0) return 0;
    const double rx = (p_.half_extent.x + std::fabs(centre_.x) + 0.5 * p_.step.x) / p_.step.x;
    const double ry = (p_.half_extent.y + std::fabs(centre_.y) + 0.5 * p_.step.y) / p_.step.y;
    return static_cast<int>(std::ceil(std::max(rx, ry)));
}

void SearchPattern::advance() noexcept {
    ++visited_;
    dwell_   = 0.0;
    arrived_ = false;

    // Walk forward until an in-bounds look is found. The spiral's rings are
    // square but the screen's tiling is not (4 deg x 3 deg tiles on a square
    // screen), so the corners of the outer rings fall off the edge and are
    // skipped rather than visited — which is why bounds are checked per cell
    // rather than by capping the ring.
    const int stop_ring = max_ring();
    while (true) {
        ++index_;
        if (spiral_ring(index_) > stop_ring) {
            // Everything reachable has been looked at. Start again.
            swept_ = true;
            index_ = 0;
            look_  = point_for(0);
            return;
        }
        const Angle2 next = point_for(index_);
        if (in_bounds(next)) { look_ = next; return; }
    }
}

Angle2 SearchPattern::step(double dt, Angle2 boresight) noexcept {
    const double tol = p_.arrive_frac * std::min(p_.step.x, p_.step.y);
    if (!arrived_ && (boresight - look_).norm() <= tol) arrived_ = true;

    if (arrived_) {
        // Dwell only starts counting once the mount has arrived. Counting from
        // the moment the look was issued would let a long slew consume the
        // whole dwell, and the pattern would move on having exposed no frame
        // at that look at all — a search that covers the screen and sees
        // nothing, which is the classic way this goes wrong.
        dwell_ += dt;
        if (dwell_ >= p_.dwell_s) advance();
    }
    return look_;
}

}  // namespace sat
