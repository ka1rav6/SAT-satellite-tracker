#include "perception/grouping.hpp"

#include <algorithm>
#include <cmath>

namespace sat {
namespace {

/// Union-find with path halving and union by rank.
///
/// Path halving rather than full path compression: it is a single extra store
/// in the find loop, gives the same near-constant amortised behaviour, and
/// needs no second pass. Crucially it is also deterministic, which full
/// compression also is — the reason to note it is that anything with a
/// heuristic depending on visit order would not be (§9.4.6).
int32_t find_root(std::span<int32_t> parent, int32_t i) noexcept {
    while (parent[static_cast<size_t>(i)] != i) {
        parent[static_cast<size_t>(i)] =
            parent[static_cast<size_t>(parent[static_cast<size_t>(i)])];
        i = parent[static_cast<size_t>(i)];
    }
    return i;
}

void unite(std::span<int32_t> parent, std::span<int32_t> rank,
           int32_t a, int32_t b) noexcept {
    a = find_root(parent, a);
    b = find_root(parent, b);
    if (a == b) return;
    // Union by rank, with the index as a tiebreaker so the outcome cannot
    // depend on anything but the data (INV-3).
    if (rank[static_cast<size_t>(a)] < rank[static_cast<size_t>(b)]) std::swap(a, b);
    else if (rank[static_cast<size_t>(a)] == rank[static_cast<size_t>(b)] && a > b) std::swap(a, b);
    parent[static_cast<size_t>(b)] = a;
    if (rank[static_cast<size_t>(a)] == rank[static_cast<size_t>(b)]) {
        ++rank[static_cast<size_t>(a)];
    }
}

}  // namespace

double BlobAccum::rms_radius() const noexcept {
    if (sw <= 0.0) return 0.0;
    const double mx = swx / sw, my = swy / sw;
    const double vx = std::max(0.0, swxx / sw - mx * mx);
    const double vy = std::max(0.0, swyy / sw - my * my);
    return std::sqrt(vx + vy);
}

size_t group_components(std::span<const uint8_t> mask,
                        std::span<const int16_t> weight,
                        int width, int height,
                        const GroupingWorkspace& ws,
                        std::vector<BlobAccum>& out,
                        size_t max_blobs,
                        size_t* dropped) {
    out.clear();
    if (dropped) *dropped = 0;
    // The blob table's bound. clear() keeps capacity, so when the caller has
    // passed the capacity it reserved, nothing here allocates — and nothing
    // here may: this runs inside the frame window INV-4's trap watches.
    const size_t cap = max_blobs;
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (width <= 0 || height <= 0 || mask.size() < n || weight.size() < n) return 0;

    // -----------------------------------------------------------------------
    // Pass 1: extract runs, in strict raster order.
    //
    // One linear scan of the mask. This is the only pass that touches every
    // pixel; everything after works on runs, of which a sparse mask has few.
    // -----------------------------------------------------------------------
    size_t run_count = 0;
    for (int y = 0; y < height; ++y) {
        const size_t row = static_cast<size_t>(y) * static_cast<size_t>(width);
        int x = 0;
        while (x < width) {
            if (!mask[row + static_cast<size_t>(x)]) { ++x; continue; }
            const int start = x;
            while (x < width && mask[row + static_cast<size_t>(x)]) ++x;
            if (run_count >= ws.runs.size()) {
                // The workspace is sized for the worst case at startup. Running
                // out means the mask is far denser than CFAR should ever
                // produce — a sign the threshold is wrong, not a reason to
                // crash. Stop here with what we have rather than overrun.
                goto runs_done;
            }
            ws.runs[run_count] = Run{static_cast<int16_t>(y),
                                     static_cast<int16_t>(start),
                                     static_cast<int16_t>(x), -1};
            ++run_count;
        }
    }
runs_done:
    if (run_count == 0) return 0;
    if (ws.parent.size() < run_count || ws.rank.size() < run_count) return 0;

    for (size_t i = 0; i < run_count; ++i) {
        ws.parent[i] = static_cast<int32_t>(i);
        ws.rank[i]   = 0;
    }

    // -----------------------------------------------------------------------
    // Pass 2: union runs on adjacent rows whose columns touch.
    //
    // Runs are in raster order, so the runs of row y-1 form a contiguous block
    // before those of row y. A moving window over that block makes this linear
    // in the number of runs rather than quadratic.
    //
    // 8-connectivity: the ranges are compared with a one-column slack, so runs
    // that only touch corner-to-corner still merge.
    // -----------------------------------------------------------------------
    size_t prev_begin = 0, prev_end = 0, cur_begin = 0;
    int prev_y = -1;
    for (size_t i = 0; i < run_count; ++i) {
        const int y = ws.runs[i].y;
        if (y != prev_y) {
            prev_begin = (prev_y == y - 1) ? cur_begin : i;
            prev_end   = (prev_y == y - 1) ? i : i;
            cur_begin  = i;
            prev_y     = y;
        }
        // Only the immediately preceding row can connect.
        if (prev_begin == prev_end) continue;
        for (size_t j = prev_begin; j < prev_end; ++j) {
            if (ws.runs[j].y != y - 1) continue;
            // 8-connected overlap test on half-open ranges.
            if (ws.runs[j].x1 >= ws.runs[i].x0 && ws.runs[j].x0 <= ws.runs[i].x1) {
                unite(ws.parent, ws.rank, static_cast<int32_t>(i), static_cast<int32_t>(j));
            }
        }
    }

    // -----------------------------------------------------------------------
    // Pass 3: flatten the union-find and accumulate moments.
    //
    // Labels are assigned in order of FIRST APPEARANCE in raster order, which
    // makes them a deterministic function of the mask alone — not of the merge
    // order, which is what §9.4.6 warns about.
    // -----------------------------------------------------------------------
    std::span<int32_t> label_of = ws.rank;    // reuse: rank is finished with
    for (size_t i = 0; i < run_count; ++i) label_of[i] = -1;

    for (size_t i = 0; i < run_count; ++i) {
        const int32_t root = find_root(ws.parent, static_cast<int32_t>(i));
        int32_t lbl = label_of[static_cast<size_t>(root)];
        if (lbl < 0) {
            // A new component, and the table is full. Drop it rather than let
            // the vector grow: growing here allocates half a megabyte inside
            // the frame window, which is exactly what INV-4 forbids and what
            // CP 14.1's fuzzer caught on a 1920x534 draw that produced 14,420
            // components. Dropping is deterministic — components are created
            // in first-appearance raster order, so which ones survive is a
            // function of the mask alone (INV-3) — and it is REPORTED, so a
            // frame whose answer is incomplete says so.
            if (out.size() >= cap) {
                ws.runs[i].label = -1;
                if (dropped) ++*dropped;
                continue;
            }
            lbl = static_cast<int32_t>(out.size());
            label_of[static_cast<size_t>(root)] = lbl;
            BlobAccum b{};
            b.x0 = ws.runs[i].x0;
            b.x1 = static_cast<int16_t>(ws.runs[i].x1 - 1);
            b.y0 = ws.runs[i].y;
            b.y1 = ws.runs[i].y;
            out.push_back(b);
        }
        ws.runs[i].label = lbl;

        BlobAccum& b = out[static_cast<size_t>(lbl)];
        const Run& r = ws.runs[i];
        const size_t row = static_cast<size_t>(r.y) * static_cast<size_t>(width);

        b.x0 = std::min(b.x0, r.x0);
        b.x1 = std::max(b.x1, static_cast<int16_t>(r.x1 - 1));
        b.y0 = std::min(b.y0, r.y);
        b.y1 = std::max(b.y1, r.y);

        for (int x = r.x0; x < r.x1; ++x) {
            // Weights are clipped at zero. A negative top-hat value means the
            // pixel is below its local background, which carries no evidence
            // about where a bright source is; letting it contribute a negative
            // weight would pull the centroid away from the light.
            const double w = std::max(0, static_cast<int>(weight[row + static_cast<size_t>(x)]));
            ++b.n;
            b.sw   += w;
            b.swx  += w * x;
            b.swy  += w * r.y;
            b.swxx += w * static_cast<double>(x) * x;
            b.swyy += w * static_cast<double>(r.y) * r.y;
            b.peak = std::max(b.peak, static_cast<float>(w));
        }
    }

    return out.size();
}

}  // namespace sat
