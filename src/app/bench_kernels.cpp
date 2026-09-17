// app/bench_kernels.cpp — see the header for why a whole-run stage table is the
// wrong instrument for optimising a kernel.

#include "app/bench_kernels.hpp"

#include "camera/splat.hpp"
#include "core/arena.hpp"
#include "core/rng.hpp"
#include "degrade/noise.hpp"
#include "degrade/sensor.hpp"
#include "perception/cfar.hpp"
#include "perception/grouping.hpp"
#include "perception/matched.hpp"
#include "perception/median.hpp"
#include "perception/morphology.hpp"
#include "perception/pipeline.hpp"
#include "perception/sat.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace sat {

namespace {

// ---------------------------------------------------------------------------
// One timed row.
//
// The reported figure is the MINIMUM over `repeats`, not the mean or the
// median. On a machine that is also doing other things, the mean measures the
// operating system and the minimum measures the kernel; since the question
// being asked is "did this change make the code faster", the minimum is the
// estimator with the least variance and no upward bias from unrelated work.
//
// The whole-run p50 that design §15 is actually scored against is still printed
// by `--headless --stages`. These two numbers answer different questions and
// both are needed.
// ---------------------------------------------------------------------------
struct Row {
    std::string name;
    double      min_us    = 0.0;
    double      median_us = 0.0;
    double      budget_us = 0.0;   ///< design §15's scalar column, 0 if none
    int         calls     = 1;     ///< calls per frame, for the budget compare
};

double now_us() {
    return std::chrono::duration<double, std::micro>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// Time `body` and file a row. `sink` is read afterwards so the optimiser
/// cannot delete the work.
Row time_it(const char* name, int warmup, int repeats, double budget_us, int calls,
            const std::function<void()>& body) {
    for (int i = 0; i < warmup; ++i) body();

    std::vector<double> samples;
    samples.reserve(static_cast<size_t>(repeats));
    for (int i = 0; i < repeats; ++i) {
        const double t0 = now_us();
        body();
        samples.push_back(now_us() - t0);
    }
    std::sort(samples.begin(), samples.end());

    Row r;
    r.name      = name;
    r.min_us    = samples.front();
    r.median_us = samples[samples.size() / 2];
    r.budget_us = budget_us;
    r.calls     = calls;
    return r;
}

// ---------------------------------------------------------------------------
// A representative frame.
//
// Not random noise: a random image would make the top-hat's opening trivial and
// the CFAR mask empty, and both kernels' costs depend on their input. This is
// the specification's own worst case — a beacon, a decoy and 120 clutter
// sources on a background pedestal, through the full damage chain — which is
// what scenarios/compliance.toml renders and therefore what the budget is
// scored on.
// ---------------------------------------------------------------------------
struct Frame {
    int                  width = 0, height = 0;
    std::vector<float>   radiance;      ///< pre-damage render
    std::vector<uint8_t> pixels;        ///< after the damage chain
};

Frame make_frame(int width, int height) {
    Frame f;
    f.width  = width;
    f.height = height;
    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    f.radiance.assign(n, 8.0f);
    f.pixels.assign(n, 0);

    Pcg32 g{20260917u};
    // One beacon in the middle, one decoy, 120 clutter sources — the same
    // population world_builder.cpp draws for scenarios/compliance.toml.
    splat_emitter(f.radiance, width, height,
                  Pixel2{width * 0.5, height * 0.5}, 10.0, ShapeKind::Square, 120.0f);
    splat_emitter(f.radiance, width, height,
                  Pixel2{width * 0.5 + 60.0, height * 0.5 + 12.0}, 10.0,
                  ShapeKind::Square, 110.0f);
    for (int i = 0; i < 120; ++i) {
        const double x = g.next_range(0.0, width);
        const double y = g.next_range(0.0, height);
        const double intensity = g.next_range(0.35, 1.6) * 120.0;
        const auto   size = static_cast<double>(3 + g.next_below(22));
        const ShapeKind shape = (g.next_below(4) == 0) ? ShapeKind::Square
                                                       : ShapeKind::Gaussian;
        splat_emitter(f.radiance, width, height, Pixel2{x, y}, size, shape,
                      static_cast<float>(intensity));
    }
    return f;
}

void print_table(const std::vector<Row>& rows) {
    std::printf("\nKERNEL BENCH  (minimum of N runs, isolated; the stage table's p50\n"
                "               is still what design section 15 is scored against)\n");
    std::printf("kernel              calls       min     median    budget     over\n");
    std::printf("---------------------------------------------------------------------\n");
    double total_min = 0.0;
    for (const Row& r : rows) {
        // Rows whose name is indented are diagnostics that overlap a budgeted
        // row above them; counting them would double-charge the total.
        if (r.name[0] != ' ') total_min += r.min_us * r.calls;
        if (r.budget_us > 0.0) {
            std::printf("%-18s %6d %9.1f  %9.1f  %8.1f  %6.1fx\n",
                        r.name.c_str(), r.calls, r.min_us, r.median_us,
                        r.budget_us, (r.min_us * r.calls) / r.budget_us);
        } else {
            std::printf("%-18s %6d %9.1f  %9.1f\n",
                        r.name.c_str(), r.calls, r.min_us, r.median_us);
        }
    }
    std::printf("---------------------------------------------------------------------\n");
    std::printf("%-18s %6s %9.1f             %8.1f  %6.1fx\n",
                "sum of kernels", "", total_min, 850.0, total_min / 850.0);
    std::printf("(microseconds; 'calls' is how many times a frame runs this kernel)\n");
}

}  // namespace

int run_bench_kernels(const BenchKernelsOptions& opt) {
    const int W = opt.width, H = opt.height;
    const size_t n = static_cast<size_t>(W) * static_cast<size_t>(H);
    std::printf("bench-kernels: %d x %d = %zu px, %d repeats, warmup %d\n",
                W, H, n, opt.repeats, opt.warmup);

    Frame frame = make_frame(W, H);

    // --- the damage chain -------------------------------------------------
    // Built by hand rather than from a Scenario so this file does not need the
    // TOML loader; the parameters are the specification's defaults.
    RngSet rng;
    rng.seed_all(20260917u);
    NoiseParams np;   // spec defaults: poisson on, sigma 20, 10% S&P, 40 hot
    SensorChain sensor;
    sensor.configure(np, Atmosphere::Clear, /*enabled=*/true, W, H, rng);

    std::vector<Row> rows;

    // The copy in the loop body is charged to the kernel deliberately: the
    // chain consumes the radiance buffer and the next repetition needs it back.
    // It is one 1.2 MB memcpy, so the row reads a little high; every
    // alternative either measures a warm buffer that the real frame never has
    // or stops the repetitions from being independent.
    {
        std::vector<float> scratch(n);
        rows.push_back(time_it("damage_chain", opt.warmup, opt.repeats, 550.0, 1, [&] {
            std::copy(frame.radiance.begin(), frame.radiance.end(), scratch.begin());
            sensor.apply(scratch, frame.pixels, rng);
        }));

        // Diagnostic rows, not budgeted. They split the chain so an
        // optimisation can be attributed: everything except the Gaussian, and
        // then the copy alone, which both of the rows above also pay.
        NoiseParams quiet = np;
        quiet.poisson_enabled = false;
        quiet.gaussian_sigma  = 0.0;
        quiet.salt_pepper_p   = 0.0;
        SensorChain quiet_chain;
        quiet_chain.configure(quiet, Atmosphere::Clear, true, W, H, rng);
        rows.push_back(time_it("  .. no gaussian", opt.warmup, opt.repeats, 0.0, 1, [&] {
            std::copy(frame.radiance.begin(), frame.radiance.end(), scratch.begin());
            quiet_chain.apply(scratch, frame.pixels, rng);
        }));
        rows.push_back(time_it("  .. copy only", opt.warmup, opt.repeats, 0.0, 1, [&] {
            std::copy(frame.radiance.begin(), frame.radiance.end(), scratch.begin());
        }));
    }

    // --- perception kernels ----------------------------------------------
    // The same 64 MB the engine reserves (engine/pipeline.hpp).
    Arena arena(64u << 20);
    PerceptionWorkspace ws;
    const int se = structuring_element_size(10);
    if (!ws.allocate(arena, W, H, se)) {
        std::fprintf(stderr, "bench-kernels: workspace allocation failed\n");
        return 1;
    }

    std::vector<uint8_t> filtered(n);
    rows.push_back(time_it("median_3x3", opt.warmup, opt.repeats, 350.0, 1, [&] {
        median_3x3(frame.pixels, ws.filtered, W, H);
    }));
    std::copy(ws.filtered.begin(), ws.filtered.end(), filtered.begin());

    rows.push_back(time_it("top_hat", opt.warmup, opt.repeats, 300.0, 1, [&] {
        top_hat(filtered, ws.tophat, W, H, se, ws.morph);
    }));

    SummedArea sa{ws.sat_sum, ws.sat_sumsq, W, H};
    rows.push_back(time_it("summed_area", opt.warmup, opt.repeats, 350.0, 2, [&] {
        build_sat(ws.tophat, W, H, sa);
    }));

    const int scale = nearest_scale(10);
    rows.push_back(time_it("matched_filter", opt.warmup, opt.repeats, 120.0, 1, [&] {
        matched_filter_at_scale(sa, W, H, scale, ws.response);
    }));

    CfarParams cp;
    rows.push_back(time_it("cfar", opt.warmup, opt.repeats, 200.0, 2, [&] {
        cfar_mask(sa, W, H, cp, ws.mask, {});
    }));

    std::vector<BlobAccum> blobs;
    blobs.reserve(4096);
    rows.push_back(time_it("grouping", opt.warmup, opt.repeats, 50.0, 1, [&] {
        blobs.clear();
        (void)group_components(ws.mask, ws.tophat, W, H, ws.grouping, blobs);
    }));

    // --- the simulator side ----------------------------------------------
    {
        std::vector<float> buf(n);
        rows.push_back(time_it("background", opt.warmup, opt.repeats, 150.0, 1, [&] {
            std::fill(buf.begin(), buf.end(), 8.0f);
        }));
    }
    {
        std::vector<float> buf(n, 8.0f);
        rows.push_back(time_it("splat", opt.warmup, opt.repeats, 40.0, 1, [&] {
            // All 122 emitters, once. A real frame runs EIGHT exposure
            // substeps but culls to the camera's view first, and a 640x480
            // camera sees 9.8% of a 2000x2000 screen — about a dozen sources.
            // 8 substeps x ~12 visible is close to 122 x 1, so this is charged
            // as one call per frame rather than eight.
            std::fill(buf.begin(), buf.end(), 8.0f);
            Pcg32 g{20260917u};
            for (int i = 0; i < 122; ++i) {
                const double x = g.next_range(0.0, W);
                const double y = g.next_range(0.0, H);
                const auto   size = static_cast<double>(3 + g.next_below(22));
                const ShapeKind shape = (g.next_below(4) == 0) ? ShapeKind::Square
                                                               : ShapeKind::Gaussian;
                splat_emitter(buf, W, H, Pixel2{x, y}, size, shape, 120.0f, 0.125);
            }
        }));
    }

    print_table(rows);
    return 0;
}

int bench_kernels_main(int argc, char** argv) {
    BenchKernelsOptions opt;
    for (int i = 0; i < argc; ++i) {
        const char* a = argv[i];
        auto next = [&](int& out) {
            if (i + 1 < argc) { out = std::atoi(argv[++i]); }
        };
        if      (std::strcmp(a, "--width")   == 0) next(opt.width);
        else if (std::strcmp(a, "--height")  == 0) next(opt.height);
        else if (std::strcmp(a, "--repeats") == 0) next(opt.repeats);
        else if (std::strcmp(a, "--warmup")  == 0) next(opt.warmup);
        else {
            std::fprintf(stderr, "sat-tracker: --bench-kernels: unrecognised '%s'\n", a);
            return 2;
        }
    }
    opt.width   = std::clamp(opt.width, 16, 8192);
    opt.height  = std::clamp(opt.height, 16, 8192);
    opt.repeats = std::clamp(opt.repeats, 1, 10000);
    opt.warmup  = std::clamp(opt.warmup, 0, 1000);
    return run_bench_kernels(opt);
}

}  // namespace sat
