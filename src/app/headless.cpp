// app/headless.cpp

#include "app/headless.hpp"

#include "app/timestamp.hpp"

#include "engine/pipeline.hpp"
#include "metrics/centroid_log.hpp"
#include "metrics/collector.hpp"
#include "metrics/report.hpp"
#include "metrics/run_report.hpp"
#include "scenario/schema.hpp"
#include "sat/version.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>

namespace sat {

int run_headless(const HeadlessOptions& opt) {
    // --- load ---------------------------------------------------------------
    auto loaded = load_scenario(opt.scenario_path);
    if (!loaded) {
        std::fprintf(stderr, "%s\n", loaded.error().c_str());
        return 1;
    }
    Scenario sc = *loaded;

    if (opt.seed_override >= 0)     sc.seed = static_cast<uint64_t>(opt.seed_override);
    if (opt.duration_override > 0.0) sc.duration_s = opt.duration_override;
    // INV-7: --no-ai must produce a fully working run, and every artifact has
    // to say which it was. Stage 11 is where a model actually loads; setting
    // the flag now means the disclosure is already plumbed when it does.
    if (opt.no_ai) sc.ai_enabled = false;

    std::error_code ec;
    std::filesystem::create_directories(opt.out_dir, ec);
    if (ec) {
        std::fprintf(stderr, "sat-tracker: cannot create output directory '%s': %s\n",
                     opt.out_dir.c_str(), ec.message().c_str());
        return 1;
    }

    // --- build --------------------------------------------------------------
    Pipeline pipe;
    pipe.build_from_scenario(sc);
    pipe.set_publish_snapshots(opt.fingerprint);

    const CameraGeometry cam    = sc.camera_geometry();
    const ScreenGeometry screen = sc.screen_geometry();
    const size_t expected = static_cast<size_t>(sc.duration_s * sc.camera_hz) + 2;

    // --- artifacts, opened BEFORE the run (§13.2's A9 rule) ----------------
    CentroidLog log;
    const std::string csv_path = opt.out_dir + "/centroid.csv";
    const bool want_csv = opt.write_artifacts && !opt.no_csv;
    if (want_csv) {
        CentroidLogHeader h;
        h.source      = opt.scenario_path;
        h.mode        = "synthetic";
        h.build       = SAT_GIT_HASH;
        h.utc         = utc_timestamp_now();
        h.screen_w    = screen.width;  h.screen_h = screen.height;
        h.camera_w    = cam.width;     h.camera_h = cam.height;
        h.fps         = sc.camera_hz;
        h.ifov_urad   = cam.ifov_urad();
        h.ai_enabled  = sc.ai_enabled;
        if (!log.open(csv_path, h)) {
            std::fprintf(stderr, "sat-tracker: cannot write '%s'\n", csv_path.c_str());
            return 1;
        }
    }

    MetricCollector metrics;
    metrics.begin(sc.name, sc.seed, cam.ifov_urad(), expected);

    // Per-frame traces for the report's plots (CP 7.6). Kept here rather than
    // inside MetricCollector because the collector's series get SORTED by the
    // first quantile call — see metrics/series.hpp — and a plot needs
    // chronological order. Reserved up front so the loop still allocates
    // nothing.
    ReportTrace trace_centroid{"Centroiding error, image frame", "px", {}, {},
                               1.0, "1 px"};
    ReportTrace trace_tracking{"Tracking error", "px", {}, {},
                               sc.tracking_error_px, "row 17"};
    if (opt.write_report) {
        for (ReportTrace* t : {&trace_centroid, &trace_tracking}) {
            t->x.reserve(expected);
            t->y.reserve(expected);
        }
    }

    // --- run ----------------------------------------------------------------
    // The wall clock is read HERE, outside the simulation, and never enters it.
    // INV-3 forbids the simulation from depending on it; measuring how long the
    // simulation took is the one legitimate use, and it happens in app/ rather
    // than in any module the invariant checker scans.
    const auto t0 = std::chrono::steady_clock::now();
    while (pipe.step()) {
        const FrameRecord& r = pipe.last();
        metrics.add(r);
        if (want_csv) log.write(r, screen);
        if (opt.write_report) {
            // Only frames where the metric is DEFINED are plotted. Plotting a
            // zero on a frame with no detection would draw a spike to the axis
            // that reads as perfect accuracy — the same flattering-zero the
            // compliance matrix had to be fixed for.
            if (r.centroid_error_valid) {
                trace_centroid.x.push_back(r.time_s);
                trace_centroid.y.push_back(r.centroid_error_px);
            }
            if (r.track_state == TrackState::Confirmed) {
                trace_tracking.x.push_back(r.time_s);
                trace_tracking.y.push_back(r.tracking_error_px);
            }
        }
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double wall_s = std::chrono::duration<double>(t1 - t0).count();

    log.close();

    const RunMetrics m = metrics.finish(pipe.timers(), wall_s,
                                        pipe.gimbal().saturation_frac());

    // --- run.json (CP 7.4) --------------------------------------------------
    if (opt.write_artifacts) {
        uint64_t fp = 0;
        for (const FrameFingerprint& f : pipe.fingerprints()) fp ^= f.combined();
        const std::string json_path = opt.out_dir + "/run.json";
        if (!write_run_json(json_path, m, sc, SAT_GIT_HASH, fp)) {
            std::fprintf(stderr, "sat-tracker: cannot write '%s'\n", json_path.c_str());
            return 1;
        }
    }

    // --- report.html (CP 7.6) ----------------------------------------------
    std::string report_path;
    if (opt.write_artifacts && opt.write_report) {
        ReportInput ri;
        ri.build                = SAT_GIT_HASH;
        ri.utc                  = utc_timestamp_now();
        ri.scenario_name        = sc.name;
        ri.scenario_description = sc.description;
        ri.metrics              = m;
        ri.requirements.acquisition_s     = sc.acquisition_s;
        ri.requirements.tracking_error_px = sc.tracking_error_px;
        ri.requirements.target_loss_frac  = sc.target_loss_frac;
        ri.requirements.reacquisition_s   = sc.reacquisition_s;
        ri.requirements.min_fps           = sc.min_fps;
        // The derived floor on row 17, from this scenario's own jitter. See
        // Requirements::tracking_floor_px.
        const double a = sc.jitter_px_per_frame;
        ri.requirements.tracking_floor_px = (a > 0.0) ? std::sqrt(2.0 * a * a / 3.0) : 0.0;
        ri.traces = {trace_centroid, trace_tracking};

        report_path = opt.out_dir + "/report.html";
        if (!write_report(report_path, ri)) {
            std::fprintf(stderr, "sat-tracker: cannot write '%s'\n", report_path.c_str());
            return 1;
        }
    }

    if (!opt.quiet) {
        std::printf("%s", format_summary(m).c_str());
        if (opt.write_artifacts) {
            std::printf("\nartifacts         %s/run.json", opt.out_dir.c_str());
            if (want_csv) {
                std::printf(", %s/centroid.csv (%lld rows)",
                            opt.out_dir.c_str(), static_cast<long long>(log.rows()));
            }
            if (!report_path.empty()) std::printf(", %s", report_path.c_str());
            std::printf("\n");
        }
    }
    return 0;
}

int headless_command(int argc, char* argv[], int& i) {
    HeadlessOptions opt;
    opt.scenario_path = std::string(SAT_SCENARIO_DIR) + "/baseline.toml";

    for (int k = i + 1; k < argc; ++k) {
        const char* a = argv[k];
        if (std::strcmp(a, "--scenario") == 0 && k + 1 < argc) {
            opt.scenario_path = argv[++k];
        } else if (std::strcmp(a, "--out") == 0 && k + 1 < argc) {
            opt.out_dir = argv[++k];
        } else if (std::strcmp(a, "--seed") == 0 && k + 1 < argc) {
            opt.seed_override = std::atoll(argv[++k]);
        } else if (std::strcmp(a, "--duration") == 0 && k + 1 < argc) {
            opt.duration_override = std::atof(argv[++k]);
        } else if (std::strcmp(a, "--no-ai") == 0) {
            opt.no_ai = true;
        } else if (std::strcmp(a, "--no-csv") == 0) {
            opt.no_csv = true;
        } else if (std::strcmp(a, "--no-report") == 0) {
            opt.write_report = false;
        } else if (std::strcmp(a, "--quiet") == 0) {
            opt.quiet = true;
        } else if (std::strcmp(a, "--bench") == 0) {
            // A pure speed run: no artifacts, no snapshots. What CP 7.3's
            // wall-time criterion is actually about.
            opt.write_artifacts = false;
            opt.fingerprint     = false;
            opt.write_report    = false;
        } else {
            std::fprintf(stderr, "sat-tracker: --headless: unrecognised option '%s'\n", a);
            return 2;
        }
        i = k;
    }
    return run_headless(opt);
}

}  // namespace sat
