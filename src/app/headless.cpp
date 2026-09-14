// app/headless.cpp

#include "app/headless.hpp"

#include "app/timestamp.hpp"

#include "engine/pipeline.hpp"
#include "metrics/centroid_log.hpp"
#include "metrics/collector.hpp"
#include "metrics/run_report.hpp"
#include "scenario/schema.hpp"
#include "sat/version.hpp"

#include <chrono>
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

    if (!opt.quiet) {
        std::printf("%s", format_summary(m).c_str());
        if (opt.write_artifacts) {
            std::printf("\nartifacts         %s/centroid.csv (%lld rows), %s/run.json\n",
                        opt.out_dir.c_str(), static_cast<long long>(log.rows()),
                        opt.out_dir.c_str());
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
        } else if (std::strcmp(a, "--quiet") == 0) {
            opt.quiet = true;
        } else if (std::strcmp(a, "--bench") == 0) {
            // A pure speed run: no artifacts, no snapshots. What CP 7.3's
            // wall-time criterion is actually about.
            opt.write_artifacts = false;
            opt.fingerprint     = false;
        } else {
            std::fprintf(stderr, "sat-tracker: --headless: unrecognised option '%s'\n", a);
            return 2;
        }
        i = k;
    }
    return run_headless(opt);
}

}  // namespace sat
