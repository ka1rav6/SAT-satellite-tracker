// app/headless.cpp

#include "app/headless.hpp"

#include "app/timestamp.hpp"

#include "engine/pipeline.hpp"
#include "engine/video_probe.hpp"
#include "engine/video_source.hpp"
#include "metrics/centroid_log.hpp"
#include "metrics/trace_log.hpp"
#include "metrics/collector.hpp"
#include "metrics/report.hpp"
#include "metrics/run_report.hpp"
#include "scenario/overlay.hpp"
#include "scenario/schema.hpp"
#include "sat/version.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace sat {

// ---------------------------------------------------------------------------
// load_with_overrides — the scenario, plus any --set keys.
//
// With no overrides this is exactly load_scenario, and deliberately so: the
// common path must not start depending on the override machinery working. Only
// when something is actually overridden do we read the text, rewrite the
// document and re-parse it through the same loader.
// ---------------------------------------------------------------------------
namespace {

Result<Scenario> load_with_overrides(const HeadlessOptions& opt) {
    if (opt.overrides.empty()) return load_scenario(opt.scenario_path);

    std::ifstream in(opt.scenario_path, std::ios::binary);
    if (!in) return Err("cannot open scenario '" + opt.scenario_path + "'");
    std::ostringstream buf;
    buf << in.rdbuf();

    std::vector<Override> ov;
    ov.reserve(opt.overrides.size());
    for (const auto& [k, v] : opt.overrides) ov.push_back(Override{k, v});

    auto text = apply_overrides(buf.str(), ov);
    if (!text) return Err(text.error());
    return parse_scenario(*text, opt.scenario_path);
}

}  // namespace

int run_headless(const HeadlessOptions& opt) {
    // --- load ---------------------------------------------------------------
    auto loaded = load_with_overrides(opt);
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
    if (opt.video_path.empty()) {
        pipe.build_from_scenario(sc);
    } else {
        VideoMode  mode{};
        VideoMode* mode_ptr = nullptr;
        if (opt.video_mode == "screen") { mode = VideoMode::Screen; mode_ptr = &mode; }
        else if (opt.video_mode == "direct") { mode = VideoMode::Direct; mode_ptr = &mode; }
        else if (!opt.video_mode.empty()) {
            std::fprintf(stderr, "sat-tracker: --video-mode must be 'screen' or "
                                 "'direct' (got '%s')\n", opt.video_mode.c_str());
            return 2;
        }
        const std::filesystem::path truth{opt.truth_path};
        if (Status st = pipe.build_from_video(sc, opt.video_path, mode_ptr,
                                              opt.truth_path.empty() ? nullptr : &truth);
            !st) {
            std::fprintf(stderr, "sat-tracker: %s\n", st.error().c_str());
            return 1;
        }
        const VideoSource* v = pipe.video();
        if (!opt.quiet) {
            std::printf("video             %s\n", opt.video_path.c_str());
            // The SOURCE resolution, not the reported screen. In direct mode
            // the screen geometry is the scenario's nominal canvas, which has
            // nothing to do with the file — printing it said "2000x2000
            // source" for a 640x480 clip.
            std::printf("mode              %s%s\n",
                        video_mode_name(v->mode()),
                        opt.video_mode.empty() ? "  (auto-detected)" : "  (from --video-mode)");
            std::printf("source            %dx%d @ %.3f fps  ->  %dx%d delivered\n",
                        v->source_width(), v->source_height(), v->fps(),
                        v->geometry().width, v->geometry().height);
            if (!opt.truth_path.empty()) {
                std::printf("truth             %s\n", opt.truth_path.c_str());
            }
            std::printf("\n");
        }
        // A video run's length is the clip's, not the scenario's: the source
        // returns false at EOF and that is the clean termination §8.3
        // requirement 7 asks for. So the duration is only a SIZING HINT here,
        // used to reserve buffers.
        //
        // The first version set it to 1e9 to mean "until EOF" and the program
        // died with bad_alloc before reading a frame: expected_frames is
        // duration * camera_hz, so it asked for thirty billion samples. A
        // sentinel that flows into an allocation is not a sentinel.
        //
        // The container's frame count is the right hint, and it is allowed to
        // be wrong — a truncated file over-reports and the vectors simply do
        // not fill. Clamped so a container claiming an absurd count cannot do
        // the same thing again.
        auto probe = probe_video(opt.video_path);
        const double frames = (probe && probe->frame_count > 0)
            ? static_cast<double>(probe->frame_count) : 3600.0;
        sc.duration_s = std::min(frames, 1.0e6) / std::max(1.0, static_cast<double>(sc.camera_hz));
    }
    pipe.set_publish_snapshots(opt.fingerprint);

    const CameraGeometry cam    = sc.camera_geometry();
    // In screen mode the video IS the canvas, so centroids are reported on the
    // FILE's scale. pipeline.cpp resolves this; reading it back rather than
    // recomputing keeps one source of truth for the number that §13.2's graded
    // artifact is denominated in.
    const ScreenGeometry screen = pipe.config().synthetic.screen;
    const size_t expected = static_cast<size_t>(sc.duration_s * sc.camera_hz) + 2;

    // --- artifacts, opened BEFORE the run (§13.2's A9 rule) ----------------
    CentroidLog log;
    const std::string csv_path = opt.out_dir + "/centroid.csv";
    const bool want_csv = opt.write_artifacts && !opt.no_csv;
    if (want_csv) {
        CentroidLogHeader h;
        h.source = opt.video_path.empty() ? opt.scenario_path : opt.video_path;
        h.mode   = pipe.is_video() ? pipe.video()->name() : "synthetic";
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

    // CP 10.x's control trace. Opened alongside centroid.csv and for the same
    // reason — a file that only appears at the end of a run is a file you do
    // not get when the run is what went wrong.
    TraceLog trace_log;
    if (opt.write_trace) {
        TraceLogHeader th;
        th.source    = opt.video_path.empty() ? opt.scenario_path : opt.video_path;
        th.build     = SAT_GIT_HASH;
        th.utc       = utc_timestamp_now();
        th.ifov_urad = cam.ifov_urad();
        th.kp   = sc.control.kp;   th.ki   = sc.control.ki;
        th.kd   = sc.control.kd;   th.k_ff = sc.control.k_ff;
        const std::string tp = opt.out_dir + "/trace.csv";
        if (!trace_log.open(tp, th)) {
            std::fprintf(stderr, "sat-tracker: cannot write '%s'\n", tp.c_str());
            return 1;
        }
    }

    MetricCollector metrics;
    metrics.begin(sc.name, sc.seed, cam.ifov_urad(), expected,
                  /*pointing_supported=*/!pipe.is_video()
                      || pipe.video()->supports_pointing());

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
        if (trace_log.is_open()) {
            TraceSample ts;
            ts.frame  = r.frame;
            ts.time_s = r.time_s;
            ts.mode   = track_mode_name(r.mode);
            if (r.truth_valid) {
                // SIGNED, in screen pixels. The magnitude alone cannot tell a
                // lag from a lead, and CP 10.1's whole finding — that a
                // one-frame-ahead aim plus full feedforward overshoots — is
                // invisible in an absolute value.
                const Pixel2 bore = screen.to_pixel(r.boresight_true);
                ts.truth_valid       = true;
                ts.err_x_px          = bore.x - r.truth_screen.x;
                ts.err_y_px          = bore.y - r.truth_screen.y;
                ts.tracking_error_px = r.tracking_error_px;
            }
            ts.cmd_rate_x = r.cmd_rate.x;  ts.cmd_rate_y = r.cmd_rate.y;
            const Rate2 gr = pipe.gimbal().rate();
            ts.gimbal_rate_x = gr.x;       ts.gimbal_rate_y = gr.y;
            ts.integ_x = pipe.controller().az().integrator();
            ts.integ_y = pipe.controller().el().integrator();
            ts.est_rate_x = r.estimate_rate.x;
            ts.est_rate_y = r.estimate_rate.y;
            const GimbalParams& gp = pipe.gimbal().az().params();
            ts.saturated = gp.max_rate_urad_s > 0.0
                        && (std::fabs(gr.x) >= gp.max_rate_urad_s * 0.999
                         || std::fabs(gr.y) >= pipe.gimbal().el().params()
                                                   .max_rate_urad_s * 0.999);
            ts.imm_cv = r.imm_mode_prob[0];
            ts.imm_ca = r.imm_mode_prob[1];
            ts.imm_ct = r.imm_mode_prob[2];
            ts.imm_turn_rate = r.imm_turn_rate;
            trace_log.write(ts);
        }
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
        if (opt.stage_timings) {
            std::printf("\n%s", format_stage_timings(pipe.timers()).c_str());
        }
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
        } else if (std::strcmp(a, "--video") == 0 && k + 1 < argc) {
            opt.video_path = argv[++k];
        } else if (std::strcmp(a, "--truth") == 0 && k + 1 < argc) {
            opt.truth_path = argv[++k];
        } else if (std::strcmp(a, "--video-mode") == 0 && k + 1 < argc) {
            opt.video_mode = argv[++k];
        } else if (std::strcmp(a, "--set") == 0 && k + 1 < argc) {
            // `--set dotted.key=value`. The value is TOML source text, so
            // strings need quoting exactly as they would in the file — which
            // is the overlay's contract and not a special case here.
            const std::string kv = argv[++k];
            const size_t eq = kv.find('=');
            if (eq == std::string::npos || eq == 0) {
                std::fprintf(stderr,
                             "sat-tracker: --set wants 'dotted.key=value', got '%s'\n",
                             kv.c_str());
                return 2;
            }
            opt.overrides.emplace_back(kv.substr(0, eq), kv.substr(eq + 1));
        } else if (std::strcmp(a, "--trace") == 0) {
            opt.write_trace = true;
        } else if (std::strcmp(a, "--stages") == 0) {
            opt.stage_timings = true;
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

int video_command(int argc, char* argv[], int& i) {
    if (i + 1 >= argc) {
        std::fprintf(stderr, "sat-tracker: --video needs a file path\n");
        return 2;
    }
    HeadlessOptions opt;
    opt.scenario_path = std::string(SAT_SCENARIO_DIR) + "/video_screen.toml";
    opt.video_path    = argv[++i];

    for (int k = i + 1; k < argc; ++k) {
        const char* a = argv[k];
        if      (std::strcmp(a, "--truth") == 0 && k + 1 < argc)      opt.truth_path = argv[++k];
        else if (std::strcmp(a, "--video-mode") == 0 && k + 1 < argc) opt.video_mode = argv[++k];
        else if (std::strcmp(a, "--scenario") == 0 && k + 1 < argc)   opt.scenario_path = argv[++k];
        else if (std::strcmp(a, "--out") == 0 && k + 1 < argc)        opt.out_dir = argv[++k];
        else if (std::strcmp(a, "--no-ai") == 0)                      opt.no_ai = true;
        else if (std::strcmp(a, "--quiet") == 0)                      opt.quiet = true;
        else if (std::strcmp(a, "--no-csv") == 0)                     opt.no_csv = true;
        else if (std::strcmp(a, "--no-report") == 0)                  opt.write_report = false;
        else {
            std::fprintf(stderr, "sat-tracker: --video: unrecognised option '%s'\n", a);
            return 2;
        }
        i = k;
    }
    return run_headless(opt);
}

}  // namespace sat
