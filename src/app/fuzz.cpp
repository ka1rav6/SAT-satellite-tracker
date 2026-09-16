// app/fuzz.cpp — CP 14.1.

#include "app/fuzz.hpp"

#include "core/rng.hpp"
#include "engine/pipeline.hpp"
#include "metrics/collector.hpp"
#include "scenario/schema.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace sat {

namespace {

// ---------------------------------------------------------------------------
// A sampler that hits its endpoints.
//
// Uniform sampling over [lo, hi] almost never produces lo or hi, and those are
// where the bugs are: a division by a value that is legally zero, a loop bound
// that is legally one, an array indexed by a size that is legally its maximum.
// One draw in six takes an endpoint, which is what §14's "including corners"
// asks for.
// ---------------------------------------------------------------------------
struct Sampler {
    Pcg32 rng;

    double real(double lo, double hi) {
        const uint32_t r = rng.next_u32() % 6u;
        if (r == 0) return lo;
        if (r == 1) return hi;
        return lo + (hi - lo) * rng.next_double();
    }
    int integer(int lo, int hi) {
        const uint32_t r = rng.next_u32() % 6u;
        if (r == 0) return lo;
        if (r == 1) return hi;
        if (hi <= lo) return lo;
        return lo + static_cast<int>(rng.next_u32() % static_cast<uint32_t>(hi - lo + 1));
    }
    bool boolean() { return (rng.next_u32() & 1u) != 0u; }

    template <size_t N>
    const char* pick(const char* const (&options)[N]) {
        return options[rng.next_u32() % static_cast<uint32_t>(N)];
    }
};

// ---------------------------------------------------------------------------
// Draw a scenario.
//
// Ranges come from scenario/schema.cpp — the same table the loader validates
// against — so that every scenario this produces is one the system claims to
// accept. Fuzzing OUTSIDE the legal ranges would be testing the validator,
// which is a different and already-covered job (tests/bad_configs).
// ---------------------------------------------------------------------------
Scenario draw(Sampler& s, double duration_s) {
    Scenario sc;
    sc.name       = "fuzz";
    sc.duration_s = duration_s;
    sc.seed       = s.rng.next_u64();

    // -----------------------------------------------------------------------
    // The rates, and why they are constructed rather than drawn independently.
    //
    // INV-3 requires truth_hz to be an exact multiple of BOTH camera_hz and
    // control_hz, so that the sub-tick divisors are integers and the clock is
    // derived rather than accumulated. Drawing all three uniformly makes that
    // true almost never: the first version of this fuzzer had the schema
    // reject 58 of every 60 draws, and a fuzzer that spends 97% of its budget
    // re-proving one validation message already covered by tests/bad_configs
    // is not testing the engine at all.
    //
    // So the two visible rates are drawn and truth_hz is built as a multiple
    // of their LCM. That keeps the interesting axes — 30 Hz against 240 Hz,
    // one sub-tick against many — and spends the draws on the simulator.
    // -----------------------------------------------------------------------
    sc.camera_hz  = s.integer(30, 240);
    sc.control_hz = s.integer(20, 240);
    {
        int a = sc.camera_hz, b = sc.control_hz;
        while (b != 0) { const int t = a % b; a = b; b = t; }   // gcd
        const int lcm = sc.camera_hz / a * sc.control_hz;
        // Capped: an LCM of 240 and 239 is 57,360, and multiplying that by 12
        // asks for a 688 kHz world for no benefit. Clamp to a multiple that
        // stays inside the schema's own ceiling.
        int mult = s.integer(1, 12);
        while (mult > 1 && static_cast<long long>(lcm) * mult > 20000) --mult;
        sc.truth_hz = lcm * mult;
    }

    sc.canvas_px[0] = s.integer(2000, 6000);
    sc.canvas_px[1] = s.integer(2000, 6000);

    sc.resolution[0] = s.integer(64, 1920);
    sc.resolution[1] = s.integer(64, 1080);
    sc.fov_deg[0]    = s.real(0.1, 60.0);
    sc.fov_deg[1]    = s.real(0.1, 60.0);
    sc.exposure_ms   = s.real(0.0, 1000.0 / sc.camera_hz);
    sc.blur_substeps = s.integer(1, 32);
    sc.initial_pos_px[0] = s.real(1.0, sc.canvas_px[0] - 1.0);
    sc.initial_pos_px[1] = s.real(1.0, sc.canvas_px[1] - 1.0);

    const int n_targets = s.integer(1, 4);
    for (int i = 0; i < n_targets; ++i) {
        TargetSpec t;
        t.intensity = s.real(1.0, 255.0);
        t.size_px   = s.integer(5, 20);
        t.random_initial = s.boolean();
        // Inside the screen, which is the canvas — spec row 11's own bound.
        // A margin keeps the draw off the exact edge except when the corner
        // sampler chooses it deliberately.
        t.initial_px[0]  = s.real(1.0, sc.canvas_px[0] - 1.0);
        t.initial_px[1]  = s.real(1.0, sc.canvas_px[1] - 1.0);
        static const char* const shapes[] = {"square", "circle", "gaussian"};
        t.shape_type = s.pick(shapes);

        const int n_motion = s.integer(0, 3);
        for (int m = 0; m < n_motion; ++m) {
            MotionSpec ms;
            static const char* const kinds[] = {
                "constant", "linear", "accel", "circular", "sinusoid",
                "lissajous", "spiral", "ou_noise"};
            ms.kind = s.pick(kinds);
            ms.velocity_px_s[0] = s.real(-2000.0, 2000.0);
            ms.velocity_px_s[1] = s.real(-2000.0, 2000.0);
            ms.accel_px_s2[0]   = s.real(-5000.0, 5000.0);
            ms.accel_px_s2[1]   = s.real(-5000.0, 5000.0);
            ms.amplitude_px[0]  = s.real(0.0, 2000.0);
            ms.amplitude_px[1]  = s.real(0.0, 2000.0);
            ms.offset_px[0]     = s.real(0.0, 4000.0);
            ms.offset_px[1]     = s.real(0.0, 4000.0);
            ms.radius_px        = s.real(0.0, 2000.0);
            // A zero period is a division by zero waiting to happen in every
            // periodic component, which is exactly the corner worth drawing.
            ms.period_s         = s.real(0.0, 120.0);
            ms.phase_deg        = s.real(0.0, 360.0);
            ms.freq_ratio       = s.real(0.0, 8.0);
            ms.r0_px            = s.real(0.0, 500.0);
            ms.growth_px_s      = s.real(-200.0, 200.0);
            ms.sigma_px_s       = s.real(0.0, 500.0);
            ms.tau_s            = s.real(0.0, 60.0);
            ms.axis             = s.integer(0, 1);
            t.motion.push_back(ms);
        }
        sc.targets.push_back(t);
    }

    sc.max_pan_dps      = s.real(5.0, 10.0);
    sc.max_tilt_dps     = s.real(5.0, 10.0);
    sc.max_accel_dps2   = s.real(0.0, 1000.0);
    sc.time_constant_s  = s.real(0.0, 0.5);
    sc.latency_s        = s.real(0.0, 0.2);
    sc.encoder_lsb_urad = s.real(0.0, 500.0);
    sc.resonance_hz     = s.real(0.0, 100.0);

    sc.noise_poisson  = s.boolean();
    sc.gaussian_sigma = s.real(0.0, 20.0);
    sc.salt_pepper    = s.real(0.0, 1.0);
    sc.hot_pixels     = s.integer(0, 5000);

    static const char* const weather[] = {"clear", "haze", "fog", "rain", "lowlight"};
    const std::string w = s.pick(weather);
    sc.atmosphere = (w == "haze") ? Atmosphere::Haze
                  : (w == "fog")  ? Atmosphere::Fog
                  : (w == "rain") ? Atmosphere::Rain
                  : (w == "lowlight") ? Atmosphere::LowLight
                                      : Atmosphere::Clear;

    sc.jitter_px_per_frame = s.real(0.0, 20.0);
    const int n_plat = s.integer(0, 2);
    for (int i = 0; i < n_plat; ++i) {
        MotionSpec ms;
        static const char* const kinds[] = {"constant", "linear", "sinusoid", "ou_noise"};
        ms.kind = s.pick(kinds);
        ms.velocity_px_s[0] = s.real(-500.0, 500.0);
        ms.velocity_px_s[1] = s.real(-500.0, 500.0);
        ms.amplitude_px[0]  = s.real(0.0, 500.0);
        ms.amplitude_px[1]  = s.real(0.0, 500.0);
        ms.period_s         = s.real(0.0, 60.0);
        ms.sigma_px_s       = s.real(0.0, 200.0);
        ms.tau_s            = s.real(0.0, 30.0);
        sc.platform.push_back(ms);
    }

    sc.static_sources = s.integer(0, 1000);
    sc.decoy_beacons  = s.integer(0, 16);

    sc.control.kp   = s.real(0.0, 40.0);
    sc.control.ki   = s.real(0.0, 40.0);
    sc.control.kd   = s.real(0.0, 10.0);
    sc.control.k_ff = s.real(0.0, 2.0);
    sc.control.i_limit    = s.real(0.0, 1.0e7);
    sc.control.anti_windup = s.boolean();
    sc.control.smith       = s.boolean();

    sc.tracking_imm        = s.boolean();
    sc.supervisor_enabled  = s.boolean();
    sc.supervisor_dwell    = s.integer(1, 300);
    sc.supervisor_ema_tau  = s.real(0.0, 300.0);

    const int n_events = s.integer(0, 4);
    for (int i = 0; i < n_events; ++i) {
        EventSpec e;
        e.t_s = s.real(0.0, duration_s);
        static const char* const actions[] = {
            "set_atmosphere", "occlude_target", "spawn_decoy", "platform_gust"};
        e.action = s.pick(actions);
        static const char* const modes[] = {"clear", "haze", "fog", "rain", "lowlight"};
        e.mode = s.pick(modes);
        e.ramp_s       = s.real(0.0, 5.0);
        e.duration_s   = s.real(0.0, 5.0);
        e.offset_px[0] = s.real(-500.0, 500.0);
        e.offset_px[1] = s.real(-500.0, 500.0);
        e.magnitude_px = s.real(0.0, 200.0);
        sc.events.push_back(e);
    }
    return sc;
}

/// Every reported number, checked. A NaN is singled out because it is the
/// failure that PROPAGATES: it survives every comparison, poisons every
/// average it enters, and surfaces three stages later as a metric nobody can
/// explain.
bool finite_metrics(const RunMetrics& m, const char*& which) {
    const struct { const char* name; double v; } checks[] = {
        {"centroid_rmse_image",  m.centroid_rmse_image_px},
        {"centroid_rmse_screen", m.centroid_rmse_screen_px},
        {"centroid_bias_x",      m.centroid_bias_x_px},
        {"centroid_bias_y",      m.centroid_bias_y_px},
        {"tracking_rms",         m.tracking_rms_px},
        {"tracking_p95",         m.tracking_p95_px},
        {"tracking_max",         m.tracking_max_px},
        {"acquisition_cold",     m.acquisition_cold_s},
        {"acquisition_in_fov",   m.acquisition_in_fov_s},
        {"lock_retention",       m.lock_retention_rate},
        {"target_loss",          m.target_loss_frac},
        {"false_track_rate",     m.false_track_rate_per_min},
        {"saturation_frac",      m.saturation_frac},
        {"handover_time",        m.handover_time_s},
    };
    for (const auto& c : checks) {
        if (!std::isfinite(c.v)) { which = c.name; return false; }
    }
    return true;
}

}  // namespace

int run_fuzz(const FuzzOptions& opt) {
    Sampler s;
    s.rng = Pcg32(opt.seed);

    int failures = 0;
    int rejected = 0;
    int64_t total_frames = 0;

    std::printf("CP 14.1: %d random scenarios, %.2f s each, seed %llu\n"
                "  every parameter across its legal range, one draw in six on "
                "an endpoint\n\n",
                opt.count, opt.duration_s,
                static_cast<unsigned long long>(opt.seed));

    for (int i = 0; i < opt.count; ++i) {
        const Scenario sc = draw(s, opt.duration_s);

        // The fuzzer builds a Scenario struct directly rather than TOML text,
        // so the loader's validation has not run. Check it here: a draw the
        // schema would refuse is not a bug in the engine, and running it would
        // be testing a configuration the system has already said no to.
        Validator v("fuzz");
        validate_scenario(sc, v);
        if (!v.ok()) {
            ++rejected;
            if (opt.verbose) {
                std::printf("  [%d] rejected: %s\n", i,
                            v.errors().front().format().c_str());
            }
            continue;
        }

        Pipeline p;
        p.build_from_scenario(sc);
        MetricCollector mc;
        mc.begin(sc.name, sc.seed, sc.camera_geometry().ifov_urad(),
                 static_cast<size_t>(sc.duration_s * sc.camera_hz) + 2,
                 /*pointing_supported=*/true);

        int64_t frames = 0;
        // A hang is a failure too, and the bound has to come from the scenario
        // rather than from a wall clock (INV-3 keeps the clock out of the
        // simulation path). Four times the expected frame count is generous
        // enough that no correct run reaches it.
        const int64_t cap = static_cast<int64_t>(sc.duration_s * sc.camera_hz) * 4 + 64;
        while (p.step()) {
            mc.add(p.last());
            if (++frames > cap) {
                std::printf("  [%d] HANG: %lld frames against an expected %d\n",
                            i, static_cast<long long>(frames),
                            static_cast<int>(sc.duration_s * sc.camera_hz));
                ++failures;
                break;
            }
        }
        total_frames += frames;

        // A nominal wall time: this is looking for NaNs in the metrics, not
        // measuring speed, and reading a real clock 5,000 times would add a
        // dependency on the machine to a test about arithmetic.
        const RunMetrics m = mc.finish(p.timers(), 1.0,
                                       p.gimbal().saturation_frac());
        const char* which = "";
        if (!finite_metrics(m, which)) {
            std::printf("  [%d] NaN in %s  (seed %llu, %dx%d px, fov %.3fx%.3f deg, "
                        "%d targets, %d clutter)\n",
                        i, which, static_cast<unsigned long long>(sc.seed),
                        sc.resolution[0], sc.resolution[1],
                        sc.fov_deg[0], sc.fov_deg[1],
                        static_cast<int>(sc.targets.size()), sc.static_sources);
            ++failures;
        } else if (opt.verbose) {
            std::printf("  [%d] ok  %lld frames\n", i, static_cast<long long>(frames));
        }
    }

    std::printf("\n  %d scenarios run, %d rejected by the schema before running,\n"
                "  %lld frames simulated, %d failures\n",
                opt.count - rejected, rejected,
                static_cast<long long>(total_frames), failures);
    if (failures == 0) {
        std::printf("\nPASS: no crash, no hang, no NaN.\n");
        return 0;
    }
    std::printf("\nFAIL: %d scenarios produced a crash, a hang or a NaN.\n", failures);
    return 1;
}

int fuzz_command(int argc, char* argv[], int& i) {
    FuzzOptions opt;
    for (int k = i + 1; k < argc; ++k) {
        const char* a = argv[k];
        if (std::strcmp(a, "--count") == 0 && k + 1 < argc) {
            opt.count = std::atoi(argv[++k]);
        } else if (std::strcmp(a, "--seed") == 0 && k + 1 < argc) {
            opt.seed = static_cast<uint64_t>(std::atoll(argv[++k]));
        } else if (std::strcmp(a, "--duration") == 0 && k + 1 < argc) {
            opt.duration_s = std::atof(argv[++k]);
        } else if (std::strcmp(a, "--verbose") == 0) {
            opt.verbose = true;
        } else if (a[0] == '-') {
            std::fprintf(stderr,
                         "sat-tracker: --fuzz-scenarios: unrecognised option '%s'\n", a);
            return 2;
        } else {
            // Bare count, so `--fuzz-scenarios 5000` reads the way §13.4
            // writes it.
            opt.count = std::atoi(a);
        }
        i = k;
    }
    if (opt.count <= 0) {
        std::fprintf(stderr, "sat-tracker: --fuzz-scenarios needs a positive count\n");
        return 2;
    }
    return run_fuzz(opt);
}

}  // namespace sat
