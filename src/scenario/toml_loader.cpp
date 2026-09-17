// scenario/toml_loader.cpp — TOML into Scenario, with validation.
//
// toml++ is the only configuration language (design §4, decision 5). It is used
// here for parsing and for SOURCE LINE NUMBERS, which is what makes design
// §7.5's error format possible at all — without them the best we could say is
// "gimbal.max_pan_dps is wrong", and the user would have to find it.
//
// ---------------------------------------------------------------------------
// THE SHAPE OF THIS FILE
// ---------------------------------------------------------------------------
// Every key is read through one of the small get_* helpers, which do three
// things at once: read the value if present, leave the default alone if not,
// and range-check it against the schema table while remembering the line it
// came from. That coupling is deliberate — a key read without a schema check is
// a key whose bounds nobody is enforcing, and keeping them in one call makes
// the omission visible.

#include "scenario/schema.hpp"

#include <toml++/toml.hpp>

#include <fstream>
#include <sstream>

namespace sat {
namespace {

/// The source line a node came from, or 0. toml++ tracks this for every node,
/// which is the whole reason it was chosen over a hand-rolled parser.
int line_of(const toml::node* n) {
    return n ? static_cast<int>(n->source().begin.line) : 0;
}

// ---------------------------------------------------------------------------
// Scalar readers. Each leaves `out` untouched when the key is absent, so the
// struct's defaults stand and an omitted key is never an error unless the
// schema marks it required.
// ---------------------------------------------------------------------------
void get_double(const toml::table& t, std::string_view path, double& out, Validator& v) {
    const toml::node* n = t.at_path(path).node();
    if (!n) return;
    if (auto d = n->value<double>()) {
        out = *d;
        v.check_range(path, out, line_of(n));
    } else {
        v.add(path, std::string(path) + " must be a number", line_of(n));
    }
}

void get_int(const toml::table& t, std::string_view path, int& out, Validator& v) {
    const toml::node* n = t.at_path(path).node();
    if (!n) return;
    if (auto i = n->value<int64_t>()) {
        out = static_cast<int>(*i);
        v.check_range(path, static_cast<double>(out), line_of(n));
    } else {
        v.add(path, std::string(path) + " must be an integer", line_of(n));
    }
}

void get_u64(const toml::table& t, std::string_view path, uint64_t& out, Validator& v) {
    const toml::node* n = t.at_path(path).node();
    if (!n) return;
    if (auto i = n->value<int64_t>()) {
        if (*i < 0) {
            v.add(path, std::string(path) + " must not be negative", line_of(n));
            return;
        }
        out = static_cast<uint64_t>(*i);
    } else {
        v.add(path, std::string(path) + " must be an integer", line_of(n));
    }
}

void get_bool(const toml::table& t, std::string_view path, bool& out, Validator& v) {
    const toml::node* n = t.at_path(path).node();
    if (!n) return;
    if (auto b = n->value<bool>()) out = *b;
    else v.add(path, std::string(path) + " must be true or false", line_of(n));
}

void get_string(const toml::table& t, std::string_view path, std::string& out, Validator& v) {
    const toml::node* n = t.at_path(path).node();
    if (!n) return;
    if (auto s = n->value<std::string>()) out = *s;
    else v.add(path, std::string(path) + " must be a string", line_of(n));
}

/// A two-element numeric array: resolution, fov, canvas, velocity, and so on.
/// Both elements are range-checked against the same schema entry, which is
/// correct for every current use (a resolution of 640x2 is as wrong as 2x640).
void get_array2(const toml::table& t, std::string_view path, double* out, Validator& v) {
    const toml::node* n = t.at_path(path).node();
    if (!n) return;
    const toml::array* a = n->as_array();
    if (!a || a->size() != 2) {
        v.add(path, std::string(path) + " must be an array of two numbers", line_of(n));
        return;
    }
    for (int i = 0; i < 2; ++i) {
        if (auto d = (*a)[static_cast<size_t>(i)].value<double>()) {
            out[i] = *d;
            v.check_range(path, out[i], line_of(n));
        } else {
            v.add(path, std::string(path) + " must contain numbers", line_of(n));
        }
    }
}

void get_array2_int(const toml::table& t, std::string_view path, int* out, Validator& v) {
    double tmp[2] = {static_cast<double>(out[0]), static_cast<double>(out[1])};
    get_array2(t, path, tmp, v);
    out[0] = static_cast<int>(tmp[0]);
    out[1] = static_cast<int>(tmp[1]);
}

// ---------------------------------------------------------------------------
// Motion components — design §7.2.
//
// The `kind` string selects which parameters are meaningful. Unknown kinds are
// rejected with the full list, because a typo in a motion kind would otherwise
// produce a stationary target and a confusing benchmark rather than an error.
// ---------------------------------------------------------------------------
MotionSpec parse_motion(const toml::table& m, const std::string& key, Validator& v) {
    MotionSpec s;
    const int line = static_cast<int>(m.source().begin.line);

    if (auto k = m["kind"].value<std::string>()) s.kind = *k;
    else v.add(key + ".kind", key + ".kind is required (design §7.2)", line);

    static constexpr const char* kKinds[] = {
        "constant", "linear", "accel", "sinusoid", "circular",
        "lissajous", "spiral", "ou_noise", "waypoints"
    };
    bool known = false;
    for (const char* k : kKinds) if (s.kind == k) { known = true; break; }
    if (!s.kind.empty() && !known) {
        v.check_enum(key + ".kind", s.kind,
                     {"constant", "linear", "accel", "sinusoid", "circular",
                      "lissajous", "spiral", "ou_noise", "waypoints"},
                     line, "row 12");
    }

    auto arr2 = [&](const char* name, double* dst) {
        if (const toml::array* a = m[name].as_array()) {
            if (a->size() == 2) {
                for (int i = 0; i < 2; ++i) {
                    if (auto d = (*a)[static_cast<size_t>(i)].value<double>()) dst[i] = *d;
                }
            } else {
                v.add(key + "." + name, key + "." + name +
                      " must be an array of two numbers", line);
            }
        }
    };
    auto num = [&](const char* name, double& dst) {
        if (auto d = m[name].value<double>()) dst = *d;
    };

    arr2("offset_px",     s.offset_px);
    arr2("velocity_px_s", s.velocity_px_s);
    arr2("accel_px_s2",   s.accel_px_s2);
    arr2("amplitude_px",  s.amplitude_px);
    num("radius_px",   s.radius_px);
    num("r0_px",       s.r0_px);
    num("growth_px_s", s.growth_px_s);
    num("period_s",    s.period_s);
    num("phase_deg",   s.phase_deg);
    num("freq_ratio",  s.freq_ratio);
    num("sigma_px_s",  s.sigma_px_s);
    num("tau_s",       s.tau_s);

    if (auto ax = m["axis"].value<std::string>()) {
        s.axis = (*ax == "y" || *ax == "Y") ? 1 : 0;
    } else if (auto ai = m["axis"].value<int64_t>()) {
        s.axis = static_cast<int>(*ai);
    }

    // A period of zero would divide by zero when computing omega. The
    // components guard against it, but catching it here says WHERE.
    if (s.period_s <= 0.0 &&
        (s.kind == "sinusoid" || s.kind == "circular" ||
         s.kind == "lissajous" || s.kind == "spiral")) {
        v.add(key + ".period_s",
              key + ".period_s = " + std::to_string(s.period_s) +
              " must be positive for a periodic motion (specification row 12)", line);
    }
    if (s.kind == "ou_noise" && s.tau_s <= 0.0) {
        v.add(key + ".tau_s",
              key + ".tau_s must be positive: it is the correlation time of the "
              "Ornstein-Uhlenbeck process (design §7.2)", line);
    }

    if (const toml::array* pts = m["points"].as_array()) {
        for (const auto& p : *pts) {
            if (const toml::array* row = p.as_array()) {
                if (row->size() == 3) {
                    std::array<double, 3> w{};
                    for (size_t i = 0; i < 3; ++i) {
                        if (auto d = (*row)[i].value<double>()) w[i] = *d;
                    }
                    s.points.push_back(w);
                }
            }
        }
        if (s.kind == "waypoints" && s.points.size() < 2) {
            v.add(key + ".points",
                  key + ".points needs at least two waypoints to define a path "
                  "(specification row 12, user-defined motion)", line);
        }
    }
    return s;
}

void parse_motion_array(const toml::table& root, std::string_view path,
                        std::vector<MotionSpec>& out, Validator& v) {
    const toml::node* n = root.at_path(path).node();
    if (!n) return;
    if (const toml::array* a = n->as_array()) {
        for (size_t i = 0; i < a->size(); ++i) {
            if (const toml::table* m = (*a)[i].as_table()) {
                out.push_back(parse_motion(*m, std::string(path) + "[" +
                                               std::to_string(i) + "]", v));
            }
        }
    }
}

}  // namespace

// ===========================================================================
// parse_scenario
// ===========================================================================
Result<Scenario> parse_scenario(std::string_view toml_text, std::string_view name) {
    Validator v{std::string(name)};
    Scenario  sc;
    sc.source_path = std::string(name);

    toml::table root;
    try {
        root = toml::parse(toml_text, name);
    } catch (const toml::parse_error& e) {
        // A syntax error is reported in exactly the same shape as a semantic
        // one, so a user never has to learn two error formats.
        std::string msg = std::string(name) + ":" +
                          std::to_string(e.source().begin.line) + ": " +
                          std::string(e.description());
        return Err(std::move(msg));
    }

    // --- [meta] -------------------------------------------------------------
    get_string(root, "meta.name",        sc.name,        v);
    get_string(root, "meta.description", sc.description, v);

    // --- [input] — design §8.3 ---------------------------------------------
    {
        std::string mode = "synthetic";
        get_string(root, "input.mode", mode, v);
        if      (mode == "synthetic")    sc.input_mode = InputMode::Synthetic;
        else if (mode == "video_screen") sc.input_mode = InputMode::VideoScreen;
        else if (mode == "video_direct") sc.input_mode = InputMode::VideoDirect;
        else v.check_enum("input.mode", mode,
                          {"synthetic", "video_screen", "video_direct"},
                          line_of(root.at_path("input.mode").node()));
        get_string(root, "input.video_file",      sc.video_file,      v);
        get_string(root, "input.video_truth_csv", sc.video_truth_csv, v);
    }

    // --- [sim] --------------------------------------------------------------
    get_int   (root, "sim.truth_hz",   sc.truth_hz,   v);
    get_int   (root, "sim.camera_hz",  sc.camera_hz,  v);
    get_int   (root, "sim.control_hz", sc.control_hz, v);
    get_double(root, "sim.duration_s", sc.duration_s, v);
    get_u64   (root, "sim.seed",       sc.seed,       v);

    // --- [world] — rows 1, 6 -----------------------------------------------
    get_array2_int(root, "world.canvas_px", sc.canvas_px, v);
    {
        std::string edge = "bounce";
        get_string(root, "world.edge_behaviour", edge, v);
        if      (edge == "bounce") sc.edge_behaviour = EdgeBehaviour::Bounce;
        else if (edge == "wrap")   sc.edge_behaviour = EdgeBehaviour::Wrap;
        else if (edge == "exit")   sc.edge_behaviour = EdgeBehaviour::Exit;
        else v.check_enum("world.edge_behaviour", edge, {"bounce", "wrap", "exit"},
                          line_of(root.at_path("world.edge_behaviour").node()));
    }

    // --- [camera] — rows 2-6 -----------------------------------------------
    {
        std::string type = "mono";
        get_string(root, "camera.type", type, v);
        if      (type == "mono")   sc.camera_type = CameraType::Mono;
        else if (type == "colour" || type == "color") sc.camera_type = CameraType::Colour;
        else v.check_enum("camera.type", type, {"mono", "colour"},
                          line_of(root.at_path("camera.type").node()), "row 2");
    }
    get_array2_int(root, "camera.resolution",    sc.resolution,    v);
    get_array2    (root, "camera.fov_deg",       sc.fov_deg,       v);
    get_double    (root, "camera.exposure_ms",   sc.exposure_ms,   v);
    get_int       (root, "camera.blur_substeps", sc.blur_substeps, v);
    get_array2    (root, "camera.initial_pos_px", sc.initial_pos_px, v);

    // Spec row 6's default is the centre of the screen, which depends on the
    // canvas size — so it can only be applied after the canvas is known.
    if (!root.at_path("camera.initial_pos_px").node()) {
        sc.initial_pos_px[0] = (sc.canvas_px[0] - 1) * 0.5;
        sc.initial_pos_px[1] = (sc.canvas_px[1] - 1) * 0.5;
    }

    // --- [[target]] — rows 7-12 --------------------------------------------
    // Accepts both `[target]` (a single table, as design §7.1 writes it) and
    // `[[target]]` (an array, for spec row 8's "multiple optional").
    auto parse_target = [&](const toml::table& t, const std::string& key) {
        TargetSpec ts;
        if (auto s = t["kind"].value<std::string>())      ts.kind = *s;
        if (auto d = t["intensity"].value<double>())      ts.intensity = *d;
        if (auto i = t["size_px"].value<int64_t>()) {
            ts.size_px = static_cast<int>(*i);
            v.check_range("target.size_px", static_cast<double>(ts.size_px),
                          line_of(t["size_px"].node()));
        }
        if (auto d = t["intensity"].value<double>()) {
            v.check_range("target.intensity", *d, line_of(t["intensity"].node()));
        }

        // Row 11: "random" or an explicit [x, y].
        if (const toml::node* n = t["initial_px"].node()) {
            if (auto s = n->value<std::string>()) {
                if (*s == "random") ts.random_initial = true;
                else v.check_enum(key + ".initial_px", *s, {"random"},
                                  line_of(n), "row 11");
            } else if (const toml::array* a = n->as_array()) {
                if (a->size() == 2) {
                    ts.random_initial = false;
                    for (int i = 0; i < 2; ++i) {
                        if (auto d = (*a)[static_cast<size_t>(i)].value<double>()) {
                            ts.initial_px[i] = *d;
                        }
                    }
                } else {
                    v.add(key + ".initial_px",
                          key + ".initial_px must be \"random\" or [x, y] "
                          "(specification row 11)", line_of(n));
                }
            }
        }

        if (auto s = t["shape"]["type"].value<std::string>()) {
            ts.shape_type = *s;
            v.check_enum(key + ".shape.type", ts.shape_type,
                         {"square", "circle", "gaussian", "mask"},
                         line_of(t["shape"]["type"].node()), "row 9");
        }
        if (auto s = t["shape"]["mask_file"].value<std::string>()) ts.mask_file = *s;

        if (const toml::array* a = t["motion"].as_array()) {
            for (size_t i = 0; i < a->size(); ++i) {
                if (const toml::table* m = (*a)[i].as_table()) {
                    ts.motion.push_back(
                        parse_motion(*m, key + ".motion[" + std::to_string(i) + "]", v));
                }
            }
        }
        sc.targets.push_back(std::move(ts));
    };

    if (const toml::node* n = root["target"].node()) {
        if (const toml::array* a = n->as_array()) {
            for (size_t i = 0; i < a->size(); ++i) {
                if (const toml::table* t = (*a)[i].as_table()) {
                    parse_target(*t, "target[" + std::to_string(i) + "]");
                }
            }
        } else if (const toml::table* t = n->as_table()) {
            parse_target(*t, "target");
        }
    }

    // --- [gimbal] — rows 13-15 ---------------------------------------------
    get_double(root, "gimbal.max_pan_dps",      sc.max_pan_dps,      v);
    get_double(root, "gimbal.max_tilt_dps",     sc.max_tilt_dps,     v);
    get_double(root, "gimbal.max_accel_dps2",   sc.max_accel_dps2,   v);
    get_double(root, "gimbal.time_constant_s",  sc.time_constant_s,  v);
    get_double(root, "gimbal.latency_s",        sc.latency_s,        v);
    get_double(root, "gimbal.encoder_lsb_urad", sc.encoder_lsb_urad, v);
    get_double(root, "gimbal.resonance_hz",     sc.resonance_hz,     v);

    // --- [noise] — rows 21-22 ----------------------------------------------
    get_bool  (root, "noise.poisson",        sc.noise_poisson,  v);
    get_double(root, "noise.gaussian_sigma", sc.gaussian_sigma, v);
    get_double(root, "noise.salt_pepper",    sc.salt_pepper,    v);
    get_int   (root, "noise.hot_pixels",     sc.hot_pixels,     v);

    // --- [atmosphere] — row 24 ---------------------------------------------
    {
        std::string mode = "clear";
        get_string(root, "atmosphere.mode", mode, v);
        if      (mode == "clear")    sc.atmosphere = Atmosphere::Clear;
        else if (mode == "haze")     sc.atmosphere = Atmosphere::Haze;
        else if (mode == "rain")     sc.atmosphere = Atmosphere::Rain;
        else if (mode == "fog")      sc.atmosphere = Atmosphere::Fog;
        else if (mode == "lowlight") sc.atmosphere = Atmosphere::LowLight;
        else v.check_enum("atmosphere.mode", mode,
                          {"clear", "haze", "rain", "fog", "lowlight"},
                          line_of(root.at_path("atmosphere.mode").node()), "row 24");
    }

    // --- [disturbance] — rows 23, 25 ---------------------------------------
    get_double(root, "disturbance.jitter_px_per_frame", sc.jitter_px_per_frame, v);
    parse_motion_array(root, "disturbance.platform", sc.platform, v);

    // --- [clutter] -----------------------------------------------------------
    get_int(root, "clutter.static_sources", sc.static_sources, v);
    get_int(root, "clutter.decoy_beacons",  sc.decoy_beacons,  v);

    // --- [ai] — design §11 --------------------------------------------------
    get_bool  (root, "ai.enabled",          sc.ai_enabled,       v);
    get_string(root, "ai.candidate_net",    sc.candidate_net,    v);
    get_string(root, "ai.centroid_net",     sc.centroid_net,     v);
    get_string(root, "ai.motion_net",       sc.motion_net,       v);
    get_string(root, "ai.recovery_net",     sc.recovery_net,     v);
    get_string(root, "ai.strategy_policy",  sc.strategy_policy,  v);
    get_bool  (root, "ai.fallback_on_fail", sc.fallback_on_fail, v);

    // --- [logging] -----------------------------------------------------------
    get_string(root, "logging.centroid_csv", sc.centroid_csv, v);
    get_string(root, "logging.metrics_json", sc.metrics_json, v);
    get_string(root, "logging.report_html",  sc.report_html,  v);

    // --- [control] — design §10.4 ------------------------------------------
    get_double(root, "control.kp",      sc.control.kp,      v);
    get_double(root, "control.ki",      sc.control.ki,      v);
    get_double(root, "control.kd",      sc.control.kd,      v);
    get_double(root, "control.k_ff",    sc.control.k_ff,    v);
    get_double(root, "control.i_limit", sc.control.i_limit, v);
    get_bool  (root, "control.anti_windup", sc.control.anti_windup, v);
    get_bool  (root, "control.smith",       sc.control.smith,       v);
    get_double(root, "control.smith_rate_blend", sc.control.smith_rate_blend, v);

    // --- [search] — design §10.5 -------------------------------------------
    get_string(root, "search.strategy", sc.search_strategy, v);
    v.check_enum("search.strategy", sc.search_strategy,
                 {"spiral", "raster", "probabilistic", "camp_and_wait"}, 0);

    // --- [supervisor] — design §10.6 ---------------------------------------
    get_bool  (root, "supervisor.enabled",          sc.supervisor_enabled, v);
    get_int   (root, "supervisor.min_dwell_frames", sc.supervisor_dwell,   v);
    get_double(root, "supervisor.ema_tau_frames",   sc.supervisor_ema_tau, v);

    // --- [tracking] — design §10.2 -----------------------------------------
    get_bool  (root, "tracking.imm", sc.tracking_imm, v);

    // --- [tracking] — §10.2's priority policy -------------------------------
    get_bool  (root, "tracking.priority",               sc.priority_enabled,     v);
    get_double(root, "tracking.priority_motion_weight", sc.priority_motion_w,    v);
    get_double(root, "tracking.priority_min_commit",    sc.priority_min_commit,  v);
    get_int   (root, "tracking.priority_min_frames",    sc.priority_min_frames,  v);
    get_int   (root, "tracking.priority_drop_frames",   sc.priority_drop_frames, v);
    get_int   (root, "tracking.priority_switch_frames", sc.priority_switch_frames, v);
    get_double(root, "tracking.priority_switch_ratio",  sc.priority_switch_ratio,  v);

    // --- [perception] — design §14.0b's detection window --------------------
    get_double(root, "perception.min_snr_factor",     sc.min_snr_factor,     v);
    get_bool  (root, "perception.roi",                sc.roi_enabled,        v);
    get_int   (root, "perception.roi_min_half_px",    sc.roi_min_half_px,    v);
    get_double(root, "perception.roi_sigma_margin",   sc.roi_sigma_margin,   v);
    get_int   (root, "perception.roi_refresh_frames", sc.roi_refresh_frames, v);

    // --- [requirements] — rows 16-20 ---------------------------------------
    get_double(root, "requirements.acquisition_s",     sc.acquisition_s,     v);
    get_double(root, "requirements.tracking_error_px", sc.tracking_error_px, v);
    get_double(root, "requirements.target_loss_frac",  sc.target_loss_frac,  v);
    get_double(root, "requirements.reacquisition_s",   sc.reacquisition_s,   v);
    get_double(root, "requirements.min_fps",           sc.min_fps,           v);

    // --- [[event]] — design §7.4 -------------------------------------------
    if (const toml::array* a = root["event"].as_array()) {
        for (size_t i = 0; i < a->size(); ++i) {
            const toml::table* t = (*a)[i].as_table();
            if (!t) continue;
            EventSpec e;
            const std::string key = "event[" + std::to_string(i) + "]";
            const int line = static_cast<int>(t->source().begin.line);

            if (auto d = (*t)["t_s"].value<double>()) e.t_s = *d;
            if (auto s = (*t)["action"].value<std::string>()) e.action = *s;
            // §7.4: "action is a validated enum. New actions require a C++
            // change — correct, since actions are behaviour."
            v.check_enum(key + ".action", e.action,
                         {"set_atmosphere", "occlude_target", "spawn_decoy",
                          "platform_gust"}, line);

            if (auto s = (*t)["mode"].value<std::string>())        e.mode = *s;
            if (auto d = (*t)["ramp_s"].value<double>())           e.ramp_s = *d;
            if (auto d = (*t)["duration_s"].value<double>())       e.duration_s = *d;
            if (auto d = (*t)["magnitude_px"].value<double>())     e.magnitude_px = *d;
            if (const toml::array* o = (*t)["offset_px"].as_array()) {
                if (o->size() == 2) {
                    for (int k = 0; k < 2; ++k) {
                        if (auto d = (*o)[static_cast<size_t>(k)].value<double>()) {
                            e.offset_px[k] = *d;
                        }
                    }
                }
            }
            sc.events.push_back(std::move(e));
        }
    }

    // --- cross-field checks --------------------------------------------------
    validate_scenario(sc, v);

    if (!v.ok()) return Err(v.format_all());
    return Ok(std::move(sc));
}

Result<Scenario> load_scenario(const std::filesystem::path& path) {
    std::error_code ec;
    if (!std::filesystem::exists(path, ec)) {
        return Err("scenario file not found: " + path.string());
    }
    std::ifstream in(path);
    if (!in) return Err("cannot open scenario file: " + path.string());

    std::ostringstream ss;
    ss << in.rdbuf();

    auto r = parse_scenario(ss.str(), path.string());
    if (r) r->source_path = path.string();
    return r;
}

}  // namespace sat
