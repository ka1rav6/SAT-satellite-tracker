// metrics/run_report.cpp

#include "metrics/run_report.hpp"

#include <nlohmann/json.hpp>

#include <cstdio>
#include <fstream>

namespace sat {

namespace {

/// The scenario, echoed key for key. Written by hand rather than by a
/// reflection macro so that adding a §7.1 key and forgetting to echo it is a
/// visible omission in a diff rather than an invisible one.
nlohmann::ordered_json scenario_json(const Scenario& sc) {
    nlohmann::ordered_json j;
    j["name"]        = sc.name;
    j["description"] = sc.description;

    j["sim"] = {
        {"truth_hz",   sc.truth_hz},
        {"camera_hz",  sc.camera_hz},
        {"control_hz", sc.control_hz},
        {"duration_s", sc.duration_s},
        {"seed",       sc.seed},
    };
    j["world"] = {
        {"canvas_px", {sc.canvas_px[0], sc.canvas_px[1]}},
        // P3-3: the NAME, not the enumerator's integer value.
        //
        // This serialised as `0` until now. run.json is a provenance record —
        // its whole job is to let someone reconstruct what produced a number
        // months later — and "0" requires the reader to have the header open
        // at the right version to decode it. Worse, it is silently wrong the
        // moment an enumerator is inserted, because the old file's 0 now means
        // something else. The name round-trips through a rename of the enum
        // and reads correctly with no context at all.
        //
        // Same treatment as the TOML input, which has always been a name.
        {"edge_behaviour", edge_behaviour_name(sc.edge_behaviour)},
    };
    j["camera"] = {
        {"resolution",     {sc.resolution[0], sc.resolution[1]}},
        {"fov_deg",        {sc.fov_deg[0], sc.fov_deg[1]}},
        {"exposure_ms",    sc.exposure_ms},
        {"blur_substeps",  sc.blur_substeps},
        {"initial_pos_px", {sc.initial_pos_px[0], sc.initial_pos_px[1]}},
    };

    nlohmann::ordered_json targets = nlohmann::ordered_json::array();
    for (const TargetSpec& t : sc.targets) {
        nlohmann::ordered_json tj = {
            {"kind",      t.kind},
            {"intensity", t.intensity},
            {"size_px",   t.size_px},
            {"shape",     t.shape_type},
            {"random_initial", t.random_initial},
            {"initial_px", {t.initial_px[0], t.initial_px[1]}},
        };
        nlohmann::ordered_json ms = nlohmann::ordered_json::array();
        for (const MotionSpec& m : t.motion) {
            // Only the fields the component actually uses would be tidier, but
            // it would require a per-kind switch that has to be kept in step
            // with the motion factory. Echoing the kind plus the whole
            // parameter block is inert and cannot drift.
            ms.push_back({
                {"kind",          m.kind},
                {"offset_px",     {m.offset_px[0], m.offset_px[1]}},
                {"velocity_px_s", {m.velocity_px_s[0], m.velocity_px_s[1]}},
                {"accel_px_s2",   {m.accel_px_s2[0], m.accel_px_s2[1]}},
                {"amplitude_px",  {m.amplitude_px[0], m.amplitude_px[1]}},
                {"radius_px",     m.radius_px},
                {"r0_px",         m.r0_px},
                {"growth_px_s",   m.growth_px_s},
                {"period_s",      m.period_s},
                {"phase_deg",     m.phase_deg},
                {"freq_ratio",    m.freq_ratio},
                {"sigma_px_s",    m.sigma_px_s},
                {"tau_s",         m.tau_s},
                {"axis",          m.axis},
            });
        }
        tj["motion"] = ms;
        targets.push_back(tj);
    }
    j["targets"] = targets;

    j["gimbal"] = {
        {"max_pan_dps",     sc.max_pan_dps},
        {"max_tilt_dps",    sc.max_tilt_dps},
        {"max_accel_dps2",  sc.max_accel_dps2},
        {"time_constant_s", sc.time_constant_s},
        {"latency_s",       sc.latency_s},
        {"encoder_lsb_urad", sc.encoder_lsb_urad},
        {"resonance_hz",    sc.resonance_hz},
    };
    j["noise"] = {
        {"gaussian_sigma", sc.gaussian_sigma},
        {"poisson",        sc.noise_poisson},
        {"salt_pepper",    sc.salt_pepper},
        {"hot_pixels",     sc.hot_pixels},
    };
    j["atmosphere"]  = nlohmann::ordered_json{{"mode", atmosphere_name(sc.atmosphere)}};
    j["disturbance"] = nlohmann::ordered_json{{"jitter_px_per_frame", sc.jitter_px_per_frame}};
    j["clutter"]     = nlohmann::ordered_json{{"static_sources", sc.static_sources},
                                      {"decoy_beacons",  sc.decoy_beacons}};
    j["requirements"] = {
        {"acquisition_s",     sc.acquisition_s},
        {"tracking_error_px", sc.tracking_error_px},
        {"target_loss_frac",  sc.target_loss_frac},
        {"reacquisition_s",   sc.reacquisition_s},
        {"min_fps",           sc.min_fps},
    };
    return j;
}

nlohmann::ordered_json metrics_json(const RunMetrics& m) {
    return {
        // Names match docs/METRICS.md exactly, so a reader of one can look up
        // the other without a translation table.
        {"centroiding", {
            {"rmse_screen_px", m.centroid_rmse_screen_px},
            {"p95_screen_px",  m.centroid_p95_screen_px},
            {"max_screen_px",  m.centroid_max_screen_px},
            {"rmse_image_px",  m.centroid_rmse_image_px},
            {"p95_image_px",   m.centroid_p95_image_px},
            {"max_image_px",   m.centroid_max_image_px},
            {"bias_x_px",      m.centroid_bias_x_px},
            {"bias_y_px",      m.centroid_bias_y_px},
            {"frames_scored",  m.centroid_frames},
            // The third frame — see FrameRecord::centroid_error_boresight_px.
            // Screen pixels with the unobservable platform displacement D(t)
            // cancelled, so it stays bounded where rmse_screen_px grows as
            // int|D|dt. A reader comparing against a sub-pixel expectation
            // wants this one.
            {"rmse_boresight_px", m.centroid_rmse_boresight_px},
            {"p95_boresight_px",  m.centroid_p95_boresight_px},
            {"max_boresight_px",  m.centroid_max_boresight_px},
            // D(t) itself, so the screen column can be checked against it.
            {"pointing_drift_rms_px", m.pointing_drift_rms_px},
            {"pointing_drift_max_px", m.pointing_drift_max_px},
        }},
        {"tracking", {
            {"rms_px",        m.tracking_rms_px},
            {"p95_px",        m.tracking_p95_px},
            {"max_px",        m.tracking_max_px},
            // The PS's Performance Log deliverable asks for "average and
            // maximum tracking error" in so many words. max_px was already
            // here; this is the literal average.
            {"mean_px",       m.tracking_mean_px},
            {"rms_urad",      m.tracking_rms_urad},
            {"frames_scored", m.tracking_frames},
            // P1-9: the acquisition slew separated from the loop. Row 17 is
            // graded on the steady-state block.
            {"steady", {
                {"rms_px",        m.tracking_rms_steady_px},
                {"p95_px",        m.tracking_p95_steady_px},
                {"max_px",        m.tracking_max_steady_px},
                {"mean_px",       m.tracking_mean_steady_px},
                {"frames_scored", m.tracking_frames_steady},
            }},
            {"transient", {
                {"rms_px",        m.tracking_rms_transient_px},
                {"max_px",        m.tracking_max_transient_px},
                {"frames_scored", m.tracking_frames_transient},
                {"settle_frames", m.settle_frames},
            }},
        }},
        {"acquisition", {
            {"acquired",        m.acquired},
            {"cold_s",          m.acquisition_cold_s},
            {"acquired_in_fov", m.acquired_in_fov},
            {"in_fov_s",        m.acquisition_in_fov_s},
        }},
        {"reacquisition", {
            {"episodes", m.reacquisitions},
            {"mean_s",   m.reacquisition_mean_s},
            {"p95_s",    m.reacquisition_p95_s},
            {"max_s",    m.reacquisition_max_s},
        }},
        // P0-2. The PS's own objective sentence — "locate and MAINTAIN the
        // remote terminal within its camera Field-of-View" — names a quantity,
        // and this is it. Its own object rather than a field inside `lock`
        // because it is prior to lock: every retention figure is conditional
        // on the beacon having been in the FOV at all.
        {"fov_containment", {
            {"frac",                m.fov_containment_frac},
            {"post_acq_frac",       m.fov_containment_post_acq},
            {"post_acq_valid",      m.post_acq_valid},
            {"frames_total",        m.frames_total},
            {"frames_in_fov",       m.frames_in_fov},
            {"frames_post_acq",     m.frames_post_acq},
            {"frames_in_fov_post_acq", m.frames_in_fov_post_acq},
            {"frames_held_post_acq",   m.frames_held_post_acq},
        }},
        {"lock", {
            {"retention_rate",   m.lock_retention_rate},
            {"target_loss_frac", m.target_loss_frac},
            // Row 18 is graded on THIS one: it is the only loss figure a run
            // that never acquires cannot flatter. See collector.cpp.
            {"target_loss_post_acq", m.target_loss_post_acq},
            {"frames_in_fov",    m.frames_in_fov},
            {"frames_confirmed", m.frames_confirmed},
            {"frames_held_in_fov", m.frames_held_in_fov},
            {"false_tracks",     m.false_tracks},
            {"false_track_rate_per_min", m.false_track_rate_per_min},
        }},
        {"plant", nlohmann::ordered_json{{"saturation_frac", m.saturation_frac}}},
        // CP 10.7. A sweep aggregates `reached` across runs into the success
        // RATE the checkpoint asks for, which is why the per-run record is a
        // bool and a time rather than a rate of its own.
        {"handover", {
            {"reached",        m.handover_reached},
            {"time_s",         m.handover_time_s},
            {"best_rms_urad",  m.handover_rms_urad_best},
        }},
        // P2-9. The PS asks for "processing time"; this is the other half of
        // that question — how stale the command is when it reaches the mount.
        // Broken out because which term dominates is the actionable part.
        {"latency", {
            {"exposure_ms",  m.latency_exposure_ms},
            {"compute_ms",   m.latency_compute_ms},
            {"transport_ms", m.latency_transport_ms},
            {"total_ms",     m.latency_total_ms},
        }},
        // P2-8. A throughput figure with no statement of how much machine it
        // used is a throughput figure divided by an unknown.
        {"resources", {
            {"known",            m.resources_known},
            {"cpu_user_s",       m.cpu_user_s},
            {"cpu_system_s",     m.cpu_system_s},
            {"peak_rss_bytes",   m.peak_rss_bytes},
            {"hardware_threads", m.hardware_threads},
        }},
        // P1-10. `model` is here so that `misses: 0` can be read correctly:
        // with the model off it means "nothing was measured", and with it on
        // it means "the system kept up". Those are the same number and
        // completely different claims.
        {"deadline", {
            {"model",            m.deadline_model},
            {"enabled",          m.deadline_enabled},
            {"budget_ms",        m.deadline_budget_ms},
            {"budget_is_period", m.deadline_budget_is_period},
            {"misses",           m.deadline_misses},
            {"shed_frames",      m.shed_frames},
            {"worst_overrun_ms", m.worst_overrun_ms},
            {"reproducible",     m.run_reproducible},
        }},
        {"speed", {
            {"frame_ms_p50", m.frame_ms_p50},
            {"frame_ms_p95", m.frame_ms_p95},
            {"frame_ms_p99", m.frame_ms_p99},
            {"fps_from_p50", m.fps_mean},
            {"fps_from_p95", m.fps_p5},
            {"wall_time_s",  m.wall_time_s},
            {"frames_total", m.frames_total},
            {"duration_s",   m.duration_s},
        }},
    };
}

}  // namespace

std::string scenario_json_text(const Scenario& sc) {
    return scenario_json(sc).dump(2);
}

std::string run_json(const RunMetrics& m, const Scenario& sc,
                     const std::string& build_hash, uint64_t fingerprint_hash) {
    nlohmann::ordered_json j;
    j["format"] = "sat-run-v1";

    // Provenance first, and deliberately so: anyone opening this file should
    // see what produced it before they see what it claims.
    //
    // This needs ordered_json rather than json. nlohmann's default object type
    // is a std::map, which sorts keys — so "metrics" came out before
    // "provenance" and the comment above was describing an intention the code
    // did not implement. A test asserting the order is what caught it.
    j["provenance"] = {
        {"build",       build_hash},
        {"seed",        sc.seed},
        {"scenario",    sc.name},
        {"ai_enabled",  sc.ai_enabled},
        // INV-3's hash over every published snapshot. A re-run with the same
        // build and seed must reproduce this exactly; if it does not, every
        // number below is suspect and this field is how that gets noticed.
        {"frame_fingerprint", fingerprint_hash},
    };
    j["metrics"]  = metrics_json(m);
    j["scenario"] = scenario_json(sc);
    return j.dump(2);
}

bool write_run_json(const std::string& path, const RunMetrics& m, const Scenario& sc,
                    const std::string& build_hash, uint64_t fingerprint_hash) {
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out << run_json(m, sc, build_hash, fingerprint_hash) << "\n";
    return static_cast<bool>(out);
}

Result<RunMetrics> metrics_from_json(std::string_view json_text) {
    nlohmann::json j;
    try {
        j = nlohmann::json::parse(json_text);
    } catch (const std::exception& e) {
        return Err(std::string("run.json does not parse: ") + e.what());
    }
    if (!j.contains("metrics") || !j.contains("provenance")) {
        return Err("run.json is missing its metrics or provenance block");
    }

    // Every field is read with a default, so a run.json written by an older
    // build loses the fields it did not have rather than failing to load. The
    // top-level blocks above are required, because their absence means this is
    // not a run.json at all.
    const nlohmann::json& m = j["metrics"];
    auto num = [](const nlohmann::json& o, const char* k, double d = 0.0) {
        return o.contains(k) ? o[k].get<double>() : d;
    };
    auto integer = [](const nlohmann::json& o, const char* k) -> int64_t {
        return o.contains(k) ? o[k].get<int64_t>() : 0;
    };
    auto flag = [](const nlohmann::json& o, const char* k) {
        return o.contains(k) && o[k].get<bool>();
    };

    RunMetrics r;
    if (m.contains("centroiding")) {
        const auto& c = m["centroiding"];
        r.centroid_rmse_screen_px = num(c, "rmse_screen_px");
        r.centroid_p95_screen_px  = num(c, "p95_screen_px");
        r.centroid_max_screen_px  = num(c, "max_screen_px");
        r.centroid_rmse_image_px  = num(c, "rmse_image_px");
        r.centroid_p95_image_px   = num(c, "p95_image_px");
        r.centroid_max_image_px   = num(c, "max_image_px");
        r.centroid_bias_x_px      = num(c, "bias_x_px");
        r.centroid_bias_y_px      = num(c, "bias_y_px");
        r.centroid_frames         = integer(c, "frames_scored");
        r.centroid_rmse_boresight_px = num(c, "rmse_boresight_px");
        r.centroid_p95_boresight_px  = num(c, "p95_boresight_px");
        r.centroid_max_boresight_px  = num(c, "max_boresight_px");
        r.pointing_drift_rms_px      = num(c, "pointing_drift_rms_px");
        r.pointing_drift_max_px      = num(c, "pointing_drift_max_px");
    }
    if (m.contains("tracking")) {
        const auto& t = m["tracking"];
        r.tracking_rms_px   = num(t, "rms_px");
        r.tracking_p95_px   = num(t, "p95_px");
        r.tracking_max_px   = num(t, "max_px");
        r.tracking_mean_px  = num(t, "mean_px");
        r.tracking_rms_urad = num(t, "rms_urad");
        r.tracking_frames   = integer(t, "frames_scored");
        if (t.contains("steady")) {
            const auto& s = t["steady"];
            r.tracking_rms_steady_px  = num(s, "rms_px");
            r.tracking_p95_steady_px  = num(s, "p95_px");
            r.tracking_max_steady_px  = num(s, "max_px");
            r.tracking_mean_steady_px = num(s, "mean_px");
            r.tracking_frames_steady  = integer(s, "frames_scored");
        }
        if (t.contains("transient")) {
            const auto& s = t["transient"];
            r.tracking_rms_transient_px = num(s, "rms_px");
            r.tracking_max_transient_px = num(s, "max_px");
            r.tracking_frames_transient = integer(s, "frames_scored");
            r.settle_frames             = integer(s, "settle_frames");
        }
    }
    if (m.contains("acquisition")) {
        const auto& a = m["acquisition"];
        r.acquired             = flag(a, "acquired");
        r.acquisition_cold_s   = num(a, "cold_s");
        r.acquired_in_fov      = flag(a, "acquired_in_fov");
        r.acquisition_in_fov_s = num(a, "in_fov_s");
    }
    if (m.contains("reacquisition")) {
        const auto& q = m["reacquisition"];
        r.reacquisitions       = integer(q, "episodes");
        r.reacquisition_mean_s = num(q, "mean_s");
        r.reacquisition_p95_s  = num(q, "p95_s");
        r.reacquisition_max_s  = num(q, "max_s");
    }
    if (m.contains("lock")) {
        const auto& l = m["lock"];
        r.lock_retention_rate  = num(l, "retention_rate");
        r.target_loss_frac     = num(l, "target_loss_frac");
        r.target_loss_post_acq = num(l, "target_loss_post_acq");
        r.frames_in_fov       = integer(l, "frames_in_fov");
        r.frames_confirmed    = integer(l, "frames_confirmed");
        r.frames_held_in_fov  = integer(l, "frames_held_in_fov");
        r.false_tracks        = integer(l, "false_tracks");
        r.false_track_rate_per_min = num(l, "false_track_rate_per_min");
    }
    if (m.contains("fov_containment")) {
        const auto& f = m["fov_containment"];
        r.fov_containment_frac     = num(f, "frac");
        r.fov_containment_post_acq = num(f, "post_acq_frac");
        if (f.contains("post_acq_valid") && f["post_acq_valid"].is_boolean()) {
            r.post_acq_valid = f["post_acq_valid"].get<bool>();
        }
        r.frames_post_acq        = integer(f, "frames_post_acq");
        r.frames_in_fov_post_acq = integer(f, "frames_in_fov_post_acq");
        r.frames_held_post_acq   = integer(f, "frames_held_post_acq");
    }
    if (m.contains("latency")) {
        const auto& l = m["latency"];
        r.latency_exposure_ms  = num(l, "exposure_ms");
        r.latency_compute_ms   = num(l, "compute_ms");
        r.latency_transport_ms = num(l, "transport_ms");
        r.latency_total_ms     = num(l, "total_ms");
    }
    if (m.contains("resources")) {
        const auto& u = m["resources"];
        if (u.contains("known") && u["known"].is_boolean()) {
            r.resources_known = u["known"].get<bool>();
        }
        r.cpu_user_s     = num(u, "cpu_user_s");
        r.cpu_system_s   = num(u, "cpu_system_s");
        r.peak_rss_bytes = static_cast<uint64_t>(integer(u, "peak_rss_bytes"));
        r.hardware_threads = static_cast<unsigned>(integer(u, "hardware_threads"));
    }
    if (m.contains("plant")) r.saturation_frac = num(m["plant"], "saturation_frac");
    if (m.contains("handover")) {
        const auto& h = m["handover"];
        if (h.contains("reached") && h["reached"].is_boolean()) {
            r.handover_reached = h["reached"].get<bool>();
        }
        r.handover_time_s        = num(h, "time_s");
        r.handover_rms_urad_best = num(h, "best_rms_urad");
    }
    if (m.contains("speed")) {
        const auto& s = m["speed"];
        r.frame_ms_p50 = num(s, "frame_ms_p50");
        r.frame_ms_p95 = num(s, "frame_ms_p95");
        r.frame_ms_p99 = num(s, "frame_ms_p99");
        r.fps_mean     = num(s, "fps_from_p50");
        r.fps_p5       = num(s, "fps_from_p95");
        r.wall_time_s  = num(s, "wall_time_s");
        r.frames_total = integer(s, "frames_total");
        r.duration_s   = num(s, "duration_s");
    }
    const auto& p = j["provenance"];
    if (p.contains("scenario"))   r.scenario_name = p["scenario"].get<std::string>();
    if (p.contains("seed"))       r.seed = p["seed"].get<uint64_t>();
    if (p.contains("ai_enabled")) r.ai_enabled = p["ai_enabled"].get<bool>();
    return Ok(std::move(r));
}

}  // namespace sat
