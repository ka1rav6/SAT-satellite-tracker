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
nlohmann::json scenario_json(const Scenario& sc) {
    nlohmann::json j;
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
        {"edge_behaviour", sc.edge_behaviour},
    };
    j["camera"] = {
        {"resolution",     {sc.resolution[0], sc.resolution[1]}},
        {"fov_deg",        {sc.fov_deg[0], sc.fov_deg[1]}},
        {"exposure_ms",    sc.exposure_ms},
        {"blur_substeps",  sc.blur_substeps},
        {"initial_pos_px", {sc.initial_pos_px[0], sc.initial_pos_px[1]}},
    };

    nlohmann::json targets = nlohmann::json::array();
    for (const TargetSpec& t : sc.targets) {
        nlohmann::json tj = {
            {"kind",      t.kind},
            {"intensity", t.intensity},
            {"size_px",   t.size_px},
            {"shape",     t.shape_type},
            {"random_initial", t.random_initial},
            {"initial_px", {t.initial_px[0], t.initial_px[1]}},
        };
        nlohmann::json ms = nlohmann::json::array();
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
    j["atmosphere"]  = nlohmann::json{{"mode", atmosphere_name(sc.atmosphere)}};
    j["disturbance"] = nlohmann::json{{"jitter_px_per_frame", sc.jitter_px_per_frame}};
    j["clutter"]     = nlohmann::json{{"static_sources", sc.static_sources},
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

nlohmann::json metrics_json(const RunMetrics& m) {
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
        }},
        {"tracking", {
            {"rms_px",        m.tracking_rms_px},
            {"p95_px",        m.tracking_p95_px},
            {"max_px",        m.tracking_max_px},
            {"rms_urad",      m.tracking_rms_urad},
            {"frames_scored", m.tracking_frames},
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
        {"lock", {
            {"retention_rate",   m.lock_retention_rate},
            {"target_loss_frac", m.target_loss_frac},
            {"frames_in_fov",    m.frames_in_fov},
            {"frames_confirmed", m.frames_confirmed},
            {"false_tracks",     m.false_tracks},
            {"false_track_rate_per_min", m.false_track_rate_per_min},
        }},
        {"plant", nlohmann::json{{"saturation_frac", m.saturation_frac}}},
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

std::string run_json(const RunMetrics& m, const Scenario& sc,
                     const std::string& build_hash, uint64_t fingerprint_hash) {
    nlohmann::json j;
    j["format"] = "sat-run-v1";

    // Provenance first, and deliberately so: anyone opening this file should
    // see what produced it before they see what it claims.
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

}  // namespace sat
