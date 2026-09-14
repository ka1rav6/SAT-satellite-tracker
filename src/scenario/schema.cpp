// scenario/schema.cpp — the schema table and the validator.

#include "scenario/schema.hpp"

#include <cmath>
#include <cstdio>
#include <limits>

namespace sat {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
}

// ===========================================================================
// THE SCHEMA TABLE
//
// Every bound here comes from the specification parameter table (design §3.2)
// or from a physical impossibility. Where the specification names a range, the
// `spec_row` field carries the row number so the error message can cite it.
//
// Where the specification is silent, the bound is ours and the note says so —
// being explicit about which limits are the customer's and which are ours is
// what lets a reviewer argue with the right one.
// ===========================================================================
const std::vector<FieldSpec>& schema() {
    static const std::vector<FieldSpec> table = {
        // --- [sim] ----------------------------------------------------------
        {"sim.truth_hz",   ValueKind::Int,   false, 30,   10000, "",
         "the world advances at this rate; must divide evenly by camera_hz and control_hz (INV-3)"},
        {"sim.camera_hz",  ValueKind::Int,   false, 30,   1000,  "row 5",
         "the specification sets a floor of 30 Hz"},
        {"sim.control_hz", ValueKind::Int,   false, 20,   1000,  "row 15",
         "the specification sets a floor of 20 Hz"},
        {"sim.duration_s", ValueKind::Float, false, 0.0,  86400, "",
         "our bound: 24 hours, beyond which the int64 tick counter is the only "
         "thing still holding up and nobody is watching"},
        {"sim.seed",       ValueKind::Int,   false, 0,    kInf,  "",
         "any non-negative value; it is run through a SplitMix64 finaliser before "
         "reaching any stream (core/rng.hpp)"},

        // --- [world] — rows 1, 6 -------------------------------------------
        {"world.canvas_px", ValueKind::Array2, false, 2000, 100000, "row 1",
         "the specification sets a minimum screen size of 2000 x 2000 px"},

        // --- [camera] — rows 2-6 -------------------------------------------
        {"camera.resolution",    ValueKind::Array2, false, 16,   16384, "row 3",
         "user-defined, default 640 x 480"},
        {"camera.fov_deg",       ValueKind::Array2, false, 0.01, 180.0, "row 4",
         "user-defined, default 4 x 3 degrees"},
        {"camera.exposure_ms",   ValueKind::Float,  false, 0.0,  1000.0, "",
         "our bound: an exposure longer than a second cannot fit inside a 30 Hz "
         "frame; 0 disables motion blur entirely"},
        {"camera.blur_substeps", ValueKind::Int,    false, 1,    64,    "",
         "1 disables motion blur; design §9.2 uses 8"},

        // --- [target] — rows 7-12 ------------------------------------------
        {"target.intensity", ValueKind::Float, false, 0.0, 255.0, "",
         "grey levels above background"},
        {"target.size_px",   ValueKind::Int,   false, 5,   20,    "row 10",
         "the specification permits 5 to 20 px, default 10"},

        // --- [gimbal] — rows 13-15 -----------------------------------------
        {"gimbal.max_pan_dps",      ValueKind::Float, false, 5.0,  10.0,  "row 13",
         "the specification permits 5 to 10 deg/s, default 5"},
        {"gimbal.max_tilt_dps",     ValueKind::Float, false, 5.0,  10.0,  "row 14",
         "the specification permits 5 to 10 deg/s, default 5"},
        {"gimbal.max_accel_dps2",   ValueKind::Float, false, 0.1,  100000.0, "",
         "our bound: the specification does not constrain acceleration, but a mount "
         "that reaches its rate limit in under a truth tick is indistinguishable "
         "from one with no limit at all"},
        {"gimbal.time_constant_s",  ValueKind::Float, false, 0.0,  1.0,   "",
         "our bound: 0 models an ideal rate loop; beyond a second the mount would be "
         "slower than the scenario is long"},
        {"gimbal.latency_s",        ValueKind::Float, false, 0.0,  0.2,   "",
         "the transport delay line holds 64 ticks, which bounds this"},
        {"gimbal.encoder_lsb_urad", ValueKind::Float, false, 0.0,  10000.0, "",
         "0 disables encoder quantisation"},
        {"gimbal.resonance_hz",     ValueKind::Float, false, 0.0,  1000.0, "",
         "0 disables the structural mode"},

        // --- [noise] — rows 21-22 ------------------------------------------
        {"noise.gaussian_sigma", ValueKind::Float, false, 0.0, 20.0, "row 22",
         "the specification caps read-noise standard deviation at 20 grey levels"},
        {"noise.salt_pepper",    ValueKind::Float, false, 0.0, 1.0,  "row 21",
         "a fraction of pixels; the specification's figure is ~0.10"},
        {"noise.hot_pixels",     ValueKind::Int,   false, 0,   100000, "",
         "our bound: one third of a 640x480 frame, past which the defect map is the "
         "image rather than a defect in it"},

        // --- [disturbance] — rows 23, 25 -----------------------------------
        {"disturbance.jitter_px_per_frame", ValueKind::Float, false, 0.0, 20.0, "row 23",
         "the specification caps camera jitter at +/- 20 px per frame"},

        // --- [clutter] -------------------------------------------------------
        {"clutter.static_sources", ValueKind::Int, false, 0, 100000, "",
         "design §9.1 calls 50-500 static sources mandatory for credibility"},
        {"clutter.decoy_beacons",  ValueKind::Int, false, 0, 16, "",
         "design §9.1 asks for at least one near-identical decoy"},

        // --- [control] — design §10.4 --------------------------------------
        //
        // The bounds are not cosmetic. kp is a bandwidth in rad/s, and a
        // discrete loop running at control_hz goes unstable somewhere below
        // its Nyquist rate: at 30 Hz that is ~94 rad/s, and the mount's 20 ms
        // lag plus 10 ms transport delay eats most of the margin long before
        // that. 40 is a generous ceiling that still refuses a typo'd 400.
        {"control.kp",      ValueKind::Float, false, 0.0, 40.0, "",
         "kp is the loop bandwidth in rad/s; above ~40 the 30 Hz discrete loop "
         "and the mount's 30 ms of lag cannot stay stable"},
        {"control.ki",      ValueKind::Float, false, 0.0, 40.0, "",
         "integral gain, 1/s^2"},
        {"control.kd",      ValueKind::Float, false, 0.0, 10.0, "",
         "derivative gain, dimensionless; taken on the measurement, so large "
         "values amplify encoder quantisation rather than the setpoint"},
        // k_ff is allowed above 1 deliberately. Full feedforward is 1.0, but
        // the transport delay means a slight OVER-feed can cancel the residual
        // lag, and CP 10.1 is supposed to be able to find that empirically
        // rather than have the schema assume it away.
        {"control.k_ff",    ValueKind::Float, false, 0.0, 2.0,  "",
         "velocity feedforward gain; 0 disables it (CP 10.1's ablation), 1 is "
         "full cancellation of the tracking lag"},
        {"control.i_limit", ValueKind::Float, false, 0.0, kInf, "",
         "integrator clamp in urad*s; caps how much history the integral term "
         "can hold, independently of the anti-windup"},

        // --- [requirements] — rows 16-20 -----------------------------------
        {"requirements.acquisition_s",     ValueKind::Float, false, 0.0, kInf, "row 16",
         "the specification requires acquisition within 2 s"},
        {"requirements.tracking_error_px", ValueKind::Float, false, 0.0, kInf, "row 17",
         "the specification requires tracking error within 10 px"},
        {"requirements.target_loss_frac",  ValueKind::Float, false, 0.0, 1.0,  "row 18",
         "the specification requires target loss below 5%"},
        {"requirements.reacquisition_s",   ValueKind::Float, false, 0.0, kInf, "row 19",
         "the specification requires re-acquisition within 1 s"},
        {"requirements.min_fps",           ValueKind::Float, false, 0.0, kInf, "row 20",
         "the specification requires at least 20 FPS"},
    };
    return table;
}

const FieldSpec* find_field(std::string_view path) {
    for (const auto& f : schema()) {
        if (path == f.path) return &f;
    }
    return nullptr;
}

// ===========================================================================
// ValidationError
// ===========================================================================
std::string ValidationError::format() const {
    std::string out = file;
    if (line > 0) out += ":" + std::to_string(line);
    out += ": " + message;
    return out;
}

// ===========================================================================
// Validator
// ===========================================================================
namespace {

/// Render a double the way a person would write it in a config file: no
/// trailing zeros, no scientific notation for ordinary magnitudes. The error
/// message quotes the value back, and "14" reads better than "14.000000".
std::string pretty(double v) {
    char buf[64];
    if (v == std::floor(v) && std::fabs(v) < 1e15) {
        std::snprintf(buf, sizeof(buf), "%.0f", v);
    } else {
        std::snprintf(buf, sizeof(buf), "%g", v);
    }
    return std::string(buf);
}

std::string range_text(const FieldSpec& f) {
    const bool lo = f.min != -kInf, hi = f.max != kInf;
    if (lo && hi) return "[" + pretty(f.min) + ", " + pretty(f.max) + "]";
    if (lo)       return ">= " + pretty(f.min);
    if (hi)       return "<= " + pretty(f.max);
    return "any value";
}

}  // namespace

void Validator::check_range(std::string_view path, double value, int line) {
    const FieldSpec* f = find_field(path);
    if (!f) return;                 // not a schema-controlled key

    if (std::isnan(value)) {
        add(path, std::string(path) + " is not a number", line);
        return;
    }
    if (value >= f->min && value <= f->max) return;

    // Design §7.5's format, assembled in the order a reader needs it: what,
    // what value, why it is wrong, and on whose authority.
    std::string msg = std::string(path) + " = " + pretty(value)
                    + " is outside the permitted range " + range_text(*f);
    if (f->spec_row[0] != '\0') {
        msg += " (specification " + std::string(f->spec_row) + ")";
    }
    if (f->note[0] != '\0') {
        msg += "\n  " + std::string(f->note);
    }
    add(path, std::move(msg), line);
}

void Validator::check_enum(std::string_view path, const std::string& value,
                           std::initializer_list<const char*> allowed, int line,
                           const char* spec_row) {
    for (const char* a : allowed) {
        if (value == a) return;
    }
    std::string msg = std::string(path) + " = \"" + value + "\" is not recognised; expected one of";
    for (const char* a : allowed) {
        msg += " \"";
        msg += a;
        msg += "\"";
    }
    if (spec_row && spec_row[0] != '\0') {
        msg += " (specification " + std::string(spec_row) + ")";
    }
    add(path, std::move(msg), line);
}

void Validator::add(std::string_view path, std::string message, int line) {
    errors_.push_back(ValidationError{file_, line, std::string(path), std::move(message)});
}

std::string Validator::format_all() const {
    std::string out;
    for (size_t i = 0; i < errors_.size(); ++i) {
        if (i) out += "\n";
        out += errors_[i].format();
    }
    return out;
}

// ===========================================================================
// Cross-field validation.
//
// These are the interesting checks: a scenario can have every individual value
// in range and still describe something impossible or self-contradictory.
// ===========================================================================
void validate_scenario(const Scenario& sc, Validator& v) {
    // --- INV-3: the clock must divide evenly ------------------------------
    // Without this, camera frames fall between truth ticks and the world has to
    // be interpolated at frame time, which is both slower and irreproducible.
    if (sc.camera_hz > 0 && sc.truth_hz % sc.camera_hz != 0) {
        v.add("sim.truth_hz",
              "sim.truth_hz = " + std::to_string(sc.truth_hz) +
              " must be an exact multiple of sim.camera_hz = " +
              std::to_string(sc.camera_hz) +
              "\n  camera frames must land on truth ticks, or the world would have to be "
              "interpolated at frame time (INV-3, design §6.2)", 0);
    }
    if (sc.control_hz > 0 && sc.truth_hz % sc.control_hz != 0) {
        v.add("sim.truth_hz",
              "sim.truth_hz = " + std::to_string(sc.truth_hz) +
              " must be an exact multiple of sim.control_hz = " +
              std::to_string(sc.control_hz) + " (INV-3)", 0);
    }

    // --- row 8: at least one target ---------------------------------------
    if (sc.targets.empty()) {
        v.add("target",
              "a scenario must define at least one target "
              "(specification row 8: one target is mandatory)", 0);
    }

    // --- the camera must fit inside the screen ----------------------------
    // Not a spec row, but physically necessary: a viewport larger than the
    // canvas makes "the camera sees 7.68% of the screen" meaningless and every
    // search strategy degenerate.
    if (sc.resolution[0] > sc.canvas_px[0] || sc.resolution[1] > sc.canvas_px[1]) {
        v.add("camera.resolution",
              "camera.resolution " + std::to_string(sc.resolution[0]) + "x" +
              std::to_string(sc.resolution[1]) + " does not fit inside world.canvas_px " +
              std::to_string(sc.canvas_px[0]) + "x" + std::to_string(sc.canvas_px[1]) +
              "\n  the camera views a window of the screen (specification rows 1 and 3)", 0);
    }

    // --- row 6: the camera must start on the screen -----------------------
    if (sc.initial_pos_px[0] < 0.0 || sc.initial_pos_px[0] >= sc.canvas_px[0] ||
        sc.initial_pos_px[1] < 0.0 || sc.initial_pos_px[1] >= sc.canvas_px[1]) {
        v.add("camera.initial_pos_px",
              "camera.initial_pos_px [" + pretty(sc.initial_pos_px[0]) + ", " +
              pretty(sc.initial_pos_px[1]) + "] lies outside the screen "
              "(specification row 6: the camera starts at the centre of the screen)", 0);
    }

    // --- row 11: an explicit initial target position must be on screen ----
    for (size_t i = 0; i < sc.targets.size(); ++i) {
        const TargetSpec& t = sc.targets[i];
        const std::string key = "target[" + std::to_string(i) + "]";
        if (!t.random_initial &&
            (t.initial_px[0] < 0.0 || t.initial_px[0] >= sc.canvas_px[0] ||
             t.initial_px[1] < 0.0 || t.initial_px[1] >= sc.canvas_px[1])) {
            v.add(key + ".initial_px",
                  key + ".initial_px [" + pretty(t.initial_px[0]) + ", " +
                  pretty(t.initial_px[1]) + "] lies outside the screen "
                  "(specification row 11)", 0);
        }
        if (t.shape_type == "mask" && t.mask_file.empty()) {
            v.add(key + ".shape.mask_file",
                  key + ".shape.type = \"mask\" requires shape.mask_file "
                  "(specification row 9, design §7.3)", 0);
        }
    }

    // --- design §8.3: a video mode needs a video file ---------------------
    if (sc.input_mode != InputMode::Synthetic && sc.video_file.empty()) {
        v.add("input.video_file",
              "input.mode = \"" + std::string(input_mode_name(sc.input_mode)) +
              "\" requires input.video_file (design §8.3)", 0);
    }

    // --- INV-8: warn rather than reject on damage in a video mode ---------
    // This is a WARNING expressed as an error only when it would be silently
    // wrong. Design §8.3 requirement 5 says "assert noise and atmosphere
    // generators are disabled; warn if the config enables them". Rejecting
    // outright would be unhelpful — the evaluators' scenario files may well
    // carry noise settings copied from a synthetic scenario — so the loader
    // disables the generators (Scenario::damage_enabled) and says so, rather
    // than refusing to run.

    // --- events ------------------------------------------------------------
    for (size_t i = 0; i < sc.events.size(); ++i) {
        const std::string key = "event[" + std::to_string(i) + "]";
        if (sc.events[i].t_s < 0.0) {
            v.add(key + ".t_s", key + ".t_s = " + pretty(sc.events[i].t_s) +
                                " is before the start of the run", 0);
        }
        if (sc.events[i].t_s > sc.duration_s) {
            v.add(key + ".t_s", key + ".t_s = " + pretty(sc.events[i].t_s) +
                  " is after sim.duration_s = " + pretty(sc.duration_s) +
                  ", so the event would never fire", 0);
        }
    }
}

// ===========================================================================
// Enum names and the atmosphere table.
// ===========================================================================
const char* input_mode_name(InputMode m) noexcept {
    switch (m) {
        case InputMode::Synthetic:   return "synthetic";
        case InputMode::VideoScreen: return "video_screen";
        case InputMode::VideoDirect: return "video_direct";
    }
    return "unknown";
}

const char* atmosphere_name(Atmosphere a) noexcept {
    switch (a) {
        case Atmosphere::Clear:    return "clear";
        case Atmosphere::Haze:     return "haze";
        case Atmosphere::Rain:     return "rain";
        case Atmosphere::Fog:      return "fog";
        case Atmosphere::LowLight: return "lowlight";
    }
    return "unknown";
}

const char* edge_behaviour_name(EdgeBehaviour e) noexcept {
    switch (e) {
        case EdgeBehaviour::Bounce: return "bounce";
        case EdgeBehaviour::Wrap:   return "wrap";
        case EdgeBehaviour::Exit:   return "exit";
    }
    return "unknown";
}

// Design §9.3's table, verbatim. I <- alpha*I + beta.
AtmosphereCoeffs atmosphere_coeffs(Atmosphere a) noexcept {
    switch (a) {
        case Atmosphere::Clear:    return {1.00,   0.0};
        case Atmosphere::Haze:     return {0.75,  20.0};
        case Atmosphere::Rain:     return {0.60,  15.0};
        case Atmosphere::Fog:      return {0.35,  60.0};
        case Atmosphere::LowLight: return {0.40, -40.0};
    }
    return {1.0, 0.0};
}

// ---------------------------------------------------------------------------
// max_accel_px_s2 — see schema.hpp for why this lives here rather than in
// tracking/.
//
// Each case is the second derivative of the position law in §7.2, evaluated at
// its maximum. Anything whose acceleration is genuinely unbounded or
// impulsive (a bounce) returns 0 and says so: pretending to bound an impulse
// with a number would be worse than admitting the model does not cover it, and
// the floor applied by the caller covers the residual.
// ---------------------------------------------------------------------------
double max_accel_px_s2(const MotionSpec& m, double duration_s) noexcept {
    const double w = (m.period_s > 0.0) ? (2.0 * kPi / m.period_s) : 0.0;

    if (m.kind == "constant" || m.kind == "linear") {
        return 0.0;                                   // x'' = 0 exactly
    }
    if (m.kind == "accel") {
        return std::hypot(m.accel_px_s2[0], m.accel_px_s2[1]);
    }
    if (m.kind == "circular") {
        return w * w * std::fabs(m.radius_px);        // centripetal
    }
    if (m.kind == "sinusoid") {
        const double a = (m.axis == 0) ? m.amplitude_px[0] : m.amplitude_px[1];
        return w * w * std::fabs(a);
    }
    if (m.kind == "lissajous") {
        // Independent frequencies per axis; the figure-of-eight is freq_ratio 2.
        const double ax = w * w * std::fabs(m.amplitude_px[0]);
        const double wy = w * m.freq_ratio;
        const double ay = wy * wy * std::fabs(m.amplitude_px[1]);
        return std::hypot(ax, ay);
    }
    if (m.kind == "spiral") {
        // r(t) = r0 + growth*t, theta = w*t. In polar form the acceleration is
        // (r'' - r w^2) radially and (2 r' w) tangentially; r'' is zero for a
        // linear growth law, so the radial term is the centripetal one at the
        // largest radius the run reaches.
        const double r_max = std::fabs(m.r0_px) + std::fabs(m.growth_px_s) * duration_s;
        return std::hypot(w * w * r_max, 2.0 * std::fabs(m.growth_px_s) * w);
    }
    if (m.kind == "ou_noise") {
        // An Ornstein-Uhlenbeck velocity process: dv = -v/tau dt + sigma dW.
        // Its mean-reverting term alone gives an acceleration of order
        // sigma/tau at the stationary velocity scale. This is a scale, not a
        // hard bound — the process is Gaussian and has no maximum — which is
        // exactly the case a Kalman filter's q is designed for.
        return (m.tau_s > 0.0) ? (m.sigma_px_s / m.tau_s) : 0.0;
    }
    if (m.kind == "waypoints") {
        // Second difference of the supplied points. Waypoints are piecewise
        // linear, so the true acceleration is impulsive at the corners; the
        // difference over the two adjacent legs is the honest smoothed figure.
        double worst = 0.0;
        for (size_t i = 2; i < m.points.size(); ++i) {
            const double t0 = m.points[i - 2][0], t1 = m.points[i - 1][0], t2 = m.points[i][0];
            const double d1 = t1 - t0, d2 = t2 - t1;
            if (d1 <= 0.0 || d2 <= 0.0) continue;
            for (int ax = 1; ax <= 2; ++ax) {
                const double v1 = (m.points[i - 1][ax] - m.points[i - 2][ax]) / d1;
                const double v2 = (m.points[i][ax]     - m.points[i - 1][ax]) / d2;
                worst = std::max(worst, std::fabs(v2 - v1) / (0.5 * (d1 + d2)));
            }
        }
        return worst;
    }
    // Unknown or impulsive (bounce): not modelled, and saying so is better than
    // inventing a number.
    return 0.0;
}

double max_accel_px_s2(const TargetSpec& t, double duration_s) noexcept {
    double sum = 0.0;
    for (const MotionSpec& m : t.motion) sum += max_accel_px_s2(m, duration_s);
    return sum;
}

double max_speed_px_s(const MotionSpec& m, double duration_s) noexcept {
    const double w = (m.period_s > 0.0) ? (2.0 * kPi / m.period_s) : 0.0;

    if (m.kind == "constant") return 0.0;
    if (m.kind == "linear") {
        return std::hypot(m.velocity_px_s[0], m.velocity_px_s[1]);
    }
    if (m.kind == "accel") {
        // v(t) = v0 + a t, largest at the end of the run.
        return std::hypot(m.velocity_px_s[0] + m.accel_px_s2[0] * duration_s,
                          m.velocity_px_s[1] + m.accel_px_s2[1] * duration_s);
    }
    if (m.kind == "circular")  return w * std::fabs(m.radius_px);
    if (m.kind == "sinusoid") {
        const double a = (m.axis == 0) ? m.amplitude_px[0] : m.amplitude_px[1];
        return w * std::fabs(a);
    }
    if (m.kind == "lissajous") {
        return std::hypot(w * std::fabs(m.amplitude_px[0]),
                          w * m.freq_ratio * std::fabs(m.amplitude_px[1]));
    }
    if (m.kind == "spiral") {
        const double r_max = std::fabs(m.r0_px) + std::fabs(m.growth_px_s) * duration_s;
        return std::hypot(std::fabs(m.growth_px_s), w * r_max);
    }
    if (m.kind == "ou_noise") {
        // Stationary standard deviation of an OU velocity process driven by
        // sigma with correlation time tau is sigma * sqrt(tau/2). Three of
        // those covers it with the same 3-sigma convention used elsewhere.
        return 3.0 * m.sigma_px_s * std::sqrt(std::max(m.tau_s, 0.0) / 2.0);
    }
    if (m.kind == "waypoints") {
        double worst = 0.0;
        for (size_t i = 1; i < m.points.size(); ++i) {
            const double dt = m.points[i][0] - m.points[i - 1][0];
            if (dt <= 0.0) continue;
            worst = std::max(worst, std::hypot(m.points[i][1] - m.points[i - 1][1],
                                               m.points[i][2] - m.points[i - 1][2]) / dt);
        }
        return worst;
    }
    return 0.0;
}

double max_speed_px_s(const TargetSpec& t, double duration_s) noexcept {
    double sum = 0.0;
    for (const MotionSpec& m : t.motion) sum += max_speed_px_s(m, duration_s);
    return sum;
}

}  // namespace sat
