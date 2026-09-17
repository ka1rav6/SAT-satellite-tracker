// control/supervisor.hpp — the SAT supervisor (design §10.6).
//
// ---------------------------------------------------------------------------
// WHAT THIS IS, AND WHY THE PROJECT IS NAMED AFTER IT
// ---------------------------------------------------------------------------
// Every stage before this one built a component with a fixed configuration: one
// CFAR threshold, one centroider, one set of gains, one filter. Each was tuned
// for a condition, and every one of them has a measured regime where a
// different choice would be better. Stage 9's ablation shows WindowedCoM
// beating SurfaceFit at high SNR and losing at low; CP 10.5 shows the IMM
// winning on a figure-8 and being a wash on a straight line; CP 10.2's ki table
// trades steady-state accuracy against overshoot.
//
// "Adaptive" in the project's name is the claim that the system picks, at
// runtime, from what it can observe. This file is that claim. §10.6's last
// line is the standard it has to meet: "Log every switch with its trigger. The
// GUI shows a strategy timeline; the report shows per-scenario strategy
// occupancy. That turns 'adaptive' from a claim into data."
//
// ---------------------------------------------------------------------------
// THREE MANDATORY PROPERTIES (§10.6), AND WHAT EACH PREVENTS
// ---------------------------------------------------------------------------
// 1. EMA SMOOTHING + HYSTERESIS. "Never react to a single frame." A single
//    frame's SNR is a noisy sample of a quantity that changes over seconds;
//    switching on it means switching several times a second. kMinDwell = 30
//    frames is one second at spec row 5's minimum rate.
//
//    This is not a nicety. The centroider, the filter and the gains all carry
//    state, and a switch discards or reinterprets some of it. A system that
//    chatters between two good strategies performs worse than one committed to
//    either.
//
// 2. BUMPLESS SWITCHING. "Carry the integrator across a gain change, or the
//    switch kicks the loop." AxisController::set_gains already does this and
//    has since Stage 1 — it rescales the integrator by the ki ratio so the
//    CONTRIBUTED TERM stays continuous, which is the part the plant sees. The
//    supervisor's job is to route gain changes through it rather than through
//    reset().
//
// 3. RULE-TABLE FALLBACK. INV-7: the system must run with no model at all.
//    §10.6 gives the table explicitly and it is transcribed here, thresholds
//    and all, so that the learned policy at CP 12.4 has something honest to be
//    measured against rather than replacing.
//
// ---------------------------------------------------------------------------
// WHAT THIS FILE DELIBERATELY DOES NOT DO
// ---------------------------------------------------------------------------
// It does not apply anything. It takes observations and returns a Strategy; the
// engine decides what to do with it. That keeps sat_control free of any
// dependency on the engine (INV-1), and it makes the supervisor testable as a
// pure function of its inputs — which matters, because "does it chatter" is a
// question about a sequence of decisions and answering it by running a
// simulation would be answering a different question.

#pragma once

#include "control/controller.hpp"
#include "core/ring.hpp"
#include "core/strategy.hpp"
#include "perception/centroid/estimators.hpp"
#include "search/pattern.hpp"

#include <cstdint>

namespace sat {

// ---------------------------------------------------------------------------
// Observation — one frame's raw evidence, before smoothing.
//
// Separate from Conditions because the two are different things and conflating
// them is how "never react to a single frame" gets lost: an Observation is a
// sample, a Conditions is a belief. Every field here is something the system
// can actually measure without truth — that is not a comment, it is the reason
// the supervisor can be part of the shipped loop at all.
// ---------------------------------------------------------------------------
struct Observation {
    bool  detected = false;       ///< was there a detection this frame
    float integrated_snr = 0.0f;  ///< the associated detection's SNR
    float bg_sigma = 0.0f;        ///< CFAR's background sigma, grey levels
    float target_contrast = 0.0f; ///< peak above background, grey levels
    float centroid_sigma_px = 0.0f;
    float ml_confidence = 0.0f;   ///< Stage 11; 0 and flagged until then
    bool  ml_available = false;

    bool  have_track = false;
    float innovation_nis = 0.0f;  ///< the filter's own consistency check
    float track_age_s = 0.0f;
    float gate_utilisation = 0.0f;///< d^2 / gate threshold, in [0, 1]
    float imm_mode_ca = 0.0f;
    float imm_mode_ct = 0.0f;

    float saturation_frac = 0.0f; ///< the plant's, cumulative
};

// ---------------------------------------------------------------------------
// Conditions — §10.6's twelve features, smoothed.
//
// The field list is §10.6's, in its order, because the learned StrategyPolicy
// at CP 12.4 consumes it as a flat vector and the order is then part of the
// model's contract. as_features() is the one place that ordering is written
// down.
// ---------------------------------------------------------------------------
struct Conditions {
    float bg_sigma            = 0.0f;
    float target_contrast     = 0.0f;
    float integrated_snr      = 0.0f;
    float detection_rate      = 0.0f;   ///< fraction of recent frames detected
    float ml_confidence_mean  = 0.0f;
    float centroid_sigma_mean = 0.0f;
    float saturation_frac     = 0.0f;
    float innovation_nis      = 0.0f;
    float track_age_s         = 0.0f;
    float gate_utilisation    = 0.0f;
    float imm_mode_ca         = 0.0f;
    float imm_mode_ct         = 0.0f;

    static constexpr int kFeatureCount = 12;

    /// The flat vector, in §10.6's declared order. Written once, here, so the
    /// model's input layout cannot drift from the struct.
    void as_features(float (&out)[kFeatureCount]) const noexcept;
};

// ---------------------------------------------------------------------------
// Strategy — what the supervisor may change.
//
// §10.6's fields exactly. Every one of them is a knob some earlier stage
// measured a regime for; the supervisor's whole content is choosing between
// those measured regimes rather than picking one at build time.
// ---------------------------------------------------------------------------
struct Strategy {
    DetectorKind   perception  = DetectorKind::Classical;
    CentroidKind   centroider  = CentroidKind::WindowedCoM;
    float          cfar_k      = 4.0f;
    /// The candidate SNR gate, as a multiple of cfar_k
    /// (PerceptionParams::min_snr_factor). Adapted alongside cfar_k and for the
    /// same reason: both are thresholds that reject a faint target, and moving
    /// one without the other only half-answers the condition.
    float          min_snr_factor = 1.5f;
    FilterKind     filter      = FilterKind::Cv;
    PredictorKind  predictor   = PredictorKind::None;
    ControlGains   gains{};
    SearchStrategy search      = SearchStrategy::Spiral;
    float          q_scale     = 1.0f;   ///< multiplies the filter's process noise

    /// Two strategies differ if anything the loop can feel differs. Used to
    /// decide whether a switch is worth logging — and, more importantly, worth
    /// spending the dwell timer on.
    [[nodiscard]] bool operator==(const Strategy& o) const noexcept;
    [[nodiscard]] bool operator!=(const Strategy& o) const noexcept { return !(*this == o); }
};

/// §10.6's rule table, transcribed. Public because CP 12.4 has to measure the
/// learned policy against it, and because it is the INV-7 fallback: with no
/// model and with --no-ai this IS the supervisor's policy.
[[nodiscard]] Strategy rule_table(const Conditions& c, const Strategy& base) noexcept;

// ---------------------------------------------------------------------------
// SupervisorParams
// ---------------------------------------------------------------------------
struct SupervisorParams {
    /// §10.6: "kMinDwell = 30 frames (1 s) between switches."
    int min_dwell_frames = 30;

    /// EMA time constant in frames. 15 is half the dwell: the belief must
    /// settle well within the time the supervisor is obliged to hold still,
    /// or every switch is made on a number still moving toward its value.
    double ema_tau_frames = 15.0;

    /// Frames of detection history behind detection_rate. One second, the same
    /// window as the dwell, so the two describe the same stretch of time.
    int detection_window = 30;

    /// Off by default. The supervisor CHANGES the configuration a run uses, so
    /// a run with it on and a run with it off are different claims and must be
    /// distinguishable — the same argument INV-7 makes for --no-ai. Every
    /// artifact records which.
    bool enabled = false;
};

/// One logged switch. §10.6: "Log every switch with its trigger."
struct StrategySwitch {
    int64_t     frame  = 0;
    double      time_s = 0.0;
    /// A literal, never an owned string: this is a fixed-capacity ring in the
    /// steady-state path and INV-4 forbids it allocating.
    const char* reason = "";
    Strategy    to{};
};

// ---------------------------------------------------------------------------
// SatSupervisor
// ---------------------------------------------------------------------------
class SatSupervisor {
public:
    /// `base` is the scenario's own configuration — the strategy the system
    /// would run with no supervisor at all. Every rule is expressed as a
    /// MODIFICATION of it rather than as an absolute, so that turning the
    /// supervisor on cannot silently discard a deliberate scenario setting.
    void reset(const SupervisorParams& p, const Strategy& base) noexcept;

    /// One frame. Returns the strategy in force, which is unchanged unless a
    /// switch was both warranted and permitted.
    const Strategy& update(const Observation& o, int64_t frame, double time_s) noexcept;

    [[nodiscard]] const Strategy&   current()    const noexcept { return current_; }
    [[nodiscard]] const Conditions& conditions() const noexcept { return smoothed_; }
    [[nodiscard]] bool  changed_this_frame() const noexcept { return changed_; }
    [[nodiscard]] int   dwell_frames()       const noexcept { return dwell_; }
    [[nodiscard]] int64_t switch_count()     const noexcept { return switches_; }

    /// The last 64 switches, newest first. Fixed capacity: a run that switches
    /// more than 64 times has a chattering problem, and the fix for that is not
    /// a bigger log.
    [[nodiscard]] const Ring<StrategySwitch, 64>& history() const noexcept {
        return log_;
    }

    /// Fraction of frames spent in each detector kind — §10.6's "per-scenario
    /// strategy occupancy". Indexed by DetectorKind.
    [[nodiscard]] double occupancy(DetectorKind k) const noexcept;

private:
    /// Fold one observation into the smoothed belief.
    void smooth(const Observation& o) noexcept;

    SupervisorParams p_{};
    Strategy         base_{};
    Strategy         current_{};
    Conditions       smoothed_{};

    bool     primed_  = false;   ///< first observation seeds rather than blends
    int      dwell_   = 0;
    bool     changed_ = false;
    int64_t  switches_ = 0;
    int64_t  frames_   = 0;

    /// Detection history as a bitmask, the same trick Track uses for its M-of-N
    /// window: N is small and fixed, and a popcount is one instruction where a
    /// ring of counters is a loop that is easy to get off by one.
    uint32_t det_window_ = 0;

    int64_t  occupancy_frames_[3] = {0, 0, 0};

    Ring<StrategySwitch, 64> log_{};
};

}  // namespace sat
