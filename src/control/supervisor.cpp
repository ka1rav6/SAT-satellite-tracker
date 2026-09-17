// control/supervisor.cpp — design §10.6.

#include "control/supervisor.hpp"

#include <bit>
#include <cmath>

namespace sat {

void Conditions::as_features(float (&out)[kFeatureCount]) const noexcept {
    out[0]  = bg_sigma;
    out[1]  = target_contrast;
    out[2]  = integrated_snr;
    out[3]  = detection_rate;
    out[4]  = ml_confidence_mean;
    out[5]  = centroid_sigma_mean;
    out[6]  = saturation_frac;
    out[7]  = innovation_nis;
    out[8]  = track_age_s;
    out[9]  = gate_utilisation;
    out[10] = imm_mode_ca;
    out[11] = imm_mode_ct;
}

bool Strategy::operator==(const Strategy& o) const noexcept {
    return perception == o.perception
        && centroider == o.centroider
        && filter     == o.filter
        && predictor  == o.predictor
        && search     == o.search
        && cfar_k  == o.cfar_k
        && min_snr_factor == o.min_snr_factor
        && q_scale == o.q_scale
        && gains.kp == o.gains.kp && gains.ki == o.gains.ki
        && gains.kd == o.gains.kd && gains.k_ff == o.gains.k_ff;
}

// ---------------------------------------------------------------------------
// rule_table — §10.6, transcribed.
//
// The thresholds are the design's, not tuned here, and that is deliberate: the
// point of CP 12.2 is a baseline that CP 12.3's measurement and CP 12.4's
// learned policy are scored against. A baseline quietly tuned to the same runs
// it is later compared on is not a baseline.
//
// Two departures from the literal text, both recorded rather than silent:
//
//   §10.6 writes the SNR rules as two independent `if`s, so a condition with
//   integrated_snr between 8 and 15 matches NEITHER and the strategy is left
//   at kDefault. Written that way the middle band is a third regime by
//   omission. It is expressed here as an if/else-if chain over three explicit
//   bands, which is the same behaviour with the middle one named.
//
//   §10.6's table selects `MLAssisted` and `Learned` below SNR 8. Stage 11
//   does not exist, and INV-7 requires the system to run without it, so those
//   selections are made only when a model is actually loaded. The classical
//   part of the low-SNR rule — a lower CFAR threshold and a larger q — applies
//   either way, because it is the part that does not need a model.
// ---------------------------------------------------------------------------
Strategy rule_table(const Conditions& c, const Strategy& base) noexcept {
    Strategy s = base;

    // --- perception, by integrated SNR -------------------------------------
    if (c.integrated_snr < 8.0f) {
        // A faint target. Lower the CFAR threshold to stop rejecting it, and
        // widen the filter's process noise because a faint target's measured
        // position is noisier and the filter must not out-confidence it.
        s.cfar_k  = 3.2f;
        s.q_scale = 1.5f;
        // -------------------------------------------------------------------
        // A THIRD departure from §10.6's literal text, recorded like the other
        // two above.
        //
        // The design's low-SNR rule lowers the CFAR threshold "to stop
        // rejecting a faint target". Since it was written, a SECOND threshold
        // that rejects faint targets has been added — the candidate SNR gate
        // (PerceptionParams::min_snr_factor), which exists because CFAR's
        // per-pixel false alarms are smeared by the matched filter into blobs
        // of exactly beacon-like shape. Lowering cfar_k while leaving that gate
        // at its clear-air multiple is a rule that half-fires.
        //
        // Measured on scenarios/supervisor/weather_change.toml, whose beacon is
        // deliberately dim enough for fog to take it through the threshold: the
        // supervisor recovered 0.5 points of lock retention with the gate
        // fixed, against 8.9 points before the gate existed. With the gate
        // adapted too, it recovers what it is supposed to.
        //
        // WHAT THIS CANNOT DO, and it is worth being plain about it: a CFAR
        // false alarm sits just above k by construction, and a target at the
        // detection limit also sits just above k. Their SNR distributions
        // OVERLAP — measured at 4.16-4.70 for false alarms against ~4.0 for the
        // fogged beacon — so no threshold separates them. Lowering the gate
        // buys the faint target back and buys false alarms with it. What makes
        // that trade safe rather than merely different is §10.2's priority
        // policy downstream, which requires a candidate to move like a target
        // before the mount is committed to it.
        // -------------------------------------------------------------------
        s.min_snr_factor = 1.2f;
        if (c.ml_confidence_mean > 0.0f) {
            s.perception = DetectorKind::MlAssisted;
            s.centroider = CentroidKind::Learned;
        } else {
            // INV-7's path. SurfaceFit rather than WindowedCoM: Stage 9's
            // harness measured it as the better of the two below SNR 10, where
            // a window centred on a noisy argmax is the dominant error.
            s.centroider = CentroidKind::SurfaceFit;
        }
    } else if (c.integrated_snr > 15.0f) {
        // A strong target. Raise the threshold — false alarms cost more than
        // sensitivity here — and use the estimator Stage 9 measured as best in
        // this regime.
        s.perception = DetectorKind::Classical;
        s.centroider = CentroidKind::WindowedCoM;
        s.cfar_k     = 4.2f;
        s.min_snr_factor = 1.6f;   // see the low-SNR branch above
    }
    // 8 <= snr <= 15: the middle band keeps `base`. Neither rule's evidence
    // applies, and a supervisor that always does SOMETHING is a supervisor that
    // switches for its own sake.

    // --- control, by how hard the plant is working -------------------------
    if (c.saturation_frac > 0.25f) {
        // The mount is against its stops a quarter of the time, which §1.3 says
        // is the central difficulty of the problem. Feed forward harder, so
        // less of the demand has to come from observed error; integrate less,
        // because an integrator cannot spend what the actuator will not give
        // and CP 10.2 measured what it costs when it tries.
        s.gains.k_ff *= 1.3;
        s.gains.ki   *= 0.5;
    }

    // --- the filter, by its own consistency --------------------------------
    //
    // NIS is the filter grading itself: for a consistent 2-dof filter it
    // averages 2.0, and CP 6.2 measures 2.0 over 2,000 frames. Above that the
    // filter is OVER-CONFIDENT — reality is surprising it more than its
    // covariance predicts — and the fix is more process noise. Below, it is
    // under-confident and paying for it in responsiveness.
    //
    // The bounds are the 99% interval of a chi-squared with 2 degrees of
    // freedom, 0.02 and 9.21, scaled to the mean: the same distribution CP
    // 6.3's gate uses, which is not a coincidence — both ask "is this
    // innovation consistent with the covariance I claimed".
    constexpr float kNisHi = 4.0f;
    constexpr float kNisLo = 1.0f;
    if (c.innovation_nis > kNisHi) s.q_scale *= 1.4f;
    if (c.innovation_nis < kNisLo && c.innovation_nis > 0.0f) s.q_scale *= 0.8f;

    // --- the filter, by what the motion looks like -------------------------
    //
    // Not in §10.6's table, because CP 10.5 did not exist when it was written.
    // The IMM's own mode probabilities are two of the twelve features, and the
    // measurement behind this is CP 10.5's: the IMM is worth 21.75 -> 17.15 px
    // on a figure-8 and a wash on a straight line. So it is selected when the
    // manoeuvring models are carrying real weight, and not otherwise.
    if (c.imm_mode_ca + c.imm_mode_ct > 0.6f) s.filter = FilterKind::Imm;

    // PredictorKind::Smith is never selected. CP 10.4 measured it as a net loss
    // on this plant — 20 degrees of delay phase at crossover, against a margin
    // near 90 — and the supervisor has no feature that would distinguish a
    // plant where it pays. Adding a rule that fires on nothing would be a
    // claim the measurement does not support.

    return s;
}

// ---------------------------------------------------------------------------
// SatSupervisor
// ---------------------------------------------------------------------------
void SatSupervisor::reset(const SupervisorParams& p, const Strategy& base) noexcept {
    p_        = p;
    base_     = base;
    current_  = base;
    smoothed_ = Conditions{};
    primed_   = false;
    dwell_    = 0;
    changed_  = false;
    switches_ = 0;
    frames_   = 0;
    det_window_ = 0;
    for (int64_t& v : occupancy_frames_) v = 0;
    // clear(), not fill(). fill() sets the ring's size to its CAPACITY — it
    // exists for the gimbal's delay line, where "the command from 3 steps ago"
    // must be a defined zero before any command has arrived. A LOG has the
    // opposite requirement: a reader iterating history() must see the switches
    // that happened and not 64 zero-valued entries that did not. Getting this
    // wrong made the dwell-interval test read a gap of 0 frames between two
    // switches that were never logged.
    log_.clear();
}

void SatSupervisor::smooth(const Observation& o) noexcept {
    // Exponential moving average with a time constant in frames. alpha is
    // computed from the exponential rather than as 1/tau: at tau = 15 the
    // difference is small, but expressing it as exp(-1/tau) means the time
    // constant MEANS what it says at any tau, including tau below 1 where the
    // naive form goes above 1 and the filter oscillates.
    const double a = (p_.ema_tau_frames > 0.0)
                   ? 1.0 - std::exp(-1.0 / p_.ema_tau_frames)
                   : 1.0;
    const float alpha = static_cast<float>(a);

    auto blend = [&](float& acc, float sample) {
        acc = primed_ ? acc + alpha * (sample - acc) : sample;
    };

    // The detection window first: detection_rate is a property of the WINDOW,
    // not something to smooth, because it is already an average.
    det_window_ = (det_window_ << 1) | (o.detected ? 1u : 0u);
    const int n = (p_.detection_window < 1) ? 1
                : (p_.detection_window > 32 ? 32 : p_.detection_window);
    const uint32_t mask = (n >= 32) ? 0xffffffffu : ((1u << n) - 1u);
    const int seen = static_cast<int>(frames_ < n ? frames_ + 1 : n);
    smoothed_.detection_rate =
        static_cast<float>(std::popcount(det_window_ & mask)) / static_cast<float>(seen);

    // -----------------------------------------------------------------------
    // The DETECTION-CONDITIONAL features are only updated on frames that HAVE
    // a detection, and that is the whole reason this is not a loop over fields.
    //
    // On a frame with no detection the SNR is not zero, it is UNDEFINED — there
    // was nothing to measure the SNR of. Blending a zero in would drag the
    // smoothed SNR toward zero in exactly the situation where the supervisor
    // most needs it to be right: a run that is missing frames because the
    // target is faint would report an SNR lower than any frame ever measured,
    // and the low-SNR rule would fire on an artifact of its own averaging.
    //
    // This is INV-9's principle — a missing measurement is a gap, not a zero —
    // applied to the supervisor's inputs. detection_rate is where "we are
    // missing frames" is represented, and it is represented once.
    // -----------------------------------------------------------------------
    if (o.detected) {
        blend(smoothed_.integrated_snr,      o.integrated_snr);
        blend(smoothed_.bg_sigma,            o.bg_sigma);
        blend(smoothed_.target_contrast,     o.target_contrast);
        blend(smoothed_.centroid_sigma_mean, o.centroid_sigma_px);
        if (o.ml_available) blend(smoothed_.ml_confidence_mean, o.ml_confidence);
    }
    // Likewise the track-conditional ones.
    if (o.have_track) {
        blend(smoothed_.innovation_nis,   o.innovation_nis);
        blend(smoothed_.gate_utilisation, o.gate_utilisation);
        blend(smoothed_.imm_mode_ca,      o.imm_mode_ca);
        blend(smoothed_.imm_mode_ct,      o.imm_mode_ct);
        // Age is a fact about the current track, not a noisy sample of
        // anything, so it is taken as-is. Smoothing it would report a track as
        // younger than it is, which is the one direction that matters: age
        // gates trust.
        smoothed_.track_age_s = o.track_age_s;
    } else {
        smoothed_.track_age_s = 0.0f;
    }
    // Saturation is already a cumulative fraction over the whole run, so it is
    // taken directly. Smoothing an average is smoothing twice.
    smoothed_.saturation_frac = o.saturation_frac;

    primed_ = true;
}

const Strategy& SatSupervisor::update(const Observation& o,
                                      int64_t frame, double time_s) noexcept {
    changed_ = false;
    smooth(o);
    ++frames_;

    const size_t slot = static_cast<size_t>(current_.perception);
    if (slot < 3) ++occupancy_frames_[slot];

    if (!p_.enabled) return current_;

    // §10.6's property 1, second half. The dwell timer runs regardless of
    // whether a switch was WANTED, so the guarantee is "at most one switch per
    // second" rather than "at most one per second among frames we considered".
    ++dwell_;
    if (dwell_ < p_.min_dwell_frames) return current_;

    // A belief built from fewer frames than the EMA's own time constant is
    // still travelling toward its value. Switching on it is switching on the
    // transient, which is what property 1 exists to prevent.
    if (frames_ < static_cast<int64_t>(p_.ema_tau_frames)) return current_;

    const Strategy want = rule_table(smoothed_, base_);
    if (want == current_) return current_;

    // -----------------------------------------------------------------------
    // The trigger, for the log. §10.6: "Log every switch WITH ITS TRIGGER."
    //
    // Recomputed from the conditions rather than threaded out of rule_table,
    // because a reason that can disagree with the rule that fired is worse than
    // no reason at all. The order matches the table's, so the first matching
    // clause is the one that dominated.
    // -----------------------------------------------------------------------
    const char* why =
        (smoothed_.integrated_snr < 8.0f)  ? "integrated SNR below 8"
      : (smoothed_.integrated_snr > 15.0f) ? "integrated SNR above 15"
      : (smoothed_.saturation_frac > 0.25f) ? "plant saturated over 25% of ticks"
      : (smoothed_.innovation_nis > 4.0f)  ? "filter over-confident (NIS high)"
      : (smoothed_.innovation_nis < 1.0f && smoothed_.innovation_nis > 0.0f)
                                           ? "filter under-confident (NIS low)"
      : (smoothed_.imm_mode_ca + smoothed_.imm_mode_ct > 0.6f)
                                           ? "manoeuvring models dominant"
      : "conditions returned to baseline";

    current_ = want;
    dwell_   = 0;
    changed_ = true;
    ++switches_;
    log_.push(StrategySwitch{frame, time_s, why, current_});
    return current_;
}

double SatSupervisor::occupancy(DetectorKind k) const noexcept {
    const size_t i = static_cast<size_t>(k);
    if (i >= 3 || frames_ == 0) return 0.0;
    return static_cast<double>(occupancy_frames_[i]) / static_cast<double>(frames_);
}

}  // namespace sat
