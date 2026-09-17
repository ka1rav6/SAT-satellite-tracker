// tests/supervisor/test_supervisor.cpp — Stage 12 (design §10.6).
//
// CP 12.1  Conditions extraction + EMA smoothing.
// CP 12.2  Rule table with hysteresis and bumpless switching.
//
// ---------------------------------------------------------------------------
// MOST OF THIS IS TESTED WITHOUT A SIMULATION, ON PURPOSE
// ---------------------------------------------------------------------------
// §10.6's first mandatory property is "never react to a single frame", and the
// question it raises — does this chatter — is a question about a SEQUENCE OF
// DECISIONS. Answering it by running the engine would answer a different and
// much weaker question: whether it chattered on the particular conditions one
// scenario happened to produce.
//
// SatSupervisor is a pure function of its observations, which is why it is. So
// the adversarial sequences are fed in directly: a condition oscillating across
// a threshold every frame, a run of frames with no detection, a feature that
// jumps. Only the end-to-end effect needs the engine, and only one case uses it.

#include <doctest/doctest.h>

#include "control/supervisor.hpp"
#include "engine/pipeline.hpp"

#include <cmath>
#include <string>
#include <vector>

using namespace sat;

namespace {

SupervisorParams params(int dwell = 30, double tau = 15.0) {
    SupervisorParams p;
    p.enabled = true;
    p.min_dwell_frames = dwell;
    p.ema_tau_frames   = tau;
    return p;
}

/// An observation with a detection at a given SNR, and a healthy track.
Observation obs_at(float snr) {
    Observation o;
    o.detected        = true;
    o.integrated_snr  = snr;
    o.target_contrast = snr * 20.0f;
    o.bg_sigma        = 20.0f;
    o.centroid_sigma_px = 0.3f;
    o.have_track      = true;
    o.innovation_nis  = 2.0f;    // a consistent 2-dof filter
    o.track_age_s     = 5.0f;
    return o;
}

}  // namespace

// ===========================================================================
// CP 12.1 — Conditions and smoothing
// ===========================================================================
TEST_CASE("CP 12.1: a missing detection is a gap in the SNR, not a zero") {
    // The failure this guards is subtle and would make the supervisor
    // self-defeating.
    //
    // On a frame with no detection the SNR is not zero, it is UNDEFINED —
    // there was nothing to measure the SNR of. Blending a zero in drags the
    // smoothed SNR toward zero in exactly the situation where the supervisor
    // most needs it to be right: a run that is missing frames BECAUSE the
    // target is faint would then report an SNR lower than any frame ever
    // measured, and the low-SNR rule would be firing on an artifact of its own
    // averaging rather than on evidence.
    //
    // This is INV-9's principle applied to the supervisor's inputs. Whether
    // frames are being missed is represented by detection_rate, once.
    SatSupervisor s;
    s.reset(params(), Strategy{});

    for (int i = 0; i < 60; ++i) s.update(obs_at(30.0f), i, i / 30.0);
    const float settled = s.conditions().integrated_snr;
    CHECK(settled == doctest::Approx(30.0).epsilon(0.05));

    // Thirty frames with nothing detected.
    Observation blank;
    blank.have_track = false;
    for (int i = 60; i < 90; ++i) s.update(blank, i, i / 30.0);

    MESSAGE("SNR after 30 blank frames: " << s.conditions().integrated_snr
            << " (was " << settled << "); detection rate "
            << s.conditions().detection_rate);

    // The SNR is untouched...
    CHECK(s.conditions().integrated_snr == doctest::Approx(settled));
    // ...and the fact that frames are being missed is represented, once.
    CHECK(s.conditions().detection_rate == doctest::Approx(0.0));
}

TEST_CASE("CP 12.1: the EMA time constant means what it says") {
    // exp(-1/tau) rather than 1/tau. At tau = 15 the two agree closely, and at
    // tau below 1 the naive form gives alpha above 1 and the filter oscillates
    // — a smoother that amplifies is worse than no smoother, and it would only
    // show up when someone configured a short tau.
    SatSupervisor s;
    s.reset(params(30, 10.0), Strategy{});

    s.update(obs_at(0.0f), 0, 0.0);              // primes at 0
    for (int i = 1; i <= 10; ++i) s.update(obs_at(100.0f), i, i / 30.0);

    // One time constant of a step: 1 - 1/e = 63.2%.
    MESSAGE("after one time constant: " << s.conditions().integrated_snr
            << " of 100 (expected 63.2)");
    CHECK(s.conditions().integrated_snr == doctest::Approx(63.2).epsilon(0.05));

    // And a very short tau must still converge rather than ring.
    SatSupervisor fast;
    fast.reset(params(30, 0.5), Strategy{});
    fast.update(obs_at(0.0f), 0, 0.0);
    for (int i = 1; i <= 20; ++i) fast.update(obs_at(50.0f), i, i / 30.0);
    CHECK(fast.conditions().integrated_snr == doctest::Approx(50.0).epsilon(0.01));
}

TEST_CASE("CP 12.1: the feature vector's order is the struct's order") {
    // CP 12.4's learned policy consumes Conditions as a flat vector, so the
    // ordering becomes part of a trained model's contract. If the struct and
    // as_features ever disagree, every inference is silently wrong in a way no
    // other test would notice.
    Conditions c;
    c.bg_sigma = 1; c.target_contrast = 2; c.integrated_snr = 3;
    c.detection_rate = 4; c.ml_confidence_mean = 5; c.centroid_sigma_mean = 6;
    c.saturation_frac = 7; c.innovation_nis = 8; c.track_age_s = 9;
    c.gate_utilisation = 10; c.imm_mode_ca = 11; c.imm_mode_ct = 12;

    float f[Conditions::kFeatureCount];
    c.as_features(f);
    for (int i = 0; i < Conditions::kFeatureCount; ++i) {
        CHECK(f[i] == doctest::Approx(static_cast<float>(i + 1)));
    }
}

// ===========================================================================
// CP 12.2 — hysteresis, dwell, bumpless switching
// ===========================================================================
TEST_CASE("CP 12.2: a condition oscillating every frame produces no chatter") {
    // The adversarial case §10.6's property 1 exists for. The SNR flips across
    // the high threshold on every single frame — the worst input a
    // threshold-based policy can be given.
    //
    // Without smoothing this switches 30 times a second. Without the dwell
    // timer it switches whenever the smoothed value happens to cross. With
    // both, §10.6's guarantee is "at most one switch per second", and 300
    // frames is ten seconds.
    SatSupervisor s;
    s.reset(params(30, 15.0), Strategy{});

    for (int i = 0; i < 300; ++i) {
        s.update(obs_at(i % 2 ? 40.0f : 5.0f), i, i / 30.0);
    }
    MESSAGE("switches over 10 s of a signal alternating every frame: "
            << s.switch_count());
    CHECK(s.switch_count() <= 10);
}

TEST_CASE("CP 12.2: switches are at least kMinDwell apart") {
    // The guarantee stated directly, over a signal designed to want a switch
    // constantly: a slow sweep back and forth across both thresholds.
    SatSupervisor s;
    s.reset(params(30, 5.0), Strategy{});

    for (int i = 0; i < 900; ++i) {
        const float snr = 20.0f + 18.0f * static_cast<float>(std::sin(i * 0.05));
        s.update(obs_at(snr), i, i / 30.0);
    }

    std::vector<int64_t> at;
    for (size_t k = 0; k < s.history().size(); ++k) {
        at.push_back(s.history().at_back(k).frame);
    }
    // The ring must report only what was pushed. It reported 64 entries — its
    // full capacity — because reset() used fill() rather than clear(), and
    // fill() sets the size to N so that the gimbal's delay line can read "the
    // command from 3 steps ago" before any command exists. A log needs the
    // opposite. Asserted here because a log that silently reports phantom
    // entries would make every occupancy and interval figure wrong.
    CHECK(at.size() == static_cast<size_t>(s.switch_count()));
    REQUIRE(at.size() >= 2);
    // Newest first, so consecutive entries differ by at least the dwell.
    int64_t worst = 1 << 30;
    for (size_t k = 1; k < at.size(); ++k) worst = std::min(worst, at[k - 1] - at[k]);
    MESSAGE(at.size() << " switches over 30 s; closest pair " << worst << " frames apart");
    CHECK(worst >= 30);
}

TEST_CASE("CP 12.2: every switch carries a trigger") {
    // §10.6: "Log every switch with its trigger." A log of switches with no
    // reasons turns "adaptive" back into a claim.
    SatSupervisor s;
    s.reset(params(30, 5.0), Strategy{});
    for (int i = 0; i < 120; ++i) s.update(obs_at(40.0f), i, i / 30.0);
    for (int i = 120; i < 300; ++i) s.update(obs_at(4.0f), i, i / 30.0);

    REQUIRE(s.switch_count() > 0);
    for (size_t k = 0; k < s.history().size(); ++k) {
        const StrategySwitch& sw = s.history().at_back(k);
        CHECK(std::string(sw.reason).size() > 0);
    }
    MESSAGE("most recent trigger: " << std::string(s.history().newest().reason));
}

TEST_CASE("CP 12.2: switching gains is bumpless") {
    // §10.6's property 2: "Carry the integrator across a gain change, or the
    // switch kicks the loop."
    //
    // The subtlety is that carrying the RAW integrator is not enough. What the
    // plant sees is the integral TERM, ki * integ, so halving ki while keeping
    // integ halves the contributed rate — a step in the command, which is
    // exactly the kick the property forbids. AxisController::set_gains rescales
    // by the ki ratio so the TERM is continuous.
    //
    // §10.6's saturation rule halves ki, so this is the exact change the
    // supervisor makes.
    ControlGains g;
    g.kp = 8.0; g.ki = 2.0; g.kd = 0.0; g.k_ff = 0.0;

    AxisController c;
    c.reset(g);
    // Wind the integrator up on a constant error.
    for (int i = 0; i < 100; ++i) {
        (void)c.compute(1000.0, 0.0, 0.0, 0.0, 1.0 / 30.0, false, true);
    }
    const double before = c.compute(1000.0, 0.0, 0.0, 0.0, 1.0 / 30.0, false, true);
    const double term_before = c.gains().ki * c.integrator();

    ControlGains half = g;
    half.ki *= 0.5;
    c.set_gains(half);
    const double after = c.compute(1000.0, 0.0, 0.0, 0.0, 1.0 / 30.0, false, true);
    const double term_after = c.gains().ki * c.integrator();

    MESSAGE("integral term across a 2x ki change: " << term_before << " -> "
            << term_after << ";  output " << before << " -> " << after);

    // The contributed term is continuous to within one step of integration.
    CHECK(term_after == doctest::Approx(term_before).epsilon(0.02));
    // And so is the output. Without the rescale this would drop by half the
    // integral term, which here is thousands of urad/s.
    CHECK(after == doctest::Approx(before).epsilon(0.02));

    // The counterfactual, so the number above means something: reset() zeroes
    // the integrator and the output jumps.
    AxisController naive;
    naive.reset(g);
    for (int i = 0; i < 101; ++i) {
        (void)naive.compute(1000.0, 0.0, 0.0, 0.0, 1.0 / 30.0, false, true);
    }
    naive.reset(half);
    const double kicked = naive.compute(1000.0, 0.0, 0.0, 0.0, 1.0 / 30.0, false, true);
    CHECK(std::fabs(kicked - before) > 0.1 * std::fabs(before));
}

TEST_CASE("CP 12.2: a rule only fires on the evidence it names") {
    // The middle band. §10.6 writes the two SNR rules as independent `if`s, so
    // a condition between 8 and 15 matches neither and the strategy is left at
    // the base. That is a third regime by omission, and it is the right
    // behaviour — a supervisor that always does SOMETHING is one that switches
    // for its own sake — so it is asserted rather than left to be noticed.
    Strategy base;
    base.cfar_k = 3.9f;

    Conditions mid;
    mid.integrated_snr = 11.0f;
    mid.innovation_nis = 2.0f;
    const Strategy s_mid = rule_table(mid, base);
    CHECK(s_mid.cfar_k == doctest::Approx(base.cfar_k));
    CHECK(s_mid.centroider == base.centroider);

    Conditions lo = mid; lo.integrated_snr = 5.0f;
    CHECK(rule_table(lo, base).cfar_k == doctest::Approx(3.2f));

    Conditions hi = mid; hi.integrated_snr = 25.0f;
    CHECK(rule_table(hi, base).cfar_k == doctest::Approx(4.2f));
}

TEST_CASE("CP 12.2: with no model the low-SNR rule stays classical (INV-7)") {
    // §10.6's table selects MLAssisted and Learned below SNR 8. Stage 11 does
    // not exist and INV-7 requires the system to run without it, so those are
    // selected only when a model actually reported a confidence. The classical
    // half of the rule — a lower threshold, a wider q — applies either way,
    // because it is the half that needs no model.
    Strategy base;
    Conditions c;
    c.integrated_snr = 4.0f;
    c.innovation_nis = 2.0f;

    const Strategy no_ml = rule_table(c, base);
    CHECK(no_ml.perception == DetectorKind::Classical);
    CHECK(no_ml.centroider == CentroidKind::SurfaceFit);
    CHECK(no_ml.cfar_k == doctest::Approx(3.2f));
    CHECK(no_ml.q_scale > 1.0f);

    c.ml_confidence_mean = 0.8f;
    const Strategy with_ml = rule_table(c, base);
    CHECK(with_ml.perception == DetectorKind::MlAssisted);
    CHECK(with_ml.centroider == CentroidKind::Learned);
}

// ===========================================================================
// The end-to-end effect — the only case that needs the engine.
// ===========================================================================
TEST_CASE("Stage 12: the supervisor recovers lock that the fixed configuration loses") {
    // scenarios/supervisor/weather_change.toml in code: fog arrives at 10 s and
    // clears at 25 s, on a beacon dim enough that the fog takes the integrated
    // SNR through §10.6's low-SNR threshold.
    //
    // At spec row 7's nominal brightness this measures nothing — the fogged SNR
    // lands in the middle band where the rule table deliberately does nothing,
    // and the loop tracks at 0.1 px either way. The scenario file carries that
    // note at length.
    auto make = [](bool supervised) {
        Scenario sc;
        sc.duration_s = 40.0;
        sc.seed       = 42;
        sc.static_sources = 0;
        sc.decoy_beacons  = 0;
        sc.jitter_px_per_frame = 0.0;
        sc.supervisor_enabled  = supervised;

        TargetSpec t;
        t.size_px        = 10;
        t.intensity      = 40.0;
        t.random_initial = false;
        t.initial_px[0]  = 1040.0;
        t.initial_px[1]  = 1020.0;
        MotionSpec m;
        m.kind = "linear";
        m.velocity_px_s[0] =  22.0;
        m.velocity_px_s[1] = -11.0;
        t.motion.push_back(m);
        sc.targets.push_back(t);

        EventSpec fog;   fog.t_s = 10.0; fog.action = "set_atmosphere"; fog.mode = "fog";
        EventSpec clear; clear.t_s = 25.0; clear.action = "set_atmosphere"; clear.mode = "clear";
        sc.events.push_back(fog);
        sc.events.push_back(clear);
        return sc;
    };

    auto retention = [](const Scenario& sc) {
        Pipeline p;
        p.build_from_scenario(sc);
        int64_t in_fov = 0, held = 0;
        while (p.step()) {
            const FrameRecord& r = p.last();
            if (!r.truth_valid || !r.truth_in_fov) continue;
            ++in_fov;
            if (r.track_state == TrackState::Confirmed) ++held;
        }
        // -------------------------------------------------------------------
        // A SAMPLE-SIZE GUARD, NOT AN ASSERTION ABOUT THE RUN.
        //
        // This was `in_fov > 900` out of 1200 frames, and it passed — because
        // the detector used to return several candidates per frame of pure
        // noise, the tracker locked onto one of them within a frame, and the
        // camera therefore stayed roughly where the beacon was instead of
        // searching. With the SNR gate in place those candidates are gone, the
        // FSM correctly goes back to Search when the fog takes the beacon below
        // the threshold, and the camera sweeps away from it. in_fov is now 547
        // for the fixed configuration and 629 for the supervised one.
        //
        // That is the scenario working as designed — it exists to put a dim
        // beacon through a threshold — so the guard is now what it was always
        // meant to be: enough frames for the retention ratio to mean something.
        // -------------------------------------------------------------------
        REQUIRE(in_fov > 300);
        return static_cast<double>(held) / static_cast<double>(in_fov);
    };

    const double fixed      = retention(make(false));
    const double supervised = retention(make(true));

    MESSAGE("lock retention through fog: fixed configuration "
            << (100.0 * fixed) << " %, supervised " << (100.0 * supervised) << " %");

    // Measured 77.4% -> 86.3% when this was written, and 55.2% -> 62.3% now.
    // Both arms fell when the SNR gate landed, for the reason in the guard
    // above; what the case measures is the GAP, and the gap is 7.1 points
    // against the 8.9 it was.
    //
    // Recovering it needed the supervisor's low-SNR rule to move the candidate
    // gate as well as the CFAR threshold — see the third departure recorded in
    // control/supervisor.cpp's rule_table. With only cfar_k adapted the
    // supervisor recovered 0.5 points, because it was lowering one of the two
    // thresholds that were rejecting the target.
    //
    // The bound is set well below the measurement so a modest regression still
    // passes while a REVERSAL — the supervisor making things worse, which is
    // the failure mode that matters — turns it red.
    CHECK(supervised > fixed + 0.03);
}

TEST_CASE("Stage 12: design 7.4's events actually happen") {
    // They did not, until this stage. The loader parsed them, the schema
    // validated them, run.json echoed them back, and no code read them: a
    // scenario could ask for fog at t = 10 s, be told the request was valid,
    // and run in clear air for the whole run.
    //
    // That is the worst shape a configuration defect can take — validation
    // says yes, the artifact says the setting was applied, the behaviour is
    // unchanged — so the check is that the WORLD changed, not that the event
    // fired.
    Scenario sc;
    sc.duration_s = 20.0;
    sc.seed       = 7;
    sc.static_sources = 0;
    sc.decoy_beacons  = 0;
    sc.jitter_px_per_frame = 0.0;

    TargetSpec t;
    t.size_px        = 10;
    t.intensity      = 120.0;
    t.random_initial = false;
    t.initial_px[0]  = 1020.0;
    t.initial_px[1]  = 1010.0;
    sc.targets.push_back(t);

    EventSpec fog; fog.t_s = 10.0; fog.action = "set_atmosphere"; fog.mode = "fog";
    sc.events.push_back(fog);

    Pipeline p;
    p.build_from_scenario(sc);
    double snr_before = 0.0, snr_after = 0.0;
    int    n_before = 0, n_after = 0;
    while (p.step()) {
        const FrameRecord& r = p.last();
        if (!r.detected) continue;
        if (r.time_s > 4.0 && r.time_s < 9.0)  { snr_before += r.detection_snr; ++n_before; }
        if (r.time_s > 14.0)                   { snr_after  += r.detection_snr; ++n_after;  }
    }
    REQUIRE(n_before > 50);
    REQUIRE(n_after  > 50);
    snr_before /= n_before;
    snr_after  /= n_after;

    MESSAGE("detection SNR before the fog event: " << snr_before
            << ", after: " << snr_after);
    CHECK(snr_after < 0.6 * snr_before);
}

TEST_CASE("Stage 12: occlude_target hides the beacon and GIVES IT BACK") {
    // The second half is the one that had a defect, and it was not in this
    // event's own logic.
    //
    // Restoring the configured intensity every frame — "not occluded, so put
    // it back" — makes the timeline the owner of that field for the whole run.
    // tests/tracking/test_reacquire.cpp blanks the beacon by zeroing exactly
    // that field, and CP 6.7's dropout cases stopped seeing a dropout: the
    // timeline handed the beacon back every frame, in scenarios that had no
    // occlusion events at all. The write is now gated on the timeline actually
    // containing one. A component must not write what it does not own.
    Scenario sc;
    sc.duration_s = 12.0;
    sc.seed       = 11;
    sc.static_sources = 0;
    sc.decoy_beacons  = 0;
    sc.jitter_px_per_frame = 0.0;

    TargetSpec t;
    t.size_px        = 10;
    t.intensity      = 120.0;
    t.random_initial = false;
    t.initial_px[0]  = 1020.0;
    t.initial_px[1]  = 1010.0;
    sc.targets.push_back(t);

    EventSpec hide;
    hide.t_s        = 5.0;
    hide.action     = "occlude_target";
    hide.duration_s = 1.0;
    sc.events.push_back(hide);

    Pipeline p;
    p.build_from_scenario(sc);

    // Counted as detections ON THE BEACON, not detections of anything.
    //
    // The first version of this asserted zero detections during the occlusion
    // and measured 19 of 24 frames. They were not the beacon: CFAR at k = 3.9
    // over 307,200 pixels has a false-alarm probability around 5e-5, so a
    // handful of noise pixels survive the threshold every frame and some
    // survive the shape gate too. The detector reporting a candidate when
    // there is nothing there is a FALSE ALARM — §13.1 counts those separately,
    // for exactly this reason — and asserting it never happens would be
    // asserting the detector is perfect rather than that the event works.
    //
    // A detection within 3 px of the truth is the beacon; anything else is
    // noise. That is the quantity the event is supposed to change.
    int before = 0, during = 0, after = 0;
    while (p.step()) {
        const FrameRecord& r = p.last();
        const bool on_beacon = r.detected && r.centroid_error_valid
                            && r.centroid_error_px < 3.0;
        if (r.time_s > 2.0 && r.time_s < 4.9) before += on_beacon ? 1 : 0;
        if (r.time_s > 5.1 && r.time_s < 5.9) during += on_beacon ? 1 : 0;
        if (r.time_s > 7.0)                   after  += on_beacon ? 1 : 0;
    }
    MESSAGE("detections ON THE BEACON before / during / after a 1 s occlusion: "
            << before << " / " << during << " / " << after);

    CHECK(before > 60);
    CHECK(during == 0);     // genuinely gone
    CHECK(after  > 100);    // and genuinely back
}

TEST_CASE("Stage 12: a gust is a smooth excursion, not a step") {
    // platform_gust is an INTERVAL, and the shape inside it matters. A step
    // would be unphysical — a platform on a mast does not teleport — and it
    // would also lie to the blur model, which differentiates the boresight: a
    // rate discontinuity smears one frame by an amount the specification never
    // asked for. A raised cosine starts and ends at zero WITH ZERO SLOPE.
    Scenario sc;
    EventSpec g;
    g.t_s = 4.0; g.action = "platform_gust"; g.duration_s = 2.0; g.magnitude_px = 40.0;
    sc.events.push_back(g);

    EventTimeline tl;
    tl.build(sc);

    auto mag = [&](double t) {
        double dx = 0.0, dy = 0.0;
        tl.gust_offset(t, dx, dy);
        return std::sqrt(dx * dx + dy * dy);
    };

    CHECK(mag(3.9) == doctest::Approx(0.0));         // before
    CHECK(mag(4.0) == doctest::Approx(0.0));         // starts at zero
    CHECK(mag(5.0) == doctest::Approx(40.0));        // peaks at the midpoint
    CHECK(mag(6.1) == doctest::Approx(0.0));         // and is over

    // Zero SLOPE at both ends, which is the part a step gets wrong. Sampled
    // just inside the interval: a linear ramp of the same amplitude would give
    // 40/1.0 = 40 px/s here, the raised cosine gives almost nothing.
    const double slope_in = (mag(4.02) - mag(4.0)) / 0.02;
    MESSAGE("gust slope 20 ms after onset: " << slope_in
            << " px/s (a linear ramp would give 40)");
    CHECK(slope_in < 5.0);
}

TEST_CASE("Stage 12: two events at the same timestamp keep file order") {
    // §7.4 permits it, and which fires first is then decided by the order the
    // file lists them. std::sort is free to swap equal elements; std::stable_
    // sort is not. Without this a scenario stops being reproducible from its
    // own text, which is INV-3 arriving somewhere unexpected.
    Scenario sc;
    EventSpec a; a.t_s = 5.0; a.action = "set_atmosphere"; a.mode = "fog";
    EventSpec b; b.t_s = 5.0; b.action = "spawn_decoy";    b.offset_px[0] = 60.0;
    EventSpec c; c.t_s = 1.0; c.action = "platform_gust";  c.magnitude_px = 10.0;
    sc.events.push_back(a);
    sc.events.push_back(b);
    sc.events.push_back(c);

    EventTimeline tl;
    tl.build(sc);
    REQUIRE(tl.count() == 3);
    // Sorted by time...
    CHECK(tl.events()[0].action == EventAction::PlatformGust);
    // ...and ties broken by the order the file listed them.
    CHECK(tl.events()[1].action == EventAction::SetAtmosphere);
    CHECK(tl.events()[2].action == EventAction::SpawnDecoy);

    // Edge-triggered: each becomes due exactly once.
    CHECK(tl.due(0.5).size() == 0);
    CHECK(tl.due(1.0).size() == 1);
    CHECK(tl.due(1.0).size() == 0);
    CHECK(tl.due(9.0).size() == 2);
    CHECK(tl.due(9.0).size() == 0);
}
