// tests/tracking/test_track.cpp — CP 6.3, 6.4 and 6.5.
//
// CP 6.3: "Mahalanobis gate (9.21) + nearest neighbour. Accept when: with a
//          decoy 60 px away, the tracker stays on the real beacon."
// CP 6.4: "Lifecycle FSM with M-of-N and coasting. Accept when: blanking
//          detection for 8 frames does NOT delete the track; the camera keeps
//          moving sensibly on prediction."
// CP 6.5: "Adaptive R from detection SNR. Accept when: in fog the filter
//          visibly relies more on prediction; error lower than with fixed R."
//
// The lifecycle is tested exhaustively against §10.2's diagram rather than on a
// happy path, because every arrow in that diagram is a failure mode someone
// will hit: a track that confirms too easily chases noise, one that confirms
// too slowly misses spec row 16's 2 s acquisition budget, one that deletes too
// eagerly loses lock on a blink, and one that never deletes follows a ghost
// forever.

#include <doctest/doctest.h>

#include "core/rng.hpp"
#include "tracking/track.hpp"

#include <cmath>
#include <vector>

using namespace sat;

namespace {

constexpr double kDt   = 1.0 / 30.0;
constexpr double kIfov = 109.08;          // urad/px, the default camera

Measurement meas(double x, double y, double sigma = kIfov, float snr = 20.0f) {
    Measurement m;
    m.angle      = Angle2{x, y};
    m.sigma_urad = sigma;
    m.snr        = snr;
    return m;
}

TrackParams params() {
    TrackParams p;
    p.kf = KalmanParams::from_max_accel(2.0e4, kDt);
    return p;
}

/// Drive a tracker with a single measurement (or none) for one frame.
int feed(Tracker& tr, int64_t frame, std::vector<Measurement> ms) {
    return tr.step(kDt, std::span<Measurement>(ms), frame);
}

}  // namespace

// ===========================================================================
// CP 6.4 — the lifecycle, arrow by arrow
// ===========================================================================

TEST_CASE("CP 6.4: three hits in three frames confirms; a Tentative track cannot drive") {
    Tracker tr;
    tr.reset(params());

    feed(tr, 0, {meas(0, 0)});
    CHECK(tr.track().state() == TrackState::Tentative);
    // The controller must not follow a track that has been seen once. This is
    // the guard against a single noise blob yanking the mount.
    CHECK_FALSE(tr.has_lock());

    feed(tr, 1, {meas(0, 0)});
    CHECK(tr.track().state() == TrackState::Tentative);

    feed(tr, 2, {meas(0, 0)});
    CHECK(tr.track().state() == TrackState::Confirmed);
    CHECK(tr.has_lock());
}

TEST_CASE("CP 6.4: 3-of-5 confirms even when the hits are not consecutive") {
    Tracker tr;
    tr.reset(params());
    feed(tr, 0, {meas(0, 0)});          // hit
    feed(tr, 1, {});                    // miss
    feed(tr, 2, {meas(0, 0)});          // hit
    CHECK(tr.track().state() == TrackState::Tentative);
    feed(tr, 3, {});                    // miss
    feed(tr, 4, {meas(0, 0)});          // hit -> 3 of the last 5
    CHECK(tr.track().state() == TrackState::Confirmed);
}

TEST_CASE("CP 6.4: a Tentative track that never reaches 3-of-5 is deleted after 5 frames") {
    Tracker tr;
    tr.reset(params());
    feed(tr, 0, {meas(0, 0)});
    for (int i = 1; i <= 3; ++i) feed(tr, i, {});
    CHECK(tr.track().state() == TrackState::Tentative);
    feed(tr, 4, {});                    // fifth frame, still only one hit
    CHECK(tr.track().state() == TrackState::Deleted);
    CHECK_FALSE(tr.has_track());
}

TEST_CASE("CP 6.4: blanking detection for 8 frames does NOT delete the track") {
    // The checkpoint's own wording. This is the criterion the whole coasting
    // mechanism exists to satisfy, and it is graded: §13.1's
    // lock_retention_rate is BP-2.
    Tracker tr;
    tr.reset(params());

    // A target crossing at a steady 240 px/s, confirmed over 10 frames.
    const double v = 240.0 * kIfov;          // urad/s
    for (int i = 0; i < 10; ++i) feed(tr, i, {meas(v * i * kDt, 0.0)});
    REQUIRE(tr.track().state() == TrackState::Confirmed);
    const Rate2 rate_at_lock = tr.track().rate();

    // Now blank it.
    for (int i = 10; i < 18; ++i) {
        feed(tr, i, {});
        CHECK(tr.track().alive());
        // "the camera keeps moving sensibly on prediction" — drivable() is
        // what the engine asks before aiming, and it stays true.
        CHECK(tr.track().drivable());
    }
    CHECK(tr.track().state() == TrackState::Coasting);

    // "sensibly" made concrete: after 8 blank frames the prediction should
    // still be within a few pixels of where the target really is, because the
    // velocity estimate is what carries it.
    const double t_end = 17 * kDt;
    const double truth_x = v * t_end;
    const double err_px = std::fabs(tr.track().position().x - truth_x) / kIfov;
    MESSAGE("prediction error after 8 blanked frames: " << err_px << " px");
    CHECK(err_px < 5.0);
    // And it did not quietly stop: the rate estimate survived the dropout.
    CHECK(tr.track().rate().x == doctest::Approx(rate_at_lock.x).epsilon(0.05));
}

TEST_CASE("CP 6.4: any hit returns a Coasting track to Confirmed; 15 misses delete it") {
    Tracker tr;
    tr.reset(params());
    for (int i = 0; i < 5; ++i) feed(tr, i, {meas(0, 0)});
    REQUIRE(tr.track().confirmed());

    feed(tr, 5, {});
    CHECK(tr.track().state() == TrackState::Coasting);
    feed(tr, 6, {meas(0, 0)});
    CHECK(tr.track().state() == TrackState::Confirmed);

    // 15 consecutive misses, and not one fewer.
    for (int i = 0; i < 14; ++i) {
        feed(tr, 7 + i, {});
        CHECK(tr.track().alive());
    }
    CHECK(tr.track().consecutive_misses() == 14);
    feed(tr, 21, {});
    CHECK(tr.track().state() == TrackState::Deleted);
}

TEST_CASE("CP 6.4: the gate widens while coasting, with no special case") {
    // §10.2: "During Coasting the covariance grows -> the gate widens
    // automatically -> short dropouts recover with no special case." That is an
    // assertion about the arithmetic, so it can be checked as one.
    Tracker tr;
    tr.reset(params());
    for (int i = 0; i < 10; ++i) feed(tr, i, {meas(0, 0)});
    REQUIRE(tr.track().confirmed());

    // A measurement 6 px off the prediction: rejected by a settled filter.
    const Measurement off = meas(6.0 * kIfov, 0.0);
    CHECK_FALSE(tr.track().gates(off));

    const double sigma_locked = tr.track().filter().position_sigma_urad();
    for (int i = 10; i < 18; ++i) feed(tr, i, {});
    const double sigma_coast = tr.track().filter().position_sigma_urad();

    MESSAGE("position sigma: " << sigma_locked << " -> " << sigma_coast << " urad");
    CHECK(sigma_coast > sigma_locked * 3.0);
    // The same measurement that was rejected is now accepted, because we have
    // become genuinely less certain. Nothing in the code says "if coasting".
    CHECK(tr.track().gates(off));
}

// ===========================================================================
// CP 6.3 — gating and association
// ===========================================================================

TEST_CASE("CP 6.3: with a decoy 60 px away the tracker stays on the real beacon") {
    // The checkpoint verbatim. The decoy is BRIGHTER than the beacon, which is
    // the case that matters: a tracker that re-picks the strongest candidate
    // every frame — which is what Stage 1's straw man did — switches to it
    // immediately. Only the gate prevents that.
    Tracker tr;
    tr.reset(params());

    const double v = 150.0 * kIfov;
    for (int i = 0; i < 10; ++i) feed(tr, i, {meas(v * i * kDt, 0.0, kIfov, 18.0f)});
    REQUIRE(tr.track().confirmed());

    for (int i = 10; i < 120; ++i) {
        const double t = i * kDt;
        std::vector<Measurement> ms = {
            meas(v * t, 0.0, kIfov, 18.0f),                  // the beacon
            meas(v * t + 60.0 * kIfov, 0.0, kIfov, 40.0f),   // a brighter decoy
        };
        const int picked = tr.step(kDt, std::span<Measurement>(ms), i);
        REQUIRE(picked == 0);
    }
    CHECK(tr.track().confirmed());
    const double final_err_px =
        std::fabs(tr.track().position().x - v * 119 * kDt) / kIfov;
    MESSAGE("final error with the decoy present: " << final_err_px << " px");
    CHECK(final_err_px < 2.0);
}

TEST_CASE("CP 6.3: the predicted position covariance is isotropic, and that is fine") {
    // This test exists because the first version of it asserted the OPPOSITE
    // and passed on floating-point noise, which is worth keeping as a test
    // rather than deleting as an embarrassment.
    //
    // The claim was the familiar one: while coasting along a velocity the
    // uncertainty ellipse stretches ALONG the motion, so a displacement in that
    // direction is less surprising than the same displacement across it. It is
    // false for this filter. The CWNA model applies the same q to both axes and
    // R is sigma^2 * I, so azimuth and elevation are exactly independent and
    // the position block of P is a scalar multiple of the identity at every
    // step, whatever the velocity is. The gate is a circle.
    //
    // Anisotropy arrives with CP 10.5's coordinate-turn model. Asserting the
    // isotropy now means that when it does, this test fails and says so.
    Tracker tr;
    tr.reset(params());
    const double v = 300.0 * kIfov;                  // moving in x only
    for (int i = 0; i < 20; ++i) feed(tr, i, {meas(v * i * kDt, 0.0)});
    REQUIRE(tr.track().confirmed());
    for (int i = 20; i < 26; ++i) feed(tr, i, {});   // coast

    const auto& P = tr.track().filter().covariance();
    CHECK(P(0, 0) == doctest::Approx(P(1, 1)).epsilon(1e-12));
    CHECK(P(0, 1) == doctest::Approx(0.0).epsilon(1e-12));

    const Angle2 pred = tr.track().position();
    const double d = 4.0 * kIfov;
    const double along  = tr.track().distance2(meas(pred.x + d, pred.y));
    const double across = tr.track().distance2(meas(pred.x, pred.y + d));
    MESSAGE("d^2 along = " << along << ", across = " << across);
    CHECK(along == doctest::Approx(across).epsilon(1e-9));
}

TEST_CASE("CP 6.3: what the gate DOES buy is a radius that tracks its own confidence") {
    // The honest justification for a chi-square gate over a fixed angular
    // radius: the threshold keeps meaning "reject 1% of true detections" as the
    // filter's confidence changes, which a fixed radius cannot.
    Tracker tr;
    tr.reset(params());
    for (int i = 0; i < 20; ++i) feed(tr, i, {meas(0, 0, 0.3 * kIfov)});
    REQUIRE(tr.track().confirmed());

    // Settled: a 3 px offset is out.
    CHECK_FALSE(tr.track().gates(meas(3.0 * kIfov, 0.0, 0.3 * kIfov)));
    // Coasting: the same offset is in, because we are genuinely less sure.
    for (int i = 20; i < 30; ++i) feed(tr, i, {});
    CHECK(tr.track().gates(meas(3.0 * kIfov, 0.0, 0.3 * kIfov)));
}

TEST_CASE("CP 6.3: association prefers the crisp detection over the smeared one") {
    // Why the score is d^2 + ln|S| and not d^2. d^2 divides by the uncertainty
    // the candidate claims, so a garbage detection reporting a huge sigma gets
    // a small d^2 for free and a plain nearest-in-d^2 rule takes it every time.
    //
    // The two candidates below are the same distance from the prediction and
    // differ only in how much they trust themselves.
    Tracker tr;
    tr.reset(params());
    for (int i = 0; i < 20; ++i) feed(tr, i, {meas(0, 0, 0.5 * kIfov)});
    REQUIRE(tr.track().confirmed());

    const double off = 1.5 * kIfov;
    const Measurement crisp   = meas(off, 0.0, 0.5 * kIfov, 30.0f);
    const Measurement smeared = meas(off, 0.0, 4.0 * kIfov,  3.0f);

    // The trap, demonstrated: on d^2 alone the smeared one wins.
    CHECK(tr.track().distance2(smeared) < tr.track().distance2(crisp));
    // On the likelihood score it does not.
    CHECK(tr.track().assoc_score(crisp) < tr.track().assoc_score(smeared));
    MESSAGE("d^2: crisp " << tr.track().distance2(crisp)
            << " vs smeared " << tr.track().distance2(smeared)
            << " | score: crisp " << tr.track().assoc_score(crisp)
            << " vs smeared " << tr.track().assoc_score(smeared));

    // And the tracker as a whole picks the crisp one.
    std::vector<Measurement> ms = {smeared, crisp};
    CHECK(tr.step(kDt, std::span<Measurement>(ms), 20) == 1);
}

TEST_CASE("CP 6.3: unassociated measurements are reported, not silently dropped") {
    // §10.5's probability grid treats "we looked and saw something we could not
    // explain" as information. Flagging the associated one rather than erasing
    // it is what makes that possible later.
    Tracker tr;
    tr.reset(params());
    for (int i = 0; i < 5; ++i) feed(tr, i, {meas(0, 0)});
    REQUIRE(tr.track().confirmed());

    std::vector<Measurement> ms = {meas(0, 0), meas(300.0 * kIfov, 0.0)};
    const int picked = tr.step(kDt, std::span<Measurement>(ms), 5);
    CHECK(picked == 0);
    CHECK(ms[0].associated);
    CHECK_FALSE(ms[1].associated);
    CHECK(tr.gated_count() == 1);
}

// ===========================================================================
// CP 6.5 — adaptive R
// ===========================================================================

TEST_CASE("CP 6.5: adaptive R beats fixed R when the detection quality varies") {
    // The ablation the checkpoint asks for, run as a controlled A/B on the same
    // measurement sequence.
    //
    // The scenario is "fog": the detection degrades part-way through, so its
    // reported sigma rises and its actual error rises with it. An adaptive
    // filter sees the rise and leans on its prediction; a fixed-R filter keeps
    // trusting the measurement exactly as much as before, and follows the
    // degraded detections into the noise.
    constexpr double v      = 200.0 * kIfov;
    constexpr double good_s = 0.5 * kIfov;
    constexpr double bad_s  = 6.0 * kIfov;

    auto run = [&](bool adaptive) {
        RngSet rng(2024);                       // the SAME noise for both arms
        Pcg32& g = rng[Stream::Misc];
        TrackParams p = params();
        p.adaptive_r         = adaptive;
        p.fixed_r_sigma_urad = good_s;          // tuned for the clear half
        Tracker tr;
        tr.reset(p);

        double sq = 0.0;
        int n = 0;
        for (int i = 0; i < 400; ++i) {
            const double t = i * kDt;
            const bool fog = i >= 150;
            const double s = fog ? bad_s : good_s;
            // The reported sigma is honest: §10.1.4's size/(2*SNR) really does
            // grow when the detection weakens, which is why it is usable as R.
            const double zx = v * t + g.next_normal() * s;
            const double zy = 0.0   + g.next_normal() * s;
            std::vector<Measurement> ms = {meas(zx, zy, s, fog ? 3.0f : 30.0f)};
            tr.step(kDt, std::span<Measurement>(ms), i);

            if (i >= 200) {                     // score the fog half only
                const Angle2 e = tr.track().position() - Angle2{v * t, 0.0};
                sq += e.x * e.x + e.y * e.y;
                ++n;
            }
        }
        return std::sqrt(sq / n);
    };

    const double rms_fixed    = run(false);
    const double rms_adaptive = run(true);
    MESSAGE("fog-half RMS error: fixed R " << rms_fixed / kIfov
            << " px, adaptive R " << rms_adaptive / kIfov << " px");
    CHECK(rms_adaptive < rms_fixed);
}

TEST_CASE("CP 6.5: R is clamped at both ends") {
    // A detection claiming an impossible SNR must not be able to drive R to
    // zero — the filter would then discard its own prediction and follow a
    // single frame, which is the straw-man detector again with extra steps.
    TrackParams p = params();
    p.min_r_sigma_urad = 5.0;
    p.max_r_sigma_urad = 1000.0;

    Tracker tr;
    tr.reset(p);
    for (int i = 0; i < 5; ++i) feed(tr, i, {meas(0, 0, kIfov)});
    REQUIRE(tr.track().confirmed());

    std::vector<Measurement> tiny = {meas(0, 0, 1e-9)};
    tr.step(kDt, std::span<Measurement>(tiny), 5);
    CHECK(tr.track().last_sigma_urad() == doctest::Approx(5.0));

    std::vector<Measurement> huge = {meas(0, 0, 1e9)};
    tr.step(kDt, std::span<Measurement>(huge), 6);
    CHECK(tr.track().last_sigma_urad() == doctest::Approx(1000.0));
}
