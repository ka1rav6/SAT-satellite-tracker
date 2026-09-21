// tests/tracking/test_priority.cpp — design §10.2's priority policy.
//
// The policy exists because the tracker used to seed its track from "the
// strongest candidate", and on the specification's own scenario — 120 clutter
// sources drawn over 0.35x to 1.6x the beacon's brightness (§9.1) — the
// strongest candidate is a rock roughly every time. See tracking/priority.hpp
// for the 30-second run that measured it.
//
// Every case here is about one claim: that the policy prefers a thing that
// MOVES, and that it cannot be talked out of it by brightness, stability,
// centrality or age.

#include <doctest/doctest.h>

#include "tracking/priority.hpp"
#include "tracking/track.hpp"

#include <cmath>

using namespace sat;

namespace {

TrackParams params(double max_accel_urad_s2 = 1000.0) {
    TrackParams p;
    // The engine sizes q from the scenario's own closed-form maximum
    // acceleration (§7.2); a test that leaves it at zero is asking a
    // constant-velocity filter to follow a curve with no process noise, and it
    // will fail the NIS quality test for a perfectly good reason.
    p.kf = KalmanParams::from_max_accel(max_accel_urad_s2, 1.0 / 30.0);
    p.max_target_speed_urad_s = 3000.0;
    p.gate_pad_urad           = 500.0;
    p.adaptive_r              = false;
    p.fixed_r_sigma_urad      = 50.0;
    return p;
}

PriorityContext ctx() {
    PriorityContext c;
    c.boresight        = Angle2{0.0, 0.0};
    c.half_fov_urad    = 35000.0;         // ~4 x 3 deg at the spec IFOV
    c.speed_ref_urad_s = 3000.0 * 0.20;   // PriorityWeights::speed_frac
    c.speed_max_urad_s = 3000.0;
    return c;
}

Measurement meas(double x, double y, float snr) {
    Measurement m;
    m.angle      = Angle2{x, y};
    m.sigma_urad = 50.0;
    m.snr        = snr;
    return m;
}

/// Run a track forward for `frames`, moving at `vx, vy` urad/s, with the whole
/// field drifting at `dx, dy` urad/s — spec row 25's platform motion as the
/// tracker sees it.
void run(Track& t, int frames, double vx, double vy, double dx, double dy,
         float snr, double dt = 1.0 / 30.0) {
    for (int i = 1; i <= frames; ++i) {
        t.predict(dt);
        const double time = i * dt;
        t.update(meas((vx + dx) * time, (vy + dy) * time, snr));
        // The tracker would supply the median step; here it is exact, which is
        // what the median is estimating.
        t.accumulate_residual(Angle2{dx * dt, dy * dt}, /*common_valid=*/true);
    }
}

}  // namespace

TEST_CASE("§10.2: a bright, stable, centred, old rock cannot take the mount") {
    // The exact failure the policy exists to prevent. This candidate scores the
    // MAXIMUM on all four of the design's original terms.
    PriorityWeights w;
    Track rock;
    rock.start(meas(0.0, 0.0, 200.0f), params(), 0);
    // Static in the world, with the platform drifting under it at row 25's
    // rate. It never misses, it sits on the boresight, and it is 60 frames old.
    run(rock, 60, /*vx=*/0.0, /*vy=*/0.0, /*dx=*/1636.0, /*dy=*/-872.0, 200.0f);

    const float score = priority_score(rock, ctx(), w);
    INFO("rock scored ", score, " relative speed ", rock.relative_speed_urad_s());
    // It has every other advantage there is, so it must be near the ceiling the
    // four non-motion weights allow — and that ceiling must be below the
    // commit threshold.
    CHECK(rock.relative_speed_urad_s() < 1.0);
    CHECK(score <= w.snr + w.stability + w.centrality + w.age + 1e-5f);
    CHECK(score < w.min_commit_score);
}

TEST_CASE("§10.2: a beacon moving under the same drift clears the threshold") {
    PriorityWeights w;
    Track beacon;
    beacon.start(meas(0.0, 0.0, 40.0f), params(), 0);
    // 2,400 urad/s of real motion, under the identical platform drift.
    run(beacon, 60, /*vx=*/2200.0, /*vy=*/-960.0, /*dx=*/1636.0, /*dy=*/-872.0, 40.0f);

    const float score = priority_score(beacon, ctx(), w);
    INFO("beacon scored ", score, " relative speed ", beacon.relative_speed_urad_s());
    // Ego-motion compensation must recover the beacon's OWN speed, not its
    // apparent one. Apparent would be |(3836, -1832)| = 4251.
    CHECK(beacon.relative_speed_urad_s() == doctest::Approx(2400.0).epsilon(0.05));
    CHECK(score > w.min_commit_score);
}

TEST_CASE("§10.2: raw apparent speed would pick the rock — the inversion is real") {
    // The measurement that justifies ego-motion compensation rather than a
    // plain speed term. Under row 25's drift the rock's UNCOMPENSATED apparent
    // speed exceeds the beacon's, so a policy scoring raw speed prefers it.
    Track rock, beacon;
    rock.start(meas(0.0, 0.0, 200.0f), params(), 0);
    beacon.start(meas(0.0, 0.0, 40.0f), params(), 0);
    // spec_defaults.toml's numbers: platform (15, -8) px/s, beacon (22, -11) px/s,
    // at the default IFOV of 109.08 urad/px. Apparent = true - platform.
    const double ifov = 109.08;
    const double px   = ifov;
    run(rock,   60, 0.0,          0.0,          -15.0 * px, 8.0 * px, 200.0f);
    run(beacon, 60, 22.0 * px, -11.0 * px,      -15.0 * px, 8.0 * px, 40.0f);

    const double rock_apparent   = rock.mean_speed_urad_s();
    const double beacon_apparent = beacon.mean_speed_urad_s();
    INFO("apparent: rock ", rock_apparent, " beacon ", beacon_apparent);
    CHECK(rock_apparent > beacon_apparent);          // the inversion

    // Compensated, the order is restored and the gap is the beacon's own speed.
    INFO("relative: rock ", rock.relative_speed_urad_s(),
         " beacon ", beacon.relative_speed_urad_s());
    CHECK(beacon.relative_speed_urad_s() > 10.0 * rock.relative_speed_urad_s());
}

TEST_CASE("§10.2: something faster than the target can move scores zero on motion") {
    // A weak track jumping between noise blobs reports an enormous apparent
    // speed and, on the motion term alone, used to out-score everything real.
    PriorityWeights w;
    Track wild;
    wild.start(meas(0.0, 0.0, 8.0f), params(), 0);
    run(wild, 60, /*vx=*/12000.0, /*vy=*/0.0, 0.0, 0.0, 8.0f);   // 4x the maximum

    const float score = priority_score(wild, ctx(), w);
    INFO("wild scored ", score, " relative speed ", wild.relative_speed_urad_s());
    CHECK(wild.relative_speed_urad_s() > ctx().speed_max_urad_s * w.speed_max_slack);
    CHECK(score < w.min_commit_score);
}

TEST_CASE("§10.2: the motion term survives a closed path") {
    // Row 12's circular and lissajous modes return to where they started. A
    // lifetime displacement would read them as stationary and the policy would
    // drop the beacon; the sliding window must not.
    PriorityWeights w;
    const double dt = 1.0 / 30.0;
    const double R  = 2000.0;           // urad
    const double period_s = 4.0;        // one full circle every 4 s
    // Centripetal acceleration of this path, which is exactly what the engine
    // would derive from the scenario and hand the filter.
    const double a_max = R * (2.0 * kPi / period_s) * (2.0 * kPi / period_s);

    Track orbit;
    orbit.start(meas(0.0, 0.0, 40.0f), params(a_max), 0);
    const int frames = static_cast<int>(period_s / dt);   // exactly one period
    for (int i = 1; i <= frames; ++i) {
        orbit.predict(dt);
        const double th = 2.0 * kPi * (i * dt) / period_s;
        orbit.update(meas(R * std::sin(th), R * (1.0 - std::cos(th)), 40.0f));
        orbit.accumulate_residual(Angle2{}, true);
    }

    INFO("state ", track_state_name(orbit.state()), " age ", orbit.age_frames(),
         " relframes ", orbit.relative_frames(), " nis ", orbit.nis_ema());
    INFO("net displacement speed ", orbit.mean_speed_urad_s(),
         " windowed ", orbit.relative_speed_urad_s());
    // Net displacement over one full period is ~0 by construction.
    CHECK(orbit.mean_speed_urad_s() < 200.0);
    // The window still sees it moving: 2 pi R / period = 3141 urad/s.
    CHECK(orbit.relative_speed_urad_s() > 1500.0);
    CHECK(priority_score(orbit, ctx(), w) > w.min_commit_score);
}

TEST_CASE("§10.2: a coasting frame contributes nothing to the motion estimate") {
    // A miss moves the estimate by the filter's own prediction, so folding it
    // into the residual would be evidence for a belief drawn from that belief.
    Track t;
    t.start(meas(0.0, 0.0, 40.0f), params(), 0);
    const double dt = 1.0 / 30.0;
    for (int i = 1; i <= 10; ++i) {
        t.predict(dt);
        t.update(meas(2400.0 * i * dt, 0.0, 40.0f));
        t.accumulate_residual(Angle2{}, true);
    }
    const int before = t.relative_frames();
    for (int i = 0; i < 5; ++i) {
        t.predict(dt);
        t.miss();
        t.accumulate_residual(Angle2{}, true);
    }
    CHECK(t.relative_frames() == before);

    // And a frame where the ego-motion could not be measured is skipped too.
    t.predict(dt);
    t.update(meas(9999.0, 0.0, 40.0f));
    t.accumulate_residual(Angle2{}, /*common_valid=*/false);
    CHECK(t.relative_frames() == before);
}
