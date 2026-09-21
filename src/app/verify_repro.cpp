#include "app/verify_repro.hpp"

#include "core/hash.hpp"
#include "degrade/sensor_simd.hpp"
#include "engine/pipeline.hpp"

#include "sat/version.hpp"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

namespace sat {
namespace {

// ---------------------------------------------------------------------------
// The built-in scenario set.
//
// These are built in code rather than loaded from scenarios/*.toml, on purpose:
// the gate must keep working when a scenario file is edited, and a file that
// changes would change every digest the CI matrix compares. They are chosen to
// exercise the parts of the pipeline most likely to be irreproducible, which is
// a different criterion from "representative":
//
//   static          nothing moves; any divergence is in the renderer itself
//   linear          spec row 12's mandatory straight line
//   fast            near the rate limit, so the plant saturates constantly and
//                   the clamping branches are all taken
//   multi           several emitters, so visit ORDER matters (design §9.4.6
//                   warns that label numbering depends on it)
//   offset          starts far off-target, so acquisition and the FSM's early
//                   transitions are covered
//
// ---------------------------------------------------------------------------
// THE GATE USED TO RUN WITH THE ENTIRE DAMAGE CHAIN TURNED OFF — P1-1
// ---------------------------------------------------------------------------
// make_config() built a bare PipelineConfig and never enabled the sensor
// model. SensorChain defaults `enabled_` to false, so every run of this gate
// had NO Gaussian noise, no Poisson, no salt and pepper, no fixed-pattern, no
// defects, no jitter, no platform motion and no atmosphere.
//
// The symptom was visible in its own output and had been printed on every run
// for months: the seed did not change the digest for four of the five
// scenarios.
//
//     static   1  IDENTICAL  6e5ea6926c621cbf
//     static   2  IDENTICAL  6e5ea6926c621cbf     <- same digest, different seed
//     linear   1  IDENTICAL  bf817002c3fc587c
//     linear   2  IDENTICAL  bf817002c3fc587c     <- same
//
// Only `multi` responded, and only because its seed drives the build-time
// clutter LAYOUT rather than anything per-frame. The seeding mechanism was
// fine; there was simply nothing stochastic left in the pipeline for it to
// seed.
//
// That is not a cosmetic gap. The AVX2 damage chain is RUNTIME-DISPATCHED
// (degrade/sensor_simd.cpp), which means different machines execute different
// code — precisely the risk INV-3 exists to catch — and the gate protecting it
// never ran it. The CI job comparing -O0 against -O2 inherited the same hole,
// and both jobs run on the same runner, so scalar-versus-AVX2 divergence was
// untested end to end.
//
// So every scenario below now carries the specification's own damage: row 21's
// impulse noise and Poisson, row 22's read noise at the cap, row 23's jitter,
// row 25's platform motion, and a non-clear atmosphere. A seed change must now
// move every digest, and `verify_reproducibility` asserts exactly that.
// ---------------------------------------------------------------------------
struct BuiltinScenario {
    const char* name;
    double      offset_px;
    double      vx_px_s;
    double      vy_px_s;
    int         extra_emitters;
    Atmosphere  atmosphere;
    double      jitter_px;
    double      platform_vx;   ///< row 25, px/s
    double      platform_vy;
};

constexpr BuiltinScenario kScenarios[] = {
    // Atmospheres are spread across the set so the affine contrast/brightness
    // branch is taken with several different (alpha, beta) pairs rather than
    // one. `clear` appears nowhere: it is the identity, and an identity
    // transform tests the code that applies it and not the code that computes
    // it.
    {"static",  60.0,   0.0,   0.0, 0, Atmosphere::Haze,     20.0,  0.0,  0.0},
    {"linear", 120.0,  20.0,  -8.0, 0, Atmosphere::Fog,      20.0, 15.0, -8.0},
    {"fast",   200.0, 260.0,  90.0, 0, Atmosphere::Rain,     20.0, 20.0, 12.0},
    {"multi",  120.0,  15.0,   5.0, 6, Atmosphere::LowLight, 12.0, -6.0,  9.0},
    {"offset", 380.0,  10.0,   0.0, 0, Atmosphere::Haze,     20.0,  9.0, -4.0},
};

// ---------------------------------------------------------------------------
// The scenario, as a Scenario.
//
// Built through the ordinary Scenario -> build_from_scenario path rather than
// by filling a PipelineConfig directly, because that is the path a real run
// takes. The old code assembled a PipelineConfig by hand, which is how it came
// to be missing the sensor model in the first place: there was no single place
// that said "this is a configured run" and could be seen to be incomplete.
// ---------------------------------------------------------------------------
Scenario make_scenario(const BuiltinScenario& sc, uint64_t seed, double duration_s) {
    Scenario s;
    s.name       = sc.name;
    s.duration_s = duration_s;
    s.seed       = seed;

    s.canvas_px[0] = s.canvas_px[1] = 2000;
    s.resolution[0] = 640;  s.resolution[1] = 480;
    s.fov_deg[0] = 4.0;     s.fov_deg[1] = 3.0;
    s.initial_pos_px[0] = s.initial_pos_px[1] = 999.5;

    // Row 21-22: the damage chain, at the specification's own values. This is
    // the block whose absence made the gate hollow.
    s.noise_poisson  = true;
    s.gaussian_sigma = 20.0;      // row 22's cap
    s.salt_pepper    = 0.10;      // row 21's ~10%
    s.hot_pixels     = 40;
    s.atmosphere     = sc.atmosphere;                 // row 24

    // Rows 23 and 25: the disturbances, which move the TRUE boresight and so
    // reach the renderer, the blur and the whole loop.
    s.jitter_px_per_frame = sc.jitter_px;
    if (sc.platform_vx != 0.0 || sc.platform_vy != 0.0) {
        MotionSpec pm;
        pm.kind = "linear";
        pm.velocity_px_s[0] = sc.platform_vx;
        pm.velocity_px_s[1] = sc.platform_vy;
        s.platform.push_back(pm);
    }

    // Row 7-12: one beacon, pinned (row 11 permits either) at the scenario's
    // offset from screen centre, on a straight line.
    TargetSpec t;
    t.intensity      = 220.0;
    t.size_px        = 10;
    t.random_initial = false;
    t.initial_px[0]  = 999.5 + sc.offset_px;
    t.initial_px[1]  = 999.5 + sc.offset_px * 0.5;
    MotionSpec tm;
    tm.kind = "linear";
    tm.velocity_px_s[0] = sc.vx_px_s;
    tm.velocity_px_s[1] = sc.vy_px_s;
    t.motion.push_back(tm);
    s.targets.push_back(t);

    // §9.4.6 warns that blob label numbering depends on visit order, so the
    // multi case needs several emitters. They come from the world builder's
    // ClutterLayout stream — the real path — rather than being placed by hand
    // here, which is one less piece of bespoke code in the gate to drift from
    // what a real run does.
    s.static_sources = sc.extra_emitters;
    s.decoy_beacons  = 0;

    // Off: it is a different subsystem, it would add its own adaptation
    // history to every digest, and a gate should test one thing.
    s.supervisor_enabled = false;
    return s;
}

/// Digest of a whole run: every frame's combined fingerprint, folded.
uint64_t digest_of(const std::vector<FrameFingerprint>& fps) {
    uint64_t d = kFnvOffsetBasis;
    for (const auto& f : fps) d = hash_value(f.combined(), d);
    return d;
}

std::vector<FrameFingerprint> run_once(const BuiltinScenario& sc, uint64_t seed,
                                       double duration_s) {
    Pipeline p;
    p.build_from_scenario(make_scenario(sc, seed, duration_s));
    p.set_publish_snapshots(true);
    while (p.step()) {}
    return p.fingerprints();
}

}  // namespace

std::vector<ReproResult> verify_reproducibility(const std::vector<uint64_t>& seeds,
                                                double duration_s) {
    std::vector<ReproResult> out;
    for (const auto& sc : kScenarios) {
        for (const uint64_t seed : seeds) {
            ReproResult r;
            r.scenario = sc.name;
            r.seed     = seed;

            const auto a = run_once(sc, seed, duration_s);
            const auto b = run_once(sc, seed, duration_s);

            r.frames = static_cast<int64_t>(a.size());
            if (a.size() != b.size()) {
                r.identical        = false;
                r.first_divergence = static_cast<int64_t>(std::min(a.size(), b.size()));
                r.diverged_field   = "frame count";
            } else {
                for (size_t i = 0; i < a.size(); ++i) {
                    if (a[i].combined() == b[i].combined()) continue;
                    r.identical        = false;
                    r.first_divergence = static_cast<int64_t>(i);
                    // Naming the component is the point of splitting the
                    // fingerprint: "the image differs" and "the image is
                    // identical but the detection moved" are different bugs in
                    // different subsystems.
                    if      (a[i].image     != b[i].image)     r.diverged_field = "image";
                    else if (a[i].boresight != b[i].boresight) r.diverged_field = "boresight";
                    else if (a[i].detection != b[i].detection) r.diverged_field = "detection";
                    else if (a[i].track     != b[i].track)     r.diverged_field = "track";
                    else                                        r.diverged_field = "mode";
                    break;
                }
            }

            r.digest = digest_of(a);

            // ---------------------------------------------------------------
            // The scalar arm. Same scenario, same seed, vector path forced
            // off; the digest must be identical.
            //
            // Only meaningful where the hardware has AVX2 — otherwise both
            // runs are already scalar and the comparison is a tautology, so it
            // is reported as not-performed rather than as a pass.
            // ---------------------------------------------------------------
            if (damage_chain_simd_available()) {
                set_damage_simd_enabled(false);
                const uint64_t scalar = digest_of(run_once(sc, seed, duration_s));
                set_damage_simd_enabled(true);

                r.simd_compared = true;
                r.scalar_digest = scalar;
                r.simd_matches  = (scalar == r.digest);
            }

            out.push_back(std::move(r));
        }
    }
    return out;
}

std::vector<SeedSensitivity> seed_sensitivity(const std::vector<ReproResult>& results) {
    std::vector<SeedSensitivity> out;
    for (const ReproResult& r : results) {
        auto it = std::find_if(out.begin(), out.end(),
                               [&](const SeedSensitivity& s) { return s.scenario == r.scenario; });
        if (it == out.end()) {
            out.push_back(SeedSensitivity{r.scenario, 0, 0});
            it = out.end() - 1;
        }
        ++it->seeds;
    }
    // Count distinct digests per scenario. A linear scan per scenario: there
    // are five scenarios and a handful of seeds, so a set would cost more in
    // ceremony than it saves.
    for (SeedSensitivity& s : out) {
        std::vector<uint64_t> seen;
        for (const ReproResult& r : results) {
            if (r.scenario != s.scenario) continue;
            if (std::find(seen.begin(), seen.end(), r.digest) == seen.end()) {
                seen.push_back(r.digest);
            }
        }
        s.distinct = seen.size();
    }
    return out;
}

int report_reproducibility(const std::vector<ReproResult>& results) {
    std::printf("\nREPRODUCIBILITY VERIFICATION (INV-3)\n");
    std::printf("build %s+%s\n\n", SAT_VERSION, SAT_GIT_HASH);
    std::printf("damage chain: %s\n",
                damage_chain_simd_available()
                    ? "AVX2 available - scalar path compared against it below"
                    : "no AVX2 on this CPU - scalar only, nothing to compare");
    std::printf("\n%-10s %6s %8s  %-10s  %-18s %-6s %s\n",
                "scenario", "seed", "frames", "status", "digest", "simd", "note");
    std::printf("%s\n", std::string(86, '-').c_str());

    int failures = 0;
    for (const auto& r : results) {
        std::string note;
        if (!r.identical) {
            ++failures;
            note = "diverged at frame " + std::to_string(r.first_divergence)
                 + " in '" + r.diverged_field + "'";
        }
        const char* simd = "n/a";
        if (r.simd_compared) {
            if (r.simd_matches) {
                simd = "match";
            } else {
                simd = "DIFFER";
                ++failures;
                if (!note.empty()) note += "; ";
                char b[64];
                std::snprintf(b, sizeof b, "scalar digest %016llx",
                              static_cast<unsigned long long>(r.scalar_digest));
                note += b;
            }
        }
        std::printf("%-10s %6llu %8lld  %-10s  %016llx  %-6s %s\n",
                    r.scenario.c_str(),
                    static_cast<unsigned long long>(r.seed),
                    static_cast<long long>(r.frames),
                    r.identical ? "IDENTICAL" : "DIVERGED",
                    static_cast<unsigned long long>(r.digest),
                    simd,
                    note.c_str());
    }

    // -----------------------------------------------------------------------
    // Sensitivity. The opposite failure from divergence, and the one that let
    // this gate run hollow for months: if two seeds produce the same digest,
    // the run has no live stochastic component and every IDENTICAL above is
    // vacuous.
    // -----------------------------------------------------------------------
    const std::vector<SeedSensitivity> sens = seed_sensitivity(results);
    bool any_multi_seed = false;
    for (const SeedSensitivity& s : sens) if (s.seeds > 1) any_multi_seed = true;
    if (any_multi_seed) {
        std::printf("\nSEED SENSITIVITY  (different seeds must NOT agree)\n");
        for (const SeedSensitivity& s : sens) {
            if (s.seeds < 2) continue;
            const bool ok = s.ok();
            if (!ok) ++failures;
            std::printf("  %-10s %zu distinct digests over %zu seeds   %s\n",
                        s.scenario.c_str(), s.distinct, s.seeds,
                        ok ? "ok" : "FAIL - the seed reaches nothing stochastic");
        }
    }

    std::printf("\n");
    if (failures == 0) {
        std::printf("PASS: %zu runs, every one bit-identical to its repeat,\n", results.size());
        if (damage_chain_simd_available()) {
            std::printf("      every one identical between the AVX2 and scalar damage chains,\n");
        }
        std::printf("      and every seed producing a different digest.\n");
        std::printf("\nEvery run carries the specification's own damage: row 21 impulse\n");
        std::printf("noise and Poisson, row 22 read noise at the cap, row 23 jitter, row 25\n");
        std::printf("platform motion, and a non-clear atmosphere.\n");
        std::printf("\nNote: this proves determinism WITHIN a build. Cross-build and\n");
        std::printf("cross-machine agreement is checked by comparing the digests above\n");
        std::printf("between the -O0 and -O2 CI jobs (design §2 CP 2.6, §11.4).\n");
    } else {
        std::printf("FAIL: %d check(s) failed. INV-3 is violated.\n", failures);
        std::printf("\nUsual causes, in order of likelihood:\n");
        std::printf("  * a wall-clock read in the simulation path (`just gate-no-chrono`)\n");
        std::printf("  * an unseeded generator or rand() (`just gate-no-rand`)\n");
        std::printf("  * iterating an unordered_map/set\n");
        std::printf("  * std::sort on a key that can tie -- use stable_sort\n");
        std::printf("  * accumulated time (t += dt) instead of tick / rate\n");
        std::printf("\nIf the SIMD column says DIFFER, the divergence is between the AVX2\n");
        std::printf("and scalar damage chains specifically (degrade/sensor_simd.cpp). The\n");
        std::printf("usual causes there are an FMA contraction — that file must target\n");
        std::printf("`avx2` and NOT `avx2,fma` — and a jump-ahead constant that does not\n");
        std::printf("leave the generator where the scalar loop would have left it.\n");
        std::printf("\nIf SEED SENSITIVITY fails, the run has no live stochastic component:\n");
        std::printf("check that the sensor model is enabled, which is the defect P1-1\n");
        std::printf("existed for.\n");
    }
    return failures == 0 ? 0 : 1;
}

}  // namespace sat
