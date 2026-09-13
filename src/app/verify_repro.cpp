#include "app/verify_repro.hpp"

#include "core/hash.hpp"
#include "engine/pipeline.hpp"

#include "sat/version.hpp"

#include <algorithm>
#include <cstdio>
#include <string>

namespace sat {
namespace {

// ---------------------------------------------------------------------------
// The built-in scenario set.
//
// These are not the TOML scenarios of Stage 3 — they do not exist yet. They are
// chosen to exercise the parts of the pipeline most likely to be
// irreproducible, which is a different criterion from "representative":
//
//   static          nothing moves; any divergence is in the renderer itself
//   linear          spec row 12's mandatory straight line
//   fast            near the rate limit, so the plant saturates constantly and
//                   the clamping branches are all taken
//   multi           several emitters, so visit ORDER matters (design §9.4.6
//                   warns that label numbering depends on it)
//   offset          starts far off-target, so acquisition and the FSM's early
//                   transitions are covered
// ---------------------------------------------------------------------------
struct BuiltinScenario {
    const char* name;
    double      offset_px;
    double      vx_px_s;
    double      vy_px_s;
    int         extra_emitters;
};

constexpr BuiltinScenario kScenarios[] = {
    {"static",  60.0,   0.0,   0.0, 0},
    {"linear", 120.0,  20.0,  -8.0, 0},
    {"fast",   200.0, 260.0,  90.0, 0},
    {"multi",  120.0,  15.0,   5.0, 6},
    {"offset", 380.0,  10.0,   0.0, 0},
};

PipelineConfig make_config(const BuiltinScenario& sc, uint64_t seed, double duration_s) {
    PipelineConfig cfg;
    cfg.synthetic.camera     = CameraGeometry::make(640, 480, 4.0, 3.0);
    cfg.synthetic.screen     = ScreenGeometry::make(2000, 2000, cfg.synthetic.camera);
    cfg.synthetic.duration_s = duration_s;
    cfg.synthetic.seed       = seed;
    cfg.pan   = GimbalParams::from_dps(5.0, 50.0);
    cfg.tilt  = GimbalParams::from_dps(5.0, 50.0);
    cfg.gains = ControlGains::proportional(4.0);
    cfg.publish_snapshots = true;
    (void)sc;
    return cfg;
}

EmitterSoA make_emitters(const BuiltinScenario& sc, uint64_t seed,
                         const ScreenGeometry& scr) {
    EmitterSoA e;
    e.reserve(static_cast<size_t>(sc.extra_emitters) + 1);

    e.add(scr.cx + sc.offset_px, scr.cy + sc.offset_px * 0.5, 220.0f, 10,
          ShapeKind::Square, EmitterKind::Target);
    e.vx[0] = sc.vx_px_s;
    e.vy[0] = sc.vy_px_s;

    // Extra emitters come from a named stream, so adding them cannot perturb
    // anything else (core/rng.hpp).
    if (sc.extra_emitters > 0) {
        RngSet rng(seed);
        Pcg32& g = rng[Stream::ClutterLayout];
        for (int i = 0; i < sc.extra_emitters; ++i) {
            const double x = scr.cx + g.next_range(-400.0, 400.0);
            const double y = scr.cy + g.next_range(-400.0, 400.0);
            const size_t idx = e.add(x, y, static_cast<float>(g.next_range(60.0, 180.0)),
                                     static_cast<uint16_t>(6 + g.next_below(10)),
                                     ShapeKind::Square, EmitterKind::Clutter);
            e.vx[idx] = g.next_range(-5.0, 5.0);
            e.vy[idx] = g.next_range(-5.0, 5.0);
        }
    }
    return e;
}

std::vector<FrameFingerprint> run_once(const BuiltinScenario& sc, uint64_t seed,
                                       double duration_s) {
    PipelineConfig cfg = make_config(sc, seed, duration_s);
    EmitterSoA e = make_emitters(sc, seed, cfg.synthetic.screen);

    Pipeline p;
    p.build(cfg, std::move(e));
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

            uint64_t digest = kFnvOffsetBasis;
            for (const auto& f : a) digest = hash_value(f.combined(), digest);
            r.digest = digest;

            out.push_back(std::move(r));
        }
    }
    return out;
}

int report_reproducibility(const std::vector<ReproResult>& results) {
    std::printf("\nREPRODUCIBILITY VERIFICATION (INV-3)\n");
    std::printf("build %s+%s\n\n", SAT_VERSION, SAT_GIT_HASH);
    std::printf("%-10s %6s %8s  %-10s  %-18s %s\n",
                "scenario", "seed", "frames", "status", "digest", "note");
    std::printf("%s\n", std::string(78, '-').c_str());

    int failures = 0;
    for (const auto& r : results) {
        std::string note;
        if (!r.identical) {
            ++failures;
            note = "diverged at frame " + std::to_string(r.first_divergence)
                 + " in '" + r.diverged_field + "'";
        }
        std::printf("%-10s %6llu %8lld  %-10s  %016llx  %s\n",
                    r.scenario.c_str(),
                    static_cast<unsigned long long>(r.seed),
                    static_cast<long long>(r.frames),
                    r.identical ? "IDENTICAL" : "DIVERGED",
                    static_cast<unsigned long long>(r.digest),
                    note.c_str());
    }

    std::printf("\n");
    if (failures == 0) {
        std::printf("PASS: %zu runs, every one bit-identical to its repeat.\n", results.size());
        std::printf("\nNote: this proves determinism WITHIN a build. Cross-build and\n");
        std::printf("cross-machine agreement is checked by comparing the digests above\n");
        std::printf("between the -O0 and -O2 CI jobs (design §2 CP 2.6, §11.4).\n");
    } else {
        std::printf("FAIL: %d of %zu runs diverged. INV-3 is violated.\n",
                    failures, results.size());
        std::printf("\nUsual causes, in order of likelihood:\n");
        std::printf("  * a wall-clock read in the simulation path (`just gate-no-chrono`)\n");
        std::printf("  * an unseeded generator or rand() (`just gate-no-rand`)\n");
        std::printf("  * iterating an unordered_map/set\n");
        std::printf("  * std::sort on a key that can tie -- use stable_sort\n");
        std::printf("  * accumulated time (t += dt) instead of tick / rate\n");
    }
    return failures == 0 ? 0 : 1;
}

}  // namespace sat
