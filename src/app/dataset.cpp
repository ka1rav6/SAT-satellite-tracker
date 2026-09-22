// app/dataset.cpp — write tracker state + FrameTruth angles for MotionNet.

#include "app/dataset.hpp"

#include "engine/pipeline.hpp"
#include "scenario/schema.hpp"
#include "scenario/sweep_spec.hpp"

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>

namespace sat {
namespace {

int32_t scenario_id_from_name(const std::string& name) noexcept {
    // Stable, process-local id. Splits key on (scenario_id, seed), not the
    // string, so a renamed file does not silently leak across splits.
    uint32_t h = 2166136261u;
    for (unsigned char c : name) {
        h ^= c;
        h *= 16777619u;
    }
    return static_cast<int32_t>(h & 0x7fffffff);
}

int dump_one_run(const Scenario& sc_in, const DatasetOptions& opt,
                 uint64_t seed, int32_t scenario_id) {
    Scenario sc = sc_in;
    sc.seed = seed;
    if (opt.duration_s > 0.0) sc.duration_s = opt.duration_s;
    sc.ai_enabled = false;  // factory must not depend on the model it trains

    // Skip the centering `constant` stack entry. The official row-12 kind is
    // the first moving component (linear / circular / lissajous / ou_noise).
    int regime = 0;
    if (!sc.targets.empty()) {
        for (const auto& m : sc.targets[0].motion) {
            if (m.kind == "constant") continue;
            regime = regime_from_kind(m.kind);
            break;
        }
    }

    Pipeline pipe;
    pipe.build_from_scenario(sc);
    pipe.set_publish_snapshots(false);
    const ScreenGeometry screen = pipe.config().synthetic.screen;

    const std::filesystem::path raw = std::filesystem::path(opt.out_dir) / "raw";
    std::error_code ec;
    std::filesystem::create_directories(raw, ec);
    if (ec) {
        std::fprintf(stderr, "sat-tracker: cannot create '%s': %s\n",
                     raw.string().c_str(), ec.message().c_str());
        return 1;
    }

    const std::string csv_path =
        (raw / (sc.name + "_" + std::to_string(seed) + ".csv")).string();
    std::ofstream out(csv_path, std::ios::out | std::ios::trunc);
    if (!out) {
        std::fprintf(stderr, "sat-tracker: cannot write '%s'\n", csv_path.c_str());
        return 1;
    }
    out << "frame,detected,"
           "track_az_urad,track_el_urad,track_vaz,track_vel,"
           "truth_az_urad,truth_el_urad,"
           "regime,scenario_id,seed\n";
    out.setf(std::ios::fmtflags(0), std::ios::floatfield);
    out.precision(9);

    int frames = 0;
    while (pipe.step()) {
        const FrameRecord& rec = pipe.last();
        const Angle2 truth = rec.truth_valid ? screen.to_angle(rec.truth_screen)
                                             : Angle2{};
        out << rec.frame << ','
            << (rec.detected ? 1 : 0) << ','
            << rec.estimate.x << ',' << rec.estimate.y << ','
            << rec.estimate_rate.x << ',' << rec.estimate_rate.y << ','
            << truth.x << ',' << truth.y << ','
            << regime << ',' << scenario_id << ',' << seed << '\n';
        ++frames;
    }
    if (frames == 0) {
        std::fprintf(stderr, "sat-tracker: --gen-dataset produced no frames for %s seed %llu\n",
                     sc.name.c_str(), static_cast<unsigned long long>(seed));
        return 1;
    }
    std::printf("wrote %d frames -> %s (regime=%d)\n", frames, csv_path.c_str(), regime);
    return 0;
}

}  // namespace

int regime_from_kind(const std::string& kind) noexcept {
    if (kind == "circular") return 1;
    if (kind == "lissajous") return 2;
    if (kind == "ou_noise") return 3;
    return 0;  // linear and every other stacked kind
}

int run_gen_dataset(const DatasetOptions& opt) {
    if (opt.task != "tracks") {
        std::fprintf(stderr,
                     "sat-tracker: --gen-dataset task '%s' is not implemented. "
                     "This branch ships tracks-only capture for MotionNet; "
                     "centroid/candidate factories remain listed in SAT-DESIGN §14.\n",
                     opt.task.c_str());
        return 2;
    }
    if (opt.out_dir.empty()) {
        std::fprintf(stderr, "sat-tracker: --gen-dataset needs --out\n");
        return 2;
    }

    if (!opt.sweep_path.empty()) {
        auto spec = load_sweep_spec(opt.sweep_path);
        if (!spec) {
            std::fprintf(stderr, "%s\n", spec.error().c_str());
            return 1;
        }
        auto loaded = load_scenario(spec->base_scenario);
        if (!loaded) {
            std::fprintf(stderr, "%s\n", loaded.error().c_str());
            return 1;
        }
        DatasetOptions one = opt;
        if (spec->duration_s > 0.0) one.duration_s = spec->duration_s;
        const int32_t sid = scenario_id_from_name(loaded->name);
        for (uint64_t seed : spec->seeds) {
            const int rc = dump_one_run(*loaded, one, seed, sid);
            if (rc != 0) return rc;
        }
        return 0;
    }

    if (opt.scenario_path.empty()) {
        std::fprintf(stderr, "sat-tracker: --gen-dataset needs --scenario or --sweep\n");
        return 2;
    }
    auto loaded = load_scenario(opt.scenario_path);
    if (!loaded) {
        std::fprintf(stderr, "%s\n", loaded.error().c_str());
        return 1;
    }
    std::vector<uint64_t> seeds = opt.seeds;
    if (seeds.empty()) seeds.push_back(loaded->seed);
    const int32_t sid = scenario_id_from_name(loaded->name);
    for (uint64_t seed : seeds) {
        const int rc = dump_one_run(*loaded, opt, seed, sid);
        if (rc != 0) return rc;
    }
    return 0;
}

int gen_dataset_command(int argc, char* argv[], int& i) {
    DatasetOptions opt;
    for (int k = i + 1; k < argc; ++k) {
        const char* a = argv[k];
        if (std::strcmp(a, "--scenario") == 0 && k + 1 < argc) {
            opt.scenario_path = argv[++k];
            i = k;
        } else if (std::strcmp(a, "--sweep") == 0 && k + 1 < argc) {
            opt.sweep_path = argv[++k];
            i = k;
        } else if (std::strcmp(a, "--out") == 0 && k + 1 < argc) {
            opt.out_dir = argv[++k];
            i = k;
        } else if (std::strcmp(a, "--task") == 0 && k + 1 < argc) {
            opt.task = argv[++k];
            i = k;
        } else if (std::strcmp(a, "--seed") == 0 && k + 1 < argc) {
            opt.seeds.push_back(static_cast<uint64_t>(std::atoll(argv[++k])));
            i = k;
        } else if (std::strcmp(a, "--duration") == 0 && k + 1 < argc) {
            opt.duration_s = std::atof(argv[++k]);
            i = k;
        } else {
            std::fprintf(stderr, "sat-tracker: --gen-dataset: unrecognised option '%s'\n", a);
            return 2;
        }
    }
    return run_gen_dataset(opt);
}

}  // namespace sat
