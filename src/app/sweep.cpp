// app/sweep.cpp

#include "app/sweep.hpp"

#include "sat/version.hpp"

#include "metrics/compliance.hpp"
#include "metrics/run_report.hpp"
#include "scenario/schema.hpp"
#include "scenario/sweep_spec.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>

namespace sat {

namespace {

/// Quote a path for the shell. Sweeps write into directories a user named, and
/// a space in one of them must not turn into two arguments.
std::string shell_quote(const std::string& s) {
#if defined(_WIN32)
    return "\"" + s + "\"";
#else
    std::string out = "'";
    for (char c : s) {
        if (c == '\'') out += "'\\''";
        else           out += c;
    }
    out += "'";
    return out;
#endif
}

/// The path of the running executable, so workers re-exec the SAME binary.
/// Taking it from argv[0] would break when the sweep is launched through a
/// symlink or from a different working directory, and a sweep that silently
/// measured a different build than the one that started it would be worse than
/// one that refused to run.
std::string self_path() {
    std::error_code ec;
#if defined(_WIN32)
    char buf[4096];
    const DWORD n = GetModuleFileNameA(nullptr, buf, sizeof buf);
    if (n > 0) return std::string(buf, n);
#else
    const auto p = std::filesystem::read_symlink("/proc/self/exe", ec);
    if (!ec) return p.string();
#endif
    return "sat-tracker";
}

}  // namespace

// ---------------------------------------------------------------------------
// expand — the cartesian product
// ---------------------------------------------------------------------------
std::vector<SweepRun> expand(const SweepSpec& spec, const std::string& out_dir) {
    std::vector<SweepRun> runs;

    // Start with one empty combination and multiply by each axis in turn. The
    // order matters only for readability: the LAST axis varies fastest, which
    // groups the matrix the way a person reads it.
    std::vector<std::vector<Override>> combos{{}};
    for (const SweepAxis& ax : spec.axes) {
        std::vector<std::vector<Override>> next;
        next.reserve(combos.size() * ax.values.size());
        for (const std::vector<Override>& c : combos) {
            for (const std::string& v : ax.values) {
                std::vector<Override> extended = c;
                extended.push_back(Override{ax.key, v});
                next.push_back(std::move(extended));
            }
        }
        combos.swap(next);
    }

    size_t index = 0;
    for (const std::vector<Override>& c : combos) {
        std::string label;
        for (size_t i = 0; i < c.size(); ++i) {
            if (i) label += ", ";
            label += c[i].key + "=" + c[i].value;
        }
        if (label.empty()) label = "base";

        for (uint64_t seed : spec.seeds) {
            SweepRun r;
            r.overrides = c;
            r.seed      = seed;
            r.label     = label;
            char dir[64];
            std::snprintf(dir, sizeof dir, "/run_%05zu", index++);
            r.dir = out_dir + dir;
            runs.push_back(std::move(r));
        }
    }
    return runs;
}

// ---------------------------------------------------------------------------
// run_sweep
// ---------------------------------------------------------------------------
int run_sweep(const SweepOptions& opt) {
    auto spec_r = load_sweep_spec(opt.spec_path);
    if (!spec_r) {
        std::fprintf(stderr, "%s\n", spec_r.error().c_str());
        return 1;
    }
    const SweepSpec spec = *spec_r;

    // Read the base scenario once. Every worker gets a COMPLETE, pre-resolved
    // scenario file rather than a base plus a list of overrides, so a run is
    // reproducible from its own directory alone — the run.json beside it
    // echoes the same configuration, and neither depends on the base file
    // still existing or still saying what it said.
    std::ifstream base_in(spec.base_scenario, std::ios::binary);
    if (!base_in) {
        std::fprintf(stderr, "sweep: cannot read base scenario '%s'\n",
                     spec.base_scenario.c_str());
        return 1;
    }
    const std::string base_text((std::istreambuf_iterator<char>(base_in)),
                                 std::istreambuf_iterator<char>());

    std::error_code ec;
    std::filesystem::create_directories(opt.out_dir, ec);
    if (ec) {
        std::fprintf(stderr, "sweep: cannot create '%s': %s\n",
                     opt.out_dir.c_str(), ec.message().c_str());
        return 1;
    }

    // -----------------------------------------------------------------------
    // Does every axis actually DO anything?
    //
    // A sweep that silently varies nothing is the worst possible outcome: it
    // produces a full matrix of identical results and looks perfectly healthy.
    // A mistyped key — `atmosphere.mod` — parses as valid TOML, is ignored by
    // the loader, and yields exactly that.
    //
    // The first version of this check looked the key up in scenario::schema().
    // That was wrong, and the sweep's own first run caught it: the schema table
    // holds NUMERIC RANGE checks, while string enums like atmosphere.mode are
    // validated in the loader against their allowed values. Every valid enum
    // key was rejected as a typo.
    //
    // So the check is behavioural instead of a registry lookup: apply the
    // override and see whether the parsed Scenario changes. That cannot drift
    // out of step with the loader, because it IS the loader, and it catches a
    // key that is inert for any reason rather than only for one.
    // -----------------------------------------------------------------------
    {
        auto base_sc = parse_scenario(base_text, spec.base_scenario);
        if (!base_sc) {
            std::fprintf(stderr, "sweep: the base scenario is not valid:\n%s\n",
                         base_sc.error().c_str());
            return 1;
        }
        const std::string base_json = scenario_json_text(*base_sc);
        for (const SweepAxis& ax : spec.axes) {
            bool moved = false;
            for (const std::string& v : ax.values) {
                auto text = apply_overrides(base_text, {Override{ax.key, v}});
                if (!text) continue;
                auto sc = parse_scenario(*text, "<axis probe>");
                if (sc && scenario_json_text(*sc) != base_json) { moved = true; break; }
            }
            if (!moved) {
                std::fprintf(stderr,
                    "sweep: axis '%s' does not change the scenario at any of its "
                    "values.\n  Either the key is mistyped, or every value equals "
                    "the base scenario's.\n  A sweep over an inert axis produces a "
                    "matrix of identical rows and looks fine.\n", ax.key.c_str());
                return 1;
            }
        }
    }

    std::vector<SweepRun> runs = expand(spec, opt.out_dir);
    const int jobs = (opt.jobs > 0)
        ? opt.jobs
        : static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));

    if (!opt.quiet) {
        std::printf("sweep: %zu runs (%zu combinations x %zu seeds) on %d workers\n",
                    runs.size(), runs.size() / std::max<size_t>(1, spec.seeds.size()),
                    spec.seeds.size(), jobs);
    }

    // --- write every run's scenario up front -------------------------------
    // Before spawning anything, so a bad override fails the sweep immediately
    // rather than after eight minutes of workers each failing the same way.
    for (SweepRun& r : runs) {
        std::vector<Override> ov = r.overrides;
        ov.push_back(Override{"sim.seed", std::to_string(r.seed)});
        if (spec.duration_s > 0.0) {
            ov.push_back(Override{"sim.duration_s", std::to_string(spec.duration_s)});
        }
        auto text = apply_overrides(base_text, ov);
        if (!text) {
            std::fprintf(stderr, "%s\n", text.error().c_str());
            return 1;
        }
        // Parse it here too, so a value that is legal TOML but out of spec is
        // reported once, by §7.5's error format, with the offending key named.
        auto check = parse_scenario(*text, r.dir + "/scenario.toml");
        if (!check) {
            std::fprintf(stderr, "sweep: run '%s' seed %llu is not a valid scenario:\n%s\n",
                         r.label.c_str(), static_cast<unsigned long long>(r.seed),
                         check.error().c_str());
            return 1;
        }
        std::filesystem::create_directories(r.dir, ec);
        std::ofstream out(r.dir + "/scenario.toml", std::ios::binary);
        if (!out) {
            std::fprintf(stderr, "sweep: cannot write into '%s'\n", r.dir.c_str());
            return 1;
        }
        out << *text;
    }

    // --- spawn workers -----------------------------------------------------
    const std::string exe = self_path();
    std::atomic<size_t> next{0};
    std::atomic<size_t> done{0};
    std::mutex print_mu;

    auto worker = [&]() {
        for (;;) {
            const size_t i = next.fetch_add(1);
            if (i >= runs.size()) return;
            SweepRun& r = runs[i];

            std::string cmd = shell_quote(exe) + " --headless --quiet"
                            + " --scenario " + shell_quote(r.dir + "/scenario.toml")
                            + " --out " + shell_quote(r.dir);
            if (spec.no_ai) cmd += " --no-ai";
            // A sweep wants the metrics, not 500 CSVs of ~2 MB each.
            if (!opt.keep_csv) cmd += " --no-csv";
#if !defined(_WIN32)
            cmd += " >/dev/null 2>&1";
#else
            cmd += " >NUL 2>&1";
#endif
            r.exit_code = std::system(cmd.c_str());
            // A non-zero exit, a signal, or no run.json at all: the run failed
            // and is REPORTED as failed rather than silently omitted. A sweep
            // that quietly drops the configurations that crashed reports the
            // average of the ones that survived, which is exactly backwards.
            //
            // And the REASON is kept, not just the count. The first real sweep
            // reported "95 failed" and nothing else, which meant an hour of
            // bisecting to discover the runs were fine and the machine had
            // been oversubscribed. A failure count with no cause attached is
            // barely better than no report.
            if (r.exit_code != 0) {
                char b[128];
                std::snprintf(b, sizeof b, "worker exited %d", r.exit_code);
                r.failure = b;
            } else {
                std::ifstream in(r.dir + "/run.json");
                if (in) r.ok = true;
                else    r.failure = "worker exited 0 but wrote no run.json";
            }
            const size_t n = done.fetch_add(1) + 1;
            if (!opt.quiet && (n % 25 == 0 || n == runs.size())) {
                std::lock_guard<std::mutex> lk(print_mu);
                std::printf("  %zu / %zu\n", n, runs.size());
                std::fflush(stdout);
            }
        }
    };

    std::vector<std::thread> pool;
    pool.reserve(static_cast<size_t>(jobs));
    for (int k = 0; k < jobs; ++k) pool.emplace_back(worker);
    for (std::thread& t : pool) t.join();

    // --- collect -----------------------------------------------------------
    size_t failed = 0;
    for (SweepRun& r : runs) {
        if (!r.ok) { ++failed; continue; }
        std::ifstream in(r.dir + "/run.json", std::ios::binary);
        const std::string text((std::istreambuf_iterator<char>(in)),
                                std::istreambuf_iterator<char>());
        auto m = metrics_from_json(text);
        if (!m) { r.ok = false; r.failure = m.error(); ++failed; continue; }
        r.metrics = *m;
    }

    // Group by condition for §13.3's per-condition breakout. The grouping key
    // is the override label — everything except the seed — so each row of the
    // breakout is a distribution over seeds at one configuration, which is the
    // only way a p95 in it means anything.
    std::vector<RunMetrics>     all;
    std::vector<ConditionGroup> groups;
    for (const SweepRun& r : runs) {
        if (!r.ok) continue;
        all.push_back(r.metrics);
        auto it = std::find_if(groups.begin(), groups.end(),
                               [&](const ConditionGroup& g) { return g.label == r.label; });
        if (it == groups.end()) {
            groups.push_back(ConditionGroup{r.label, {r.metrics}});
        } else {
            it->runs.push_back(r.metrics);
        }
    }

    // The requirements come from the BASE scenario's [requirements] block, so
    // a scenario that states a stricter budget is checked against its own.
    Requirements req;
    if (auto base = parse_scenario(base_text, spec.base_scenario)) {
        req.acquisition_s     = base->acquisition_s;
        req.tracking_error_px = base->tracking_error_px;
        req.target_loss_frac  = base->target_loss_frac;
        req.reacquisition_s   = base->reacquisition_s;
        req.min_fps           = base->min_fps;

        // The derived floor on row 17, from the base scenario's own jitter.
        // Computed here rather than assumed, so a scenario with the
        // disturbance disabled gets no floor and is judged against row 17
        // directly. See Requirements::tracking_floor_px for the derivation.
        const double a = base->jitter_px_per_frame;
        req.tracking_floor_px = (a > 0.0) ? std::sqrt(2.0 * a * a / 3.0) : 0.0;
    }

    char prov[512];
    std::snprintf(prov, sizeof prov,
                  "%zu runs - %zu conditions x %zu seeds - build %s - ai %s",
                  all.size(), groups.size(), spec.seeds.size(), SAT_GIT_HASH,
                  spec.no_ai ? "disabled" : "enabled");
    const std::string matrix = compliance_matrix(all, groups, req, prov);

    if (!opt.quiet) {
        std::printf("\n%zu of %zu runs completed", runs.size() - failed, runs.size());
        if (failed) std::printf("  (%zu FAILED)", failed);
        std::printf("\n");
    }
    if (failed) {
        // Grouped by reason, with one command that reproduces each. Printed to
        // stderr and ALWAYS, even under --quiet: a quiet sweep still has to say
        // that part of its input is missing, or the matrix above it is being
        // read as if it covered everything.
        std::map<std::string, std::pair<size_t, const SweepRun*>> why;
        for (const SweepRun& r : runs) {
            if (r.ok) continue;
            auto& e = why[r.failure];
            e.first++;
            if (!e.second) e.second = &r;
        }
        std::fprintf(stderr, "\n%zu runs failed:\n", failed);
        for (const auto& [reason, info] : why) {
            std::fprintf(stderr, "  %5zu x  %s\n", info.first, reason.c_str());
            std::fprintf(stderr, "           reproduce: %s --headless --scenario %s --out %s\n",
                         exe.c_str(), (info.second->dir + "/scenario.toml").c_str(),
                         info.second->dir.c_str());
        }
        std::fprintf(stderr,
            "\n  The matrix below covers only the %zu runs that completed.\n\n",
            runs.size() - failed);
    }
    if (!opt.quiet) std::printf("\n%s", matrix.c_str());

    // Written beside the runs, so the matrix is an artifact rather than
    // something that scrolled past.
    std::ofstream mat(opt.out_dir + "/compliance.txt", std::ios::binary);
    if (mat) mat << matrix;

    // A non-zero exit when any run failed. A sweep whose workers crashed and
    // which still exits 0 will be believed by CI.
    return failed ? 2 : 0;
}

int sweep_command(int argc, char* argv[], int& i) {
    SweepOptions opt;
    if (i + 1 >= argc) {
        std::fprintf(stderr, "sat-tracker: --sweep needs a sweep .toml file\n");
        return 2;
    }
    opt.spec_path = argv[++i];
    for (int k = i + 1; k < argc; ++k) {
        const char* a = argv[k];
        if (std::strcmp(a, "--out") == 0 && k + 1 < argc)       opt.out_dir = argv[++k];
        else if (std::strcmp(a, "--jobs") == 0 && k + 1 < argc) opt.jobs = std::atoi(argv[++k]);
        else if (std::strcmp(a, "--quiet") == 0)                opt.quiet = true;
        else if (std::strcmp(a, "--keep-csv") == 0)             opt.keep_csv = true;
        else {
            std::fprintf(stderr, "sat-tracker: --sweep: unrecognised option '%s'\n", a);
            return 2;
        }
        i = k;
    }
    return run_sweep(opt);
}

}  // namespace sat
