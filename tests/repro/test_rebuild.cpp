// tests/repro/test_rebuild.cpp — INV-3 across a REBUILD, not just across two
// processes.
//
// ---------------------------------------------------------------------------
// THE GAP THIS FILE EXISTS TO CLOSE
// ---------------------------------------------------------------------------
// Every other reproducibility test in this repository — test_fingerprint.cpp,
// test_ablation.cpp's digest pair, --verify-reproducibility — compares two runs
// made by two FRESH Pipeline objects:
//
//     { Pipeline p; p.build_from_scenario(sc); ... }      // a
//     { Pipeline p; p.build_from_scenario(sc); ... }      // b
//
// That is the headless shape, and it was the only shape anything checked. The
// GUI's Reset button is a different shape: Dashboard::rebuild() calls
// build_from_scenario on the SAME Pipeline, because the dashboard owns exactly
// one and the window, the triple buffer and the preview texture are all bound
// to it. So Reset is "start this run over on the object that just finished a
// run", and nothing tested that the second run is the first one.
//
// It was not. Pressing Reset and then Play produced a visibly different run
// every time, which is the opposite of the property INV-3 is supposed to
// guarantee and made the GUI useless as a way to compare two settings. The
// cause was per-run state in SyntheticSource that build() never cleared, and
// `sim_time_s_` above all: §7.2's motion algebra evaluates positions at
// ABSOLUTE time, so a second run that starts at t = 20 s puts the beacon on a
// completely different phase of its motion while every RNG stream is correctly
// reseeded and the world is correctly rebuilt. Nothing looked wrong anywhere.
//
// The test is written over EVERY scenario in scenarios/ rather than one,
// because the state that leaks is per-feature: a scenario with no turbulence
// cannot catch a turbulence filter that is not reset, and one with no platform
// motion cannot catch a stale disturbance phase.

#include <doctest/doctest.h>

#include "engine/pipeline.hpp"
#include "scenario/schema.hpp"
#include "core/profile.hpp"

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

using namespace sat;

namespace {

/// Run `sc` to completion on `p` and return the per-frame fingerprints.
/// Copied out, because the next run overwrites the pipeline's own vector.
std::vector<FrameFingerprint> run(Pipeline& p, const Scenario& sc) {
    p.build_from_scenario(sc);
    while (p.step()) {}
    return p.fingerprints();
}

}  // namespace

// ---------------------------------------------------------------------------
// The GUI's Reset button, in one assertion.
// ---------------------------------------------------------------------------
TEST_CASE("INV-3: rebuilding the same Pipeline reproduces the run exactly") {
    std::vector<std::string> files;
    for (const auto& e :
         std::filesystem::recursive_directory_iterator(SAT_SCENARIO_DIR)) {
        if (e.path().extension() == ".toml") files.push_back(e.path().string());
    }
    // Directory iteration order is filesystem-defined; sorting keeps a failure
    // reproducible and the output readable.
    std::sort(files.begin(), files.end());
    REQUIRE(!files.empty());

    for (const std::string& f : files) {
        auto loaded = load_scenario(f);
        // A sweep spec is not a scenario. Skipping what the loader rejects is
        // correct here: scenarios/ holds both, and the schema is the authority
        // on which is which.
        if (!loaded) continue;

        Scenario sc = *loaded;
        sc.duration_s = std::min(sc.duration_s, 2.0);   // keep the suite quick

        INFO("scenario = " << f);

        // ONE pipeline, two runs. This is the whole point — a fresh Pipeline
        // per run is what every other test already does and it passes there.
        Pipeline p;
        const std::vector<FrameFingerprint> a = run(p, sc);
        const std::vector<FrameFingerprint> b = run(p, sc);

        REQUIRE(a.size() == b.size());
        REQUIRE(!a.empty());

        // Compared component by component rather than on combined(), so a
        // failure names WHICH part of the frame diverged — the image, the
        // pointing, the detection, the filter or the FSM — which is the same
        // reason verify_repro.cpp splits the fingerprint.
        for (size_t i = 0; i < a.size(); ++i) {
            INFO("frame = " << i);
            CHECK(a[i].image     == b[i].image);
            CHECK(a[i].boresight == b[i].boresight);
            CHECK(a[i].detection == b[i].detection);
            CHECK(a[i].track     == b[i].track);
            CHECK(a[i].mode      == b[i].mode);
        }
    }
}

// ---------------------------------------------------------------------------
// And the same property against a fresh object, so that a failure above can be
// attributed to the REBUILD rather than to the scenario being non-deterministic
// in the first place.
// ---------------------------------------------------------------------------
TEST_CASE("INV-3: a rebuilt run matches a freshly constructed one") {
    auto loaded = load_scenario(std::string(SAT_SCENARIO_DIR) + "/fog_figure8.toml");
    REQUIRE_MESSAGE(loaded.has_value(), loaded.error());
    Scenario sc = *loaded;
    sc.duration_s = 2.0;

    // `warm` runs something else first, so anything it leaves behind has a
    // chance to contaminate the run being compared. A different seed and a
    // different duration make the leak as large as possible.
    Scenario other = sc;
    other.seed       = sc.seed + 7;
    other.duration_s = 3.0;

    Pipeline warm;
    (void)run(warm, other);
    const std::vector<FrameFingerprint> rebuilt = run(warm, sc);

    Pipeline cold;
    const std::vector<FrameFingerprint> fresh = run(cold, sc);

    REQUIRE(rebuilt.size() == fresh.size());
    for (size_t i = 0; i < fresh.size(); ++i) {
        INFO("frame = " << i);
        CHECK(rebuilt[i].combined() == fresh[i].combined());
    }
}

// ---------------------------------------------------------------------------
// The state a rebuild has to clear that is not part of the fingerprint.
//
// The fingerprint covers what a frame LOOKS like, which is what the two cases
// above compare. It does not cover what the run REPORTS about itself, and the
// GUI's Reset button resets both or neither: a demo that shows the frame-time
// percentiles of three pooled runs after two Resets is as wrong as one that
// shows a different beacon.
// ---------------------------------------------------------------------------
TEST_CASE("rebuilding clears the run's accumulated reports") {
    auto loaded = load_scenario(std::string(SAT_SCENARIO_DIR) + "/spec_defaults.toml");
    REQUIRE_MESSAGE(loaded.has_value(), loaded.error());
    Scenario sc = *loaded;
    sc.duration_s = 1.0;
    const auto expected = static_cast<uint64_t>(sc.duration_s * sc.camera_hz);

    Pipeline p;
    p.build_from_scenario(sc);
    while (p.step()) {}
    // The run's own count, whatever it is — the zone also covers the final
    // step() that reports the end of the run, so it is expected + 1 and the
    // exact figure is not what this test is about.
    const uint64_t first = p.timers()[Stage::FrameTotal].count();
    REQUIRE(first >= expected);

    // Design 15's stage table and every SPEED block read these histograms, and
    // they accumulate for the life of the object. Without the reset the second
    // run reported first + first samples — the pooled timings of both runs.
    p.build_from_scenario(sc);
    CHECK(p.timers()[Stage::FrameTotal].count() == 0u);
    while (p.step()) {}
    CHECK(p.timers()[Stage::FrameTotal].count() == first);
}

TEST_CASE("rebuilding clears the deadline ladder") {
    auto loaded = load_scenario(std::string(SAT_SCENARIO_DIR) + "/spec_defaults.toml");
    REQUIRE_MESSAGE(loaded.has_value(), loaded.error());
    Scenario sc = *loaded;
    sc.duration_s = 1.0;

    Pipeline p;
    p.build_from_scenario(sc);
    // One injected stall, late enough that the track is confirmed (the ladder
    // only climbs while there is a lock to protect) and late enough that the
    // eight comfortable frames recovery needs do not fit before the run ends.
    // Injected mode, so the clock is never read and this is the same on any
    // machine.
    p.set_deadline(Pipeline::DeadlineMode::Injected, 1000.0);
    p.inject_stall(25, 1.0e6);
    while (p.step()) {}
    REQUIRE(p.deadline().level() > 0);

    // Starting a run over must not begin it already shedding. The budget and
    // the mode are a run OPTION the caller set, so those stay.
    p.build_from_scenario(sc);
    CHECK(p.deadline().level() == 0);
    CHECK(p.deadline_mode() == Pipeline::DeadlineMode::Injected);
}
