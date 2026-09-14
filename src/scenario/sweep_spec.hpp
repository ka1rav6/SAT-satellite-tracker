// scenario/sweep_spec.hpp — what a sweep.toml describes (CP 7.5).
//
// ---------------------------------------------------------------------------
// WHY THIS IS IN scenario/ AND NOT IN app/ WITH THE REST OF THE SWEEP
// ---------------------------------------------------------------------------
// It was written in app/sweep.cpp first, and the build rejected it: toml++ is
// linked PRIVATE to sat_scenario on purpose, so that exactly one module in the
// project knows what TOML is. Parsing a config file in app/ would have meant
// making it public and giving every future caller a second place to parse
// configuration from — which is how two subtly different parsers end up
// disagreeing about the same file.
//
// The split is also the right one on its own merits. A sweep.toml is a
// configuration document and belongs with the code that validates
// configuration; spawning worker processes and aggregating their output is an
// application concern. app/sweep.cpp does the second and nothing else.

#pragma once

#include "core/result.hpp"
#include "scenario/overlay.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace sat {

// ---------------------------------------------------------------------------
// SweepSpec — what sweep.toml describes.
// ---------------------------------------------------------------------------
struct SweepAxis {
    std::string key;                  ///< dotted scenario key, e.g. "atmosphere.mode"
    std::vector<std::string> values;  ///< TOML source text, one per point
};

struct SweepSpec {
    std::string base_scenario;        ///< path to the scenario to vary
    std::vector<SweepAxis> axes;      ///< the cartesian product
    std::vector<uint64_t>  seeds;     ///< every combination is run at every seed
    double duration_s = 0.0;          ///< override; 0 keeps the scenario's
    bool   no_ai      = false;

    /// How many runs this expands to.
    [[nodiscard]] size_t run_count() const noexcept {
        size_t combos = 1;
        for (const SweepAxis& a : axes) combos *= a.values.size();
        return combos * seeds.size();
    }
};

[[nodiscard]] Result<SweepSpec> load_sweep_spec(const std::string& path);

/// Parse from text, for tests and for anything that already has the document.
[[nodiscard]] Result<SweepSpec> parse_sweep_spec(std::string_view toml_text,
                                                 std::string_view name = "<memory>");

}  // namespace sat
