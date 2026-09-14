// scenario/sweep_spec.cpp

#include "scenario/sweep_spec.hpp"

#include <toml++/toml.hpp>

#include <cstdio>
#include <fstream>
#include <sstream>

namespace sat {

Result<SweepSpec> load_sweep_spec(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return Err("sweep: cannot read '" + path + "'");
    const std::string text((std::istreambuf_iterator<char>(in)),
                            std::istreambuf_iterator<char>());
    return parse_sweep_spec(text, path);
}

Result<SweepSpec> parse_sweep_spec(std::string_view toml_text, std::string_view name) {
    const std::string path(name);
    toml::table doc;
    try {
        doc = toml::parse(toml_text);
    } catch (const toml::parse_error& e) {
        std::ostringstream os;
        os << path << ":" << e.source().begin.line << ": " << e.description();
        return Err(os.str());
    }

    SweepSpec spec;
    const toml::table* sw = doc["sweep"].as_table();
    if (!sw) return Err(path + ": missing [sweep] table");

    if (auto v = (*sw)["base"].value<std::string>()) spec.base_scenario = *v;
    else return Err(path + ": [sweep].base must name the scenario to vary");

    if (auto v = (*sw)["duration_s"].value<double>()) spec.duration_s = *v;
    if (auto v = (*sw)["no_ai"].value<bool>())        spec.no_ai = *v;

    // Seeds: either an explicit list or a count, which means 1..n. A count is
    // what a 500-run sweep actually wants and spelling out 50 integers in a
    // config file is how typos get in.
    if (const toml::array* arr = (*sw)["seeds"].as_array()) {
        for (const auto& e : *arr) {
            if (auto v = e.value<int64_t>()) spec.seeds.push_back(static_cast<uint64_t>(*v));
        }
    } else if (auto n = (*sw)["seed_count"].value<int64_t>()) {
        for (int64_t k = 1; k <= *n; ++k) spec.seeds.push_back(static_cast<uint64_t>(k));
    }
    if (spec.seeds.empty()) spec.seeds.push_back(1);

    if (const toml::array* axes = doc["axis"].as_array()) {
        for (const auto& node : *axes) {
            const toml::table* t = node.as_table();
            if (!t) continue;
            SweepAxis ax;
            if (auto k = (*t)["key"].value<std::string>()) ax.key = *k;
            else return Err(path + ": every [[axis]] needs a key");
            const toml::array* vals = (*t)["values"].as_array();
            if (!vals || vals->empty()) {
                return Err(path + ": axis '" + ax.key + "' has no values");
            }
            for (const auto& v : *vals) {
                // Re-serialise each value as TOML source, so the overlay can
                // insert it verbatim and TOML's own rules decide its type. A
                // string has to be re-quoted: `os << node` prints `fog`, which
                // is not valid TOML on the right-hand side of an assignment,
                // and the overlay would then fail on every string axis.
                std::ostringstream os;
                if (auto str = v.value<std::string>()) {
                    os << toml::value<std::string>(*str);
                } else if (auto d = v.value<double>()) {
                    // Shortest round-tripping form. toml++'s default prints
                    // 0.1 as 0.10000000000000001, which is correct and which
                    // then appears verbatim as a row label in the compliance
                    // matrix — the one place in the project most likely to be
                    // read by someone else.
                    char b[64];
                    std::snprintf(b, sizeof b, "%.10g", *d);
                    os << b;
                } else {
                    v.visit([&os](const auto& n) {
                        if constexpr (toml::is_value<decltype(n)>) os << n;
                    });
                }
                ax.values.push_back(os.str());
            }
            spec.axes.push_back(std::move(ax));
        }
    }
    return Ok(std::move(spec));
}

}  // namespace sat
