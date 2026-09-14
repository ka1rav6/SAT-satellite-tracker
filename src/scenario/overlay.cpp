// scenario/overlay.cpp

#include "scenario/overlay.hpp"

#include <toml++/toml.hpp>

#include <sstream>

namespace sat {

namespace {

/// Split "a.b.c" into its segments.
std::vector<std::string> split_path(std::string_view p) {
    std::vector<std::string> out;
    size_t start = 0;
    while (start <= p.size()) {
        const size_t dot = p.find('.', start);
        if (dot == std::string_view::npos) {
            out.emplace_back(p.substr(start));
            break;
        }
        out.emplace_back(p.substr(start, dot - start));
        start = dot + 1;
    }
    return out;
}

}  // namespace

Result<std::string> apply_overrides(std::string_view toml_text,
                                    const std::vector<Override>& ov) {
    toml::table doc;
    try {
        doc = toml::parse(toml_text);
    } catch (const toml::parse_error& e) {
        std::ostringstream os;
        os << "sweep: the base scenario does not parse: " << e.description()
           << " at line " << e.source().begin.line;
        return Err(os.str());
    }

    for (const Override& o : ov) {
        const std::vector<std::string> path = split_path(o.key);
        if (path.empty() || path.back().empty()) {
            return Err("sweep: empty override key");
        }

        // Parse the value on its own, as a one-key document. This is what makes
        // the value's TYPE come from TOML's own rules rather than from a guess
        // in this file: `true` is a bool, `"fog"` is a string, `0.1` is a
        // float, `[1, 2]` is an array, all without a single branch here.
        toml::table holder;
        try {
            holder = toml::parse("v = " + o.value);
        } catch (const toml::parse_error& e) {
            std::ostringstream os;
            os << "sweep: override '" << o.key << " = " << o.value
               << "' is not valid TOML: " << e.description();
            return Err(os.str());
        }

        // Walk to the parent table, creating tables that do not exist yet.
        toml::table* cur = &doc;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            toml::node* child = cur->get(path[i]);
            if (!child) {
                cur->insert(path[i], toml::table{});
                child = cur->get(path[i]);
            }
            toml::table* as_table = child->as_table();
            if (!as_table) {
                return Err(
                    "sweep: override key '" + o.key + "' runs through '" + path[i] +
                    "', which is not a table in the base scenario");
            }
            cur = as_table;
        }

        cur->insert_or_assign(path.back(), *holder.get("v"));
    }

    std::ostringstream os;
    os << doc;
    return Ok(os.str());
}

}  // namespace sat
