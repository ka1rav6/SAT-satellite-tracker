// scenario/overlay.cpp

#include "scenario/overlay.hpp"

#include "scenario/schema.hpp"

#include <toml++/toml.hpp>

#include <algorithm>
#include <sstream>
#include <vector>

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

/// Would `v` be read by a human as a bare word — a name, not a number?
///
/// Deliberately narrow: ASCII letters, digits, underscore and hyphen, starting
/// with a letter. That covers every enum spelling in the schema (`lowlight`,
/// `figure_of_eight`, `salt-pepper`) and nothing else. It is checked only
/// AFTER toml++ has already refused the value, so `true`, `inf`, `nan` and
/// every numeric form have been taken by their own rules first and never
/// reach here.
bool is_bare_word(std::string_view v) {
    if (v.empty()) return false;
    if (!((v[0] >= 'a' && v[0] <= 'z') || (v[0] >= 'A' && v[0] <= 'Z'))) return false;
    for (char c : v) {
        // Dot and slash are included so `--set ai.motion_net=models/foo.onnx`
        // is a string. A path is not valid TOML on the right of `=`, same as
        // a bare word, so quoting it cannot change a value that already parsed.
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
                     || (c >= '0' && c <= '9') || c == '_' || c == '-'
                     || c == '.' || c == '/' || c == '\\';
        if (!ok) return false;
    }
    return true;
}

/// Edit distance, capped — only used to suggest a nearer key in an error.
size_t edit_distance(std::string_view a, std::string_view b) {
    std::vector<size_t> prev(b.size() + 1), cur(b.size() + 1);
    for (size_t j = 0; j <= b.size(); ++j) prev[j] = j;
    for (size_t i = 1; i <= a.size(); ++i) {
        cur[0] = i;
        for (size_t j = 1; j <= b.size(); ++j) {
            const size_t sub = prev[j - 1] + (a[i - 1] == b[j - 1] ? 0 : 1);
            cur[j] = std::min({cur[j - 1] + 1, prev[j] + 1, sub});
        }
        prev = cur;
    }
    return prev[b.size()];
}

/// The closest schema key to `key`, or empty when nothing is close enough.
///
/// A suggestion that is not actually similar is worse than none: it sends the
/// reader off to check a key they never meant. The threshold is a third of the
/// key's length, so `control.kpp` finds `control.kp` and `this.is.not.a.key`
/// finds nothing.
std::string nearest_key(std::string_view key) {
    size_t best = key.size() / 3 + 1;
    std::string out;
    for (const FieldSpec& f : schema()) {
        const size_t d = edit_distance(key, f.path);
        if (d < best) { best = d; out = f.path; }
    }
    return out;
}

}  // namespace

Result<void> check_override_keys(const std::vector<Override>& ov,
                                 std::string_view context) {
    const std::string pre(context);
    for (const Override& o : ov) {
        if (find_field(o.key) != nullptr) continue;

        std::ostringstream os;
        os << pre << ": '" << o.key << "' is not a scenario key, so setting "
           << "it would change nothing";
        const std::string near = nearest_key(o.key);
        if (!near.empty()) {
            os << "\n  Did you mean '" << near << "'?";
        }
        if (o.key.find('[') != std::string::npos) {
            os << "\n  Keys inside an array of tables — [[target.motion]],"
                  " [[disturbance.platform]], [[event]] — cannot be set this"
                  " way.\n  Copy the scenario and edit the stack there;"
                  " docs/GUIDE.md section 12 has the recipe.";
        }
        return Err(os.str());
    }
    return Ok();
}

Result<std::string> apply_overrides(std::string_view toml_text,
                                    const std::vector<Override>& ov,
                                    std::string_view context) {
    const std::string pre(context);
    toml::table doc;
    try {
        doc = toml::parse(toml_text);
    } catch (const toml::parse_error& e) {
        std::ostringstream os;
        os << pre << ": the base scenario does not parse: " << e.description()
           << " at line " << e.source().begin.line;
        return Err(os.str());
    }

    for (const Override& o : ov) {
        const std::vector<std::string> path = split_path(o.key);
        if (path.empty() || path.back().empty()) {
            return Err(pre + ": empty override key");
        }


        // Parse the value on its own, as a one-key document. This is what makes
        // the value's TYPE come from TOML's own rules rather than from a guess
        // in this file: `true` is a bool, `"fog"` is a string, `0.1` is a
        // float, `[1, 2]` is an array, all without a single branch here.
        // ------------------------------------------------------------------
        // Parse the value on its own, as a one-key document. This is what
        // makes the value's TYPE come from TOML's own rules rather than from a
        // guess in this file: `true` is a bool, `"fog"` is a string, `0.1` is
        // a float, `[1, 2]` is an array, all without a single branch here.
        //
        // A-6: if that fails and the value reads as a bare word, try it again
        // quoted. `--set atmosphere.mode=lowlight` is the command a judge is
        // most likely to type live, and a bare word is not valid TOML on the
        // right of an assignment under any reading — so there is nothing for
        // the retry to be ambiguous with, and nothing that used to work can
        // change meaning. The schema still decides whether the STRING is a
        // legal value for that key.
        // ------------------------------------------------------------------
        toml::table holder;
        bool parsed = false;
        try {
            holder = toml::parse("v = " + o.value);
            parsed = true;
        } catch (const toml::parse_error&) {
            // fall through to the bare-word retry
        }
        if (!parsed && is_bare_word(o.value)) {
            try {
                std::string escaped;
                escaped.reserve(o.value.size());
                for (char c : o.value) {
                    if (c == '\\' || c == '"') escaped.push_back('\\');
                    escaped.push_back(c);
                }
                holder = toml::parse("v = \"" + escaped + "\"");
                parsed = true;
            } catch (const toml::parse_error&) {
                // fall through to the error below
            }
        }
        if (!parsed) {
            // Re-parse once more purely to recover toml++'s own description
            // for the message; the value is known bad by this point.
            std::string why = "could not determine value type";
            try {
                holder = toml::parse("v = " + o.value);
            } catch (const toml::parse_error& e) {
                why = e.description();
            }
            std::ostringstream os;
            os << pre << ": override '" << o.key << " = " << o.value
               << "' is not valid TOML: " << why;
            // Offer the quoting fix only when quoting could plausibly BE the
            // fix. Suggesting --set 'sim.seed="[1,"' for a truncated array is
            // worse than saying nothing: it is confidently wrong advice, and
            // the value's first character already rules it out.
            const char c0 = o.value.empty() ? '\0' : o.value[0];
            const bool looks_structural =
                c0 == '[' || c0 == '{' || c0 == '"' || c0 == '\'' ||
                (c0 >= '0' && c0 <= '9') || c0 == '-' || c0 == '+';
            if (!looks_structural) {
                os << "\n  Values are TOML source, so a string needs quotes:"
                   << "\n      " << pre << " '" << o.key << "=\"" << o.value << "\"'";
            }
            os << "\n  Numbers, booleans and arrays do not need them: "
                  "sim.seed=7, control.smith=true, camera.fov_deg=[8,6]";
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
                // Was hardcoded to "sweep: " regardless of context, so the
                // same defect reported through --headless blamed a sweep that
                // was not running. Same class as audit A-6.
                return Err(
                    pre + ": override key '" + o.key + "' runs through '" + path[i] +
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
