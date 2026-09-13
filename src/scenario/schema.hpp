// scenario/schema.hpp — validation that cites the specification.
//
// Design §7.5 fixes the error format, and the requirement is unusual enough to
// be worth restating:
//
//     scenarios/bad.toml:41: gimbal.max_pan_dps = 14.0 is outside the permitted
//       range [5.0, 10.0] (specification row 13).
//
// Four things in one line: the file, the LINE, the offending value, and the
// specification row that makes it wrong.
//
// ---------------------------------------------------------------------------
// WHY THE SPEC ROW IS THE IMPORTANT PART
// ---------------------------------------------------------------------------
// The evaluators supply the scenarios (§3.1). When one of them is rejected, the
// useful response is not "invalid config" — it is a sentence that lets someone
// holding the specification see immediately that the file asks for something the
// specification does not permit. Anything less turns a correct rejection into an
// argument.
//
// It also works in our favour during development: CP 14.1 fuzzes 5000 scenarios
// across every parameter's legal range, and a rejection that names the row makes
// it obvious whether the fuzzer found a real bug or simply generated something
// out of spec.

#pragma once

#include "core/result.hpp"
#include "scenario/scenario.hpp"

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace sat {

// ---------------------------------------------------------------------------
// FieldSpec — one row of the schema table.
// ---------------------------------------------------------------------------
enum class ValueKind : uint8_t { Int, Float, Bool, String, Array2, ArrayN, Table };

struct FieldSpec {
    const char* path;       ///< dotted TOML path, e.g. "gimbal.max_pan_dps"
    ValueKind   kind;
    bool        required;
    double      min;        ///< inclusive; -inf when unbounded
    double      max;        ///< inclusive; +inf when unbounded
    const char* spec_row;   ///< "row 13", or "" for keys the spec does not name
    const char* note;       ///< extra context appended to an error
};

/// The whole schema. Defined in schema.cpp so the table lives in one place and
/// can be walked for documentation generation (the user manual cross-references
/// every spec row, CP 15.4).
[[nodiscard]] const std::vector<FieldSpec>& schema();

/// Look up one field, or nullptr.
[[nodiscard]] const FieldSpec* find_field(std::string_view path);

// ---------------------------------------------------------------------------
// ValidationError — one problem, with everything needed to report it.
// ---------------------------------------------------------------------------
struct ValidationError {
    std::string file;
    int         line = 0;      ///< 0 when the problem is not tied to a line
    std::string path;          ///< the TOML key
    std::string message;       ///< already formatted, spec row included

    /// The §7.5 rendering:  file:line: message
    [[nodiscard]] std::string format() const;
};

// ---------------------------------------------------------------------------
// Validator — accumulates errors rather than stopping at the first.
//
// Reporting every problem at once matters here: someone handed a rejected
// scenario should be able to fix it in one pass, not discover a second error
// only after correcting the first. CP 3.2's fifteen bad configs each test one
// specific message, but a real hand-written file often has several.
// ---------------------------------------------------------------------------
class Validator {
public:
    explicit Validator(std::string file) : file_(std::move(file)) {}

    /// Range check against the schema table. `line` is the TOML source line, so
    /// the error can point at it.
    void check_range(std::string_view path, double value, int line);

    /// One of a fixed set of strings. Used for every enum-valued key —
    /// atmosphere mode, shape type, input mode, event action.
    void check_enum(std::string_view path, const std::string& value,
                    std::initializer_list<const char*> allowed, int line,
                    const char* spec_row = "");

    /// A problem that is not a simple range or enum: a cross-field constraint,
    /// a missing required table, a file that does not exist.
    void add(std::string_view path, std::string message, int line);

    [[nodiscard]] bool ok() const noexcept { return errors_.empty(); }
    [[nodiscard]] const std::vector<ValidationError>& errors() const noexcept {
        return errors_;
    }

    /// Every error, one per line, in the §7.5 format.
    [[nodiscard]] std::string format_all() const;

private:
    std::string                  file_;
    std::vector<ValidationError> errors_;
};

/// Cross-field and structural checks that no per-key range can express.
///
/// Separate from the per-key checks because these are the interesting ones: a
/// scenario can have every individual value in range and still be impossible.
void validate_scenario(const Scenario& sc, Validator& v);

/// Load, parse and validate. The single entry point design §7.5 names.
///
/// Returns the scenario, or an error string containing EVERY problem found,
/// each on its own line in the §7.5 format.
[[nodiscard]] Result<Scenario> load_scenario(const std::filesystem::path& path);

/// Parse and validate from an in-memory string. Used by the tests and by the
/// fuzzer, neither of which should need to touch the filesystem.
[[nodiscard]] Result<Scenario> parse_scenario(std::string_view toml_text,
                                              std::string_view name = "<memory>");

}  // namespace sat
