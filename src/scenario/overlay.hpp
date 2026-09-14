// scenario/overlay.hpp — CP 7.5's mechanism for varying one key at a time.
//
// A sweep is a cartesian product over scenario keys. To run it, something has
// to be able to say "this scenario, but with atmosphere.mode = fog" without
// hand-writing a file per combination — 500 of them, in CP 7.5's case.
//
// ---------------------------------------------------------------------------
// WHY THIS RETURNS TOML TEXT RATHER THAN SETTING STRUCT FIELDS
// ---------------------------------------------------------------------------
// The obvious implementation is a path -> member-pointer table: given
// "gimbal.max_pan_dps", assign to Scenario::max_pan_dps. It is also the wrong
// one, for two reasons.
//
//   It would be a SECOND schema. scenario/schema.cpp already has a table of
//   every legal key, and the whole point of that table is that there is one
//   place a key is described. A parallel table mapping the same keys to struct
//   members would drift, and the failure mode of drift here is a sweep that
//   silently does not vary the axis it claims to.
//
//   It would bypass validation. Overriding a key to something out of range
//   must produce §7.5's error — file, line, value, specification row — and
//   that machinery runs on parsed TOML. A struct-field setter would write the
//   bad value straight into the Scenario and the sweep would report results
//   for a configuration the specification forbids.
//
// So an override is applied to the TOML document and the result is re-parsed
// through the ordinary loader. Every key is checked by the one schema, every
// error is the one format, and there is nothing to keep in step.

#pragma once

#include "core/result.hpp"

#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace sat {

/// One `dotted.key = value` override. The value is TOML source text, so it can
/// be a number, a quoted string, a bool or an array — whatever the key needs —
/// and the parser decides whether it is valid for that key.
struct Override {
    std::string key;
    std::string value;
};

/// Apply overrides to a TOML document and return the new document text.
///
/// Creates intermediate tables as needed, so an override can set a key whose
/// table the base file never mentions. Fails if `toml_text` does not parse, or
/// if a key's path runs through something that is not a table.
[[nodiscard]] Result<std::string> apply_overrides(std::string_view toml_text,
                                                  const std::vector<Override>& ov);

}  // namespace sat
