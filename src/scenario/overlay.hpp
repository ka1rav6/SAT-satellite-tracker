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

/// Reject any override key the scenario schema does not define.
///
/// SEPARATE FROM apply_overrides ON PURPOSE. That function is a general TOML
/// overlay and is unit-tested with synthetic documents (`[a] x = 1`); this is
/// the scenario-specific POLICY, and it belongs at the CLI boundary where a
/// human typed the key. The division is the one this header already describes:
/// the overlay does syntax, the schema does meaning.
///
/// WHAT IT PREVENTS. `apply_overrides` creates any intermediate table it
/// cannot find, so before this check an unrecognised key was a silent no-op
/// that still exited 0:
///
///     --set control.kpp=99                          (a typo for control.kp)
///     --set target.motion[0].velocity_px_s=[500,0]  (an array-of-tables path)
///
/// Both wrote a key the loader never reads, ran the BASE scenario, and printed
/// a full report. Measured before this check, both produced 16.939 px —
/// byte-identical to passing no --set at all.
///
/// That is the worst failure mode available to a tuning switch. In a demo it
/// answers "what if you double the target speed?" with the number for the
/// speed that was not changed; in a script it records an experiment that never
/// ran. `--set` is meant to be typed by hand under time pressure, which is
/// exactly when a key gets mistyped.
///
/// Array-of-table paths are rejected by the same rule and correctly so: the
/// overlay addresses named tables and cannot reach an array element at all.
/// Motion stacks are edited in a scenario file — docs/GUIDE.md §12.
[[nodiscard]] Result<void> check_override_keys(const std::vector<Override>& ov,
                                               std::string_view context = "--set");

/// Apply overrides to a TOML document and return the new document text.
///
/// Creates intermediate tables as needed, so an override can set a key whose
/// table the base file never mentions. Fails if `toml_text` does not parse, or
/// if a key's path runs through something that is not a table.
///
/// `context` prefixes every error message. It exists because every message
/// here used to begin "sweep:" unconditionally, including during a plain
/// `--headless --set ...`, where the word names a subsystem the user is not
/// using and sends them to the wrong documentation (A-6).
///
/// BARE WORDS ARE ACCEPTED AS STRINGS. `--set atmosphere.mode=lowlight` is the
/// single most likely thing to be typed live in front of judges, and it used
/// to fail with:
///
///     sweep: override 'atmosphere.mode = lowlight' is not valid TOML:
///     Error while parsing value: could not determine value type
///
/// which is true, unhelpful, and does not mention the fix. A bare word is not
/// valid TOML on the right-hand side of an assignment under ANY reading, so
/// re-trying it as a quoted string cannot change the meaning of anything that
/// previously worked — there is nothing to be ambiguous with. Values that DO
/// parse (`true`, `0.1`, `[1, 2]`, `inf`) are untouched and keep their types.
///
/// Semantic validation stays where it belongs: the quoted value goes through
/// the one schema, which rejects `atmosphere.mode = "lowlite"` naming the key,
/// its legal values and its specification row. This function does syntax.
[[nodiscard]] Result<std::string> apply_overrides(std::string_view toml_text,
                                                  const std::vector<Override>& ov,
                                                  std::string_view context = "sweep");

}  // namespace sat
