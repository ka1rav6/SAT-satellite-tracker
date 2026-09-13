#!/usr/bin/env bash
#
# tools/verify_inv1_guard.sh — prove that the INV-1 guard actually fires.
#
# Design §2 CP 2.6 establishes the pattern this script follows: after building a
# safety check, "deliberately inject a violation and confirm the job goes red."
# A guard that has never been seen to fail is not evidence of anything — it is
# just a passing message.
#
# This injects a sat_world dependency into a tracker-side target, configures,
# and asserts that CMake refuses. It always restores modules.cmake, including on
# failure or interrupt.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
modules="${repo_root}/cmake/modules.cmake"
backup="$(mktemp)"
scratch="$(mktemp -d)"

cleanup() {
    cp "${backup}" "${modules}"
    rm -f "${backup}"
    rm -rf "${scratch}"
}
trap cleanup EXIT INT TERM

cp "${modules}" "${backup}"

echo "INV-1 self-test: injecting 'sat_search -> sat_world' into the module graph…"

# sat_search is chosen because sat_control links it. A guard that only checked
# direct edges would catch sat_search and miss sat_control; asserting that BOTH
# are reported is what proves the closure walk is transitive.
# The injection finds the sat_search block and appends sat_world to its
# PUBLIC_DEPS line, rather than matching the whole declaration literally. The
# literal version broke the first time sat_search gained a source file
# (CP 6.7's search/pattern.cpp) — which is a bad failure mode for a self-test:
# a guard's own check going red for an unrelated reason trains people to ignore
# it. It still asserts that the target and its PUBLIC_DEPS line exist, so a real
# structural change is still reported rather than silently skipped.
python3 - "${modules}" <<'PY'
import re, sys
path = sys.argv[1]
src = open(path).read()

start = src.find("sat_add_module(sat_search")
assert start >= 0, "sat_search target not found; update this self-test"
end = src.find("\n)", start)
assert end > start, "sat_search declaration is not closed; update this self-test"

block = src[start:end]
patched, n = re.subn(r"(?m)^(\s*PUBLIC_DEPS .*)$", r"\1 sat_world", block, count=1)
assert n == 1, "sat_search has no PUBLIC_DEPS line; update this self-test"

open(path, "w").write(src[:start] + patched + src[end:])
PY

output="${scratch}/configure.log"
if cmake -S "${repo_root}" -B "${scratch}/build" > "${output}" 2>&1; then
    echo "FAIL: configure succeeded with a deliberate INV-1 violation in place." >&2
    echo "      The guard in cmake/modules.cmake is not doing its job." >&2
    exit 1
fi

if ! grep -q "INV-1 VIOLATION" "${output}"; then
    echo "FAIL: configure failed, but not because of the INV-1 guard." >&2
    echo "      Something else is broken; here is the tail of the log:" >&2
    tail -20 "${output}" >&2
    exit 1
fi

echo "  guard fired as expected:"
grep -m1 "INV-1 VIOLATION" "${output}" | sed 's/^/    /'
echo "INV-1 self-test passed: the guard rejects a violation it is meant to catch."
