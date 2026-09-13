#!/usr/bin/env bash
#
# tools/verify_repro_guard.sh — prove that --verify-reproducibility can fail.
#
# Design §14 CP 2.6 does not stop at "CI green". It says:
#
#     "Then deliberately inject a std::chrono call into the sim path and confirm
#      the job goes red. Fix any failure before proceeding."
#
# That instruction is the whole reason this script exists. A reproducibility
# check that has never been observed to fail is indistinguishable from one that
# always prints PASS, and INV-3 is the invariant every other number in the
# project rests on. If the compliance matrix is not reproducible, it is not
# evidence.
#
# The injection is a wall-clock read that perturbs the rendered frame by a
# fraction of a grey level — the smallest realistic version of the mistake,
# rather than something that would break the build outright. It is inserted into
# the synthetic source's render path, the file is always restored via a trap,
# and the build tree used is a throwaway.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
target="${repo_root}/src/engine/synthetic_source.cpp"
backup="$(mktemp)"
scratch="$(mktemp -d)"

cleanup() {
    cp "${backup}" "${target}"
    rm -f "${backup}"
    rm -rf "${scratch}"
}
trap cleanup EXIT INT TERM

cp "${target}" "${backup}"

echo "INV-3 self-test: building a clean baseline…"
cmake -S "${repo_root}" -B "${scratch}/build" -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF > "${scratch}/cfg.log" 2>&1
cmake --build "${scratch}/build" --target sat-tracker --parallel \
      > "${scratch}/build.log" 2>&1

if ! "${scratch}/build/sat-tracker" --verify-reproducibility --seeds 1 --duration 1.0 \
        > "${scratch}/clean.log" 2>&1; then
    echo "FAIL: the UNMODIFIED build is already irreproducible." >&2
    tail -30 "${scratch}/clean.log" >&2
    exit 1
fi
echo "  baseline is reproducible, as expected."

echo "INV-3 self-test: injecting a wall-clock read into the render path…"
python3 - "${target}" <<'PYINJECT'
import sys
path = sys.argv[1]
src = open(path).read()

# Inject into advance_world rather than render_frame. Two reasons:
#
#   * It is the REALISTIC version of the mistake. The way a wall clock actually
#     gets into a simulation is someone computing dt from one, not someone
#     adding nanoseconds to a pixel. INV-3's first bullet — "No std::chrono or
#     any wall-clock read in the simulation path" — exists because of exactly
#     this.
#   * It is observable. An earlier version of this script perturbed
#     radiance_[0] at the top of render_frame, which is BEFORE the buffer is
#     cleared, so the perturbation was overwritten and the self-test wrongly
#     reported that the check had failed to detect anything.
needle = "void SyntheticSource::advance_world(double dt) noexcept {"
assert needle in src, "advance_world signature changed; update this self-test"

injection = needle + """
    // ---- DELIBERATE INV-3 VIOLATION, injected by tools/verify_repro_guard.sh --
    // The classic mistake: deriving a timestep from a wall clock. The
    // perturbation is a few parts per million of one tick, which is about as
    // subtle as this error ever is in practice.
    {
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        const auto ns  = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
        dt += dt * static_cast<double>(ns % 1000) * 1e-6;
    }
    // ---- END INJECTION -------------------------------------------------------"""

src = src.replace(needle, injection)
src = src.replace('#include <algorithm>', '#include <algorithm>\n#include <chrono>')
open(path, "w").write(src)
PYINJECT

cmake --build "${scratch}/build" --target sat-tracker --parallel \
      > "${scratch}/build2.log" 2>&1

if "${scratch}/build/sat-tracker" --verify-reproducibility --seeds 1 --duration 1.0 \
        > "${scratch}/dirty.log" 2>&1; then
    echo "FAIL: --verify-reproducibility PASSED with a wall-clock read in the" >&2
    echo "      render path. The check is not detecting what it exists to detect." >&2
    tail -30 "${scratch}/dirty.log" >&2
    exit 1
fi

echo "  check went red as expected:"
grep -E "DIVERGED|FAIL:" "${scratch}/dirty.log" | head -4 | sed 's/^/    /'
echo "INV-3 self-test passed: the reproducibility check detects a clock read."
