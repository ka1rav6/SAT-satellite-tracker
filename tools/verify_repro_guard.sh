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
#
# ---------------------------------------------------------------------------
# PART TWO: THE AVX2 LANE — added with P1-1
# ---------------------------------------------------------------------------
# The gate grew a second arm: it now runs each scenario once more with the
# vector damage chain forced off and requires the digests to agree. That arm
# needs its own self-test, for exactly the reason this file already argues —
# a comparison nobody has watched fail proves nothing.
#
# So a second injection perturbs the AVX2 kernel's read-noise sigma by one part
# in a thousand. It is the realistic form of the bug — a constant transcribed
# slightly differently into the vector path — and it is the hardest kind for
# the old gate to see: the scalar path is untouched, every repeat is still
# bit-identical to itself, and the only thing that changes is agreement BETWEEN
# the two paths.
#
# WHY NOT ONE ULP, which was the first attempt. The frame is quantised to 8
# bits, so a relative perturbation of 1e-7 on a sigma of 20 grey levels moves
# every pixel by ~2e-6 of a level and not one of them crosses a rounding
# boundary. The injected build produced byte-identical frames and the self-test
# reported, correctly, that the gate had not fired. One ULP is below the
# observable floor of the thing being compared, so it tests nothing. 1e-3 is
# ~0.02 grey levels, which flips a scattering of pixels per frame — still far
# subtler than any bug that would be noticed by eye.
#
# On a machine without AVX2 this half is skipped and says so.

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

# Restore before part two, so the two injections cannot compound.
cp "${backup}" "${target}"

# ---------------------------------------------------------------------------
# Part two — the scalar/vector arm.
# ---------------------------------------------------------------------------
simd_target="${repo_root}/src/degrade/sensor_simd.cpp"
simd_backup="$(mktemp)"
cp "${simd_target}" "${simd_backup}"
cleanup_simd() {
    cp "${simd_backup}" "${simd_target}"
    rm -f "${simd_backup}"
    cleanup
}
trap cleanup_simd EXIT INT TERM

if ! "${scratch}/build/sat-tracker" --has-avx2 > /dev/null 2>&1; then
    echo
    echo "INV-3 self-test: this CPU has no AVX2, so the scalar/vector arm cannot"
    echo "                 be exercised here. windows-video.yml and the CI matrix"
    echo "                 cover other hardware; the arm reports n/a rather than"
    echo "                 passing when there is nothing to compare."
    exit 0
fi

echo
echo "INV-3 self-test: perturbing the AVX2 read-noise sigma by 1 part in 1000…"
python3 - "${simd_target}" <<'PYINJECT2'
import sys
path = sys.argv[1]
src = open(path).read()

# The perturbation goes on the read-noise sigma the vector kernel uses. The
# scalar path keeps the true value, so the two chains now disagree by one part
# in a thousand — the kind of difference a "looks equivalent" rewrite of a
# constant actually produces.
needle = "size_t run_avx2(const DamageArgs& a, Pcg32& g) noexcept {"
assert needle in src, "run_avx2 signature changed; update this self-test"

injection = needle + """
    // ---- DELIBERATE AVX2/SCALAR MISMATCH, injected by verify_repro_guard.sh --
    // Nudges lane 0 of the read-noise sigma by one ULP. The scalar path is
    // untouched, so every run stays bit-identical to its own repeat and only
    // the cross-path comparison can see this.
    DamageArgs a_injected = a;
    a_injected.sigma = a.sigma * 1.001f;
    a_injected.rvar  = a_injected.sigma * a_injected.sigma;
    #define a a_injected
    // ---- END INJECTION -------------------------------------------------------"""

src = src.replace(needle, injection)
open(path, "w").write(src)
PYINJECT2

cmake --build "${scratch}/build" --target sat-tracker --parallel \
      > "${scratch}/build3.log" 2>&1

if "${scratch}/build/sat-tracker" --verify-reproducibility --seeds 1 --duration 1.0 \
        > "${scratch}/simd.log" 2>&1; then
    echo "FAIL: --verify-reproducibility PASSED with the AVX2 damage chain" >&2
    echo "      disagreeing with the scalar one. The arm added for P1-1 is not" >&2
    echo "      detecting what it exists to detect." >&2
    tail -40 "${scratch}/simd.log" >&2
    exit 1
fi

echo "  check went red as expected:"
grep -E "DIFFER|FAIL:" "${scratch}/simd.log" | head -4 | sed 's/^/    /'
echo "INV-3 self-test passed: the gate detects an AVX2/scalar divergence."
