#!/usr/bin/env python3
"""CP 12.3 — per-condition Monte Carlo: where does adapting actually pay?

The checkpoint asks for "(conditions -> best strategy) pairs FROM MEASUREMENT",
which is the input CP 12.4's learned policy would be trained on and the only
honest way to say whether the rule table is any good.

The grid is beacon brightness crossed with seeds, on a scenario whose weather
changes mid-run (design §7.4's event timeline). Brightness is the axis because
it is what moves the integrated SNR across §10.6's two thresholds: at spec row
7's nominal 120 the fogged SNR lands in the middle band where the rule table
deliberately does nothing, and the supervisor is correctly a no-op.

What this prints is the thing worth knowing and the thing a single headline
number would hide: the supervisor's value is ZERO where the fixed configuration
already copes, and grows as conditions worsen. A system that adapts when there
is nothing to adapt to would be a worse system, not a better one.
"""

import argparse
import json
import os
import subprocess
import sys

INTENSITIES = [80, 60, 50, 40, 30, 25]
SEEDS = [1, 2, 3, 4, 5]


def run(binary, scenario, out_dir, intensity, seed, supervised):
    tag = f"i{intensity}_s{seed}_{'on' if supervised else 'off'}"
    run_dir = os.path.join(out_dir, tag)
    subprocess.run(
        [binary, "--headless", "--scenario", scenario, "--duration", "40",
         "--seed", str(seed),
         "--set", f"target.intensity={intensity}",
         "--set", f"supervisor.enabled={'true' if supervised else 'false'}",
         "--no-csv", "--no-report", "--quiet", "--out", run_dir],
        check=True, stdout=subprocess.DEVNULL)
    with open(os.path.join(run_dir, "run.json"), encoding="utf-8") as fh:
        m = json.load(fh)["metrics"]
    return (100.0 * m["lock"]["retention_rate"],
            m["tracking"]["rms_px"],
            m["centroiding"]["rmse_image_px"])


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--binary", required=True)
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)

    out_dir = os.path.join(a.out, "cp123")
    os.makedirs(out_dir, exist_ok=True)

    print(f"  {len(SEEDS)} seeds per cell, fog from 10 s to 25 s\n")
    print("  intensity   retention fixed -> supervised    tracking RMS      centroid RMSE")
    rows = []
    for inten in INTENSITIES:
        acc = {False: [0.0, 0.0, 0.0], True: [0.0, 0.0, 0.0]}
        for sup in (False, True):
            for seed in SEEDS:
                r, t, c = run(a.binary, a.scenario, out_dir, inten, seed, sup)
                acc[sup][0] += r / len(SEEDS)
                acc[sup][1] += t / len(SEEDS)
                acc[sup][2] += c / len(SEEDS)
        rows.append((inten, acc[False], acc[True]))
        print(f"  {inten:6d}      {acc[False][0]:6.2f} %% -> {acc[True][0]:6.2f} %%"
              f"       {acc[False][1]:6.2f} -> {acc[True][1]:6.2f}"
              f"    {acc[False][2]:7.2f} -> {acc[True][2]:7.2f}")

    # The conclusion, stated rather than left to the reader. "Best strategy" per
    # condition is what CP 12.4 would train on, and with two strategies it is
    # just which of the two won.
    print("\n  (conditions -> best strategy), by lock retention:")
    for inten, fixed, sup in rows:
        gain = sup[0] - fixed[0]
        verdict = ("supervised" if gain > 1.0
                   else "fixed" if gain < -1.0
                   else "no measurable difference")
        print(f"    intensity {inten:3d}   {verdict:26s} ({gain:+.2f} points)")

    best = max(rows, key=lambda r: r[2][0] - r[1][0])
    print(f"\n  The adaptation is worth most at intensity {best[0]}: "
          f"{best[2][0] - best[1][0]:+.2f} points of retention.")
    print("  Where the fixed configuration already copes it is worth nothing, "
          "which is the\n  correct behaviour and not a disappointing result.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
