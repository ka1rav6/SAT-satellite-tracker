#!/usr/bin/env python3
"""CP 10.7 — handover success rate and time-to-handover over a sweep.

The checkpoint asks for a RATE, which means many runs: whether one run reaches
Handover says almost nothing, because whether the loop settles inside a 333
urad box depends on where the beacon happened to start and what the noise did.

Two arms, and the comparison is the point:

  clean      scenarios/control/fast_linear.toml as committed
  row 23     the same, with the specification's maximum camera jitter

Row 23 allows +/- 20 px per frame on the TRUE boresight. A quadrant cell is
co-boresighted and sees all of it; the encoder sees none. Uniform[-A,A] over
two axes gives an RMS of sqrt(2*400/3) = 16.33 px = 1781 urad against a
criterion of capture/3 = 333 urad — unreachable by a factor of five, and not by
anything a control law can fix. The sweep is what turns that arithmetic into a
measured rate.
"""

import argparse
import json
import os
import subprocess
import sys

SEEDS = list(range(1, 21))


def arm(binary, scenario, out_dir, label, extra):
    reached, times, best = 0, [], []
    for seed in SEEDS:
        run_dir = os.path.join(out_dir, f"{label}_s{seed}")
        subprocess.run(
            [binary, "--headless", "--scenario", scenario, "--duration", "11",
             "--seed", str(seed), "--no-csv", "--no-report", "--quiet",
             "--out", run_dir] + extra,
            check=True, stdout=subprocess.DEVNULL)
        with open(os.path.join(run_dir, "run.json"), encoding="utf-8") as fh:
            h = json.load(fh)["metrics"]["handover"]
        if h["reached"]:
            reached += 1
            times.append(h["time_s"])
        best.append(h["best_rms_urad"])
    return reached, times, best


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--binary", required=True)
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)

    out_dir = os.path.join(a.out, "cp107")
    os.makedirs(out_dir, exist_ok=True)

    print(f"  {len(SEEDS)} seeds per arm, capture/3 = 333.3 urad\n")
    for label, extra in (("clean", []),
                         ("row 23 jitter",
                          ["--set", "disturbance.jitter_px_per_frame=20"])):
        n, times, best = arm(a.binary, a.scenario, out_dir,
                             label.split()[0], extra)
        rate = 100.0 * n / len(SEEDS)
        if times:
            times.sort()
            mid = times[len(times) // 2]
            print(f"  {label:16s} handover on {n:2d}/{len(SEEDS)} runs "
                  f"({rate:5.1f} %)   median {mid:5.2f} s, "
                  f"worst {max(times):5.2f} s")
        else:
            # The reason matters more than the rate when the rate is zero.
            best.sort()
            mid = best[len(best) // 2]
            print(f"  {label:16s} handover on {n:2d}/{len(SEEDS)} runs "
                  f"({rate:5.1f} %)   best sustained RMS offset: "
                  f"median {mid:7.1f} urad, best {min(best):7.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
