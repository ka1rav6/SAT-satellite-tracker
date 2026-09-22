"""Compare --no-ai vs MotionNet on official reacq / lock fields.

Same short sweep (figure-8 + OU + fog, seeds 1-20) both ways. SAT-ML §10.3
grades those log fields, not next-point RMSE.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import statistics
from pathlib import Path


SCENARIOS = [
    "scenarios/ml/motion_figure8.toml",
    "scenarios/ml/motion_ou.toml",
    "scenarios/fog_figure8.toml",
]


def run_one(binary: Path, scenario: str, seed: int, out: Path, no_ai: bool,
            model: str | None, duration: float) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    cmd = [str(binary), "--headless", "--scenario", scenario, "--seed", str(seed),
           "--duration", str(duration), "--out", str(out), "--quiet"]
    if no_ai:
        cmd.append("--no-ai")
    elif model:
        cmd.extend(["--set", f'ai.motion_net="{model}"'])
    subprocess.run(cmd, check=True)
    report = json.loads((out / "run.json").read_text(encoding="utf-8"))
    return report


def metric(report: dict, key: str) -> float:
    m = report.get("metrics", report)
    if key == "reacquisition_s":
        block = m.get("reacquisition", m)
        for alt in ("mean_s", "reacquisition_s", "reacquisition_mean_s"):
            if alt in block:
                return float(block[alt])
    if key == "target_loss_frac":
        block = m.get("lock", m)
        for alt in ("target_loss_frac", "target_loss_post_acq"):
            if alt in block:
                return float(block[alt])
        if "lock_retention_rate" in block:
            return 1.0 - float(block["lock_retention_rate"])
    if key in m:
        return float(m[key])
    return float("nan")


def summarise(rows: list[dict], key: str) -> float:
    vals = [metric(r, key) for r in rows]
    vals = [v for v in vals if v == v]
    return statistics.mean(vals) if vals else float("nan")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bin", type=Path, default=Path("build/sat-tracker"))
    parser.add_argument("--model", default="models/motionnet_v1.onnx")
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--out", type=Path, default=Path("logs/motion_ablate"))
    args = parser.parse_args()

    classical, learned = [], []
    for sc in SCENARIOS:
        for seed in range(1, args.seeds + 1):
            tag = Path(sc).stem
            classical.append(run_one(args.bin, sc, seed,
                                     args.out / "no_ai" / tag / str(seed),
                                     True, None, args.duration))
            learned.append(run_one(args.bin, sc, seed,
                                   args.out / "motion" / tag / str(seed),
                                   False, args.model, args.duration))

    table = {
        "no_ai": {
            "reacquisition_s": summarise(classical, "reacquisition_s"),
            "target_loss_frac": summarise(classical, "target_loss_frac"),
        },
        "motion": {
            "reacquisition_s": summarise(learned, "reacquisition_s"),
            "target_loss_frac": summarise(learned, "target_loss_frac"),
        },
    }
    print(json.dumps(table, indent=2))
    (args.out / "ablate.json").write_text(json.dumps(table, indent=2) + "\n",
                                          encoding="utf-8")


if __name__ == "__main__":
    main()
