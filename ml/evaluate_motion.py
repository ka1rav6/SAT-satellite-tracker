"""Evaluate MotionNet on the held-out test split (SAT-ML.md §6.6, ML-7).

Train never opens these shards. The gate is:

    +5  RMSE  MotionNet <= 0.80 * CV
    +15 RMSE  MotionNet <= 0.65 * CV
    regime accuracy after 1 s of history >= 0.90

A fail writes the numbers and does not ship ONNX.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch

from ml.datasets import TrackWindowDataset
from ml.models.motion import HORIZON, MotionNet, constant_velocity_forecast

PLUS5 = 4    # 0-based index of the +5-frame step
PLUS15 = 14  # last step


def rmse_at(pred: np.ndarray, target: np.ndarray, step: int) -> float:
    """RMSE in µrad at one horizon index over a batch of windows."""
    err = pred[:, step, :] - target[:, step, :]
    return float(np.sqrt(np.mean(np.sum(err * err, axis=-1))))


def regime_accuracy(logits: np.ndarray, regime: np.ndarray) -> float:
    pred = np.argmax(logits, axis=-1)
    return float(np.mean(pred == regime))


def compare_table(cv_pred: np.ndarray, mn_pred: np.ndarray, target: np.ndarray,
                  logits: np.ndarray, regime: np.ndarray) -> dict[str, float]:
    a = rmse_at(cv_pred, target, PLUS5)
    b = rmse_at(cv_pred, target, PLUS15)
    c = rmse_at(mn_pred, target, PLUS5)
    d = rmse_at(mn_pred, target, PLUS15)
    e = regime_accuracy(logits, regime)
    return {
        "cv_plus5_rmse": a,
        "cv_plus15_rmse": b,
        "motion_plus5_rmse": c,
        "motion_plus15_rmse": d,
        "regime_acc": e,
        "plus5_ratio": (c / a) if a > 0 else 1.0,
        "plus15_ratio": (d / b) if b > 0 else 1.0,
        "plus5_improvement": ((a - c) / a) if a > 0 else 0.0,
        "plus15_improvement": ((b - d) / b) if b > 0 else 0.0,
        "gate_plus5": 1.0 if c <= 0.80 * a else 0.0,
        "gate_plus15": 1.0 if d <= 0.65 * b else 0.0,
        "gate_regime": 1.0 if e >= 0.90 else 0.0,
    }


def gate_passed(table: dict[str, float]) -> bool:
    return (table["gate_plus5"] > 0.5 and table["gate_plus15"] > 0.5
            and table["gate_regime"] > 0.5)


def evaluate(dataset_root: Path, ckpt: Path, device_name: str = "cpu") -> dict[str, object]:
    """Open the test split once. Never used by train_motion (ML-7)."""
    data = TrackWindowDataset(dataset_root, "test")
    device = torch.device(device_name)
    model = MotionNet().to(device)
    payload = torch.load(ckpt, map_location=device, weights_only=True)
    model.load_state_dict(payload.get("model", payload))
    model.eval()

    hist_norm = torch.from_numpy(np.stack([
        data[i][0] for i in range(len(data))
    ])).to(device)
    future = np.stack([data.future[i] for i in range(len(data))]).astype(np.float32)
    regime = np.asarray([int(data.regime[i]) for i in range(len(data))], dtype=np.int64)
    hist_raw = torch.from_numpy(data.history.astype(np.float32)).to(device)

    with torch.no_grad():
        residual, logits = model(hist_norm)
        cv = constant_velocity_forecast(hist_raw)
        mn = residual + cv
    table = compare_table(
        cv.cpu().numpy(), mn.cpu().numpy(), future,
        logits.cpu().numpy(), regime,
    )
    table["n_windows"] = float(len(data))
    result = {
        "is_real_result": "dummy" not in str(dataset_root).lower(),
        "gate_passed": gate_passed(table),
        "table": table,
        "ckpt": str(ckpt),
        "dataset": str(dataset_root),
    }
    print_table(table)
    if not result["gate_passed"]:
        print("GATE FAILED — do not ship ONNX. Record the numbers on the model card.")
    else:
        print("GATE PASSED — SAT-ML §6.6 RMSE / regime checks.")
    out = ckpt.with_suffix(".eval.json")
    out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def print_table(table: dict[str, float]) -> None:
    print(f"{'':16} {'+5 RMSE (urad)':>16} {'+15 RMSE (urad)':>16} {'regime acc @ >=1s':>18}")
    print(f"{'CV':16} {table['cv_plus5_rmse']:16.3f} {table['cv_plus15_rmse']:16.3f} {'n/a':>18}")
    print(f"{'MotionNet':16} {table['motion_plus5_rmse']:16.3f} "
          f"{table['motion_plus15_rmse']:16.3f} {table['regime_acc']:18.3f}")
    print(f"{'vs CV':16} {table['plus5_improvement']:16.3f} "
          f"{table['plus15_improvement']:16.3f}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--ckpt", type=Path, required=True)
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args()
    evaluate(args.dataset, args.ckpt, args.device)


if __name__ == "__main__":
    main()
