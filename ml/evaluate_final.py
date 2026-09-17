"""Touch the held-out test split once and print SAT-ML.md acceptance metrics.

Training never imports this module.  It exists as a visibly separate final
evaluation step so ML-7 is enforceable in practice.  Dummy fixtures do not
constitute a claim about real performance; their results are labelled as such
in the model cards until CP 11.1 produces simulator data.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import torch

from ml.datasets import normalise_candidate_scalars, normalise_patch, normalise_scalars
from ml.inference import classical_centroid
from ml.models.candidate import CandidateNet
from ml.models.centroid import CentroidNet


def _raw_test_arrays(root: Path) -> dict[str, np.ndarray]:
    """Read test shards once for the final report, retaining condition metadata."""
    keys = ("patches", "scalars", "labels", "conditions")
    values = {key: [] for key in keys}
    for shard in sorted((root / "test").glob("shard_*.npz")):
        with np.load(shard) as data:
            for key in keys:
                values[key].append(data[key])
    return {key: np.concatenate(items, axis=0) for key, items in values.items()}


def _centroid_report(root: Path, checkpoint: Path | None) -> None:
    raw = _raw_test_arrays(root)
    classical = np.stack([classical_centroid(patch) for patch in raw["patches"]])
    model = CentroidNet().eval()
    if checkpoint is not None:
        state = torch.load(checkpoint, map_location="cpu", weights_only=True)
        model.load_state_dict(state.get("model", state))
    learned_parts = []
    start = time.perf_counter()
    patches = torch.from_numpy(np.stack([normalise_patch(item) for item in raw["patches"]])[:, None])
    scalars = torch.from_numpy(normalise_scalars(raw["scalars"]))
    with torch.no_grad():
        for start_index in range(0, len(patches), 512):
            learned_parts.append(model(patches[start_index:start_index + 512],
                                       scalars[start_index:start_index + 512]).numpy())
    torch_ms = (time.perf_counter() - start) * 1000.0 / max(len(raw["patches"]), 1)
    learned = np.concatenate(learned_parts)
    target = raw["labels"].astype(np.float32)
    snr = raw["scalars"][:, 0]
    print("CentroidNet final test evaluation (test split touched once)")
    for lo, hi in zip((0, 3, 5, 8, 12, 15, 25), (3, 5, 8, 12, 15, 25, 1000)):
        mask = (snr >= lo) & (snr < hi)
        if not mask.any():
            continue
        error = np.sqrt(np.mean(np.sum((learned[mask] - target[mask]) ** 2, axis=1)))
        bias = np.mean(learned[mask] - target[mask], axis=0)
        classical_error = np.sqrt(np.mean(np.sum((classical[mask] - target[mask]) ** 2, axis=1)))
        improvement = 1.0 - error / max(classical_error, 1e-12)
        print(f"  SNR [{lo}, {hi}): rmse_px={error:.6f} classical={classical_error:.6f} "
              f"improvement={improvement:.3%} bias=({bias[0]:+.6f},{bias[1]:+.6f})")
    print(f"  residual_bias_max_px={np.max(np.abs(np.mean(learned - target, axis=0))):.6f}")
    print(f"  inference_latency_ms_per_sample: pytorch={torch_ms:.6f}")
    try:
        import onnxruntime as ort
        session = ort.InferenceSession("models/centroidnet_v1.onnx", providers=["CPUExecutionProvider"])
        patch = patches.numpy().astype(np.float32)
        scalars = scalars.numpy().astype(np.float32)
        start = time.perf_counter()
        session.run(["offset"], {"patch": patch, "scalars": scalars})
        ort_ms = (time.perf_counter() - start) * 1000.0 / max(len(patch), 1)
        print(f"  inference_latency_ms_per_sample: onnxruntime={ort_ms:.6f}")
    except Exception as error:
        print(f"  inference_latency_ms_per_sample: onnxruntime=unavailable ({error})")


def _binary_auc(scores: np.ndarray, labels: np.ndarray) -> float:
    order = np.argsort(scores, kind="stable")
    ranks = np.empty_like(order, dtype=np.float64)
    ranks[order] = np.arange(1, len(order) + 1)
    positives = labels == 1
    negatives = ~positives
    return float((ranks[positives].sum() - positives.sum() * (positives.sum() + 1) / 2)
                 / max(positives.sum() * negatives.sum(), 1))


def _ece(probabilities: np.ndarray, labels: np.ndarray, bins: int = 15) -> float:
    edges = np.linspace(0.0, 1.0, bins + 1)
    result = 0.0
    for lo, hi in zip(edges[:-1], edges[1:]):
        mask = (probabilities > lo) & (probabilities <= hi)
        if mask.any():
            result += mask.mean() * abs(labels[mask].mean() - probabilities[mask].mean())
    return float(result)


def _candidate_report(root: Path, checkpoint: Path | None) -> None:
    raw = _raw_test_arrays(root)
    model = CandidateNet().eval()
    if checkpoint is not None:
        state = torch.load(checkpoint, map_location="cpu", weights_only=True)
        model.load_state_dict(state.get("model", state))
    outputs = []
    patches = torch.from_numpy(np.stack([normalise_patch(item) for item in raw["patches"]])[:, None])
    scalars = torch.from_numpy(normalise_candidate_scalars(raw["scalars"]))
    with torch.no_grad():
        for start_index in range(0, len(patches), 512):
            outputs.append(torch.softmax(model(patches[start_index:start_index + 512],
                                               scalars[start_index:start_index + 512]), dim=1).numpy())
    probabilities = np.concatenate(outputs)
    labels = raw["labels"].astype(np.int64)
    beacon = (labels == 0).astype(np.int64)
    score = probabilities[:, 0]
    mask = np.isin(raw["conditions"][:, 2].astype(np.int64), [3, 4])
    print("CandidateNet final test evaluation (test split touched once)")
    print(f"  auc_beacon_vs_rest_overall={_binary_auc(score, beacon):.6f}")
    print(f"  auc_beacon_vs_rest_fog_lowlight={_binary_auc(score[mask], beacon[mask]):.6f}")
    print(f"  expected_calibration_error={_ece(score, beacon):.6f}")
    thresholds = np.unique(score[beacon])
    threshold = thresholds[-1] if len(thresholds) else 1.0
    for candidate in thresholds:
        if np.mean(score[beacon] >= candidate) >= 0.99:
            threshold = candidate
            break
    decoys = labels == 1
    rejection = np.mean(score[decoys] < threshold) if decoys.any() else float("nan")
    print(f"  decoy_rejection_at_1pct_beacon_miss={rejection:.6f}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--task", choices=("centroid", "candidate"), required=True)
    parser.add_argument("--checkpoint", type=Path)
    args = parser.parse_args()
    if args.task == "centroid":
        _centroid_report(args.dataset, args.checkpoint)
    else:
        _candidate_report(args.dataset, args.checkpoint)


if __name__ == "__main__":
    main()