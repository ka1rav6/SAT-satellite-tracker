"""Create deterministic dummy ML shards for SAT-ML.md section 2.

The C++ ``--gen-dataset`` command is the real data factory and remains a
blocking CP 11.1 dependency.  This tool deliberately does not impersonate
that command: it creates small, synthetic fixtures so the Python loaders,
split guard, training loops, export code and evaluation plumbing can be
validated before the simulator starts writing real labels.

Each shard follows the documented aligned-array contract: ``patches``,
``scalars``, ``labels``, ``run_id``, ``scenario_id``, ``seed`` and
``conditions``.  A run is represented by one (scenario_id, seed) pair and is
assigned to exactly one split.  The fixed ordering and local RNGs are
intentional: rerunning this command with the same arguments must produce the
same bytes and therefore the same experiments.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from ml.datasets import verify_split_disjoint


SCENARIOS = tuple(chr(ord("A") + index) for index in range(20))
SPLIT_SCENARIOS = {
    "train": tuple(range(16)),
    "val": tuple(range(16)),
    "test": tuple(range(16, 20)),
}
SPLIT_SEEDS = {
    "train": range(1, 701),
    "val": range(701, 851),
    "test": range(851, 1001),
}


def _split_for(scenario_id: int, seed: int) -> str:
    """Return the only split allowed by SAT-ML.md section 2.3."""
    if scenario_id < 16 and seed <= 700:
        return "train"
    if scenario_id < 16 and seed <= 850:
        return "val"
    if scenario_id >= 16 and seed >= 851:
        return "test"
    raise ValueError(f"(scenario_id={scenario_id}, seed={seed}) is not in the documented split")


def _centroid_sample(rng: np.random.Generator, patch_px: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Render a noisy square-like beacon at a sub-pixel offset.

    This is intentionally a fixture generator, not a second simulator.  The
    smooth spot gives CentroidNet a learnable signal and the exact offset is
    retained as the regression label; real training data must still come from
    the C++ analytic-coverage renderer.
    """
    offset = rng.uniform(-0.48, 0.48, size=2).astype(np.float32)
    snr = float(rng.uniform(3.0, 35.0))
    size = float(rng.uniform(5.0, 20.0))
    weather = float(rng.integers(0, 5))
    axis = np.arange(patch_px, dtype=np.float32) - (patch_px - 1) / 2.0
    yy, xx = np.meshgrid(axis, axis, indexing="ij")
    sigma = max(size / 3.0, 1.0)
    signal = np.exp(-((xx - offset[0]) ** 2 + (yy - offset[1]) ** 2) / (2.0 * sigma * sigma))
    noise_std = 1.0 / max(snr, 1.0)
    patch = signal + rng.normal(0.0, noise_std, signal.shape).astype(np.float32)
    scalars = np.asarray([snr, size], dtype=np.float32)
    conditions = np.asarray([snr, size, weather, noise_std, 0.0], dtype=np.float32)
    return patch.astype(np.float32), scalars, np.concatenate((offset, conditions))


def _candidate_sample(rng: np.random.Generator, patch_px: int) -> tuple[np.ndarray, np.ndarray, int, np.ndarray]:
    """Create a four-class candidate fixture with six documented scalars."""
    label = int(rng.choice(4, p=[0.06, 0.04, 0.31, 0.59]))
    patch = rng.normal(0.0, 0.08, size=(patch_px, patch_px)).astype(np.float32)
    if label in (0, 1):
        centre = (patch_px - 1) / 2.0
        axis = np.arange(patch_px, dtype=np.float32)
        yy, xx = np.meshgrid(axis, axis, indexing="ij")
        amplitude = 1.0 if label == 0 else 0.75
        patch += amplitude * np.exp(-((xx - centre) ** 2 + (yy - centre) ** 2) / 8.0)
    snr = float(rng.uniform(3.0, 35.0))
    size = float(rng.uniform(5.0, 20.0))
    weather = float(rng.integers(0, 5))
    scalars = np.asarray(
        [snr, size * size, rng.uniform(0.25, 1.0), rng.uniform(1.0, 2.5),
         rng.uniform(0.0, 1.0), rng.uniform(0.0, 100.0)],
        dtype=np.float32,
    )
    conditions = np.asarray([snr, size, weather, scalars[2], scalars[3]], dtype=np.float32)
    return patch, scalars, label, conditions


def _write_split(root: Path, split: str, samples_per_run: int, shard_size: int,
                 patch_px: int, task: str) -> int:
    rows: list[tuple[np.ndarray, np.ndarray, np.ndarray, str, int, int, np.ndarray]] = []
    shard_index = 0
    sample_count = 0

    def flush() -> None:
        nonlocal rows, shard_index
        if not rows:
            return
        patches = np.stack([row[0] for row in rows])
        scalars = np.stack([row[1] for row in rows])
        labels = np.stack([row[2] for row in rows])
        np.savez(
            root / split / f"shard_{shard_index:04d}.npz",
            patches=patches.astype(np.float32),
            scalars=scalars.astype(np.float32),
            labels=labels.astype(np.float32 if task == "centroid_patches" else np.int64),
            run_id=np.asarray([row[3] for row in rows]),
            scenario_id=np.asarray([row[4] for row in rows], dtype=np.int32),
            seed=np.asarray([row[5] for row in rows], dtype=np.int32),
            conditions=np.stack([row[6] for row in rows]).astype(np.float32),
        )
        rows = []
        shard_index += 1

    for scenario_id in SPLIT_SCENARIOS[split]:
        for seed in SPLIT_SEEDS[split]:
            rng = np.random.default_rng((scenario_id + 1) * 1_000_003 + seed)
            run_id = f"{SCENARIOS[scenario_id]}-{seed:04d}"
            for _ in range(samples_per_run):
                if task == "centroid_patches":
                    patch, scalar, target = _centroid_sample(rng, patch_px)
                    label, conditions = target[:2], target[2:]
                else:
                    patch, scalar, label, conditions = _candidate_sample(rng, patch_px)
                rows.append((patch, scalar, label, run_id, scenario_id, seed, conditions))
                sample_count += 1
                if len(rows) >= shard_size:
                    flush()
    flush()
    return sample_count


def generate(root: Path, samples_per_run: int, shard_size: int, patch_px: int, task: str) -> None:
    root.mkdir(parents=True, exist_ok=True)
    for split in ("train", "val", "test"):
        (root / split).mkdir(exist_ok=True)
        for old_shard in sorted((root / split).glob("shard_*.npz")):
            old_shard.unlink()

    counts = {
        split: _write_split(root, split, samples_per_run, shard_size, patch_px, task)
        for split in ("train", "val", "test")
    }
    manifest = {
        "name": root.name,
        "task": task,
        # A wall-clock timestamp would make identical fixture runs differ.
        "generated_utc": "1970-01-01T00:00:00Z",
        "generator_build": "dummy-v1",
        "sweep_config_sha256": "synthetic-fixture",
        "n_samples": counts,
        "patch_px": patch_px,
        "conditions": {"clear": 0.2, "haze": 0.2, "rain": 0.2, "fog": 0.2, "lowlight": 0.2},
        "snr_bins": [3, 5, 8, 12, 18, 25, 35, 50],
        "size_bins": [5, 8, 11, 14, 17, 20],
        "split": counts,
        "scenarios": {split: [SCENARIOS[index] for index in SPLIT_SCENARIOS[split]] for split in counts},
        "seed_ranges": {split: [min(SPLIT_SEEDS[split]), max(SPLIT_SEEDS[split])] for split in counts},
        "normalisation": {
            "mode": "per_patch_std",
            "eps": 1e-6,
            "patch_mean": 0.0,
            "patch_std": 1.0,
            "scalar_mean": [0.0, 0.0],
            "scalar_std": [1.0, 1.0],
        },
    }
    manifest_bytes = json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode("utf-8")
    manifest["sha256"] = hashlib.sha256(manifest_bytes).hexdigest()
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    verify_split_disjoint(root)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--task", choices=("centroid_patches", "candidates"), default="centroid_patches")
    parser.add_argument("--samples-per-run", type=int, default=2)
    parser.add_argument("--shard-size", type=int, default=4096)
    parser.add_argument("--patch-px", type=int, default=15)
    args = parser.parse_args()
    if args.samples_per_run < 1 or args.shard_size < 1 or args.patch_px != 15:
        parser.error("samples and shard size must be positive; patch-px is fixed at the documented 15")
    generate(args.out, args.samples_per_run, args.shard_size, args.patch_px, args.task)
    print(f"wrote deterministic {args.task} shards to {args.out}")


if __name__ == "__main__":
    main()