"""Window SAT --gen-dataset CSVs into MotionNet shards (SAT-ML.md §2 / §6.5).

C++ writes one CSV per (scenario, seed) of live tracker state plus FrameTruth
angles. This module is the only place those rows become (30, 4) -> (15, 2)
windows, and it calls verify_split_disjoint before returning.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

from ml.datasets import HISTORY_LEN, HORIZON, verify_split_disjoint


def split_for(scenario_id: int, seed: int, test_scenario_ids: set[int] | None = None) -> str:
    """SAT-ML §2.3: split by run, never by frame.

    Unseen scenario ids (the hold-out file) and seeds ≡ 9 (mod 10) are test.
    Seeds ≡ 7 or 8 (mod 10) are val. Everything else is train.
    """
    if test_scenario_ids and scenario_id in test_scenario_ids:
        return "test"
    rem = int(seed) % 10
    if rem == 9:
        return "test"
    if rem in (7, 8):
        return "val"
    return "train"


def load_run_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"{path} has no data rows")
    def col(name: str, dtype) -> np.ndarray:
        return np.asarray([row[name] for row in rows], dtype=dtype)
    return {
        "detected": col("detected", np.uint8),
        "track": np.stack(
            [col("track_az_urad", np.float32), col("track_el_urad", np.float32),
             col("track_vaz", np.float32), col("track_vel", np.float32)],
            axis=-1,
        ),
        "truth": np.stack(
            [col("truth_az_urad", np.float32), col("truth_el_urad", np.float32)],
            axis=-1,
        ),
        "regime": col("regime", np.int64),
        "scenario_id": col("scenario_id", np.int32),
        "seed": col("seed", np.int32),
    }


def windows_from_run(run: dict[str, np.ndarray], *, require_dropouts: bool = False
                     ) -> list[dict[str, np.ndarray]]:
    """Slice one run into 30-history / 15-future samples.

    History is tracker state (what C++ will feed). Future is FrameTruth
    residual from the current-frame truth, in µrad. Windows that span a
    detection miss are kept; if require_dropouts, a run with zero misses
    is rejected so we cannot silently train only on clean lock.
    """
    n = int(run["track"].shape[0])
    if require_dropouts and int(run["detected"].sum()) == n:
        raise ValueError("include_dropouts is set but this run never missed a detection")
    samples = []
    for t in range(HISTORY_LEN - 1, n - HORIZON):
        hist = run["track"][t - HISTORY_LEN + 1 : t + 1]
        future = run["truth"][t + 1 : t + 1 + HORIZON] - run["truth"][t]
        det = run["detected"][t - HISTORY_LEN + 1 : t + 1]
        samples.append({
            "history": hist.astype(np.float32),
            "future": future.astype(np.float32),
            "regime": np.int64(run["regime"][t]),
            "detected_hist": det.astype(np.uint8),
            "scenario_id": np.int32(run["scenario_id"][t]),
            "seed": np.int32(run["seed"][t]),
            "run_id": f"{int(run['scenario_id'][t])}-{int(run['seed'][t])}",
        })
    return samples


def pack_shards(root: Path, samples: list[dict[str, np.ndarray]],
                test_scenario_ids: set[int] | None = None,
                shard_size: int = 4096) -> dict[str, int]:
    by_split: dict[str, list[dict[str, np.ndarray]]] = defaultdict(list)
    for sample in samples:
        by_split[split_for(int(sample["scenario_id"]), int(sample["seed"]),
                           test_scenario_ids)].append(sample)

    counts = {"train": 0, "val": 0, "test": 0}
    for split in ("train", "val", "test"):
        split_dir = root / split
        split_dir.mkdir(parents=True, exist_ok=True)
        for old in split_dir.glob("shard_*.npz"):
            old.unlink()
        rows = by_split.get(split, [])
        counts[split] = len(rows)
        for i in range(0, len(rows), shard_size):
            chunk = rows[i : i + shard_size]
            np.savez(
                split_dir / f"shard_{i // shard_size:04d}.npz",
                history=np.stack([r["history"] for r in chunk]),
                future=np.stack([r["future"] for r in chunk]),
                regime=np.asarray([r["regime"] for r in chunk], dtype=np.int64),
                detected_hist=np.stack([r["detected_hist"] for r in chunk]),
                run_id=np.asarray([r["run_id"] for r in chunk]),
                scenario_id=np.asarray([r["scenario_id"] for r in chunk], dtype=np.int32),
                seed=np.asarray([r["seed"] for r in chunk], dtype=np.int32),
            )
    return counts


def write_manifest(root: Path, counts: dict[str, int], scenarios: dict[str, list[str]]) -> None:
    manifest = {
        "name": root.name,
        "task": "tracks",
        "generated_utc": "1970-01-01T00:00:00Z",
        "generator_build": "datagen-tracks-v1",
        "n_samples": counts,
        "patch_px": 0,
        "scenarios": scenarios,
        "seed_ranges": {k: [0, 0] for k in counts},
        "split": counts,
    }
    payload = json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode("utf-8")
    manifest["sha256"] = hashlib.sha256(payload).hexdigest()
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def build_from_raw(raw_dir: Path, out_dir: Path,
                   test_scenario_ids: set[int] | None = None,
                   require_dropouts: bool = False) -> dict[str, int]:
    csvs = sorted(raw_dir.glob("*.csv"))
    if not csvs:
        raise FileNotFoundError(f"no raw CSVs under {raw_dir}")
    samples: list[dict[str, np.ndarray]] = []
    names: dict[str, set[str]] = {"train": set(), "val": set(), "test": set()}
    any_miss = False
    for csv_path in csvs:
        run = load_run_csv(csv_path)
        if int(run["detected"].sum()) < run["detected"].shape[0]:
            any_miss = True
        windows = windows_from_run(run, require_dropouts=False)
        samples.extend(windows)
        sid = int(run["scenario_id"][0])
        seed = int(run["seed"][0])
        names[split_for(sid, seed, test_scenario_ids)].add(csv_path.stem)
    if require_dropouts and not any_miss:
        raise ValueError(
            "include_dropouts is true but every raw dump has detected==1 on every frame. "
            "Turn weather/jitter on; a model trained only on clean lock is useless."
        )
    out_dir.mkdir(parents=True, exist_ok=True)
    counts = pack_shards(out_dir, samples, test_scenario_ids)
    write_manifest(out_dir, counts, {k: sorted(v) for k, v in names.items()})
    verify_split_disjoint(out_dir)
    return counts


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, required=True, help="directory of --gen-dataset CSVs")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--require-dropouts", action="store_true")
    parser.add_argument("--test-scenario-id", type=int, action="append", default=[])
    args = parser.parse_args()
    counts = build_from_raw(
        args.raw, args.out,
        test_scenario_ids=set(args.test_scenario_id) or None,
        require_dropouts=args.require_dropouts,
    )
    print(f"wrote track shards {counts} -> {args.out}")


if __name__ == "__main__":
    main()
