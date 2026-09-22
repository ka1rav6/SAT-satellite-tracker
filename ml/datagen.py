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
import subprocess
import tomllib
from collections import defaultdict
from dataclasses import dataclass
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


@dataclass(frozen=True)
class MotionSweep:
    """Python-side factory list. Not a SAT SweepSpec (those have one base)."""

    scenarios: list[str]
    holdout: list[str]
    seed_start: int
    seed_end: int
    duration_s: float
    include_dropouts: bool


def load_motion_sweep(path: Path) -> MotionSweep:
    data = tomllib.loads(path.read_text(encoding="utf-8"))
    motion = data.get("motion", {})
    capture = data.get("capture", {})
    return MotionSweep(
        scenarios=list(motion.get("scenarios", [])),
        holdout=list(motion.get("holdout", [])),
        seed_start=int(motion.get("seed_start", 1)),
        seed_end=int(motion.get("seed_end", 1)),
        duration_s=float(motion.get("duration_s", 20.0)),
        include_dropouts=bool(capture.get("include_dropouts", True)),
    )


def capture_motion_sweep(sweep: MotionSweep, sat_tracker: Path, out_dir: Path,
                         seed_end: int | None = None, duration_s: float | None = None
                         ) -> Path:
    """Invoke sat-tracker --gen-dataset once per (scenario, seed)."""
    raw = out_dir / "raw"
    raw.mkdir(parents=True, exist_ok=True)
    last = seed_end if seed_end is not None else sweep.seed_end
    dur = sweep.duration_s if duration_s is None else duration_s
    files = list(sweep.scenarios) + list(sweep.holdout)
    if not files:
        raise ValueError(f"{sweep} has no scenarios")
    for scenario in files:
        for seed in range(sweep.seed_start, last + 1):
            cmd = [
                str(sat_tracker),
                "--gen-dataset",
                "--scenario", scenario,
                "--out", str(out_dir),
                "--task", "tracks",
                "--seed", str(seed),
                "--duration", str(dur),
            ]
            subprocess.run(cmd, check=True)
    return raw


def scenario_id_from_name(name: str) -> int:
    """Must match sat::scenario_id_from_name in src/app/dataset.cpp (FNV-1a)."""
    h = 2166136261
    for byte in name.encode("utf-8"):
        h ^= byte
        h = (h * 16777619) & 0xFFFFFFFF
    return h & 0x7FFFFFFF


def holdout_ids_from_csvs(raw_dir: Path, holdout_stems: set[str]) -> set[int]:
    ids: set[int] = set()
    for csv_path in raw_dir.glob("*.csv"):
        stem = csv_path.stem  # name_seed
        name = stem.rsplit("_", 1)[0]
        if name in holdout_stems:
            run = load_run_csv(csv_path)
            ids.add(int(run["scenario_id"][0]))
    return ids


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, help="directory of --gen-dataset CSVs")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--require-dropouts", action="store_true")
    parser.add_argument("--test-scenario-id", type=int, action="append", default=[])
    parser.add_argument("--sweep", type=Path, help="ml/sweeps/motion_v1.toml")
    parser.add_argument("--bin", type=Path, default=Path("build/sat-tracker"))
    parser.add_argument("--seed-end", type=int, default=None)
    parser.add_argument("--duration", type=float, default=None)
    args = parser.parse_args()

    test_ids = set(args.test_scenario_id)
    raw = args.raw
    require = args.require_dropouts
    if args.sweep is not None:
        sweep = load_motion_sweep(args.sweep)
        raw = capture_motion_sweep(sweep, args.bin, args.out, args.seed_end, args.duration)
        require = require or sweep.include_dropouts
        holdout_stems = {Path(p).stem for p in sweep.holdout}
        test_ids |= holdout_ids_from_csvs(raw, holdout_stems)
    if raw is None:
        raise SystemExit("need --raw or --sweep")
    counts = build_from_raw(
        raw, args.out,
        test_scenario_ids=test_ids or None,
        require_dropouts=require,
    )
    print(f"wrote track shards {counts} -> {args.out}")


if __name__ == "__main__":
    main()
