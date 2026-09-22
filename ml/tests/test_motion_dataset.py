"""Track-window contract tests for MotionNet dummy shards (SAT-ML.md §6)."""

from __future__ import annotations

from pathlib import Path

from ml.datasets import TrackWindowDataset, verify_split_disjoint
from tools.make_dummy_shards import generate


def test_dummy_track_shards_are_run_disjoint_and_load(tmp_path: Path):
    """Dummy tracks must split by (scenario, seed) and yield 30x4 / 15x2 windows."""
    generate(tmp_path, task="tracks", n_per_split=32)
    verify_split_disjoint(tmp_path)
    data = TrackWindowDataset(tmp_path, "train")
    hist, fut, regime = data[0]
    assert hist.shape == (30, 4)
    assert fut.shape == (15, 2)
    assert regime.ndim == 0
    assert 0 <= int(regime) <= 3


def test_datagen_windows_a_csv_into_run_disjoint_shards(tmp_path: Path):
    """Python windowing must use tracker history and FrameTruth residuals."""
    from ml.datagen import build_from_raw

    raw = tmp_path / "raw"
    raw.mkdir()
    # 50 frames: 30 hist + 15 future + a few extras. Seed 1 => train.
    dt = 1.0 / 30.0
    lines = ["frame,detected,track_az_urad,track_el_urad,track_vaz,track_vel,"
             "truth_az_urad,truth_el_urad,regime,scenario_id,seed"]
    for i in range(50):
        az = 1000.0 * i * dt
        lines.append(f"{i},1,{az},0,1000,0,{az},0,0,1,1")
    (raw / "line_1.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")

    out = tmp_path / "packed"
    counts = build_from_raw(raw, out)
    assert counts["train"] >= 1
    verify_split_disjoint(out)
    data = TrackWindowDataset(out, "train")
    hist, fut, regime = data[0]
    assert hist.shape == (30, 4)
    assert fut.shape == (15, 2)
    assert int(regime) == 0


def test_require_dropouts_rejects_a_sweep_with_zero_misses(tmp_path: Path):
    """Fail loud when include_dropouts is set but CFAR never dropped.

    A model trained only on clean lock cannot help official row 19.
    """
    import pytest
    from ml.datagen import build_from_raw

    raw = tmp_path / "raw"
    raw.mkdir()
    lines = ["frame,detected,track_az_urad,track_el_urad,track_vaz,track_vel,"
             "truth_az_urad,truth_el_urad,regime,scenario_id,seed"]
    for i in range(50):
        lines.append(f"{i},1,{i},0,0,0,{i},0,0,1,1")
    (raw / "clean_1.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="include_dropouts"):
        build_from_raw(raw, tmp_path / "packed", require_dropouts=True)


def test_require_dropouts_accepts_a_run_that_actually_missed(tmp_path: Path):
    from ml.datagen import build_from_raw

    raw = tmp_path / "raw"
    raw.mkdir()
    lines = ["frame,detected,track_az_urad,track_el_urad,track_vaz,track_vel,"
             "truth_az_urad,truth_el_urad,regime,scenario_id,seed"]
    for i in range(50):
        det = 0 if i == 20 else 1
        lines.append(f"{i},{det},{i},0,0,0,{i},0,1,2,1")
    (raw / "fog_1.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")
    counts = build_from_raw(raw, tmp_path / "packed", require_dropouts=True)
    assert sum(counts.values()) >= 1
