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
