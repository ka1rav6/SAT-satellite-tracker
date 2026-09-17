"""Data-factory contract tests for the deterministic dummy fixtures."""

from __future__ import annotations

from pathlib import Path

from ml.datasets import CandidatePatchDataset, CentroidPatchDataset, verify_split_disjoint
from tools.make_dummy_shards import generate


def test_dummy_centroid_shards_are_run_disjoint_and_load(tmp_path: Path):
    root = tmp_path / "centroid"
    generate(root, samples_per_run=1, shard_size=97, patch_px=15, task="centroid_patches")
    verify_split_disjoint(root)
    patch, scalars, target = CentroidPatchDataset(root, "train")[0]
    assert patch.shape == (1, 15, 15)
    assert scalars.shape == (2,)
    assert target.shape == (2,)


def test_dummy_candidate_shards_are_run_disjoint_and_load(tmp_path: Path):
    root = tmp_path / "candidate"
    generate(root, samples_per_run=1, shard_size=97, patch_px=15, task="candidates")
    verify_split_disjoint(root)
    patch, scalars, label = CandidatePatchDataset(root, "test")[0]
    assert patch.shape == (1, 15, 15)
    assert scalars.shape == (6,)
    assert int(label) in range(4)