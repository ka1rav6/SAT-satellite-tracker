"""SAT-ML §6.6 evaluate helpers — CV has nothing to beat on a linear dummy."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import torch

from ml.evaluate_motion import compare_table, rmse_at
from ml.models.motion import constant_velocity_forecast
from tools.make_dummy_shards import generate


def test_cv_comparison_is_zero_improvement_on_a_linear_dummy(tmp_path: Path):
    generate(tmp_path, task="tracks", n_per_split=16)
    from ml.datasets import TrackWindowDataset

    data = TrackWindowDataset(tmp_path, "test")
    hist = torch.from_numpy(data.history.astype(np.float32))
    future = data.future.astype(np.float32)
    cv = constant_velocity_forecast(hist).numpy()
    logits = np.zeros((len(data), 4), dtype=np.float32)
    logits[np.arange(len(data)), data.regime.astype(int)] = 1.0
    table = compare_table(cv, cv, future, logits, data.regime.astype(np.int64))
    assert abs(table["plus5_improvement"]) < 1e-6
    assert abs(table["plus15_improvement"]) < 1e-6
    assert rmse_at(cv, cv, 4) == 0.0


def test_load_motion_sweep_lists_four_regimes():
    from ml.datagen import load_motion_sweep

    sweep = load_motion_sweep(Path("ml/sweeps/motion_v1.toml"))
    assert len(sweep.scenarios) == 4
    assert sweep.include_dropouts
    assert any("figure8" in s for s in sweep.scenarios)
    assert any("linear_fast" in s for s in sweep.holdout)
