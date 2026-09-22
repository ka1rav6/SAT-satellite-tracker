"""MotionNet shape and parameter-budget tests (SAT-ML.md §6.2)."""

from __future__ import annotations

import torch

from ml.models.motion import HISTORY_LEN, HORIZON, INPUT_SIZE, MotionNet, N_REGIMES


def test_motionnet_shapes_and_param_budget():
    """GRU must emit 15-step residuals plus 4 logits and stay under 12k params."""
    model = MotionNet()
    hist = torch.zeros(3, HISTORY_LEN, INPUT_SIZE)
    forecast, logits = model(hist)
    assert forecast.shape == (3, HORIZON, 2)
    assert logits.shape == (3, N_REGIMES)
    n = sum(p.numel() for p in model.parameters())
    assert n < 12_000  # spec ~6k; fail if someone "improves" it into a transformer
