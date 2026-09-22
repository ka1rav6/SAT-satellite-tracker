"""MotionNet — 30x4 track history to 15-step residual + regime (SAT-ML.md §6.2).

A tiny GRU is the whole model. Anything larger blows the 0.05 ms ONNX budget
and official row 20 (20 FPS). It does not look at the image: perception already
handled noise; this net keeps a kinematic guess alive through CFAR dropouts.
"""

from __future__ import annotations

import torch
from torch import Tensor, nn

HISTORY_LEN, HORIZON, INPUT_SIZE, N_REGIMES, HIDDEN = 30, 15, 4, 4, 24


class MotionNet(nn.Module):
    """30x4 track history -> 15-step (dx, dy) + 4-class regime logits."""

    def __init__(self, hidden: int = HIDDEN, horizon: int = HORIZON):
        super().__init__()
        self.horizon = horizon
        self.gru = nn.GRU(input_size=INPUT_SIZE, hidden_size=hidden, batch_first=True)
        self.forecast = nn.Linear(hidden, horizon * 2)
        self.regime = nn.Linear(hidden, N_REGIMES)

    def forward(self, hist: Tensor) -> tuple[Tensor, Tensor]:
        if hist.ndim != 3 or hist.shape[1:] != (HISTORY_LEN, INPUT_SIZE):
            raise ValueError(
                f"expected (B, {HISTORY_LEN}, {INPUT_SIZE}), got {tuple(hist.shape)}"
            )
        _, hidden = self.gru(hist)
        hidden = hidden.squeeze(0)
        forecast = self.forecast(hidden).view(-1, self.horizon, 2)
        return forecast, self.regime(hidden)
