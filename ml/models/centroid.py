"""CentroidNet (SAT-ML.md section 3.2).

Centroiding is a regression problem where the useful signal is the position
of intensity within a pixel.  Consequently this network intentionally has no
pooling or strided convolution: either operation would discard the spatial
detail the output is meant to recover.  The two scalar inputs are joined only
after the convolutional features have been flattened, exactly as specified.
"""

from __future__ import annotations

import torch
from torch import Tensor, nn


class CentroidNet(nn.Module):
    """Regress ``(dx, dy)`` in pixels from a 15x15 patch and two scalars."""

    def __init__(self, patch_px: int = 15) -> None:
        super().__init__()
        if patch_px != 15:
            raise ValueError("CentroidNet is specified for a fixed 15x15 patch")
        self.conv = nn.Sequential(
            nn.Conv2d(1, 8, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(8, 16, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(16, 16, 3, padding=1),
            nn.ReLU(),
        )
        self.head = nn.Sequential(
            nn.Linear(16 * 15 * 15 + 2, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 2),
        )

    def forward(self, patch: Tensor, scalars: Tensor) -> Tensor:
        """Return one sub-pixel offset per input row."""
        if patch.ndim != 4 or patch.shape[1:] != (1, 15, 15):
            raise ValueError(f"expected patch shape (B, 1, 15, 15), got {tuple(patch.shape)}")
        if scalars.ndim != 2 or scalars.shape[1] != 2 or scalars.shape[0] != patch.shape[0]:
            raise ValueError(f"expected scalar shape (B, 2), got {tuple(scalars.shape)}")
        features = self.conv(patch).flatten(1)
        return self.head(torch.cat([features, scalars], dim=1))