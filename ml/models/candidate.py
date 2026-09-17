"""CandidateNet (SAT-ML.md section 5.2).

Candidate classification is different from sub-pixel regression: spatial
detail within a proposed blob is useful, but exact pixel phase is not.  The
specified pooling therefore belongs here and reduces each proposal to a
compact representation before the six tracker-observable scalar features are
joined to it.  The output remains logits; callers choose softmax only when
they need probabilities for calibration or association.
"""

from __future__ import annotations

import torch
from torch import Tensor, nn


class CandidateNet(nn.Module):
    """Classify proposals as beacon, decoy, clutter, or noise."""

    def __init__(self, patch_px: int = 15) -> None:
        super().__init__()
        if patch_px != 15:
            raise ValueError("CandidateNet is specified for a fixed 15x15 patch")
        self.conv = nn.Sequential(
            nn.Conv2d(1, 8, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(8, 16, 3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(16, 16, 3, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d(1),
        )
        self.head = nn.Sequential(
            nn.Linear(16 + 6, 32),
            nn.ReLU(),
            nn.Linear(32, 4),
        )

    def forward(self, patch: Tensor, scalars: Tensor) -> Tensor:
        """Return four class logits in beacon/decoy/clutter/noise order."""
        if patch.ndim != 4 or patch.shape[1:] != (1, 15, 15):
            raise ValueError(f"expected patch shape (B, 1, 15, 15), got {tuple(patch.shape)}")
        if scalars.ndim != 2 or scalars.shape[1] != 6 or scalars.shape[0] != patch.shape[0]:
            raise ValueError(f"expected scalar shape (B, 6), got {tuple(scalars.shape)}")
        features = self.conv(patch).flatten(1)
        return self.head(torch.cat([features, scalars], dim=1))