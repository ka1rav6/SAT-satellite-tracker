"""Train MotionNet as a residual over constant-velocity (SAT-ML.md §6.4).

CV already owns clean lines. The net spends capacity on turns, figure-8
crossings, and coasts — the only place SAT-ML §6.6 can be beaten. Only
train/val shards are opened here; the test split is reserved for
ml/evaluate_motion.py (ML-7).
"""

from __future__ import annotations

import argparse
import json
import random
import pathlib
from pathlib import Path

import numpy as np
import torch
from torch import Tensor, nn
from torch.utils.data import DataLoader, Dataset

from ml.datasets import TrackWindowDataset, verify_split_disjoint
from ml.models.motion import HORIZON, MotionNet, RESIDUAL_SCALE, constant_velocity_forecast


def posix(p) -> str:
    """A path as forward slashes, whatever platform wrote it.

    These strings land in a committed JSON sidecar that a reader on another OS
    opens, and `str(Path)` on Windows produces "models\\motionnet_v1.pt".
    Backslashes there are not merely ugly: they are a second escape level
    inside JSON, they do not round-trip as a path on POSIX, and they advertise
    which machine trained the model rather than which dataset did.
    """
    return pathlib.PurePath(p).as_posix()


def set_all_seeds(seed: int) -> None:
    """Pin every RNG used by the training process, including CUDA if present."""
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    # GRU has a deterministic CPU path; warn_only keeps CUDA from aborting.
    torch.use_deterministic_algorithms(True, warn_only=True)
    torch.backends.cudnn.benchmark = False


def horizon_weights(horizon: int, device: torch.device, dtype: torch.dtype) -> Tensor:
    """Near-term accuracy matters more for control; far-term for reacquisition."""
    return torch.linspace(1.0, 0.3, horizon, device=device, dtype=dtype)


def motion_loss(pred_residual: Tensor, cv: Tensor, target: Tensor,
                logits: Tensor, regime: Tensor) -> Tensor:
    """Smooth-L1 on CV+residual, horizon-weighted, plus 0.3 * regime CE."""
    weights = horizon_weights(pred_residual.shape[1], pred_residual.device, pred_residual.dtype)
    # §14.0i: divide by the residual scale so β=0.5 is the quadratic bowl
    # around a pixel-scale error, then the head still emits µrad.
    scale = pred_residual.new_tensor(RESIDUAL_SCALE)
    per_step = nn.functional.smooth_l1_loss(
        (pred_residual + cv) / scale, target / scale, beta=0.5, reduction="none"
    ).mean(-1)
    forecast_loss = (weights * per_step).mean()
    return forecast_loss + 0.3 * nn.functional.cross_entropy(logits, regime)


class _RawTrackView(Dataset):
    """Expose raw history beside the normalised window so CV stays in µrad."""

    def __init__(self, inner: TrackWindowDataset):
        self.inner = inner

    def __len__(self) -> int:
        return len(self.inner)

    def __getitem__(self, idx: int):
        hist_norm, future, regime = self.inner[idx]
        return (
            hist_norm,
            self.inner.history[idx].astype(np.float32),
            future,
            int(regime),
        )


def train(dataset_root: Path, output: Path, epochs: int = 60, batch_size: int = 256,
          seed: int = 1337, device_name: str = "cpu") -> dict[str, object]:
    set_all_seeds(seed)
    verify_split_disjoint(dataset_root)
    device = torch.device(device_name)
    train_data = _RawTrackView(TrackWindowDataset(dataset_root, "train"))
    val_data = _RawTrackView(TrackWindowDataset(dataset_root, "val"))
    generator = torch.Generator().manual_seed(seed)
    train_loader = DataLoader(train_data, batch_size=batch_size, shuffle=True,
                              generator=generator, num_workers=0)
    val_loader = DataLoader(val_data, batch_size=batch_size, shuffle=False, num_workers=0)

    model = MotionNet().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=3.0e-4, weight_decay=1.0e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=max(epochs, 1))
    best_val = float("inf")
    best_epoch = -1
    stale_epochs = 0
    history: list[dict[str, float]] = []

    for epoch in range(epochs):
        model.train()
        for hist_norm, hist_raw, future, regime in train_loader:
            hist_norm = hist_norm.to(device)
            hist_raw = hist_raw.to(device)
            future = future.to(device)
            regime = regime.to(device)
            cv = constant_velocity_forecast(hist_raw)
            optimizer.zero_grad(set_to_none=True)
            pred_res, logits = model(hist_norm)
            loss = motion_loss(pred_res, cv, future, logits, regime)
            loss.backward()
            optimizer.step()
        scheduler.step()
        val_loss = _val_loss(model, val_loader, device)
        history.append({"epoch": float(epoch + 1), "val_loss": val_loss})
        if val_loss < best_val:
            best_val = val_loss
            best_epoch = epoch + 1
            stale_epochs = 0
            output.parent.mkdir(parents=True, exist_ok=True)
            torch.save({"model": model.state_dict(), "seed": seed, "epoch": best_epoch,
                        "val_loss": best_val}, output)
        else:
            stale_epochs += 1
            if stale_epochs >= 8:
                break

    dummy = "dummy" in str(dataset_root).lower()
    result = {
        "status": ("PRELIMINARY — dummy data, not a trained model" if dummy else "trained"),
        "is_real_result": not dummy,
        "trained_on": posix(dataset_root),
        "model_card": f"docs/models/{output.stem}.md",
        "metrics_from_dummy_data" if dummy else "metrics": {
            "best_epoch": best_epoch,
            "best_val_loss": best_val,
            "epochs_completed": len(history),
            "seed": seed,
            "history": history,
        },
    }
    if dummy:
        result["_README"] = [
            "NOT A RESULT. Trained on dummy shards from "
            "tools/make_dummy_shards.py, which exists to prove the training "
            "loop runs end to end and explicitly refuses to impersonate a real "
            "dataset.",
            "There is no trained MotionNet and no real track factory until "
            "--gen-dataset writes SAT FrameTruth windows.",
        ]
    output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"best val_loss={best_val:.6f} at epoch {best_epoch}; seed={seed}")
    return result


def _val_loss(model: nn.Module, loader: DataLoader, device: torch.device) -> float:
    model.eval()
    total = 0.0
    count = 0
    with torch.no_grad():
        for hist_norm, hist_raw, future, regime in loader:
            hist_norm = hist_norm.to(device)
            hist_raw = hist_raw.to(device)
            future = future.to(device)
            regime = regime.to(device)
            pred_res, logits = model(hist_norm)
            cv = constant_velocity_forecast(hist_raw)
            total += float(motion_loss(pred_res, cv, future, logits, regime).item())
            count += 1
    return total / max(count, 1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("models/motionnet_v1.pt"))
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--seed", type=int, default=1337)
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args()
    train(args.dataset, args.out, args.epochs, args.batch_size, args.seed, args.device)


if __name__ == "__main__":
    main()
