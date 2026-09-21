"""Train CentroidNet according to SAT-ML.md sections 3.3 and 3.4.

Only train and validation splits are opened here.  The test split is reserved
for a separate final-evaluation command so ML-7 is visible in the workflow and
cannot be violated accidentally by a training diagnostic.
"""

from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader

from ml.datasets import CentroidPatchDataset, verify_split_disjoint
from ml.models.centroid import CentroidNet


def set_all_seeds(seed: int) -> None:
    """Pin every RNG used by the training process, including CUDA if present."""
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.use_deterministic_algorithms(True)
    torch.backends.cudnn.benchmark = False


def _rmse(model: nn.Module, loader: DataLoader, device: torch.device) -> float:
    model.eval()
    squared = 0.0
    count = 0
    with torch.no_grad():
        for patches, scalars, targets in loader:
            prediction = model(patches.to(device), scalars.to(device))
            squared += float(torch.sum((prediction - targets.to(device)) ** 2).item())
            count += int(targets.numel())
    return float(np.sqrt(squared / max(count, 1)))


def train(dataset_root: Path, output: Path, epochs: int = 60, batch_size: int = 512,
          seed: int = 1337, device_name: str = "cpu") -> dict[str, object]:
    set_all_seeds(seed)
    verify_split_disjoint(dataset_root)
    device = torch.device(device_name)
    train_data = CentroidPatchDataset(dataset_root, "train")
    val_data = CentroidPatchDataset(dataset_root, "val")
    generator = torch.Generator().manual_seed(seed)
    train_loader = DataLoader(train_data, batch_size=batch_size, shuffle=True, generator=generator,
                              num_workers=0)
    val_loader = DataLoader(val_data, batch_size=batch_size, shuffle=False, num_workers=0)

    model = CentroidNet().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=3.0e-4, weight_decay=1.0e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    best_rmse = float("inf")
    best_epoch = -1
    stale_epochs = 0
    history: list[dict[str, float]] = []

    for epoch in range(epochs):
        model.train()
        for patches, scalars, targets in train_loader:
            optimizer.zero_grad(set_to_none=True)
            prediction = model(patches.to(device), scalars.to(device))
            loss = nn.functional.smooth_l1_loss(prediction, targets.to(device), beta=0.05)
            loss.backward()
            optimizer.step()
        scheduler.step()
        val_rmse = _rmse(model, val_loader, device)
        history.append({"epoch": float(epoch + 1), "val_rmse_px": val_rmse})
        if val_rmse < best_rmse:
            best_rmse = val_rmse
            best_epoch = epoch + 1
            stale_epochs = 0
            output.parent.mkdir(parents=True, exist_ok=True)
            torch.save({"model": model.state_dict(), "seed": seed, "epoch": best_epoch,
                        "val_rmse_px": best_rmse}, output)
        else:
            stale_epochs += 1
            if stale_epochs >= 8:
                break

    # ------------------------------------------------------------------
    # P2-10: the sidecar says WHERE THE NUMBER CAME FROM, not just what it is.
    #
    # This used to write the metrics block alone. A JSON file whose only
    # content is `"best_val_rmse_px": 0.2744` reads as a result to a person and
    # parses as one to a script, and models/centroidnet_v1.json sat in the
    # repository looking exactly like a trained model's scorecard while
    # docs/SAT-ML.md declared ML unbuilt. The only disclaimer was in the model
    # card beside it, which is the one place a script will never look.
    #
    # `is_real_result` is derived from the dataset path rather than passed in,
    # so it cannot be forgotten: a run over tools/make_dummy_shards.py output
    # is self-evidently not a result, and saying so is the file's job.
    #
    # The classical baseline is carried alongside because it is the number that
    # decides whether a learned centroider is worth shipping at all — 0.141 px
    # from `just sweep`, against which 0.274 px on dummy data is a regression.
    # A model card that reports only the model's own score invites the wrong
    # comparison, which is to nothing.
    # ------------------------------------------------------------------
    dummy = "dummy" in str(dataset_root).lower()
    result = {
        "status": ("PRELIMINARY — dummy data, not a trained model" if dummy
                   else "trained"),
        "is_real_result": not dummy,
        "trained_on": str(dataset_root),
        "classical_baseline_px": 0.141,
        "classical_baseline_source":
            "just sweep, clear air, clutter-free, 30 s per cell",
        "model_card": f"docs/models/{output.stem}.md",
        "metrics_from_dummy_data" if dummy else "metrics": {
            "best_epoch": best_epoch,
            "best_val_rmse_px": best_rmse,
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
            "There is no trained model and no real dataset, because "
            "--gen-dataset is not implemented (design 13.4).",
            f"The CLASSICAL centroider measures 0.141 px on the same graded "
            f"metric, so {best_rmse:.4f} px would be a REGRESSION if it were "
            f"real. That is the argument for targeting CandidateNet first.",
        ]
    output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"best val_rmse_px={best_rmse:.6f} at epoch {best_epoch}; seed={seed}")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("models/centroidnet_v1.pt"))
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--seed", type=int, default=1337)
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args()
    train(args.dataset, args.out, args.epochs, args.batch_size, args.seed, args.device)


if __name__ == "__main__":
    main()