"""Train CandidateNet under SAT-ML.md sections 5.2 and 5.3.

This command follows the same ML-7 boundary as CentroidNet: only train and
validation data are opened during fitting.  The exact class weights and
label-smoothing value come from the specification; no resampling or data
augmentation is introduced because either would change the calibrated class
prior.
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

from ml.datasets import CandidatePatchDataset, verify_split_disjoint
from ml.models.candidate import CandidateNet


def set_all_seeds(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.use_deterministic_algorithms(True)
    torch.backends.cudnn.benchmark = False


def train(dataset_root: Path, output: Path, epochs: int = 60, batch_size: int = 512,
          seed: int = 1337, device_name: str = "cpu") -> dict[str, object]:
    set_all_seeds(seed)
    verify_split_disjoint(dataset_root)
    device = torch.device(device_name)
    train_data = CandidatePatchDataset(dataset_root, "train")
    val_data = CandidatePatchDataset(dataset_root, "val")
    generator = torch.Generator().manual_seed(seed)
    train_loader = DataLoader(train_data, batch_size=batch_size, shuffle=True, generator=generator,
                              num_workers=0)
    val_loader = DataLoader(val_data, batch_size=batch_size, shuffle=False, num_workers=0)

    model = CandidateNet().to(device)
    raw_weights = torch.tensor([1 / 0.06, 1 / 0.04, 1 / 0.31, 1 / 0.59], dtype=torch.float32)
    weights = raw_weights / raw_weights.sum() * 4.0
    loss_fn = nn.CrossEntropyLoss(weight=weights.to(device), label_smoothing=0.02)
    optimizer = torch.optim.AdamW(model.parameters(), lr=3.0e-4, weight_decay=1.0e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    best_accuracy = -1.0
    stale_epochs = 0
    history: list[dict[str, float]] = []

    for epoch in range(epochs):
        model.train()
        for patches, scalars, labels in train_loader:
            optimizer.zero_grad(set_to_none=True)
            logits = model(patches.to(device), scalars.to(device))
            loss_fn(logits, labels.to(device)).backward()
            optimizer.step()
        scheduler.step()
        accuracy = _accuracy(model, val_loader, device)
        history.append({"epoch": float(epoch + 1), "val_accuracy": accuracy})
        if accuracy > best_accuracy:
            best_accuracy = accuracy
            stale_epochs = 0
            output.parent.mkdir(parents=True, exist_ok=True)
            torch.save({"model": model.state_dict(), "seed": seed, "epoch": epoch + 1,
                        "val_accuracy": accuracy}, output)
        else:
            stale_epochs += 1
            if stale_epochs >= 8:
                break

    result = {"best_val_accuracy": best_accuracy, "epochs_completed": len(history),
              "seed": seed, "history": history}
    output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"best val_accuracy={best_accuracy:.6f}; seed={seed}")
    return result


def _accuracy(model: nn.Module, loader: DataLoader, device: torch.device) -> float:
    model.eval()
    correct = total = 0
    with torch.no_grad():
        for patches, scalars, labels in loader:
            prediction = model(patches.to(device), scalars.to(device)).argmax(dim=1)
            correct += int((prediction == labels.to(device)).sum().item())
            total += int(labels.numel())
    return correct / max(total, 1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("models/candidatenet_v1.pt"))
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--seed", type=int, default=1337)
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args()
    train(args.dataset, args.out, args.epochs, args.batch_size, args.seed, args.device)


if __name__ == "__main__":
    main()