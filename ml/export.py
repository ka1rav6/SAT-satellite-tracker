"""Export fixed-shape SAT models to ONNX Runtime's deployment boundary."""

from __future__ import annotations

from pathlib import Path

import torch

from ml.models.centroid import CentroidNet
from ml.models.motion import HISTORY_LEN, INPUT_SIZE, MotionNet


def export(model: torch.nn.Module, out_path: Path, patch_px: int = 15) -> None:
    """Export one fixed batch-1 CentroidNet graph as specified by section 4.1."""
    if patch_px != 15:
        raise ValueError("SAT models use fixed 15x15 inputs")
    model.eval()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    dummy_patch = torch.zeros(1, 1, patch_px, patch_px)
    dummy_scalars = torch.zeros(1, 2)
    torch.onnx.export(
        model,
        (dummy_patch, dummy_scalars),
        out_path,
        input_names=["patch", "scalars"],
        output_names=["offset"],
        opset_version=17,
        dynamic_axes=None,
        do_constant_folding=True,
    )


def export_motion(model: torch.nn.Module, out_path: Path) -> None:
    """Fixed batch-1 MotionNet graph: hist (1,30,4) -> forecast, logits."""
    model.eval()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    dummy = torch.zeros(1, HISTORY_LEN, INPUT_SIZE)
    torch.onnx.export(
        model,
        dummy,
        out_path,
        input_names=["hist"],
        output_names=["forecast", "logits"],
        opset_version=17,
        dynamic_axes=None,
        do_constant_folding=True,
    )


def export_motion_checkpoint(checkpoint_path: Path, out_path: Path) -> None:
    model = MotionNet()
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    model.load_state_dict(checkpoint.get("model", checkpoint))
    export_motion(model, out_path)


def export_checkpoint(checkpoint_path: Path, out_path: Path) -> None:
    model = CentroidNet()
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    model.load_state_dict(checkpoint.get("model", checkpoint))
    export(model, out_path)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--task", choices=("centroid", "motion"), default="centroid")
    args = parser.parse_args()
    if args.task == "motion":
        export_motion_checkpoint(args.checkpoint, args.out)
    else:
        export_checkpoint(args.checkpoint, args.out)