"""Mandatory PyTorch/ONNX parity and graceful-fallback tests (§4.2/§4.3)."""

from __future__ import annotations

import logging

import numpy as np
import pytest
import torch

from ml.inference import CentroidInference
from ml.models.centroid import CentroidNet


def test_onnx_matches_pytorch(tmp_path):
    """Fixed-shape exported output must agree to the specified 1e-5 tolerance."""
    pytest.importorskip("onnx")
    ort = pytest.importorskip("onnxruntime")
    from ml.export import export

    torch.manual_seed(1337)
    model = CentroidNet().eval()
    patch = torch.randn(1, 1, 15, 15)
    scalars = torch.randn(1, 2)
    onnx_path = tmp_path / "centroid.onnx"
    export(model, onnx_path)
    with torch.no_grad():
        torch_output = model(patch, scalars).numpy()
    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    ort_output = session.run(None, {"patch": patch.numpy(), "scalars": scalars.numpy()})[0]
    assert np.max(np.abs(torch_output - ort_output)) < 1e-5


@pytest.mark.parametrize("model_name", ["missing.onnx", "corrupt.onnx"])
def test_missing_or_corrupt_model_uses_classical_fallback(tmp_path, caplog, model_name):
    """ML-1: deleting or corrupting an optional model warns and still predicts."""
    model_path = tmp_path / model_name
    if model_name.startswith("corrupt"):
        model_path.write_bytes(b"this is not an ONNX graph")
    with caplog.at_level(logging.WARNING):
        inference = CentroidInference(model_path)
    assert inference.using_fallback
    assert "CentroidNet" in caplog.text
    output = inference.predict(np.ones((15, 15), dtype=np.float32), np.asarray([8.0, 10.0]))
    assert output.shape == (2,)
    assert np.isfinite(output).all()
