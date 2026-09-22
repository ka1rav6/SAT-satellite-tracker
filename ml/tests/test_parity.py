"""Mandatory PyTorch/ONNX parity and graceful-fallback tests (§4.2/§4.3)."""

from __future__ import annotations

import logging

import numpy as np
import pytest
import torch

from ml.inference import CentroidInference, MotionInference
from ml.models.centroid import CentroidNet
from ml.models.motion import MotionNet, constant_velocity_forecast


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


def test_motion_onnx_matches_pytorch(tmp_path):
    """MotionNet ONNX must match PyTorch to 1e-5 on a fixed dummy."""
    pytest.importorskip("onnx")
    ort = pytest.importorskip("onnxruntime")
    from ml.export import export_motion

    torch.manual_seed(1337)
    model = MotionNet().eval()
    hist = torch.randn(1, 30, 4)
    onnx_path = tmp_path / "motion.onnx"
    export_motion(model, onnx_path)
    with torch.no_grad():
        forecast, logits = model(hist)
    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    ort_f, ort_l = session.run(None, {"hist": hist.numpy()})
    assert np.max(np.abs(forecast.numpy() - ort_f)) < 1e-5
    assert np.max(np.abs(logits.numpy() - ort_l)) < 1e-5


@pytest.mark.parametrize("model_name", ["missing_motion.onnx", "corrupt_motion.onnx"])
def test_missing_motion_model_uses_cv_fallback(tmp_path, caplog, model_name):
    """INV-7 / ML-1: a missing MotionNet file must warn and use CV."""
    model_path = tmp_path / model_name
    if model_name.startswith("corrupt"):
        model_path.write_bytes(b"this is not an ONNX graph")
    hist = np.zeros((30, 4), dtype=np.float32)
    hist[:, 2] = 1000.0
    with caplog.at_level(logging.WARNING):
        inference = MotionInference(model_path)
    assert inference.using_fallback
    assert "MotionNet" in caplog.text
    forecast, logits = inference.predict(hist)
    assert forecast.shape == (15, 2)
    expected = constant_velocity_forecast(torch.from_numpy(hist[None])).numpy()[0]
    assert np.allclose(forecast, expected, atol=1e-5)
    assert logits.shape == (4,)
    assert np.isfinite(forecast).all()
