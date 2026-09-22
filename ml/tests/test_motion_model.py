"""MotionNet shape and parameter-budget tests (SAT-ML.md §6.2)."""

from __future__ import annotations

import torch

from ml.models.motion import HISTORY_LEN, HORIZON, INPUT_SIZE, MotionNet, N_REGIMES


def test_constant_velocity_forecast_is_exact_on_a_linear_track():
    """A constant-rate polyline must match CV to 1e-4 µrad — that is the baseline."""
    from ml.models.motion import constant_velocity_forecast

    dt = 1.0 / 30.0
    rate_az = 1000.0
    hist = torch.zeros(1, HISTORY_LEN, INPUT_SIZE)
    hist[0, :, 2] = rate_az
    hist[0, :, 0] = torch.arange(HISTORY_LEN, dtype=torch.float32) * rate_az * dt
    forecast = constant_velocity_forecast(hist, dt_s=dt)
    expected = torch.arange(1, HORIZON + 1, dtype=torch.float32) * rate_az * dt
    assert forecast.shape == (1, HORIZON, 2)
    assert torch.allclose(forecast[0, :, 0], expected, atol=1e-4)


def test_train_on_dummy_shards_writes_non_result_sidecar(tmp_path):
    """A dummy-path train run must refuse to look like a scored SAT-ML §6.6 result."""
    from tools.make_dummy_shards import generate
    from ml.train_motion import train

    root = tmp_path / "dummy_tracks"
    generate(root, task="tracks", n_per_split=16)
    out = tmp_path / "motionnet_dummy.pt"
    sidecar = train(root, out, epochs=2, batch_size=8)
    assert sidecar["is_real_result"] is False
    assert out.with_suffix(".json").is_file()


def test_motionnet_shapes_and_param_budget():
    """GRU must emit 15-step residuals plus 4 logits and stay under 12k params."""
    model = MotionNet()
    hist = torch.zeros(3, HISTORY_LEN, INPUT_SIZE)
    forecast, logits = model(hist)
    assert forecast.shape == (3, HORIZON, 2)
    assert logits.shape == (3, N_REGIMES)
    n = sum(p.numel() for p in model.parameters())
    assert n < 12_000  # spec ~6k; fail if someone "improves" it into a transformer
