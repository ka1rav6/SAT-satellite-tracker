"""Optional CentroidNet inference with the mandatory classical fallback.

The C++ application owns production inference, but this small Python adapter
makes ML-1 and ML-2 executable before that integration lands.  Loading a
missing, corrupt, or unavailable ONNX model emits a warning and routes every
prediction through a deterministic windowed centre-of-mass estimator instead
of turning an optional enhancement into a startup failure.
"""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np

from ml.datasets import normalise_patch, normalise_scalars

LOGGER = logging.getLogger(__name__)


def classical_centroid(patch: np.ndarray) -> np.ndarray:
    """Return the background-subtracted, weighted centre offset in pixels."""
    normalised = normalise_patch(np.asarray(patch))
    axis = np.arange(normalised.shape[0], dtype=np.float32) - (normalised.shape[0] - 1) / 2.0
    total = float(normalised.sum())
    if total <= 1e-6:
        return np.zeros(2, dtype=np.float32)
    x_mass = float((normalised.sum(axis=0) * axis).sum() / total)
    y_mass = float((normalised.sum(axis=1) * axis).sum() / total)
    return np.asarray([x_mass, y_mass], dtype=np.float32)


class CentroidInference:
    """Run ONNX when available; otherwise use the classical fallback."""

    def __init__(self, model_path: Path | None):
        self.session = None
        self.using_fallback = True
        if model_path is None:
            LOGGER.warning("CentroidNet model is disabled; using classical fallback")
            return
        try:
            import onnxruntime as ort

            self.session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
            self.using_fallback = False
        except Exception as error:  # model absence/corruption must never crash ML-1
            LOGGER.warning("CentroidNet failed to load (%s); using classical fallback", error)

    def predict(self, patch: np.ndarray, scalars: np.ndarray) -> np.ndarray:
        if self.session is None:
            return classical_centroid(patch)
        patch_input = normalise_patch(patch)[None, None, ...].astype(np.float32)
        scalar_input = normalise_scalars(np.asarray(scalars))[None, ...].astype(np.float32)
        return np.asarray(self.session.run(["offset"], {"patch": patch_input, "scalars": scalar_input})[0][0])