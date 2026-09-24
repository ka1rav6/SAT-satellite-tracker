"""Neural models specified by SAT-ML.md.

The package keeps model definitions independent from training and inference
entry points so the same module is used for checkpoint loading, ONNX export,
and parity tests.  Runtime C++ integration remains optional and must always
retain the classical fallbacks described in SAT-DESIGN.md section 11.
"""

from .candidate import CandidateNet
from .centroid import CentroidNet
from .motion import MotionNet

__all__ = ["CandidateNet", "CentroidNet", "MotionNet"]