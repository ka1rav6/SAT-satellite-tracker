#!/usr/bin/env python3
"""Build the deterministic ONNX fixture that exercises the MotionNet wrapper.

WHY THIS EXISTS
---------------
SAT-ML §4.2 makes a C++ parity test MANDATORY and shows one: load a committed
model, run a committed fixture through it, compare against a committed
expected value. The MotionNet wrapper shipped with a test shaped like that
which cannot fail:

    auto r = MotionNet::load("models/motionnet_v1.onnx");
    if (!r) { CHECK(r.error().size() > 0); return; }   // <- always taken

`models/motionnet_v1.onnx` is a training artifact and is gitignored, so in CI
that branch is always the early return. Everything between `normalise_hist`
and `argmax_regime` — the input scaling, the constant-velocity composition,
the softmax, the off-training-rate flag — had no test that ran anywhere.

THIS IS NOT A MODEL
-------------------
It is a FIXTURE, and the distinction is the point. It has MotionNet's exact
input and output signature and none of its weights; it computes a closed form
chosen so the C++ side's expected values can be worked out by hand rather than
read back from a previous run of the code under test.

It lives in `tests/ai/`, never in `models/`, and is named `fixture` so it
cannot be mistaken for `motionnet_v1.onnx` in a directory listing, in a log
line, or by a future reader deciding what to ship. It must never be used to
produce a published number.

WHAT THE GRAPH COMPUTES
-----------------------
Input  `hist`     (1, 30, 4)  -- the NORMALISED history the wrapper builds
Output `forecast` (1, 15, 2)  -- a fixed residual, R[k] = (k+1, -(k+1)) urad
Output `logits`   (1, 4)      -- per-channel sums of the normalised history

`forecast` is constant, so the composed result the wrapper returns is exactly

    step[k] = R[k] + v_last * (k + 1) * dt

which isolates the composition and the dt handling: any error in either shows
up as a mismatch against arithmetic the test does in one line.

`logits` is a function OF THE INPUT, which is what pins the normalisation. If
`normalise_hist` stopped subtracting the last position, or used the wrong
scale, the sums change and the test fails. Making the logits a reduction also
gives the argmax and the softmax something non-trivial to chew on.

REGENERATE
----------
    python3 tools/make_motion_fixture.py

Deterministic: the same file, byte for byte, every time. Commit the result.
"""

import pathlib
import sys

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

HISTORY_LEN, HORIZON, INPUT_SIZE, N_REGIMES = 30, 15, 4, 4

OUT = pathlib.Path(__file__).resolve().parent.parent / "tests" / "ai" / "motion_fixture.onnx"


def build() -> onnx.ModelProto:
    # R[k] = (k+1, -(k+1)) urad. Asymmetric between the axes on purpose: a
    # wrapper that transposed or mirrored the two would pass against any
    # symmetric fixture.
    residual = np.zeros((1, HORIZON, 2), dtype=np.float32)
    for k in range(HORIZON):
        residual[0, k, 0] = float(k + 1)
        residual[0, k, 1] = -float(k + 1)

    nodes = [
        # forecast: the constant above, ignoring the input entirely.
        helper.make_node(
            "Constant", [], ["forecast"],
            value=numpy_helper.from_array(residual, name="residual_const"),
        ),
        # logits[c] = sum over the 30 timesteps of the normalised channel c.
        helper.make_node("ReduceSum", ["hist", "axes"], ["logits"], keepdims=0),
    ]

    graph = helper.make_graph(
        nodes,
        "motion_fixture",
        inputs=[helper.make_tensor_value_info(
            "hist", TensorProto.FLOAT, [1, HISTORY_LEN, INPUT_SIZE])],
        outputs=[
            helper.make_tensor_value_info(
                "forecast", TensorProto.FLOAT, [1, HORIZON, 2]),
            helper.make_tensor_value_info(
                "logits", TensorProto.FLOAT, [1, N_REGIMES]),
        ],
        initializer=[
            numpy_helper.from_array(np.array([1], dtype=np.int64), name="axes"),
        ],
    )

    # Opset 17, matching SAT-ML §4.1's pin and the real exporter, so the
    # fixture exercises the same ORT code path the model will.
    model = helper.make_model(
        graph, producer_name="sat-motion-fixture",
        opset_imports=[helper.make_operatorsetid("", 17)])
    model.ir_version = 9   # ORT 1.17 rejects anything newer; pinned, not latest
    model.doc_string = (
        "NOT A TRAINED MODEL. Deterministic fixture for the MotionNet C++ "
        "wrapper (SAT-ML 4.2). forecast is a fixed residual; logits are "
        "per-channel sums of the normalised history. Regenerate with "
        "tools/make_motion_fixture.py."
    )
    onnx.checker.check_model(model)
    return model


def main() -> int:
    model = build()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_bytes(model.SerializeToString())
    print(f"wrote {OUT.relative_to(OUT.parent.parent.parent)} "
          f"({OUT.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
