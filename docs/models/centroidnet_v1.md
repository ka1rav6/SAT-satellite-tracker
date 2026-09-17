# CentroidNet v1

> **PRELIMINARY — trained on synthetic dummy data, re-run once CP 11.1 exists.**
> No performance number below is a real-data claim.

Purpose        Sub-pixel beacon centre from a 15x15 patch
Input          15x15 float patch (background-subtracted, per-patch normalised)
               + scalars [snr, size_est]
Output         (dx, dy) offset from patch centre, in pixels

Parameters     236,306 (the exact architecture in SAT-ML.md §3.2)
Size           Not measured yet (FP32 checkpoint and INT8 export pending)
Latency        Not measured yet; target 0.02 ms, 1 call per frame
Opset          17
ORT version    Not pinned in the current C++ vcpkg manifest; record at export

Dataset        Dummy centroid_patches fixture only
               manifest sha256 recorded by `tools/make_dummy_shards.py`
               sweep config sha256: synthetic-fixture
Split          BY RUN: seeds 1-700 train, 701-850 val,
               851-1000 test on scenarios Q-T (UNSEEN during training)
Training seed  1337

Test RMSE      Pending: run `ml/evaluate_final.py --task centroid`
vs bound       Pending real simulator data
Residual bias  Pending real simulator data
Calibration    Not applicable to this two-output version; no uncertainty head

End-to-end     Pending C++ ONNX integration and ablation

Fails when     A patch contains overlapping beacon and clutter peaks, or the
               beacon is absent from the proposed patch; the regression cannot
               recover information that CFAR did not provide.
Fallback       Bias-corrected WindowedCoM in C++; deterministic classical
               centre-of-mass proof path in `ml/inference.py`