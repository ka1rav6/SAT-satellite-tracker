# CandidateNet v1

> **PRELIMINARY — trained on synthetic dummy data, re-run once CP 11.1 exists.**
> No performance number below is a real-data claim.

Purpose        Classify a CFAR proposal as beacon, decoy, clutter, or noise
Input          15x15 float patch (background-subtracted, per-patch normalised)
               + scalars [snr, area, fill_ratio, aspect, matched_response,
               dist_from_prediction]
Output         Four logits in beacon / decoy / clutter / noise order

Parameters     4,436 (the exact architecture in SAT-ML.md §5.2)
Size           Not measured yet (FP32 checkpoint and INT8 export pending)
Latency        Not measured yet; target 0.15 ms for 20 proposals
Opset          Not exported yet
ORT version    Not pinned in the current C++ vcpkg manifest; record at export

Dataset        Dummy candidates fixture only
               manifest sha256 recorded by `tools/make_dummy_shards.py`
               sweep config sha256: synthetic-fixture
Split          BY RUN: seeds 1-700 train, 701-850 val,
               851-1000 test on scenarios Q-T (UNSEEN during training)
Training seed  1337

Test AUC       Pending: overall and fog + lowlight subset
Calibration    Pending; target expected calibration error < 0.05
Decoy reject   Pending at 1% beacon-miss rate; target > 90%

End-to-end     Pending C++ CandidateNet integration and ablation

Fails when     A decoy has beacon-like shape and lies near the tracker's
               prediction, or when the proposal gate rejects the beacon before
               this model is called.
Fallback       The classical shape / area / fill / aspect gate from
               SAT-DESIGN.md §9.4.7