# MotionNet v1

SAT-ML §6 GRU. Input `(30, 4)` tracker `[az, el, vaz, vel]` in µrad. Output
15-step residual over constant-velocity plus 4-class regime. ~6k parameters.

## Train

SAT simulator tracks only (`just motion-data`). Official SIH Dataset Link is
NA. Split by `(scenario_id, seed)`. Test shards opened only by
`ml/evaluate_motion.py`.

Factory used here: four regimes + hold-out `motion_linear_fast`, seeds 1–8,
8 s/run, 4704 / 1568 / 1568 windows. 20 AdamW epochs, seed 1337. Val loss
moved from 1394.39 to 1394.30 — the residual head stayed near zero.

Norm constants: `models/motion_norm.json` (1e5 / 1e5), baked into
`src/ai/motion_net.hpp`.

## SAT-ML §6.6 gate — FAILED (do not ship ONNX)

Measured by `python -m ml.evaluate_motion --dataset data/motion_v1 --ckpt models/motionnet_v1.pt`
on the **test** split only (`models/motionnet_v1.eval.json`).

| | +5 RMSE (urad) | +15 RMSE (urad) | regime acc @ ≥1s |
|---|---|---|---|
| CV | 2764.143 | 8292.429 | n/a |
| MotionNet | 2763.680 | 8291.701 | 0.000 |
| vs CV | +0.017 % | +0.009 % | |

Gate needed +5 ≤ 0.80 CV, +15 ≤ 0.65 CV, regime ≥ 0.90. None passed.
ONNX is implemented and parity-tested; the trained weights are **not**
committed as `models/motionnet_v1.onnx`.

Likely cause: 8 s windows, hold-out is an unseen *linear* file, and 20 epochs
did not move the residual off CV. Not a unit-mismatch — RMSE matches CV to
three figures, which is what a near-zero residual must do.

## End-to-end (official reacq / lock)

Short sweep (figure-8 + OU, seeds 1–2 / 1, 8 s). This build has
`SAT_WITH_ONNX=OFF` and no shipped ONNX, so both arms are IMM (INV-7).

| | reacquisition_s | target_loss_frac |
|---|---|---|
| `--no-ai` | 0.022 s | 0.114 |
| MotionNet | 0.022 s | 0.114 |

Identical rows: the model did not enter the loop. Do **not** claim CP 11.5.
See `issues_till_now.md`.

## Fails when

- first 1 s / empty history (row 16 first lock stays search+IMM)
- extreme low light
- a missing ONNX file (INV-7: IMM only)
- this v1 checkpoint: test-set regime head is 0 % and RMSE is CV
