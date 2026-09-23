# MotionNet v1

SAT-ML §6 GRU. Input `(30, 4)` tracker `[az, el, vaz, vel]` in µrad. Output
15-step residual over constant-velocity plus 4-class regime. ~6k parameters.
The head is multiplied by `RESIDUAL_SCALE` (1e4) so the exported tensor is
µrad; training applies SmoothL1 after dividing by that scale (SAT-DESIGN §14.0i).

## Train

SAT simulator tracks only (`just motion-data`). Official SIH Dataset Link is
NA. Split by `(scenario_id, seed)`. Test shards opened only by
`ml/evaluate_motion.py`.

Factory: linear, circular (period 2.5 s), figure-8 (period 4 s and 7.5 s),
OU, plus a held-out figure-8 (period 5.5 s, 4 seeds). Main seeds 1–30, 10 s/run.
Windows whose tracker was more than 12_000 µrad from FrameTruth were dropped.
Short `occlude_target` events create the CFAR misses.

Norm constants for the *input*: `models/motion_norm.json` (1e5 / 1e5), baked
into `src/ai/motion_net.hpp` as `kPosScale` / `kRateScale`. The residual scale
is inside the graph, not a second C++ multiply.

## SAT-ML §6.6 gate — PASSED

Measured by `python -m ml.evaluate_motion --dataset data/motion_v1 --ckpt models/motionnet_v1.pt`
on the **test** split only (`models/motionnet_v1.eval.json`). 60 AdamW epochs,
seed 1337, best val at epoch 54.

| | +5 RMSE (µrad) | +15 RMSE (µrad) | regime acc @ ≥1s |
|---|---|---|---|
| CV | 2428.825 | 12221.627 | n/a |
| MotionNet | 1318.304 | 5634.252 | 0.921 |
| vs CV | 45.7% lower | 53.9% lower | |

Gate needed +5 ≤ 0.80 CV (20% better), +15 ≤ 0.65 CV (35% better), regime ≥ 0.90.
All three passed. `models/motionnet_v1.onnx` is the export of this checkpoint.
Weights are gitignored; regenerate with `just train-motion` then `just export-motion`.

## End-to-end (official reacq / lock)

With ONNX Runtime 1.17.1 linked, the same 3-seed / 8 s sweep (figure-8, OU,
fog_figure8) loads the net. `set_regime_prior` runs (117 applies on the
figure-8 unit test). Official means:

| | reacquisition_s | target_loss_frac |
|---|---|---|
| `--no-ai` | 0.234 s | 0.089 |
| MotionNet | 0.234 s | 0.089 |

Reacquisition and target loss did not move. On figure-8 seed 1, tracking
RMS went from 29.08 px to 29.65 px. The coast gate (`k * position_sigma`)
rejects the turn residual, so the forecast never recentres search, and the
regime prior does not buy a lock frame. Do **not** claim CP 11.5. The §6.6
forecast gate is what this checkpoint passes.

## Fails when

- first 1 s / empty history (row 16 first lock stays search+IMM)
- the tracker is on clutter (those windows are not in the training set)
- a figure-8 whose period is far from both 4 s and 7.5 s (the held-out 5.5 s
  period is the one that was measured)
- extreme low light beyond the factory weather
- a missing ONNX file (INV-7: IMM only)
