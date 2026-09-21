# SAT-ML — Machine Learning and Computer Vision Specification
### Companion to `SAT-DESIGN.md` · SIH PS 26169

---

# 0. HOW TO USE THIS DOCUMENT

This document covers everything on the **Python / training side**, plus the OpenCV usage boundary.
`SAT-DESIGN.md` covers the C++ system and the runtime integration contract (its §11).

**Working order.** The models here cannot be built until the simulator can generate labelled data,
which is Stage 7 of the main roadmap. Do not start §3 before the compliance matrix works — you will
have no way to tell whether a model helped.

**The build order within this document is fixed and deliberate:**

```
§2  Data factory        →  nothing else works without it
§3  CentroidNet         →  worth the most marks; train first
§4  Integration + --no-ai  →  prove ML is optional before adding more
§5  CandidateNet
§6  MotionNet
§7  RecoveryNet         (optional)
§8  StrategyPolicy      →  needs the others to exist first
§9  SearchPolicy        (optional, high risk)
§10 Evaluation + ablation  →  the artifact that actually earns marks
```

---

# 1. THE ML STRATEGY

## 1.1 Why this is not decoration

The problem statement is titled *"AI-Based Virtual Camera Tracking System"*, and **"AI and computer
vision"** is a named criterion in the 20% Technical Evaluation. But ML has to be placed honestly, or
it costs you marks rather than earning them.

**The honest starting position:** the beacon is a uniform square, 5–20 px, with no texture, no
pattern and no variety. There is genuinely nothing about its *appearance* for a network to learn that
a matched filter does not already compute optimally, and the matched filter is ~1000× faster.

**Therefore: reject object detectors explicitly, and say why in the report.** YOLO, DETR, SSD and
friends are the wrong tool, and an ISRO evaluator will recognise that. Writing a paragraph explaining
the rejection is worth more under "Selection of Algorithms" than using one would be.

## 1.2 Where learning genuinely helps

Five places where classical methods are demonstrably weak:

| Weakness | Why classical struggles | Model | Marks it attacks |
|---|---|---|---|
| **Sub-pixel centre at low SNR** | The estimator is both biased and noisy; window truncation causes pixel-locking | **M1 `CentroidNet`** | **60%** — centroiding error, both benchmark stages |
| **Beacon vs decoy vs clutter** | Hand-written shape gates capture only a few cues | M2 `CandidateNet` | False track rate, lock retention |
| **Predicting through a dropout** | One fixed kinematic model cannot cover four dissimilar motion types | M3 `MotionNet` | Re-acquisition ≤1 s, lock retention |
| **Total detection failure in extreme conditions** | CFAR returns nothing when signal ≈ noise | M4 `RecoveryNet` | Acquisition, lock retention |
| **Choosing what to do as conditions change** | A hand-written rule table is a guess; the right answer can be *measured* | M5 `StrategyPolicy` | All of them; the innovation story |

Plus one optional, high-risk, high-reward:

| **Where to look during acquisition** | The hardest requirement is search-limited | M6 `SearchPolicy` | Acquisition time |

## 1.3 Non-negotiable rules

| # | Rule | Why |
|---|---|---|
| ML-1 | **Every model has a classical fallback.** | INV-7. A missing model file must never crash. |
| ML-2 | **`--no-ai` must pass the full compliance matrix.** | Makes ML an enhancement, not a dependency. Insurance for the live demo. |
| ML-3 | **Split datasets by run, never by frame.** | §2.3. Frame-level splitting inflates scores and hides the exact failure the evaluators' unseen videos will expose. |
| ML-4 | **Report end-to-end ablations, not just model accuracy.** | §10.3. Almost nobody does this; it is the highest-value ML artifact. |
| ML-5 | **Never claim ML is universally better. Show the crossover.** | §10.4. Honest and quantified beats "99% accuracy". |
| ML-6 | **No data augmentation.** | The simulator produces genuine variation across conditions. Synthetic augmentation of a fixed set is strictly worse. |
| ML-7 | **The test set is touched once, at the end.** | Standard hygiene; also the only measurement that predicts BP-2. |
| ML-8 | **Every model ships a model card.** | §11. The "fails when" line is what demonstrates understanding. |

---

# 2. THE DATA FACTORY

## 2.1 Why this is your biggest advantage

Most ML projects are limited by labelled data. **You have a machine that produces perfectly labelled
data at ~400× real time, in any condition you choose, for free.**

Every label comes from `FrameTruth`:
- exact sub-pixel beacon centre (to ~0.001 px, because rendering uses analytic coverage — see
  `SAT-DESIGN.md` §9.2)
- exact identity of every blob (target / decoy / clutter / hot pixel)
- exact motion regime
- exact SNR, contrast and every condition parameter

Zero annotation error. Say this prominently in the report — it is a direct consequence of having
built the simulator correctly.

## 2.2 Generation

```
sat-tracker --gen-dataset --sweep ml/sweeps/centroid_v2.toml --out data/centroid_v2/
```

```toml
# ml/sweeps/centroid_v2.toml
[sweep]
scenarios = ["baseline", "fog", "haze", "rain", "lowlight",
             "maxnoise", "clutter_dense", "decoy", "figure8", "random_walk",
             ...]                              # 20 scenarios, A-T
seeds     = { start = 1, end = 1000 }
duration_s = 20

[capture]
what          = "centroid_patches"     # centroid_patches | candidates | tracks | strategy
patch_px      = 15
max_per_run   = 400
negative_ratio = 0.0                   # centroid task: positives only
```

Output layout:

```
data/centroid_v2/
├── manifest.json
├── train/  shard_0000.npz  shard_0001.npz  …
├── val/    shard_0000.npz  …
└── test/   shard_0000.npz  …
```

Each shard is an `.npz` with aligned arrays: `patches`, `scalars`, `labels`, `run_id`, `scenario_id`,
`seed`, `conditions`.

## 2.3 ★ The splitting rule

**Split by (scenario, seed). Never by frame.**

Frames within one run are near-duplicates — the beacon moves a few pixels between them. If you
shuffle frames and then split, near-identical images land in both training and test sets, your test
score is inflated, and the model fails the moment it sees the evaluators' footage.

```
train : seeds   1-700  × scenarios A-P
val   : seeds 701-850  × scenarios A-P     (same scenarios, unseen seeds)
test  : seeds 851-1000 × scenarios Q-T     ← UNSEEN SCENARIOS *and* UNSEEN SEEDS
```

The test set uses scenarios the model has never seen. **This is the closest proxy you have for the
evaluators' MP4 files.** Describing this split explicitly in the report is one of the cheapest ways
to look competent under "AI and computer vision".

```python
# ml/datasets.py
def split_by_run(manifest):
    train_scen = SCENARIOS[:16]
    test_scen  = SCENARIOS[16:]
    return {
        "train": lambda r: r.scenario in train_scen and   1 <= r.seed <= 700,
        "val":   lambda r: r.scenario in train_scen and 701 <= r.seed <= 850,
        "test":  lambda r: r.scenario in test_scen  and 851 <= r.seed <= 1000,
    }

def assert_no_leakage(train_runs, val_runs, test_runs):
    """Fail loudly if any run_id appears in more than one split."""
    assert not (set(train_runs) & set(val_runs))
    assert not (set(train_runs) & set(test_runs))
    assert not (set(val_runs)   & set(test_runs))
```

Run `assert_no_leakage` in CI. Leakage is silent and catastrophic.

## 2.4 Manifest

```json
{
  "name": "centroid_v2",
  "generated_utc": "2026-09-14T10:22:31Z",
  "generator_build": "a3f21c9",
  "sweep_config_sha256": "9f2a…",
  "n_samples": 806400,
  "patch_px": 15,
  "conditions": { "clear": 0.2, "haze": 0.2, "rain": 0.2, "fog": 0.2, "lowlight": 0.2 },
  "snr_bins": [3, 5, 8, 12, 18, 25, 35, 50],
  "size_bins": [5, 8, 11, 14, 17, 20],
  "split": { "train": 564480, "val": 120960, "test": 120960 },
  "normalisation": { "patch_mean": 0.0, "patch_std": 1.0,
                     "scalar_mean": [...], "scalar_std": [...] }
}
```

Because the generator is reproducible, **manifest + sweep config fully reconstructs the dataset**.
State this — it is proper experimental hygiene and another paragraph of credit.

## 2.5 Normalisation

Compute statistics from the **training split only**, store them in the manifest, and bake the same
values into the C++ preprocessing. A mismatch between training and inference normalisation is the
single most common silent ML deployment bug.

```python
# ml/datasets.py
def compute_norm(train_shards):
    # patches are background-subtracted top-hat values; normalise per-patch by its own std
    # to make the model invariant to absolute brightness (critical for unseen videos)
    return {"mode": "per_patch_std", "eps": 1e-6}
```

**Per-patch normalisation, not dataset-global.** The evaluators' videos will have brightness
statistics you have never seen; a model that depends on absolute levels will fail. Normalising each
patch by its own standard deviation makes the model invariant to that.

---

# 3. M1 — `CentroidNet` ★ train this first

## 3.1 Purpose

Regress the beacon's sub-pixel centre from a small patch. **Centroiding error is named in both 30%
benchmark stages — this is the model closest to marks.**

It also learns the pixel-locking bias correction implicitly, because it is trained against exact
truth (see `SAT-DESIGN.md` §10.1.3 for the classical S-curve correction it must beat).

## 3.2 Architecture

```python
# ml/models/centroid.py
class CentroidNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(1,  8, 3, padding=1), nn.ReLU(),
            nn.Conv2d(8, 16, 3, padding=1), nn.ReLU(),
            nn.Conv2d(16,16, 3, padding=1), nn.ReLU(),
        )
        self.head = nn.Sequential(
            nn.Linear(16*15*15 + 2, 64), nn.ReLU(),
            nn.Linear(64, 32),           nn.ReLU(),
            nn.Linear(32, 2),
        )
    def forward(self, patch, scalars):        # patch (B,1,15,15), scalars (B,2)
        z = self.conv(patch).flatten(1)
        return self.head(torch.cat([z, scalars], dim=1))   # (dx, dy) in pixels
```

| Property | Value |
|---|---|
| Input | 15×15 background-subtracted patch, per-patch normalised; scalars `[snr, size_est]` |
| Output | `(dx, dy)` offset from patch centre, in pixels |
| Parameters | ~11,000 |
| Size (INT8) | ~45 KB |
| Latency budget | **0.02 ms**, 1 call per frame |
| Fallback | bias-corrected `WindowedCoM` |

**No pooling.** Pooling discards exactly the sub-pixel spatial information you are trying to recover.

## 3.3 Loss

```python
loss = F.smooth_l1_loss(pred, target, beta=0.05)    # beta in pixels
```

Smooth L1 (Huber) rather than MSE: MSE over-weights the rare large errors from badly-cropped patches
and drags the model away from the sub-pixel precision that matters.

Optionally add a heteroscedastic head predicting `log σ²` and train with a Gaussian NLL — this gives
the model its own uncertainty estimate, which feeds the Kalman `R` directly. Worth doing if time
allows; report the calibration.

## 3.4 Training config

```yaml
# ml/configs/centroid_v2.yaml
dataset: data/centroid_v2
model: centroid
seed: 1337
epochs: 60
batch_size: 512
optimizer: adamw
lr: 3.0e-4
weight_decay: 1.0e-4
scheduler: cosine
early_stopping:
  metric: val_rmse_px
  patience: 8
augmentation: none          # ML-6
```

## 3.5 Acceptance criteria

| Condition | Must achieve |
|---|---|
| Test RMSE, SNR > 15 | ≤ bias-corrected classical (it should not be *worse* in easy conditions) |
| Test RMSE, SNR < 10 | **≥ 30% better** than bias-corrected classical |
| Test RMSE vs theoretical bound | within 1.5× at every SNR bin |
| Residual bias | < 0.02 px at every SNR bin |
| Latency in C++ | < 0.05 ms |

If it fails the SNR<10 criterion, it is not worth shipping — say so and ship the classical path.

---

# 4. INTEGRATION AND `--no-ai`

**Do this immediately after M1, before building any more models.** Proving the fallback path works
while there is only one model is far easier than retrofitting it to five.

## 4.1 ONNX export

```python
# ml/export.py
def export(model, out_path, patch_px=15):
    model.eval()
    dummy_patch   = torch.zeros(1, 1, patch_px, patch_px)
    dummy_scalars = torch.zeros(1, 2)
    torch.onnx.export(
        model, (dummy_patch, dummy_scalars), out_path,
        input_names=["patch", "scalars"], output_names=["offset"],
        opset_version=17,
        dynamic_axes=None,            # FIXED shapes — faster and more predictable in ORT
        do_constant_folding=True,
    )
```

**Fixed shapes, not dynamic axes.** Dynamic shapes force ORT to re-plan allocations and cost you
latency and determinism. Batch the 20 candidate patches into a fixed-size tensor and mask unused
slots instead.

**Pin the opset.** Record it in the model card and in `run.json`.

## 4.2 Parity test — mandatory

```python
# ml/tests/test_parity.py
def test_onnx_matches_pytorch():
    x, s = load_fixture()
    torch_out = model(x, s).detach().numpy()
    ort_out   = ort_session.run(None, {"patch": x.numpy(), "scalars": s.numpy()})[0]
    assert np.abs(torch_out - ort_out).max() < 1e-5
```

Also test from C++ against a committed fixture:

```cpp
TEST_CASE("CentroidNet C++ matches PyTorch fixture") {
    auto model = OnnxModel::load("models/centroid_v2.onnx");
    auto [input, expected] = load_fixture("tests/ai/centroid_fixture.bin");
    std::array<float, 2> out;
    model->run(input, out);
    CHECK(std::abs(out[0] - expected[0]) < 1e-5f);
    CHECK(std::abs(out[1] - expected[1]) < 1e-5f);
}
```

## 4.3 Fallback test — mandatory

```cpp
TEST_CASE("missing model falls back, does not crash") {
    Scenario s = load("scenarios/spec_defaults.toml");
    s.ai.centroid_net = "models/does_not_exist.onnx";
    s.ai.fallback_on_fail = true;
    auto result = run_headless(s, /*seed=*/1);
    CHECK(result.ok());
    CHECK(result.warnings_contain("CentroidNet"));
    CHECK(result.centroid_rmse_px < 1.0);        // classical path still works
}
```

## 4.4 `--no-ai` compliance

```bash
sat-tracker --sweep scenarios/sweep_full.toml --no-ai --out results/noai/
```

**Must produce a passing compliance matrix.** This is ML-2 and it is checked in CI.

## 4.5 Reproducibility with ML

Network output is deterministic given `intra_op_num_threads=1` and `ORT_ENABLE_BASIC`, but it is
floating-point — a different ONNX Runtime *version* could shift the last bits.

| Approach | Detail |
|---|---|
| Hash **decisions**, not outputs | Selected candidate index, chosen strategy, track confirmation — all integers, all stable |
| Pin the ORT version | In `vcpkg.json`; record it in `run.json` and in the `centroid.csv` header |
| Bit-exact mode | `--no-ai` is fully bit-exact and is what the reproducibility CI job uses |

Document this precisely. Stating exactly what is and is not bit-reproducible is more credible than an
unqualified claim, and it is a good Q&A answer.

---

# 5. M2 — `CandidateNet`

## 5.1 Purpose

CFAR proposes ~20 suspicious spots per frame. This decides which is the real beacon, and which are
decoys, clutter or noise clusters.

**Why two-stage, and why this is the right architecture:** running a network over the whole 640×480
frame costs ~8 ms. Running it on twenty 15×15 patches that CFAR already proposed costs 0.15 ms.
Cheap classical proposal plus expensive learned decision is exactly how modern detectors are built,
and it is an easy answer under Q&A.

## 5.2 Architecture

```python
class CandidateNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(1,  8, 3, padding=1), nn.ReLU(),
            nn.Conv2d(8, 16, 3, padding=1), nn.ReLU(),
            nn.MaxPool2d(2),                              # pooling IS fine here
            nn.Conv2d(16,16, 3, padding=1), nn.ReLU(),
            nn.AdaptiveAvgPool2d(1),
        )
        self.head = nn.Sequential(
            nn.Linear(16 + 6, 32), nn.ReLU(),
            nn.Linear(32, 4),
        )
    def forward(self, patch, scalars):
        z = self.conv(patch).flatten(1)
        return self.head(torch.cat([z, scalars], dim=1))   # logits over 4 classes
```

| Property | Value |
|---|---|
| Input | 15×15 patch + 6 scalars: `[snr, area, fill_ratio, aspect, matched_response, dist_from_prediction]` |
| Output | 4-way softmax: `beacon · decoy · clutter · noise` |
| Parameters | ~15,000 |
| Runs | ~20 patches/frame, batched into one fixed-size call |
| Latency budget | **0.15 ms** for all 20 |
| Fallback | the shape/area/fill/aspect gate (`SAT-DESIGN.md` §9.4.7) |

**`dist_from_prediction` is important.** It lets the model use the tracker's context, not just
appearance. But be careful: it must be computed from the *tracker's own prediction*, never from
truth (INV-1).

## 5.3 Class imbalance

Typical distribution: ~6% beacon, ~4% decoy, ~31% clutter, ~59% noise. Handle with class-weighted
cross-entropy rather than resampling — resampling changes the prior and breaks calibration, and you
need calibration because this model's output feeds the Kalman `R`.

```python
weights = torch.tensor([1/0.06, 1/0.04, 1/0.31, 1/0.59])
weights = weights / weights.sum() * 4
loss = F.cross_entropy(logits, labels, weight=weights, label_smoothing=0.02)
```

## 5.4 Acceptance criteria

| Metric | Must achieve |
|---|---|
| Test AUC (beacon vs rest), overall | > 0.97 |
| Test AUC, fog + lowlight | > 0.92 |
| Expected calibration error | < 0.05 |
| Decoy rejection rate | > 90% at 1% beacon-miss rate |
| End-to-end false track rate | measurably lower than classical |

---

# 6. M3 — `MotionNet`

## 6.1 Purpose

Predict where the beacon is going, and classify which of the four motion regimes it is in.

**Why this is well-motivated:** the spec mandates four dissimilar motion types (straight line,
circular, figure-8, random). No single kinematic model covers all four. This model attacks two
graded metrics directly:

- **Re-acquisition time ≤ 1 s** — a better forecast means a smaller search region
- **Lock retention rate** — a better forecast means longer survivable dropouts

## 6.2 Architecture

```python
class MotionNet(nn.Module):
    def __init__(self, hidden=24, horizon=15):
        super().__init__()
        self.gru      = nn.GRU(input_size=4, hidden_size=hidden, batch_first=True)
        self.forecast = nn.Linear(hidden, horizon * 2)     # (dx, dy) × 15 steps
        self.regime   = nn.Linear(hidden, 4)               # line/circular/fig8/random
    def forward(self, hist):                               # (B, 30, 4)
        _, h = self.gru(hist)
        h = h.squeeze(0)
        return self.forecast(h).view(-1, 15, 2), self.regime(h)
```

| Property | Value |
|---|---|
| Input | last 30 track states `[az, el, az_rate, el_rate]`, normalised |
| Output A | 15-step position forecast |
| Output B | motion-regime logits |
| Parameters | ~6,000 |
| Latency budget | **0.05 ms** |
| Fallback | IMM prediction |

## 6.3 The regime head is not decoration

It feeds the **IMM's model priors** (`SAT-DESIGN.md` §10.2). Classifying "this is a coordinated turn"
makes the *classical* filter better. Learned and classical parts reinforce each other — draw this in
the report, it is a genuinely nice piece of system design.

```cpp
// tracking/imm.cpp
void ImmFilter::set_regime_prior(int regime, float confidence) {
    // bias the Markov transition matrix toward the predicted regime
    const double boost = 1.0 + 0.5 * confidence;
    for (int j = 0; j < M; ++j)
        pi_[j][regime_to_model(regime)] *= boost;
    renormalise_rows();
}
```

## 6.4 Loss

```python
forecast_loss = F.smooth_l1_loss(pred_fc, target_fc, beta=0.5)     # in µrad
regime_loss   = F.cross_entropy(pred_rg, target_rg)
loss = forecast_loss + 0.3 * regime_loss
```

**Weight the forecast loss by horizon.** Near-term accuracy matters more for control; far-term
matters more for reacquisition. A linearly decaying weight from 1.0 to 0.3 across the 15 steps works
well.

## 6.5 Training data

```toml
[capture]
what        = "tracks"
history_len = 30
horizon     = 15
include_dropouts = true     # ← essential
```

**Include sequences that span detection dropouts.** That is the case the model exists to handle, and
a model trained only on clean continuous tracks will be useless exactly when needed.

## 6.6 Acceptance criteria

| Metric | Must achieve |
|---|---|
| Forecast RMSE at +5 frames | ≥ 20% better than constant-velocity |
| Forecast RMSE at +15 frames | ≥ 35% better than constant-velocity |
| Regime classification accuracy | > 90% after 1 s of track |
| End-to-end reacquisition time | measurably lower |
| End-to-end lock retention | measurably higher |

---

# 7. M4 — `RecoveryNet` (optional)

## 7.1 Purpose

Find the beacon when CFAR returns nothing at all.

**The trigger condition IS the design.** This model is expensive; it earns its cost precisely when
the cheap path has already failed.

```cpp
if (candidates.empty()
    && (mode == SysMode::Search || mode == SysMode::Reacquire)
    && (now - last_recovery_call) > 0.1)         // ≤ 10 Hz
{
    run_recovery_net(frame);
}
```

## 7.2 Architecture

```python
class RecoveryNet(nn.Module):
    """Fully convolutional; outputs an 80×60 heatmap from a downsampled frame."""
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(1, 16, 3, stride=2, padding=1), nn.ReLU(),   # 320×240
            nn.Conv2d(16,32, 3, stride=2, padding=1), nn.ReLU(),   # 160×120
            nn.Conv2d(32,32, 3, stride=2, padding=1), nn.ReLU(),   #  80× 60
            nn.Conv2d(32,32, 3, padding=1),           nn.ReLU(),
            nn.Conv2d(32, 1, 1),                                   # logits
        )
    def forward(self, frame):                                      # (B,1,640,480)
        return self.net(frame)
```

| Property | Value |
|---|---|
| Parameters | ~40,000 |
| Latency | ~2.5 ms (INT8), capped at 10 Hz |
| Loss | focal loss on a Gaussian-blurred target heatmap (σ = 1 cell) |
| Fallback | continue the search pattern |

Focal loss, not plain BCE — the positive class is ~1 cell in 4800.

## 7.3 Acceptance criteria

| Metric | Must achieve |
|---|---|
| Detection rate where CFAR fails | > 40% |
| False positive rate | < 1 per 100 invocations |
| Latency | < 4 ms |

If it cannot beat 40%, drop it. It is the least important of the five.

---

# 8. M5 — `StrategyPolicy` ★ the innovation story

## 8.1 Purpose

Choose which detector, centroider, filter and control gains to use, based on current conditions.
This is what makes the name "Self-Adaptive Tracking" honest rather than marketing, and it is your
strongest answer to **"Innovation and Novelty"**.

## 8.2 How it is trained — the interesting part

**You do not hand-label this.** You measure it. Run a Monte Carlo sweep where every scenario is run
**once per strategy**, and record which strategy produced the lowest error.

```python
# ml/strategy_labels.py
records = []
for scenario in SWEEP_SCENARIOS:
    for seed in SEEDS:
        for strat_id in range(N_STRATEGIES):
            r = run_headless(scenario, seed, forced_strategy=strat_id)
            records.append({
                "conditions":     r.mean_conditions,       # 12 features
                "strategy":       strat_id,
                "centroid_rmse":  r.centroid_rmse_px,
                "tracking_rmse":  r.tracking_rmse_px,
                "lock_retention": r.lock_retention,
            })

# label each condition bin with the strategy that minimised a weighted objective
def objective(r):
    return 0.6 * r["centroid_rmse"] + 0.3 * r["tracking_rmse"] + 0.1 * (1 - r["lock_retention"])

labels = (pd.DataFrame(records)
            .groupby(["condition_bin", "strategy"])
            .apply(lambda g: g.apply(objective, axis=1).mean())
            .groupby("condition_bin").idxmin())
```

The objective weights mirror the marks: centroiding error is worth the most, so it dominates.

**This is a system that learns its own control policy from its own simulator.** That sentence belongs
in the report and in the presentation.

## 8.3 Model

```python
# gradient-boosted trees as the baseline — fast, interpretable, no overfitting worry at this size
from sklearn.ensemble import HistGradientBoostingClassifier
clf = HistGradientBoostingClassifier(max_depth=4, max_iter=200, l2_regularization=1.0)

# MLP alternative, if you want a single ONNX runtime path
class StrategyPolicy(nn.Module):
    def __init__(self, n_features=12, n_strategies=6):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(n_features, 32), nn.ReLU(),
            nn.Linear(32, 16),         nn.ReLU(),
            nn.Linear(16, n_strategies),
        )
```

| Property | Value |
|---|---|
| Input | 12-feature `Conditions` vector (`SAT-DESIGN.md` §10.6) |
| Output | strategy index (6 predefined strategies) |
| Parameters | ~1,500 |
| Latency budget | **0.005 ms** |
| Fallback | `rule_table()` |

Use the MLP if you want a single ONNX path for everything; use GBT if you want interpretability
(feature importances are a nice report figure). Either is defensible. Trees export to ONNX via
`skl2onnx`.

## 8.4 The runtime constraints the model does not control

The supervisor enforces these regardless of what the model says — they are stability requirements,
not learned behaviour:

1. **EMA smoothing** of the condition vector. Never react to a single frame.
2. **Minimum dwell of 30 frames (1 s)** between switches. Without this the system chatters, which
   looks terrible live and degrades tracking.
3. **Bumpless transfer** — carry the integrator state across a gain change, or the switch itself
   kicks the loop.

## 8.5 Acceptance criteria

| Metric | Must achieve |
|---|---|
| Agreement with the measured-optimal strategy | > 70% of condition bins |
| End-to-end vs the rule table | Better, or report honestly that it is not |
| Switch rate | < 1 per second, always |
| No error spike at a switch | Verified on the error trace |

**If the learned policy does not beat the rule table, report that.** A measured negative result with
a learning curve is a creditable outcome and far better than hiding it.

---

# 9. M6 — `SearchPolicy` (optional, high risk)

Reinforcement learning over the probability grid to minimise acquisition time.

| Property | Value |
|---|---|
| Input | 32×32 probability grid + boresight + elapsed time |
| Method | PPO or DQN against the simulator |
| Reward | `−time_to_detection`, with a movement-cost penalty |
| Output | next look direction |
| Fallback | greedy Bayesian search |

**Attempt only after Stage 13 of the main roadmap is complete and benchmarked.** RL is fiddly and can
simply fail to converge.

**Be prepared to report a negative result.** "We trained a PPO agent; it did not beat the greedy
Bayesian search; here is the learning curve and our analysis of why" is an honest, creditable outcome
that demonstrates you understood the method well enough to evaluate it.

---

# 10. EVALUATION

Report three levels. **The third is the one that earns marks.**

## 10.1 Level 1 — does the model do its own job?

- ROC curves and AUC, **broken down by weather mode and motion type**
- Precision-recall curves (better than ROC under this class imbalance)
- Confusion matrices
- For regression models: RMSE by SNR bin and by beacon size

A single overall accuracy number hides everything interesting. Never report only that.

## 10.2 Level 2 — is it honestly calibrated?

A reliability diagram: when the model says 80% confident, is it right 80% of the time?

This matters concretely: `CandidateNet`'s confidence and `CentroidNet`'s uncertainty both feed the
Kalman filter's measurement noise. **A badly calibrated model actively misleads the tracker** — worse
than no model at all.

```python
# ml/evaluate.py
def expected_calibration_error(probs, labels, n_bins=15):
    bins = np.linspace(0, 1, n_bins + 1)
    ece = 0.0
    for lo, hi in zip(bins[:-1], bins[1:]):
        m = (probs > lo) & (probs <= hi)
        if m.sum() == 0: continue
        ece += m.mean() * abs(labels[m].mean() - probs[m].mean())
    return ece
```

Target ECE < 0.05. If it is worse, apply temperature scaling on the validation set.

## 10.3 ★ Level 3 — does it move the graded numbers?

**This is the single most valuable ML artifact you can produce.** Its columns are the metrics the
evaluators actually score.

```
ABLATION — effect on graded metrics   (500 runs per row, seeds 1-500)

Configuration                     Centroid RMSE   Track err   Lock ret.   Reacq
                                       (px)          (px)        (%)       (s)
──────────────────────────────────────────────────────────────────────────────
Classical, no bias correction          0.34          4.8        96.9      0.44
+ S-curve bias correction              0.11          4.6        97.1      0.43
+ CentroidNet  (M1)                    0.09          4.4        97.3      0.42
+ CandidateNet (M2)                    0.09          4.1        97.6      0.41
+ MotionNet    (M3)                    0.09          3.6        98.1      0.24
+ StrategyPolicy (M5)                  0.08          3.2        98.2      0.21

Fog + low light subset:
Classical, no bias correction          1.02          9.9        91.3      0.91
Full stack                             0.41          6.4        96.7      0.38
```

Generate it with the same batch infrastructure that produces the compliance matrix:

```bash
for cfg in classical bias m1 m1m2 m1m2m3 full; do
  sat-tracker --sweep scenarios/sweep_full.toml --ai-config $cfg \
              --jobs 8 --out results/ablation/$cfg/
done
python tools/ablation_table.py results/ablation/ > docs/report/ablation.md
```

Nearly every team will report model accuracy. Almost none will report whether the model moved the
number that is graded.

## 10.4 The framing that lands

Never claim ML is universally better. Show the crossover, with numbers:

> Bias-corrected classical centroiding matches the learned estimator above SNR ≈ 15 and runs 30×
> faster. The learned estimator reduces RMS centroid error by 60% below SNR 8, where the classical
> estimator's residual bias dominates. The deployed hybrid switches on measured SNR and uses the
> classical path for 82% of frames.

Honest, quantified, and far stronger than "our AI achieves 99% accuracy".

## 10.5 Inference cost table (put this in the report)

| Model | Params | INT8 size | Latency | Calls/frame | Budget share |
|---|---|---|---|---|---|
| CentroidNet | 11 k | 45 KB | 0.02 ms | 1 | 2.4% |
| CandidateNet | 15 k | 61 KB | 0.15 ms | 1 (batched ×20) | 17.6% |
| MotionNet | 6 k | 24 KB | 0.05 ms | 1 | 5.9% |
| StrategyPolicy | 1.5 k | 6 KB | 0.005 ms | 1 | 0.6% |
| RecoveryNet | 40 k | 160 KB | 2.5 ms | ≤ 0.33 | conditional |
| **Total (typical frame)** | | **296 KB** | **0.225 ms** | | **26.5%** |

Showing you budgeted inference latency against a real-time control loop is a strong systems signal.

## 10.6 Quantisation

```python
# ml/quantise.py
from onnxruntime.quantization import quantize_dynamic, QuantType
quantize_dynamic("models/candidate_v3.onnx", "models/candidate_v3_int8.onnx",
                 weight_type=QuantType.QInt8)
```

**Measure the accuracy delta and report it.** If INT8 costs more than 5% of the metric, ship FP32 —
you have latency headroom. Quantise `RecoveryNet` for sure (it is the expensive one); the others are
optional.

---

# 11. MODEL CARDS

One per model, in `docs/models/`. Template:

```markdown
# CentroidNet v2

Purpose        Sub-pixel beacon centre from a 15×15 patch
Input          15×15 float patch (background-subtracted, per-patch normalised)
               + scalars [snr, size_est]
Output         (dx, dy) offset from patch centre, in pixels

Parameters     11,204
Size           45 KB (INT8) / 178 KB (FP32)
Latency        0.02 ms, 1 call per frame, 2.4% of the frame budget
Opset          17
ORT version    1.17.1

Dataset        centroid_v2 — 806,400 samples
               manifest sha256 4a91e2…
               sweep config sha256 9f2a7b…
Split          BY RUN: seeds 1-700 train, 701-850 val,
               851-1000 test on scenarios Q-T (UNSEEN during training)
Training seed  1337

Test RMSE      0.087 px overall
                 0.041 px clear
                 0.206 px fog + lowlight
vs bound       1.19× the theoretical limit at SNR 30
               1.34× at SNR 8
Residual bias  0.011 px max across SNR bins
Calibration    ECE 0.019 (heteroscedastic head)

End-to-end     centroid RMSE 0.11 → 0.09 px
               fog + lowlight 0.68 → 0.41 px

Fails when     beacon overlaps a clutter source by >60% — the patch contains
               two peaks and the regression splits the difference.
               Detected by low predicted confidence; falls back to SurfaceFit.
Fallback       bias-corrected WindowedCoM
```

**The "Fails when" line is what impresses.** Knowing your model's failure mode, and having a detector
for it, is the difference between having trained a model and understanding it.

---

# 12. THE OPENCV BOUNDARY

## 12.1 The rule

**Anything that runs 30 times a second and touches every pixel, we write ourselves. Everything else,
use OpenCV.**

| Use OpenCV for | Do NOT use OpenCV for |
|---|---|
| `cv::VideoCapture` — MP4 decode (the reason it is a dependency at all) | Anything in the per-frame hot loop |
| `cv::imread` / `imwrite` — shape masks, debug dumps | Median filter — own implementation |
| `cv::connectedComponentsWithStats` — genuinely good, hard to beat | Morphology — own van Herk implementation |
| **Test oracle for every own kernel** | Box filtering / CFAR — OpenCV has no CFAR at all |
| Offline dataset tooling, visualisation | Centroiding |

## 12.2 Why the hot loop is our own

| Reason | Detail |
|---|---|
| **Allocation** | `cv::Mat` allocates on construction, violating INV-4 and inflating p99 latency |
| **Dispatch** | Generic type/depth switching costs measurable time per call |
| **Determinism** | OpenCV picks SIMD kernels at runtime from detected CPU features; different machines can take different paths, which risks INV-3 for anything float |
| **Missing primitives** | There is no CFAR, no multi-scale matched filter, no S-curve correction |

## 12.3 OpenCV as a test oracle — use this hard

This is the part that earns marks. Every own kernel is validated against OpenCV:

```cpp
// tests/kernels/test_median.cpp
TEST_CASE("median3x3 matches cv::medianBlur bit-for-bit") {
    Pcg32 rng; rng.seed(42, 0);
    for (int trial = 0; trial < 1000; ++trial) {
        auto img = random_image(640, 480, rng);
        auto ours = sat::median3x3(img);
        cv::Mat cv_in(480, 640, CV_8U, img.data()), cv_out;
        cv::medianBlur(cv_in, cv_out, 3);
        CHECK(bitwise_equal(ours, cv_out));      // interior pixels; borders differ by policy
    }
}
```

Then write in the report:

> Every image-processing kernel is validated bit-for-bit against OpenCV on 1,000 randomised inputs.
> The shipped implementations are our own, for latency predictability and cross-machine
> reproducibility; OpenCV remains linked as the test oracle and for video decode.

That sentence gets you OpenCV's credibility *and* the credit for writing the kernels.

## 12.4 Border policy

OpenCV and your implementation will differ at image borders unless you match its `BORDER_REPLICATE`
default exactly. Either match it, or restrict the equality test to interior pixels and test the
border separately against your own documented policy. **Document whichever you choose** — an
unexplained border difference looks like a bug.

---

# 13. ML BUILD ORDER

| # | Task | Depends on | Done when |
|---|---|---|---|
| 1 | Data factory: `--gen-dataset`, manifests, run-level split, `assert_no_leakage` in CI | Main roadmap Stage 7 | 400k samples with a manifest; leakage assertion green |
| 2 | Normalisation statistics, baked into both Python and C++ | 1 | The same patch produces the same normalised tensor in both languages |
| 3 | **`CentroidNet`** — train, evaluate, export | 1, 2 | Meets the §3.5 acceptance criteria |
| 4 | **ONNX wrapper + parity test + fallback test + `--no-ai`** | 3 | C++ matches PyTorch to 1e-5; deleting the file warns not crashes; `--no-ai` passes the compliance matrix |
| 5 | `CandidateNet` | 4 | Meets §5.4 |
| 6 | `MotionNet` + IMM prior integration | 4 | Meets §6.6 |
| 7 | `RecoveryNet` *(optional)* | 4 | Meets §7.3, or is dropped |
| 8 | Strategy sweep → labels → `StrategyPolicy` | 5, 6 | Meets §8.5, or the rule table is reported as better |
| 9 | `SearchPolicy` *(optional)* | Main roadmap Stage 13 | Beats greedy Bayesian search, or a negative result is written up |
| 10 | **Ablation table** | 3–8 | The §10.3 table exists, with a fog subset |
| 11 | Calibration, ROC/PR, inference cost table | 3–8 | All report figures generated |
| 12 | Model cards | 3–8 | One per shipped model, each with a "fails when" line |

**If time runs short, ship tasks 1–6 and 10.** That is `CentroidNet` + `CandidateNet` + `MotionNet`
with a measured ablation — a complete, defensible, honestly-evaluated ML contribution that touches
every graded metric. Tasks 7–9 are upside.

---

# 14. ML DIRECTORY LAYOUT

```
ml/
├── README.md
├── requirements.txt              # pinned versions
├── datagen.py                    # drives sat-tracker --gen-dataset
├── datasets.py                   # loaders, split_by_run, assert_no_leakage, normalisation
├── models/
│   ├── centroid.py  candidate.py  motion.py  recovery.py  strategy.py
├── train.py                      # single entry point, config-driven, seeded
├── evaluate.py                   # ROC, PR, calibration, per-condition breakdown
├── ablate.py                     # the §10.3 table
├── strategy_labels.py            # the §8.2 sweep-to-labels pipeline
├── export.py                     # → ONNX, fixed shapes, pinned opset
├── quantise.py                   # INT8 + accuracy delta measurement
├── sweeps/
│   ├── centroid_v2.toml  candidate_v3.toml  motion_v2.toml  strategy_v1.toml
├── configs/
│   ├── centroid_v2.yaml  candidate_v3.yaml  motion_v2.yaml
├── tests/
│   ├── test_parity.py  test_no_leakage.py  test_norm_match.py
└── fixtures/                     # committed tensors for the C++ parity tests
```

**Determinism on the Python side, matching the C++ side:**

```python
def set_all_seeds(seed: int):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.use_deterministic_algorithms(True)
    torch.backends.cudnn.benchmark = False
```

Log the seed in every model card.

---

# 15. WHAT TO SAY IN THE TECHNICAL EVALUATION

Under **"AI and computer vision"**, the strongest things you can say, in order:

1. **"We rejected object detection, and here is why."** A uniform square has no learnable appearance;
   a matched filter is provably optimal for a known shape in additive noise and 1000× faster. This
   demonstrates you understood the problem rather than reaching for a familiar tool.
2. **"Our simulator is a data factory with zero annotation error."** 800k perfectly labelled samples
   across every condition, fully reconstructible from a manifest and a config hash.
3. **"We split by run, not by frame, and our test set uses scenarios the model never saw."** This is
   the closest proxy to your MP4 benchmark, and most teams get it wrong.
4. **"Here is the end-to-end ablation on the metrics you grade."** Not model accuracy — centroiding
   RMSE, tracking error, lock retention, reacquisition time, per configuration.
5. **"Here is where classical wins, and we deploy classical there."** The crossover at SNR ≈ 15, with
   the hybrid using the classical path for 82% of frames.
6. **"The system passes full compliance with `--no-ai`."** ML is an enhancement, not a dependency.
7. **"Our strategy selector learned its policy from measured outcomes, not from our guesses."** The
   per-strategy Monte Carlo sweep that generated its training labels.
8. **"Every model has a documented failure mode and a detector for it."** The "fails when" lines in
   the model cards.
