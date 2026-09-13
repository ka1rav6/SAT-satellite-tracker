# SAT — Metric definitions

CP 7.1 requires design §13.1's definitions to be written "verbatim into
`docs/METRICS.md` and the report". They are reproduced below exactly as the
specification states them, followed — separately and clearly marked — by the
implementation notes that say how each one is actually computed, where the
specification leaves a choice, and which choice was made.

The two halves are kept apart on purpose. A definition that has been quietly
edited to match whatever the code does is not a definition.

---

## 1. The definitions, verbatim (design §13.1)

```
centroiding_error    : |reported beacon centre − true beacon centre|, in SCREEN pixels
                       RMSE, bias (mean signed), p95, max
                       Computed ONLY on frames with a detection.       ← GRADED, 60%

tracking_error       : |camera boresight − true beacon angle|, in px AND µrad
                       RMS, p95, max. Measured ONLY while Confirmed.   ← spec row 17

acquisition_cold_s   : run start → first Confirmed track, beacon at random position
acquisition_in_fov_s : beacon first enters FOV → first Confirmed track
reacquisition_s      : Confirmed→lost → Confirmed again
lock_retention_rate  : frames Confirmed ÷ frames beacon within FOV
target_loss_frac     : 1 − lock_retention_rate                          ← spec row 18
false_track_rate     : Confirmed tracks on non-beacons, per minute
handover_success     : fraction of runs holding error < capture range for 30 frames
saturation_frac      : frames at the rate limit ÷ total frames
processing_ms        : per stage, p50 / p95 / p99  — NEVER the mean
fps                  : sustained closed-loop, GUI on and off, reported separately
```

---

## 2. Implementation notes

Source: [`src/metrics/collector.cpp`](../src/metrics/collector.cpp).

### 2.1 The four denominators

Four different frame sets appear in the definitions above, and each metric uses
the one the specification names for it:

| Denominator | Used by |
|---|---|
| every frame | `saturation_frac`, `fps`, `processing_ms` |
| frames with the beacon inside the FOV | `lock_retention_rate`, `target_loss_frac` |
| frames while the track is Confirmed | `tracking_error` |
| frames with a detection | `centroiding_error` |

Using a single denominator for all of them is the easiest way to produce a
flattering number. A run that loses the beacon for half its length and tracks
perfectly for the other half has excellent tracking error and poor lock
retention; either figure reported alone misrepresents it.

### 2.2 `centroiding_error` — two frames, both reported

§13.1 says "in SCREEN pixels". The implementation reports **both** the screen
and the image figure, labelled, because they measure different things (INV-6):

- **image frame** — `|detection − true position on the sensor|`. The detector's
  accuracy and nothing else. This is the quantity §10.1.1's bound
  `σ ≥ w / (2·SNR)` applies to, and the one §10.1.3's S-curve correction
  improves.
- **screen frame** — the same, converted through the **commanded** boresight,
  which is all the system knows. It therefore also contains the unmeasured
  pointing error: jitter (row 23) and platform drift (row 25) move the true
  boresight and the encoder does not see them.

The screen figure can never beat the jitter amplitude. Reporting it alone would
make the 60 %-weighted metric dominated by a disturbance the detector has no
influence over. Reporting only the image figure would overstate what the system
delivers. Both, always.

In video modes the two coincide: INV-8 disables the disturbances and the crop
offset is known exactly (§8.3 requirement 6). That is the mode Benchmark
Performance-2 grades centroiding in.

### 2.3 `centroiding_error` — one added condition

The specification says "computed ONLY on frames with a detection". The
implementation adds a second condition: **the beacon must also be within the
field of view.**

Scoring a reported centroid against a beacon that is not on the sensor is not a
measurement of centroiding accuracy — the detector cannot locate something that
is not there, and whatever it reported is a *false alarm*, which is a different
metric. Without this condition an early ablation measured 38 px of "centroiding
error" that was really the distance to an off-screen target.

### 2.4 `bias` is signed, everything else is a magnitude

RMSE, p95 and max are computed over `|error|`. `bias` is the mean of the
**signed** per-axis error, reported as a pair.

The distinction is the point of the metric. An estimator with 0.5 px RMSE and no
bias is *noisy*; one with 0.5 px RMSE and 0.5 px bias is *broken in a fixable
way* — subtract a constant. §10.1.3's S-curve correction is that fix, so the
metric that reveals it has to exist before the fix can be justified.

### 2.5 `reacquisition_s` is measured from the loss, not from the reappearance

The clock starts when a Confirmed track is lost and stops when one is Confirmed
again. It is deliberately not started when the beacon becomes visible again:
the system does not know when the target came back, and spec row 19 is asking
how long the *system* takes, not how lucky it was.

An episode is recorded on the transition, so a run that ends mid-episode
contributes no sample at all. A half-finished re-acquisition is not a fast one.

### 2.6 `lock_retention_rate` is clamped, and its denominator is printed

`frames Confirmed ÷ frames in FOV` can exceed 1: the filter coasts correctly
through a brief occlusion, staying Confirmed on frames where the beacon is not
visible. Unclamped, that reports retention above 100 % and a **negative** target
loss. It is clamped to 1.

When the beacon is never in view the ratio is 0/0. It is reported as zero with
`frames_in_fov = 0` printed beside it, so a reader can see the ratio is
undefined rather than bad.

### 2.7 `false_track_rate`

§13.1: "Confirmed tracks on non-beacons, per minute". Counted as frames where
the system claims a **Confirmed** lock while the beacon is not within the field
of view, scaled to a per-minute rate.

A frame where perception reported a candidate but no track was confirmed is
**not** counted. That is the gate correctly rejecting a noise blob, and CFAR's
measured false-alarm probability (CP 5.5) guarantees several such candidates per
frame at any useful threshold. Counting them would penalise the pipeline for the
thing it does right.

### 2.8 `fps` is derived from percentiles, never from a mean

§13.1 says "NEVER the mean" of `processing_ms`, and decision 17 explains why:
"A 0.3 ms mean hiding a 25 ms p99 is a broken control loop."

The same reasoning applies to the frame rate derived from it, so:

- `fps_mean` is `1000 / frame_ms_p50` — the *typical* frame, not the average.
- `fps_p5` is `1000 / frame_ms_p95` — the slow tail.

Spec row 20's "≥ 20 FPS" is a floor, so the number that decides compliance is
the slow one.

### 2.9 Quantiles are exact, by the nearest-rank definition

The p-th quantile is the smallest sample `x` such that at least a fraction `p`
of the samples are `≤ x`, i.e. element `ceil(p·n) − 1` of the sorted series.

Nearest-rank rather than an interpolating variant, so that **every reported
value is a value that actually occurred**. "p95 = 7.1 px" should mean some frame
really had 7.1 px of error, not that two neighbouring frames were averaged into
a figure nothing ever measured.

Every sample is kept rather than estimated by a streaming quantile algorithm. A
120 s run at 30 Hz is 3,600 samples — 29 KB. Approximating a number that can be
computed exactly, to save 29 KB, is a poor trade when the output is a compliance
matrix.

### 2.10 Not yet implemented

| Metric | Status |
|---|---|
| `handover_success` | The mode FSM's `Track → Handover` transition exists and is tested, but is **disabled by default** until CP 10.7 builds the quadrant detector. Entering the state would claim a capability the system does not have. |
| `fps` with the GUI on | §13.1 asks for it "reported separately". The headless figure is produced now; the GUI-on figure arrives with CP 14.4's measurement from the shipped binary. |
