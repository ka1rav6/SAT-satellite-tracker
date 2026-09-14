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

### 2.10 A derived floor on `tracking_error` (spec rows 17 vs 23)

Spec row 17 caps tracking error at **10 px**. Spec row 23 independently
specifies a camera jitter of up to **±20 px per frame**, applied to the *true*
boresight and invisible to the encoder.

These two requirements are in tension, and the arithmetic is short. Jitter is
drawn uniformly on `[−A, A]` per axis (see
[`src/degrade/disturbance.cpp`](../src/degrade/disturbance.cpp), which explains
why uniform rather than Gaussian). A uniform distribution has variance `A²/3`,
so with `A = 20 px` each axis contributes 133.3 px² and the magnitude over two
independent axes has

```
RMS = sqrt(2 · 400/3) = 16.33 px
```

**That is a floor no controller can go below**, because the disturbance
displaces the boresight *after* the command has been issued and nothing in the
system observes it. A system reporting under 10 px on this metric with row 23's
jitter applied is either not applying the specified jitter or not measuring
against the true boresight.

This is reported rather than engineered around, exactly as §10.5's acquisition
bound is. The controllable part is measured by running the same scenario with
the disturbance removed. Measured on `scenarios/baseline.toml`, beacon in view,
10 s:

| Configuration | tracking RMS |
|---|---|
| spec row 23 jitter (±20 px/frame) | 17.14 px |
| derived floor from the jitter alone | 16.33 px |
| jitter disabled | 2.86 px |

The loop contributes ~0.8 px on top of the disturbance, and meets row 17 with
room to spare once the disturbance is removed. Both figures are reported in the
compliance matrix, labelled.

### 2.11 Centroiding accuracy against the theoretical bound (Stage 9)

§10.1.1 gives a limit for an ideal estimator:

```
σ_centroid ≳ w / (2 · SNR)
```

CP 9.6 asks for measured accuracy **within 1.5× of it at every SNR bin**. We do
not meet that at low SNR, and the shortfall is recorded here rather than
absorbed into a looser threshold.

Measured on a 10 px square beacon, `WindowedCoM`, with the calibrated bias
table, 80 sub-pixel offsets per bin:

| SNR | bound (px) | measured (px) | ratio |
|---|---|---|---|
| 3 | 1.667 | 2.29 | 1.37× |
| 5 | 1.000 | 2.03 | 2.03× |
| 8 | 0.625 | 1.84 | 2.94× |
| 13 | 0.385 | 1.40 | 3.64× |
| 20 | 0.250 | 0.52 | 2.06× |
| 32 | 0.156 | 0.37 | 2.35× |
| 50 | 0.100 | 0.19 | 1.89× |
| 80 | 0.063 | 0.11 | 1.69× |

**Why.** The bound assumes an ideal estimator on an isolated, well-sampled
profile. Two things here are not that:

- the blob contour is found at half-max, and at low SNR that contour wanders,
  adding error the bound does not model;
- the default beacon is a **flat-topped square** (spec row 9), so all of its
  positional information is in the edges and a moment estimator has fewer
  effective samples than the bound assumes.

**What would close it.** §10.1.2 already names the fix, and it is not a better
moment estimator: *"MatchedPeak — best at low SNR (noise already integrated
away)"*, with the supervisor switching to it there and `Learned` below that.
Those are Stage 11 and Stage 12.

The regime the graded scenarios actually run in is the upper half of this
table: a 10 px beacon at spec row 22's maximum noise integrates to about
SNR 60, where the ratio is 1.7–1.9×.

### 2.12 The S-curve correction (CP 9.3)

§10.1.3's procedure, run by `sat-tracker --calibrate-centroid`: 200 sub-pixel
offsets × 6 sizes × 8 SNR bins × 3 estimators, fitting
`bias(u) = a·sin(2πu) + b·sin(4πu)` per cell.

**A cell's coefficients are stored only when the curve is actually
resolvable** — at least 10% of the error's variance explained by the fit. Least
squares always returns two numbers; at low SNR they are fitted to noise, and
subtracting them made the estimate 2–5% *worse*. 30 of 144 candidate cells
qualify, and the rest are left empty, which `BiasTable::correct()` treats as
"do nothing".

Measured gain, `WindowedCoM`, 5 px beacon:

| SNR | before (px) | after (px) | gain |
|---|---|---|---|
| ≤ 13 | — | — | 1.00× (no cell stored) |
| 20 | 0.334 | 0.252 | 1.33× |
| 32 | 0.246 | 0.145 | 1.69× |
| 50 | 0.171 | 0.084 | **2.02×** |
| 80 | 0.119 | 0.064 | 1.86× |

§10.1.3 expects 2–5× at high SNR; we are at the bottom of that range.

### 2.13 Frame budget (CP 14.2, spec row 20)

Spec row 20 requires **≥ 20 FPS**. §15 additionally sets an internal budget of
**≈ 0.85 ms per frame**. The first is met with margin; the second is not, and
the gap is stated here rather than absorbed.

Per-stage figures come from the shipped binary — `sat-tracker --headless
--stages` (CP 14.4) — so they can be reproduced on any machine rather than
taken on trust.

| Stage | before (µs) | after (µs) | §15 budget |
|---|---|---|---|
| frame_acquire *(simulator only)* | 24,805 | 24,805 | 740 |
| median_3x3 | 621 | 621 | 350 |
| top_hat | 6,436 | **2,187** | 300 |
| summed_area (×2) | 1,999 | 1,999 | 350 |
| matched_filter | 22,671 | **2,618** | 120 |
| cfar (×2) | 14,460 | **5,376** | 200 |
| grouping | 433 | 433 | 50 |
| **frame total, synthetic** | **87,379** | **46,555** | 850 |
| **frame total, video mode** | 87,700 | **32,488** | — |

| | before | after | requirement |
|---|---|---|---|
| synthetic | 11.4 FPS | **21.5 FPS** | ≥ 20 ✓ |
| video (2000×2000 clip) | 11.4 FPS | **30.8 FPS** | ≥ 20 ✓ |

**Three defects, all the same shape.** None was a slow algorithm; each was a
fast algorithm applied to far more data than anything read.

1. **The matched filter ran seven full-image passes.** Six of them built a
   per-pixel winning-scale map, which was read at the surviving candidates —
   at most 24 of 307,200 pixels. Evaluating `matched_best()` per candidate
   gives the identical value. 22.7 ms → 2.6 ms.
2. **CFAR wrote a per-pixel SNR map** that was likewise read only at the
   candidates, and paid a square root per pixel to produce it. Removing it,
   hoisting the four annulus row pointers out of the x loop, and testing
   `(cell−mean)² > k²·var` instead of `cell−mean > k·sd` — the same decision
   without the root — took it from 14.5 ms to 5.4 ms per pass.
3. **The top-hat's vertical pass walked one column at a time**, which is a
   cache miss per pixel for a provably O(1) algorithm. Processing 64 columns
   in lockstep — one cache line of `uint8` — took it from 6.4 ms to 2.2 ms.

All three are bit-identical to what they replaced, which the kernel suite
checks against brute force at every structuring-element size. That check caught
a real bug during the third fix: the `fwd`/`bwd` scratch halves were still
split for a single column and overlapped once the strip was wider than one.

**What is not done.** §15's 0.85 ms total assumes something the design does not
state — the budget allows 0.12 ms for a six-scale matched filter over 307,200
pixels, which is ~100 GOPS of summed-area lookups and not reachable scalar or
with AVX2. CP 14.2 also names explicit AVX2 kernels; the wins above are
algorithmic, and SIMD on what remains would plausibly give another 2–3×
(≈ 60–90 FPS), not the 20× the internal budget would need. The graded
requirement is met; the internal budget is not, and closing it would need a
different processing strategy (a decimated response grid, or a region of
interest around the track) rather than faster arithmetic.

### 2.14 Not yet implemented

| Metric | Status |
|---|---|
| `handover_success` | The mode FSM's `Track → Handover` transition exists and is tested, but is **disabled by default** until CP 10.7 builds the quadrant detector. Entering the state would claim a capability the system does not have. |
| `fps` with the GUI on | §13.1 asks for it "reported separately". The headless figure is produced now; the GUI-on figure arrives with CP 14.4's measurement from the shipped binary. |

### 2.12 Where these numbers come from

| Artifact | Produced by | Contains |
|---|---|---|
| `centroid.csv` | `--headless` | one row per frame, §13.2's format — the graded submission artifact |
| `run.json` | `--headless` | every metric above, plus the full scenario echoed and the INV-3 fingerprint |
| `report.html` | `--headless` | the same, rendered, with inline SVG plots; self-contained |
| `compliance.txt` | `--sweep` | §13.3's matrix, broken out per condition |
| `report.html` | `--sweep` | the matrix plus sweep aggregates |

`report.html` is generated in **C++, not Jinja**. §14 specifies Jinja, but the
checkpoint's criterion is "finishing a run produces a showable report with zero
manual steps", and a Python post-process is a manual step unless the binary
invokes it — at which point the shipping binary depends on a Python
installation, which §4's stack does not include and CP 15.5's "unzip and
double-click" rules out. The cost is that the template is a string in a source
file; the benefit is that the criterion is met on a clean machine. Recorded
here rather than silently taken.
