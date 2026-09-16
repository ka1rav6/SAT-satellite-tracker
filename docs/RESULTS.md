# SAT — Measured results

Every number here comes from a test or a command in this repository, and each
row says which. Nothing is quoted from memory; if a figure cannot be
reproduced by the command beside it, that is a bug.

Machine: 8-core x86-64, Release build (`-O2`), GCC 13.

---

## 1. Graded requirements (spec §3.2 rows 16–20)

Produced by `just sweep` — 200 runs over 40 conditions × 5 seeds — and written
to `logs/sweep/compliance.txt`.

**Clear air, no noise, no clutter, no decoy — the loop on its own:**

| Row | Requirement | Spec | Measured | Status |
|---|---|---|---|---|
| 16 | Acquisition (in view) | ≤ 2 s | **0.067 s** (p95 0.067) | PASS |
| 16 | Acquisition (cold) | ≤ 2 s | 0.067 s | bound derived — §10.5 |
| 17 | Tracking error | ≤ 10 px | 17.64 px (p95 29.07) | bound derived — floor 16.33 px |
| 18 | Target loss | < 5 % | **1.67 %** (p95 1.67) | PASS |
| 19 | Re-acquisition | ≤ 1 s | *no episodes in this arm* | — |
| 20 | Processing speed | ≥ 20 FPS | 32.24 FPS, **p5 13.70** | MARGINAL |
| — | Centroiding (image) | graded, 60 % | **0.141 px RMSE** (p95 0.150) | PASS |
| — | Centroiding (screen) | graded | 43.14 px | carries pointing error (INV-6) |

Two rows in that table cannot be answered from this arm alone.

**Row 19 records no episodes here** — the loop never loses lock on clear air
with nothing in the frame, so there is nothing to re-acquire. Across all 200
runs, where harder conditions do produce dropouts, re-acquisition is
**0.090 s** (p95 0.633), comfortably inside the 1 s budget. An earlier version
of this table printed that aggregate figure in the clean row, which read as a
measurement of a condition that never occurs in it.

**Row 20 is MARGINAL, not PASS.** The verdict rule is deliberately strict —
`PASS` requires the *tail* inside the limit, not just the central figure,
because "a requirement met on average and missed one run in twenty is not met"
(§13.3). The median is 32.24 FPS and the 5th percentile is 13.70. The
dedicated frame-budget measurement in §7, which times the shipped binary rather
than a sweep worker sharing eight cores with seven others, gives 21.5 FPS
synthetic and 30.8 FPS on video — that is the figure to quote for the
requirement, and the sweep's low tail is contention, not the tracker.

Two rows are marked **bound derived** rather than PASS or FAIL. Both are cases
where the specification is internally inconsistent, and both are reported the
way §10.5 prescribes for the first of them.

The figures above are the `clear / no noise / no clutter / no decoy` row of the
matrix. The other 39 conditions are in `logs/sweep/compliance.txt` and several
of them FAIL — with 120 clutter sources target loss reaches 4.4 %, and in
`lowlight` it reaches 86 %. Those are recorded in §9 rather than hidden: they
are what Stage 12's supervisor and Stage 13's acquisition strategy exist for.

### Row 16 (cold acquisition) — a geometric bound

Screen 12.5° × 12.5°, camera 4° × 3° ⇒ 25 tiles and ~18.7 s of travel at
5 °/s. P(beacon visible at t=0) is 7.68 %. **A cold search cannot meet 2 s**,
and no amount of cleverness in the search pattern changes that.

```
just cp67          # prints the sweep bound and asserts it exceeds 2 s
```

The metric that *is* meetable — and, per §10.5, the intended reading — is
acquisition from the moment the beacon enters the field of view: **0.067 s**,
two frames.

### Row 17 vs row 23 — an arithmetic bound

Row 17 caps tracking error at 10 px. Row 23 independently specifies up to
±20 px/frame of jitter applied to the **true** boresight, which the encoder
never observes. Uniform[−A,A] has variance A²/3, so over two axes:

```
RMS floor = sqrt(2 · 400/3) = 16.33 px
```

No controller can go below that — the disturbance displaces the boresight
*after* the command is issued. Measured **17.60 px**, so the loop contributes
0.7 px of its own; with the disturbance removed it is **3.75 px**, comfortably
inside row 17.

```
just spec-run      # prints both arms and the derived floor
```

---

## 2. Perception (Stage 5)

```
just test-one kernels          # the kernels against OpenCV oracles
just test-one perception       # the whole pipeline against the simulator
```

| Claim | Measured |
|---|---|
| median removes spec-row-21 impulse noise | 1,593 impulses in → **5** surviving |
| median is bit-exact vs `cv::medianBlur` | 1,000 random images, **0** mismatches |
| van Herk is O(1) in structuring-element size | k=5 → k=51 time ratio **1.10** |
| summed-area tables are exact | 10,000 random rectangles, **0** error (float32 would inject 37.3 grey levels/px) |
| matched filter SNR at scales 5 / 11 / 20 | **16.7 / 43.7 / 23.7** |
| CFAR false-alarm rate vs theory | **4.69 × 10⁻⁵** measured, 4.81 × 10⁻⁵ predicted |
| CFAR guard band | SNR **11.80** with, 5.26 without |
| candidate reduction (CP 5.7) | 15,929 bright px → 84 blobs → **10** candidates |
| ★ CP 5.9 worst case (fog + max noise + 120 clutter + decoy) | beacon in the top 5 on **100 %** of frames |

---

## 3. Tracking (Stage 6)

```
just test-tracking
just cp67          # the ★ end-to-end reacquisition checkpoint
```

| Checkpoint | Measured |
|---|---|
| 6.2 speed estimate vs the analytic velocity | **0.01 %** error (checkpoint allows 2 %) |
| 6.2 noise suppression | 300 → **160 µrad** RMS |
| 6.2 filter consistency (NIS, want 2.0) | **2.0** over 2,000 frames |
| 6.3 gate acceptance vs nominal 99 % | **98.8 %** over 20,000 draws |
| 6.3 decoy 60 px away | d² = **2,446**; tracker holds the beacon for 110 frames, final error 7 × 10⁻¹⁴ px |
| 6.4 eight blanked frames | lock retained through 7 of 8 on the prediction |
| 6.4 gate widening while coasting | σ **75 → 451 µrad**, no special case |
| 6.5 adaptive vs fixed R, in fog | **25.5 → 3.78 px** RMS, same seed |
| ★ 6.7 hide 2 s then reveal | reacquired in **5 frames** (0.17 s); checkpoint allows 15 |
| 6.7 cold sweep bound | 18.7 s at 5 °/s, 10.6 s at 10 °/s |

End to end, spec-row-23 jitter included, on the synthetic scenario
`tests/metrics/test_spec_run.cpp` builds — **not** on `compliance.toml`, which
an earlier version of this table mislabelled. The difference matters:
`compliance.toml` carries §9.1's 120 clutter sources, and those cost two orders
of magnitude (see §8).

| | Measured |
|---|---|
| lock retention | **98.3 %** (118 of 120 in-FOV frames) |
| acquisition (in view) | **0.067 s** |
| tracking error | **17.60 px** with row-23 jitter, **3.75 px** without |
| centroiding RMSE (image) | **0.201 px** |

---

## 4. Video ingest (Stage 8 — Benchmark Performance-2, 30 %)

```
just test-video
just video tests/video/clips/screen_2000x2000_30fps.mp4
```

| Checkpoint | Measured |
|---|---|
| 8.1 decoder keeps ahead | consumer waited **1** time in 60 frames (the first pop) |
| 8.2 clock divisor re-derived | 30 fps → 10, 25 → 12, 60 → 5, VFR → **0 and reported** |
| 8.3 bicubic vs bilinear, half-pixel offset | peak **226.6** vs 210, analytic 230 |
| **8.6 crop's own centroid error** | **0.0033 px RMS**, 0.0099 px worst, over 400 sub-pixel phases |
| 8.6 integer offset is an identity | exact, pixel for pixel |
| **8.7 self-scoring, whole path** | **0.0036 px RMSE**, bias −0.0025 px |
| 8.8 fourteen awkward clips | all run or refuse cleanly; **none crash** |
| 8.5 auto-detection | 2000×2000 → screen, 641×481 → direct |

---

## 5. Centroiding (Stage 9 — 60 % of the marks)

```
just test-centroid
just calibrate              # regenerates the compiled-in bias table
```

### The S-curve correction (CP 9.3)

`WindowedCoM`, 5 px beacon, before → after:

| SNR | before | after | gain |
|---|---|---|---|
| ≤ 13 | — | — | 1.00× (no cell stored) |
| 20 | 0.334 px | 0.252 px | 1.33× |
| 32 | 0.246 px | 0.145 px | 1.69× |
| 50 | 0.171 px | **0.084 px** | **2.02×** |
| 80 | 0.119 px | 0.064 px | 1.86× |

§10.1.3 expects 2–5× at high SNR; we are at the bottom of that range. 30 of
144 candidate cells qualify for storage — a cell is stored only where the fit
explains at least 10 % of the error's variance, because below that it is
fitting noise and subtracting it made the estimate 2–5 % *worse*.

### Against the theoretical bound (CP 9.5, CP 9.6)

`σ ≳ w / (2·SNR)`, 10 px square beacon, calibrated:

| SNR | bound | measured | ratio |
|---|---|---|---|
| 3 | 1.667 px | 2.29 px | 1.37× |
| 5 | 1.000 px | 2.03 px | 2.03× |
| 8 | 0.625 px | 1.84 px | 2.94× |
| 13 | 0.385 px | 1.40 px | 3.64× |
| 20 | 0.250 px | 0.52 px | 2.06× |
| 32 | 0.156 px | 0.37 px | 2.35× |
| 50 | 0.100 px | 0.19 px | 1.89× |
| 80 | 0.063 px | 0.11 px | 1.69× |

**CP 9.6 asks for 1.5× at every bin and we do not meet it below SNR 20.** See
[`METRICS.md` §2.11](METRICS.md) for why and what would close it. The graded
scenarios run in the upper half of this table — a 10 px beacon at spec row 22's
maximum noise integrates to about SNR 60.

---

## 6. Control refinement (Stage 10)

```
just test-control          # every number below, as an assertion
just cp101                 # feedforward on/off, with the plot
just cp102                 # the slew, with and without anti-windup
just cp105                 # the IMM on the figure-8, with the mode panel
just cp106                 # error and saturation against disturbance
just cp107                 # handover success rate over 20 seeds per arm
```

`scenarios/control/*.toml` are **instruments, not compliance claims**. Each one
removes whatever is larger than the effect under test — usually row 23's
jitter, whose 16.33 px floor buries anything smaller — and says so at length in
its own header. No row of the compliance matrix is measured on them.

### CP 10.1 — velocity feedforward

200 px/s target, scored over the two seconds after acquisition:

| | RMS | signed (azimuth) |
|---|---|---|
| `k_ff = 0` | 21.60 px | −15.20 px (lagging) |
| `k_ff = 1` | **2.53 px** | −1.81 px |

The checkpoint asks for "under 4 px". The design quotes ~11 px for the *before*
case on a 3 Hz loop; ours is kp = 8 rad/s = 1.27 Hz, so the predicted lag is
v/kp = 25 px and the test checks against the formula rather than the literal
number.

**The k_ff sweep found a defect, not a tuning.** The optimum sat at 0.75, and
1 − kp·dt = 1 − 8·0.0333 = 0.733 predicts that exactly. The aim point was the
filter's prediction *one frame ahead*, which is itself a velocity feedforward
implemented in the setpoint — so with `k_ff = 1` the velocity was fed twice and
the mount pointed 6.7 px *ahead* of the beacon. Tuning k_ff to 0.75 would have
produced a fine number and left a gain that silently depended on kp and the
frame rate. The aim is now the estimate at the current frame; `k_ff = 1` is
correct for the reason it is supposed to be.

**The lag is a transient, not a steady state.** With integral action a PI loop
nulls a constant-velocity lag in about kp/ki = 4 s:

| `k_ff = 0` | 0.5–1.5 s | 1.5–3 s | 3–5 s | 5–8 s | 8–11 s |
|---|---|---|---|---|---|
| RMS | 24.3 px | 17.5 px | 11.2 px | 6.0 px | 2.8 px |

So a whole-run RMS is largely a statement about run length. On a *manoeuvring*
target the integrator never arrives: a 200 px/s sinusoid scored from 4 s
onwards gives 25.64 px against feedforward's **16.56 px**. The 16.56 px
remainder is acceleration lag, which velocity feedforward cannot touch — that
is CP 10.5's job.

### CP 10.2 — anti-windup, and where `ki` comes from

A saturating 375 px slew (the loop demands 3.75× the mount's rate ceiling):

| | overshoot | lobes | settled |
|---|---|---|---|
| anti-windup on | **7.50 px** | 1 | 2.44 px RMS |
| anti-windup off | 12.06 px | 1 | — |

`ki` was 0.5, which was a guess, and it showed: the integral term was 3 % of
the slew rate, so "anti-windup is essential" was unfalsifiable. It is now
chosen by a criterion — *the largest integral gain whose full-field-slew
overshoot stays inside row 17's 10 px budget*:

| `ki` | slew overshoot | 200 px/s residual (p95) |
|---|---|---|
| 0.5 | 2.07 px | 4.73 px |
| **2.0** | **6.88 px** | **4.04 px** |
| 3.0 | 10.02 px | 3.57 px |
| 8.0 | 23.80 px | 1.50 px |

`ki = 8` is refused despite its residual: a 23.8 px overshoot leaves row 17's
budget on every acquisition, and the overshoot is scored.

**Counting zero crossings is the wrong ringing test.** At `ki = 8` the loop
holds the error to 0.12 px RMS, so noise flips the sign constantly and a
crossing counter reports violent ringing on the best-regulated run in the set.
A lobe only counts if its peak clears a floor.

### CP 10.3 — the design is wrong here

The checkpoint asks for platform drift estimation and cancellation. **The drift
is already cancelled, and implementing the checkpoint as written makes it
worse.**

`measurement.hpp` reconstructs the target's angle through the *commanded*
boresight, which is all the system knows. With the platform displacing the true
boresight by D:

```
B_true = B_cmd + D        beacon lands at  T − B_true
z = B_cmd + (T − B_true) = T − D
```

The filter sees the target at T − D moving at v − D′, so driving B_cmd there
puts B_true exactly on the beacon. The drift is indistinguishable from target
motion and CP 10.1's feedforward carries it.

| platform drift | 0 | 17 px/s | 67 px/s |
|---|---|---|---|
| tracking RMS | 2.53 px | 2.46 px | 2.29 px |

Flat to four times spec row 25's value. And feeding the controller the **true**
drift — a perfect estimator — triples the error, 2.29 px → **6.79 px**.

`platform_rate_est` stays in the signature, reachable from a test, because it
would be needed if the measurement came through the *true* boresight (an
IMU-stabilised mount). The tests pin that today it must be zero.

### CP 10.4 — the Smith predictor, built and left off

| demand | 200 px/s | 600 px/s | 800 px/s |
|---|---|---|---|
| off | 3.55 / p95 6.15 | 11.48 / 18.55 | 16.10 / 25.90 px |
| on | 3.88 / p95 8.26 | 12.41 / 25.66 | 16.96 / 34.78 px |

Worse everywhere, worse at p95 than at RMS — the harm is in transients. **This
loop is not delay-limited**: the horizon is 43 ms and the crossover is
kp = 8 rad/s, so the delay costs 8 × 0.043 = 0.34 rad = **19.9°** of phase out
of a margin starting near 90°. A Smith predictor buys bandwidth when delay is
binding; here the binding constraints are the acceleration limit during
acquisition and the rate ceiling at high demand.

§10.4 anticipated this: *"if it destabilises, leave it off — optional"*. The
test asserts the **threshold** (45° of delay phase) rather than the verdict, so
a future plant with a slower camera or a longer bus turns it red.

Three things were found while building it, all kept in the code:

- Advancing the measurement alone is CP 10.1's bias with the sign flipped.
  Both sides of the error move or neither does.
- `round(10 ms / 33 ms)` is **zero**. The delay that matters is the loop's
  sampling period, not the mount's transport delay.
- `build()` set the controller's plant model and `build_from_scenario()` did
  not, so every scenario run used a model with no rate ceiling. In both cases
  the tell was *identical numbers after a change that should have mattered*.

### CP 10.5 — the IMM

Spec row 12's mandatory figure-8, two laps scored after the first:

| | RMS | peak |
|---|---|---|
| single CV filter | 21.75 px | 30.30 px |
| **IMM (CV/CA/CT)** | **17.15 px** | **22.52 px** |

Six states, `[az, el, ȧz, ėl, äz, ël]`, because a constant-acceleration model
needs somewhere to keep the acceleration and the IMM's mixing step needs one
shared state space. Opt-in (`tracking.imm`) so the reproducibility fingerprints
stay meaningful on both settings; it costs nothing to leave off — slightly
*better* on a straight line (3.03 → 2.81 px), neutral on `compliance.toml`.

**§10.2 is wrong about where the error is.** It says the overshoot is "at the
figure-8 crossing". For x = A sin(2πt/T), y = B sin(4πt/T), the curve
self-intersects where both are zero, and there both second derivatives are
proportional to sin(0) — the acceleration is not reversing, it is **zero**, and
the path is locally straight. That is exactly where a CV model is right:

| phase in the 8 s lap | 0–1 | 1–2 | 2–3 | 3–4 | 4–5 | 5–6 | 6–7 | 7–8 |
|---|---|---|---|---|---|---|---|---|
| single CV, px | 13.55 | **26.06** | 17.95 | **26.51** | 13.45 | **26.07** | 18.04 | **26.61** |

Smallest at the crossing (t = 0, 4), roughly double at the **lobe ends** a
quarter-lap away. The effect is real and the IMM reduces it; the location in
the design's sentence is not where it happens.

Mode probabilities over two laps: CV [0.32, 0.43], CA [0.08, 0.18],
CT [0.44, 0.53] — they sum to 1 to float epsilon, they move, and none reaches
zero (zero is absorbing). Peak cross-axis position correlation **0.041**,
which is §10.2's predicted anisotropy arriving with the coordinate-turn model.

### CP 10.6 — where the loop breaks down

200 px/s target, row 25's platform motion scaled up and opposing it:

| drift, px/s | 0 | 100 | 200 | 300 | 400 | 500 | 600 | 700 | 800 |
|---|---|---|---|---|---|---|---|---|---|
| demand, px/s | 200 | 300 | 400 | 500 | 600 | 700 | 800 | 900 | 1000 |
| RMS, px | 3.55 | 5.47 | 7.45 | 9.47 | **11.48** | 13.57 | 16.10 | 20.69 | 29.43 |
| saturation, % | 0 | 0 | 0 | 0 | 0 | 0 | **0.37** | 2.90 | 8.34 |

Two numbers, and they are not the same one:

- **The mount saturates at 600 px/s of drift**, exactly where the geometry
  says: the ceiling is 5 °/s = 800 px/s per axis and the demand is 200 + d.
- **Row 17 is lost at 400 px/s** — three quarters of the actuator's authority,
  with the plant never once at its stops.

Saturation is not what breaks it. The error is proportional to demand and
decays within each run, which is transport lag plus a 4 s integrator, not a
rate limit.

### CP 10.7 — handover

| | success | time |
|---|---|---|
| clean, 20 seeds | **20/20 (100 %)** | median 2.43 s |
| spec row 23 jitter, 20 seeds | **0/20 (0 %)** | best sustained offset 1467–1606 µrad |

The criterion is RMS offset below `capture/3` = 333 µrad for 30 consecutive
frames. Row 23 allows ±20 px/frame on the **true** boresight — the encoder sees
none of it, a co-boresighted quadrant cell sees all of it — and
√(2·400/3) = 16.33 px = **1781 µrad**. The criterion is five times out of
reach, by the same arithmetic that already puts a floor under row 17.

The system does not hand over, and reports the number it got to. Claiming an
alignment a real fine sensor would immediately lose is the failure worth
avoiding.

---

## 7. The SAT supervisor (Stage 12)

```
just test-supervisor
just cp122          # the strategy timeline through a weather change
just cp123          # per-condition Monte Carlo: where adapting pays
```

The claim in the project's name: the system picks its configuration at runtime
from what it can observe. §10.6's standard for it is *"log every switch with
its trigger … that turns 'adaptive' from a claim into data."*

### It is worth nothing where the fixed configuration copes

`scenarios/supervisor/weather_change.toml` — fog at 10 s, clear at 25 s — with
the supervisor off and on:

| Beacon | Retention (fixed → supervised) | Best metric moved |
|---|---|---|
| intensity 120 | 99.8 % → 99.8 % | nothing; fogged SNR lands in the middle band |
| intensity 60 | 93.8 % → **97.6 %** | centroiding 64.2 → 39.3 px |
| intensity 40 | 77.4 % → **86.3 %** | centroiding 125.7 → 97.5 px |
| intensity 30 | 64.7 % → **80.1 %** | tracking 16.42 → **5.42 px** |

The gain grows as conditions worsen and is **zero** where they do not. That is
the correct behaviour, not a disappointing result: a supervisor that adapts
when there is nothing to adapt to would be a worse system.

At spec row 7's nominal brightness the fogged SNR is ≈ 12, inside §10.6's
middle band (8–15) where the rule table deliberately does nothing — neither
rule's evidence applies.

### The three mandatory properties (§10.6)

| Property | Measured |
|---|---|
| never react to one frame | an SNR flipping across a threshold **every frame** for 10 s produces **1** switch |
| at most one switch per second | over 30 s of a sweeping signal, closest pair **30 frames** apart |
| bumpless gain switching | integral term continuous to **2 %** across a 2× ki change; `reset()` instead kicks by half the term |
| every switch has a trigger | e.g. `integrated SNR below 8` |

### Design §7.4's events did not exist

Found while looking for a scenario whose conditions change. The timeline was
parsed, validated and echoed into `run.json`, and **nothing read
`sc.events`** — a scenario could ask for fog at 8 s, be told the request was
valid, and run in clear air. All four actions now execute.

```
just test-supervisor   # "design 7.4's events actually happen"
```

Measured: detection SNR **40.76 before** the fog event, **12.09 after**.

---

## 8. Performance (CP 14.2)

```
just stages        # per-stage p50/p95/p99 from the shipped binary
```

| Stage | before | after | §15 budget |
|---|---:|---:|---:|
| frame_acquire *(simulator only)* | 24,805 µs | 24,805 µs | 740 |
| median_3x3 | 621 | 621 | 350 |
| top_hat | 6,436 | **2,187** | 300 |
| summed_area (×2) | 1,999 | 1,999 | 350 |
| matched_filter | 22,671 | **2,618** | 120 |
| cfar (×2) | 14,460 | **5,376** | 200 |
| grouping | 433 | 433 | 50 |
| **total, synthetic** | **87,379** | **46,555** | 850 |
| **total, video mode** | 87,700 | **32,488** | — |

| | before | after | requirement |
|---|---|---|---|
| synthetic | 11.4 FPS | **21.5 FPS** | ≥ 20 ✓ |
| video, 2000×2000 clip | 11.4 FPS | **30.8 FPS** | ≥ 20 ✓ |

All three fixes are bit-identical to what they replaced. §15's 0.85 ms internal
budget is **not** met and is not reachable by these means — see
[`METRICS.md` §2.13](METRICS.md).

---

## 9. Reproducibility (INV-3)

```
just gate-repro           # every scenario twice, fingerprints compared
just gate-repro-selftest  # inject a clock read, confirm the gate goes red
```

Every built-in scenario produces identical per-frame fingerprints across runs,
including video modes and with the whole Stage 6–9 apparatus in the loop.

---

## 10. Known gaps

Recorded with measurements rather than described, so each has something to be
improved against.

| Gap | Measured | Owner |
|---|---|---|
| **Clutter and decoy discrimination** | over 200 runs: tracking 17.64 px clean → 44.57 px with one decoy → **205.20 px** with 120 clutter + decoy; centroiding 0.141 → 198.33 px | Stage 11 `CandidateNet`, Stage 12 priority policy |
| **Low light** | target loss reaches **86 %** in `lowlight` — the beacon is below the detector's floor, not mis-associated | Stage 11 `CandidateNet`, Stage 13 acquisition strategy |
| **Low-SNR centroiding** | 3.64× the bound at SNR 13 against CP 9.6's 1.5× | Stage 11 `Learned`, `MatchedPeak` via Stage 12 |
| **§15's 0.85 ms frame budget** | 46.6 ms synthetic, 32.5 ms video | a different processing strategy, not faster arithmetic |
| **CP 7.3's 2 s for a 120 s scenario** | ~5 minutes | same as above |
| **CP 7.5's 500 runs in 3 minutes** | ~13 minutes | same as above |
| **Handover under spec row 23** | 100 % clean, **0 %** at ±20 px/frame of jitter | nothing in the control law; row 23 is 5× the criterion (§6) |
| **Figure-8 acceleration lag** | 16.56 px residual with feedforward on a 200 px/s sinusoid | Stage 11 `MotionNet` priors into the IMM |
| **§15's 0.85 ms budget, IMM included** | IMM adds ~0.01 ms; the frame is still 46.6 ms | same as the row above it |
| **Adversarial scenarios** | `scenarios/adversarial/` is empty | not yet written; the awkward *video* cases exist and are exercised (CP 8.8) |
| **`--fuzz-scenarios`** | not implemented | CP 14.1 — 5,000 random scenarios, no crash, no hang, no NaN |
| **Supervisor on clutter** | the adaptation recovers **+15 points** of retention in fog and **0** against clutter — it varies thresholds, not the association policy | §10.2's `priority_score` with hysteresis; not yet built |
| **Stage 13 acquisition strategy** | not started | the low-light row above is what it is for |
| **CP 12.4 learned `StrategyPolicy`** | not started (ML) | Stage 11 |
| **Test suite** | 19 suites, all green | — |
