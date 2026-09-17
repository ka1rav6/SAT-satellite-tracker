# Issues till now

A running, honest list of everything in SAT that is **not yet up to the mark** —
unimplemented, implemented but under-performing, or implemented in a way that a
measurement has since shown to be wrong.

Rules for this file:

* Every open item carries a **measurement**, not an adjective. "Slow" is not an
  issue; "10,090 µs against a 40 µs budget" is.
* Items are checked off only when a command in the `Justfile` reproduces the fix.
* Nothing is deleted when it closes. A closed item keeps its before/after so the
  file doubles as the change log for quality, which is what `docs/RESULTS.md`
  cites.

Legend: `[ ]` open · `[x]` closed · `[~]` partially closed · `[-]` closed as
*won't fix*, with the reason stated.

---

## 1. Performance — design §15's 0.85 ms frame budget

Measured with `just stages` on `scenarios/compliance.toml` (120 clutter sources,
1 decoy, full damage chain, 640 × 480 sensor), Release `-O3`, single core.

### 1.0 The stage table was measuring the wrong things

- [x] **`frame_acquire` was one opaque 22.7 ms line.** Design §15 budgets
      *Background render*, *Emitter splat* and *Damage chain* as three separate
      rows, and the profiler collapsed all three into one, so there was no way
      to tell which of them was over. Split into `background`, `splat` and
      `damage_chain`, with `frame_acquire` kept as a non-additive parent.
- [x] **`centroid` was reported as 865× over its budget.** The zone labelled
      `Stage::Centroid` in the engine wrapped the *entire* detector, so the
      number printed against §15's 0.02 ms "Centroid + bias correction" line was
      really the whole of B6–B13. Renamed to `Stage::Perception`;
      `Stage::Centroid` now times B12/B13 alone and measures **24 µs against a
      20 µs budget**, which is the honest figure and is essentially on target.

### 1.1 Open performance gaps

| Stage | Measured p50 | §15 budget | Over | Status |
|---|---:|---:|---:|---|
| `splat` | 10,090 µs → **973 µs** | 40 | 24× | [~] separable factorisation landed; see 1.2 |
| `damage_chain` | 12,079 µs | 550 | 22× | [ ] |
| `cfar` (×2) | 4,914 µs each | 200 | 25× | [ ] |
| `matched_filter` | 2,393 µs | 120 | 20× | [ ] |
| `summed_area` (×2) | 1,827 µs each | 350 | 5× | [ ] |
| `top_hat` | 1,999 µs | 300 | 7× | [ ] |
| `median_3x3` | 567 µs | 350 | 2× | [ ] |
| `grouping` | 396 µs | 50 | 8× | [ ] |
| `background` | 135 µs | 150 | 0.9× | [x] within budget |
| `centroid` | 24 µs | 20 | 1.2× | [x] within budget |
| **`frame_total`** | **42,551 µs** | **850** | **50×** | [ ] |

### 1.2 What has been done, and what is left

- [x] **Emitter splat: 10,090 µs → 973 µs (10.4×).** The splat walked its
      footprint pixel by pixel calling a 2-D coverage function. Two of the three
      shapes are *separable* — a square's coverage is `overlap_x(i)·overlap_y(j)`
      and a Gaussian's is `gauss_x(i)·gauss_y(j)·norm` — so a W×H footprint needs
      W+H one-dimensional evaluations, not W·H two-dimensional ones. For a 24 px
      Gaussian clutter source that is 496 `erf` calls instead of 61,504. The
      circle is not separable but decomposes by inclusion-exclusion over shared
      grid corners, which is exactly 4× fewer evaluations. Every expression
      multiplies its factors in the same order as the function it replaces, so
      the result is bit-identical, not merely equal.
- [ ] **Emitter splat, the remaining 24×.** Two further levers, neither taken
      yet: a Gaussian clutter source is drawn out to 6σ, which is 2.9× more area
      than the 3.5σ at which its contribution drops below one 8-bit grey level
      (6σ is required for the *graded* beacon, where truncation biases the
      centroid — it is not required for clutter); and the inner row loop still
      carries a per-pixel branch that blocks vectorisation.
- [ ] **Damage chain.** Five separate full-frame passes (atmosphere, shot noise,
      read noise, fixed pattern, quantise) over a 1.2 MB float buffer, each one
      reading and writing it. Two `next_normal()` calls per pixel, each a
      Marsaglia polar rejection loop with a `log` and a `sqrt`. Needs fusing to a
      single pass and a vectorised generator.
- [ ] **The detector runs full-frame in every mode.** 640 × 480 = 307,200
      pixels of median, top-hat, two summed-area tables, matched filter and two
      CFAR passes, on every frame, including frames where the tracker already
      knows where the target is to within a few pixels.
- [ ] **AVX2 kernels (CP 14.2's second half).** The build has no `-march`
      beyond baseline x86-64, so every kernel is SSE2. The host supports AVX2
      and AVX-VNNI.

### 1.3 Downstream of the frame budget

- [ ] **CP 7.3 — "a 120 s scenario completes in under 2 s wall time".** Measured
      ~5 minutes. It is the frame budget, multiplied by 3,600 frames.
- [ ] **CP 7.5 — "500 runs complete in under 3 minutes on 8 cores".** Measured
      ~13 minutes. Same cause.

---

## 2. Accuracy and robustness

### 2.0 Four defects found by running the scenarios for THIRTY seconds

Every scenario in this project had been exercised at six seconds. Running
`scenarios/baseline.toml` — the specification's own defaults, its own seed — for
thirty produced this:

```
retention      n/a   (the beacon was never in view)
false tracks   1798 /min   (every frame of the run)
tracking RMS   1272 px
```

Four independent defects, each of which alone would produce that line.

- [x] **Spec row 8's `edge_behaviour` was never implemented.** Parsed,
      validated by the schema, echoed into `run.json`, read by nothing. The
      beacon started at (1936, 1831) on a 2000 × 2000 canvas and left it after
      three seconds; every metric that run produced was a measurement of an
      empty screen. Implemented as a triangle-wave fold of the analytically
      evaluated coordinate, which keeps §7.2's exactness properties that a
      reflected-velocity implementation would break.
      `just test-motion` · design amendment §14.0c.
- [x] **The detector returned six candidates a frame on pure read noise.**
      CFAR's `k` is a per-pixel statement, and at the specification's operating
      point about 29 pixels of a 640 × 480 frame fire every frame — by design.
      But CFAR runs on the matched-filter RESPONSE, which is box-summed over the
      target scale, so each one is smeared into a contiguous blob of exactly
      beacon-like area, fill and aspect. Measured: six candidates of SNR 4.16 to
      4.70 against a beacon at 248.92, every one passing the shape gate. Fixed
      with an SNR gate at 1.5 × k, which turns 29 false alarms a frame into one
      every 650 frames while the beacon clears it by 40× in clear air and 2× in
      fog.
- [x] **A track fed by occasional noise lived for ever.** `Confirmed → miss →
      Coasting → any hit → Confirmed`, with the consecutive-miss counter reset
      by every hit, so an alternating hit/miss pattern could never reach
      `coast_max_misses`. Worse, the gate widens with the miss count, so each
      miss made the next spurious hit easier to catch. Fixed with two quality
      tests — a hit ratio over a long window, and an EMA of the normalised
      innovation squared, which is 2 for a real track and near the gate's 9.21
      for one being fed by whatever falls inside it.
- [x] **§10.2's priority score chooses the clutter.** All four of the design's
      terms — SNR, stability, centrality, age — are things a bright static
      source scores the maximum on, and `Tracker::step` made it worse by seeding
      from "the strongest candidate". Fixed by adding a fifth term (motion), by
      measuring it RELATIVELY, and by structuring the weights so the four
      non-motion terms cannot reach the commit threshold on their own. See §2.1.

### 2.1 What the motion term took to get right

Recorded because three successive versions were wrong in instructive ways.

| version | why it failed | measured |
|---|---|---|
| raw apparent speed | the tracker converts pixels to angles through the COMMANDED boresight, so rows 23 and 25 make static clutter *appear* to move at minus the platform rate | clutter 1854 µrad/s vs beacon 831 — the discriminator **inverts** |
| lifetime displacement, ego-motion subtracted at the end | row 23's ±20 px/frame jitter needs 45 frames to average down — 1.5 s of row 16's 2 s budget | velocity standard error 2542 µrad/s against a 2682 µrad/s signal |
| per-frame median subtraction, lifetime window | correct for straight lines; a closed path (row 12's circular and lissajous) returns to its start, so a beacon on an orbit reads as stationary | net displacement ≈ 0 over one period |
| **per-frame median, 32-frame sliding window** | — | clutter < 1 µrad/s, beacon recovers its own 2400 µrad/s to 5 % |

Two further rules were needed and both came from measurement:

- a **coasting frame contributes nothing**. A miss moves the estimate by the
  filter's own prediction, so folding it in is evidence for a belief drawn from
  that belief. A weak track that picked up one spurious fast step kept
  confirming it and took the mount on the motion term alone.
- a frame where the **ego-motion could not be measured** contributes nothing
  either. The median needs three live tracks; with fewer there is no way to tell
  a target's motion from the platform's, and a stale estimate is worse than no
  answer. This matters most right after a lock, when the camera has centred one
  object and everything else has left the field of view.

### 2.2 A fifth defect, found by the other four

- [x] **Four Stage 10 control cases were scoring a transient.** The loop's
      integral time constant is kp/ki = 8/2 = **4 seconds**, and what the
      integrator is removing is real — the plant's 10 ms of transport delay and
      20 ms of lag (rows 13–15) leave the mount about 6 px behind at 200 px/s
      however perfect the feedforward is. The cases scored 0.5 s to 2.5 s, by
      which point the integrator has removed 46 % of it.
      They all moved by a factor of two when §10.2's policy shifted acquisition
      by **one frame**, with the tracker's own estimates unaffected (0.005 px
      and 0.10 px of position error against truth; rate estimates agreeing to
      0.08 %). They now score 8 s to 12 s. Two of them also needed their claims
      restating — see below.
- [x] **The same for the Stage 5 ablation.** It ran 2 s and scored tracking over
      the last quarter — 1.5 s to 2.0 s, deep inside the same transient. Its
      clutter arm read 1.78 px and passed. At 8 s it reads **121.55 px**,
      because two seconds is not long enough for the failure to happen. The
      run is now 8 s and the clutter arm's numbers are pinned rather than
      asserted away; the noise arm still meets row 17 at 0.92 px.
      *The wall-time argument for 2 s has also gone: the frame is 7 ms rather
      than 32, so eight seconds now costs less than two used to.*

### 2.3 Three claims that turned out not to hold, and what replaced them

Recorded rather than quietly re-tuned, because each was a statement about the
system and each is now a different statement about the system.

- **"Over-feeding the feedforward makes things worse."** It does not. Measured
  across four windows, `k_ff = 1.5` has the lower RMS in every one. The reason
  is the plant, not the tracker: a rate command of exactly *v* produces a mount
  rate of *v* only after 30 ms, so the mount is permanently a little behind and
  commanding more than *v* compensates. The RMS minimum sits above 1.
  **What replaced it:** the SIGN, which is unambiguous in every window —
  behind the target at `k_ff = 1`, ahead at `k_ff = 1.5`, monotone in between.
  That is what identifies a lead in the setpoint, which is the defect the case
  was written to catch.
- **"Row 17 is lost at 400 px/s of drift; saturation starts at 600."** Both
  moved up one step of the sweep — lost at **600**, saturation at **800** —
  because the priority policy and the SNR gate stop the loop being driven by
  candidates that are not the beacon. The checkpoint's finding is unchanged and
  is now asserted as the ORDER of the two crossings rather than as two
  constants, so an improvement cannot turn it red.
- **"Cancelling the drift twice is 2× worse."** Scored settled it is **1.75×**,
  because the integrator absorbs part of the double correction — which is what
  an integrator is for. The finding, that it is worse at all, is unchanged.

### 2.4 Still open

- [ ] **Acquisition from cold, in clutter.** `scenarios/baseline.toml` starts
      the beacon at a random screen position (row 11), so the camera sees 9.8 %
      of the screen and must search. With 120 clutter sources the policy
      correctly refuses to commit to any of them, but the spiral has not swept
      back over the beacon within 30 s. The beacon is never acquired.
      This is Stage 13's problem, not §10.2's: the sweep bound is 18.72 s at
      5 °/s and the target moves while it runs.
      *With `clutter.static_sources = 0` the same scenario acquires and holds.*
- [ ] **A moving decoy is not separated.** It moves, it is bright, it is
      stable, and it is placed 60–300 px from the beacon. The motion term cannot
      help — the decoy really is moving. Owner: Stage 11's `CandidateNet`
      (out of scope — ML).
- [ ] **A genuinely stationary target is not separated from clutter either.**
      Motion is one weighted term and not a gate, so such a target competes on
      the footing everything had before this policy existed. The specification's
      row 12 makes all four mandatory motion modes moving, so this is a
      limitation on configurations the specification does not describe — but it
      is a real one and `tracking.priority = false` is the escape hatch.
- [ ] **The supervisor recovers nothing against clutter.** `just cp123`
      measures **+15 points** of lock retention in fog and **0** against
      clutter. It adapts thresholds and estimators; it does not adapt the
      association policy.
- [ ] **Low light.** Target loss reaches **86 %** in the `lowlight` weather mode.
      The beacon is below the detector's noise floor, not mis-associated, so
      this is a sensitivity problem and not an association one.
- [ ] **Low-SNR centroiding.** 3.64× the theoretical bound at SNR 13, against
      CP 9.6's requirement of 1.5× at every bin. Above SNR 20 the requirement is
      met. Owner: Stage 11's `Learned` estimator (out of scope — ML).
- [ ] **Figure-8 acceleration lag.** 16.56 px residual with feedforward on a
      200 px/s sinusoid — the feedforward carries velocity, not acceleration.
- [-] **Handover under spec row 23.** 100 % success on a clean run, **0 %** at
      row 23's ±20 px/frame of jitter. Closed as *won't fix in the control law*:
      row 23's jitter amplitude is 5× the quadrant cell's capture criterion, so
      no controller can hold the beam inside a capture range smaller than the
      disturbance. Recorded with the derivation in `docs/RESULTS.md` §6.

---

## 3. Deliverables not yet complete

- [ ] **CP 15.1 — "every §12 panel complete".** The dashboard has the camera
      view, screen overview, both error traces, the compliance table, the stage
      latency table, the parameter table and live damage controls. Missing: the
      IMM mode-probability panel, the SAT strategy timeline and the mode-FSM
      graph.
- [ ] **Documentation for a first-time user.** `QUICKSTART.md` and
      `docs/MANUAL.md` exist but contain no screenshots, and neither walks
      someone through the product end to end.
- [-] **Stage 11 — machine learning, all of it.** Out of scope by explicit
      instruction. `CentroidNet`, `CandidateNet`, `MotionNet`, the ablation table
      and the model cards are not built, and `--gen-dataset` is not implemented.
      `--no-ai` is the shipped path and INV-7 requires it to stay that way.
- [-] **CP 12.4 — the learned `StrategyPolicy`.** Out of scope for the same
      reason. The rule table of CP 12.2 is what ships.

---

## 4. Closed

- [x] **CP 14.1 `--fuzz-scenarios`** — implemented, with explicit corner
      sampling.
- [x] **Adversarial scenarios** — `scenarios/adversarial/` now holds six.
- [x] **Stage 13 acquisition strategy** — probability grid with negative
      information, and the strategy benchmark.
- [x] **Retention reported 100 % on a run that had lost the beacon** — the
      numerator counted confirmed frames without intersecting the denominator's
      in-FOV condition.
- [x] **Design §7.4 events were parsed, validated, echoed to `run.json` and read
      by nothing.** All four actions now execute.
- [x] **INV-4's allocation trap was never armed.** It is now, in Debug, and it
      caught four steady-state allocations.
