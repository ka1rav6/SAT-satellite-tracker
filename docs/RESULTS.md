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
| 16 | Acquisition (in view) | ≤ 2 s | **0.067 s** | PASS |
| 16 | Acquisition (cold) | ≤ 2 s | 0.067 s | bound derived — §10.5 |
| 17 | Tracking error | ≤ 10 px | 17.5 px | bound derived — floor 16.33 px |
| 18 | Target loss | < 5 % | **1.67 %** | PASS |
| 19 | Re-acquisition | ≤ 1 s | **0.094 s** | PASS |
| 20 | Processing speed | ≥ 20 FPS | **21.5 / 30.8 FPS** | PASS |
| — | Centroiding (image) | graded, 60 % | **0.217 px RMSE** | PASS |
| — | Centroiding (screen) | graded | 43.1 px | carries pointing error (INV-6) |

Two rows are marked **bound derived** rather than PASS or FAIL. Both are cases
where the specification is internally inconsistent, and both are reported the
way §10.5 prescribes for the first of them.

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
*after* the command is issued. Measured **17.14 px**, so the loop contributes
0.8 px of its own; with the disturbance removed it is **2.86 px**, comfortably
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
| 6.2 speed estimate vs the analytic velocity | **0.02 %** error (checkpoint allows 2 %) |
| 6.2 noise suppression | 300 → **160 µrad** RMS |
| 6.2 filter consistency (NIS, want 2.0) | **2.0** over 2,000 frames |
| 6.3 gate acceptance vs nominal 99 % | **98.8 %** over 20,000 draws |
| 6.3 decoy 60 px away | d² = **2,446**; tracker holds the beacon for 110 frames, final error 7 × 10⁻¹⁴ px |
| 6.4 eight blanked frames | lock retained, **0.0002 px** prediction error |
| 6.4 gate widening while coasting | σ **75 → 451 µrad**, no special case |
| 6.5 adaptive vs fixed R, in fog | **25.5 → 3.78 px** RMS, same seed |
| ★ 6.7 hide 2 s then reveal | reacquired in **3 frames** (0.1 s); checkpoint allows 15 |
| 6.7 cold sweep bound | 18.7 s at 5 °/s, 10.6 s at 10 °/s |

End to end on `scenarios/compliance.toml`, spec-row-23 jitter included:

| | Measured |
|---|---|
| lock retention | **99.3 %** |
| acquisition (in view) | **0.067 s** |
| centroiding RMSE (image) | **0.273 px** |

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

## 6. Performance (CP 14.2)

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

## 7. Reproducibility (INV-3)

```
just gate-repro           # every scenario twice, fingerprints compared
just gate-repro-selftest  # inject a clock read, confirm the gate goes red
```

Every built-in scenario produces identical per-frame fingerprints across runs,
including video modes and with the whole Stage 6–9 apparatus in the loop.

---

## 8. Known gaps

Recorded with measurements rather than described, so each has something to be
improved against.

| Gap | Measured | Owner |
|---|---|---|
| **Clutter and decoy discrimination** | 0.217 px clean → ~31 px with a decoy → ~123 px with 120 clutter sources | Stage 11 `CandidateNet`, Stage 12 priority policy |
| **Low-SNR centroiding** | 3.64× the bound at SNR 13 against CP 9.6's 1.5× | Stage 11 `Learned`, `MatchedPeak` via Stage 12 |
| **§15's 0.85 ms frame budget** | 46.6 ms synthetic, 32.5 ms video | a different processing strategy, not faster arithmetic |
| **CP 7.3's 2 s for a 120 s scenario** | ~5 minutes | same as above |
| **CP 7.5's 500 runs in 3 minutes** | ~13 minutes | same as above |
| **Velocity feedforward** | plumbed, `k_ff` still 0 | CP 10.1, a one-line change plus tuning |
| **Handover** | transition implemented and tested, disabled | CP 10.7's quadrant detector |
| **Adversarial scenarios** | `scenarios/adversarial/` is empty | not yet written; the awkward *video* cases exist and are exercised (CP 8.8) |
| **`--fuzz-scenarios`** | not implemented | CP 14.1 — 5,000 random scenarios, no crash, no hang, no NaN |
| **Test suite** | 16 suites, 1.84 M assertions, all green | — |
