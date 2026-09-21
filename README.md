# SAT — Satellite Adaptive Tracker

AI-based virtual camera tracking for coarse alignment of mobile FSOC terminals.
**SIH PS 26169** · Department of Space / ISRO.

A pan–tilt camera has to find a beacon spot on a 2000 × 2000 screen, locate it
to sub-pixel accuracy, and keep it centred through atmospheric degradation,
sensor noise, platform motion and camera jitter — at 30 frames per second, with
a disturbance that can exceed the mount's own authority.

---

## Quick start

```bash
just build         # configure + build
just gui           # the live dashboard
just test          # 20 suites, about 80 seconds
```

**New here? Read [`docs/GUIDE.md`](docs/GUIDE.md).** It is a fifteen-minute
guided tour with screenshots: what the panels mean, what to break first, and how
to score a run.

Three things worth running after that:

```bash
just headless                # one run: metrics, centroid.csv, run.json, report.html
just video tests/video/clips/screen_2000x2000_30fps.mp4    # MP4 ingest
just sweep                   # 200 runs -> the requirement compliance matrix
```

![The dashboard](docs/img/01-overview.png)

---

## Where it stands

Stages 0–10 and 12–15 of the roadmap are complete. All five ★ gates pass.
Machine learning (Stage 11) is deliberately out of scope — INV-7 requires the
system to run fully without it, and it does.

On clear air with nothing else in the frame — 200 runs of 30 s each,
`just sweep`:

| Graded requirement | Spec | Measured |
|---|---|---|
| Acquisition, beacon in view (row 16) | ≤ 2 s | **0.067 s** |
| Target loss, post-acquisition (row 18) | < 5 % | **0.00 %** |
| Re-acquisition (row 19) | ≤ 1 s | **0.109 s** (over all 200 runs) |
| Processing speed (row 20) | ≥ 20 FPS | **394 FPS** (p50) |
| Centroiding accuracy (60 % of the marks) | — | **0.141 px RMSE** |
| FOV containment — the PS's own objective | — | **100 %** |

### Through row 24's weather, measured rather than asserted

`just sweep`, 30 s per cell, five seeds, clutter and decoy removed so the axis
is weather alone. Both impulse-noise settings, because row 21 specifies ~10 %
and the difference it makes is the interesting part:

| Row 24 mode | centroiding, S&P 0 | centroiding, S&P 10 % | p95 | target loss (post-acq) |
|---|---:|---:|---:|---:|
| clear | 0.141 px | **0.497 px** | 1.71 | 0.00 % |
| haze | 0.182 px | **1.291 px** | 5.41 | 0.00 % |
| rain | 0.252 px | **1.355 px** | 5.42 | 0.00 % |
| fog | 0.472 px | **1.543 px** | 5.44 | 0.05 % |
| **low light** | 90.787 px | **202.593 px** | 365.79 | **57.96 %** |

**Four of row 24's five modes hold.** Low light does not, and it is not close:
the beacon does not clear the detector's noise floor, so what is being measured
is a false lock rather than a degraded one.

> This table replaces a claim that read *"through every weather mode …
> centroiding stays at 0.14–3.4 px and target loss at 1.7 %"*. That sentence
> was wrong in three ways at once: "every mode" excluded the one that fails,
> the range quoted means where the project's own tool grades p95, and the
> figures came from 4-second cells. Anyone could disprove it in five minutes
> by running `just sweep` — which, in a project whose whole claim is that
> every number is measured, costs more than the number was worth.
>
> The sweep now runs 30 s per cell for the reason
> [`scenarios/sweeps/weather.toml`](scenarios/sweeps/weather.toml) gives at
> length: the project's own issues log records four defects that 6-second runs
> hid and 30-second runs exposed.

Two specification rows are internally inconsistent and are reported as derived
bounds rather than pass or fail — cold acquisition cannot meet 2 s by geometry,
and tracking error cannot go below 16.33 px while row 23's jitter is applied.
Both derivations are in [`docs/RESULTS.md`](docs/RESULTS.md).

### What does not pass

**Appearance discrimination.** A second beacon-shaped object in the frame — a
moving decoy, or design §9.1's 120 clutter sources — costs two orders of
magnitude of centroiding accuracy. The tracker locks onto a clutter source and
holds it. [`scenarios/hard/clutter_field.toml`](scenarios/hard/clutter_field.toml)
is that failure isolated, with the numbers in its own header, and it is what
Stage 11's `CandidateNet` exists for.

**Low light.** A third condition, and a different problem: the beacon is below
the detector's floor rather than mis-associated, so it is a sensitivity failure
and no amount of association logic fixes it.

Both are recorded with their measurements in
[`issues_till_now.md`](issues_till_now.md).

### Benchmark Performance-2 — the video path, 30 % of the marks

`just bp2` runs three committed clips end to end against committed truth, with
the full closed loop. Truth is generated from the same analytic expression that
draws each beacon, so it is exact by construction.

| Fixture | what it isolates | centroiding | tracking | frame p99 |
|---|---|---:|---:|---:|
| clean direct | the control — a noiseless box has an exact centroid | **0.0000 px** | n/a | 14.8 ms |
| noisy direct | H.264 + row-22 noise, no crop | **0.0014 px** | n/a | 16.8 ms |
| screen 2000×2000 | the full rehearsal: acquire, crop, track, hand over | **0.1037 px** | **0.888 px** | **26.1 ms** |

On the rehearsal: acquisition 0.067 s (row 16), post-acquisition target loss
0.00 % (row 18), FOV containment 100 %, handover reached at 2.000 s, and
**2.70× real time** — the system runs comfortably ahead of a 30 fps source.

### Speed

The synthetic frame is **2.54 ms** — 394 FPS, against a requirement of 20 and
a design target of 350–500. That is 17× faster than it was before Stage 14's
work:

| | before | after |
|---|---:|---:|
| frame time p50 | 42,551 µs | **2,537 µs** |
| the detector (B6–B13) | 17,309 µs | **1,119 µs** |
| the sensor model | 12,079 µs | **1,269 µs** |
| the provenance snapshot | 813 µs | **96 µs** |
| the full test suite | 359 s | **38 s** |

Measured on an Intel Core Ultra 7 256V (8 cores, AVX2, no AVX-512), Linux,
Release. Absolute figures will differ on other hardware; the ratios hold.

It does **not** reach design §15's 0.85 ms. `docs/SAT-DESIGN.md` §14.0d derives
that budget as 9.7 CPU cycles per pixel for the entire frame, against 16 for
the one Gaussian noise sample per pixel that specification rows 21–22 require.

> An earlier version of this section added *"all of the excess is the
> SIMULATOR, which does not exist in the video path that 30 % of the marks are
> scored on."* That was false when written: the video path was then **12×
> slower** than the synthetic one. It has since been fixed — the cost was a
> per-pixel bicubic crop, not the decode everyone assumed — and the video path
> is now 10.8 ms p50 against 2.54 ms synthetic. The sentence is still not one
> to make: the video path carries a real 2000×2000 decode and crop that the
> synthetic path does not, and that work is not free.

---

## Documentation

| File | Role |
|---|---|
| [`docs/GUIDE.md`](docs/GUIDE.md) | **start here** — a guided tour with screenshots |
| [`QUICKSTART.md`](QUICKSTART.md) | five minutes from a clean checkout to a tracked beacon |
| [`issues_till_now.md`](issues_till_now.md) | everything not yet up to the mark, each with a measurement |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | what is built, how it fits, why each decision went that way |
| [`docs/RESULTS.md`](docs/RESULTS.md) | every measured number, with the command that reproduces it |
| [`docs/MANUAL.md`](docs/MANUAL.md) | building, running, the CLI, the scenario format, the artifacts |
| [`docs/METRICS.md`](docs/METRICS.md) | §13.1's metric definitions verbatim, then how each is computed |
| [`docs/DEMO.md`](docs/DEMO.md) | the ten-minute demo, minute by minute, every step a command that exists |
| [`docs/SAT-DESIGN.md`](docs/SAT-DESIGN.md) | the master specification and checkpoint roadmap |
| [`docs/SAT-ML.md`](docs/SAT-ML.md) | the ML companion spec (not implemented) |
| [`AGENTS.md`](AGENTS.md) | rules for contributors |

---

## The nine invariants

Everything the project claims rests on these, and each is tied to a mechanism
that fails loudly rather than to a convention that can be forgotten.

1. **The tracker cannot see ground truth** — enforced by the linker, by a
   configure-time closure walk, and by a source scan. Each is self-tested by
   injecting a violation.
2. **The loop is closed** — the controller's output determines the boresight
   the next frame is produced at.
3. **Bit-exact reproducibility** — no wall clock, no `rand()`, named RNG
   streams. Every scenario runs twice in CI and the fingerprints must match.
4. **Zero heap allocation in steady state.**
5. **Work in angles, not pixels.**
6. **Centroiding error and tracking error are distinct.**
7. **The system runs fully without AI.**
8. **No damage is added in video modes.**
9. **Never emit a stale or interpolated centroid.**

[`ARCHITECTURE.md` §3](docs/ARCHITECTURE.md) explains how each is enforced.

---

## Licence

See [`LICENSE`](LICENSE).
