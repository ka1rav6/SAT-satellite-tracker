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

On clear air with nothing else in the frame — 200 runs, `just sweep`:

| Graded requirement | Spec | Measured |
|---|---|---|
| Acquisition, beacon in view (row 16) | ≤ 2 s | **0.067 s** |
| Target loss (row 18) | < 5 % | **1.67 %** |
| Re-acquisition (row 19) | ≤ 1 s | **0.109 s** (over all 200 runs) |
| Processing speed (row 20) | ≥ 20 FPS | **251 FPS** |
| Centroiding accuracy (60 % of the marks) | — | **0.143 px RMSE** |

Through every weather mode the specification lists, with row 21's 10 % impulse
noise and row 22's read noise at the cap, centroiding stays at **0.14–3.4 px**
and target loss at **1.7 %**.

Two specification rows are internally inconsistent and are reported as derived
bounds rather than pass or fail — cold acquisition cannot meet 2 s by geometry,
and tracking error cannot go below 16.33 px while row 23's jitter is applied.
Both derivations are in [`docs/RESULTS.md`](docs/RESULTS.md).

**Two conditions do not pass, and they are the same condition twice:** a second
beacon-shaped object in the frame. A moving decoy, or 120 clutter sources, cost
two orders of magnitude of centroiding accuracy; so does light too low for the
beacon to clear the detector's floor. Both are recorded with the measurement
against them in [`issues_till_now.md`](issues_till_now.md), and both are what
Stage 11's `CandidateNet` exists for.

### Speed

The frame is **2.62 ms** — 382 FPS, against a requirement of 20 and a design
target of 350–500. That is 16× faster than it was before Stage 14's work:

| | before | after |
|---|---:|---:|
| frame time p50 | 42,551 µs | **2,618 µs** |
| the detector (B6–B13) | 17,309 µs | **1,275 µs** |
| the sensor model | 12,079 µs | **1,395 µs** |
| the full test suite | 359 s | **78 s** |

It does **not** reach design §15's 0.85 ms, and that budget is not reachable:
`docs/SAT-DESIGN.md` §14.0d derives it as 9.7 CPU cycles per pixel for the
entire frame, against 16 for the one Gaussian noise sample per pixel that
specification rows 21–22 require. All of the excess is the SIMULATOR, which does
not exist in the video path that 30 % of the marks are scored on.

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
