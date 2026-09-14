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
just test          # 16 suites, 1.84M assertions
```

Three things worth running first:

```bash
just headless                # one run: metrics, centroid.csv, run.json, report.html
just video tests/video/clips/screen_2000x2000_30fps.mp4    # MP4 ingest
just sweep                   # 200 runs -> the requirement compliance matrix
```

---

## Where it stands

Stages 0–9 of the roadmap are complete, plus the performance work at CP 14.2.
All five ★ gates pass. Machine learning (Stage 11) is deliberately out of
scope — INV-7 requires the system to run fully without it, and it does.

| Graded requirement | Spec | Measured |
|---|---|---|
| Acquisition, beacon in view (row 16) | ≤ 2 s | **0.067 s** |
| Target loss (row 18) | < 5 % | **1.67 %** |
| Re-acquisition (row 19) | ≤ 1 s | **0.094 s** |
| Processing speed (row 20) | ≥ 20 FPS | **21.5 / 30.8 FPS** |
| Centroiding accuracy (60 % of the marks) | — | **0.217 px RMSE** |

Two specification rows are internally inconsistent and are reported as derived
bounds rather than pass or fail — cold acquisition cannot meet 2 s by geometry,
and tracking error cannot go below 16.33 px while row 23's jitter is applied.
Both derivations are in [`docs/RESULTS.md`](docs/RESULTS.md).

Known gaps are recorded with measurements, not descriptions:
[`RESULTS.md` §8](docs/RESULTS.md).

---

## Documentation

| File | Role |
|---|---|
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | what is built, how it fits, why each decision went that way |
| [`docs/RESULTS.md`](docs/RESULTS.md) | every measured number, with the command that reproduces it |
| [`docs/MANUAL.md`](docs/MANUAL.md) | building, running, the CLI, the scenario format, the artifacts |
| [`docs/METRICS.md`](docs/METRICS.md) | §13.1's metric definitions verbatim, then how each is computed |
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
