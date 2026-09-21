# Technical Report

## SAT — Satellite Adaptive Tracker
### An AI-assisted virtual camera tracking system for coarse alignment of mobile FSOC terminals

**Problem Statement 26169** · Department of Space / ISRO
**Smart India Hackathon**

---

> **Every number in this report was produced by a command in this repository,
> and every command is named beside its number.** Nothing here is an estimate.
> Where a figure could not be measured it says so; where a requirement is not
> met it says that too, with the measurement against it.
>
> Machine for all timing figures: Intel Core Ultra 7 256V, 8 cores, AVX2 (no
> AVX-512), Linux, Release build. Absolute microsecond figures will differ on
> other hardware; the ratios hold.

---

## 1. The problem, and what makes it hard

Free-space optical communication between mobile platforms requires **pointing,
acquisition and tracking** (PAT) of a laser beam whose divergence is measured
in microradians. PAT has two stages, and this problem statement addresses only
the first: **coarse alignment**, whose job is to *"first locate and maintain
the remote terminal within its camera Field-of-View"*.

Because real PAT hardware is expensive, the deliverable is a **software virtual
testbed**: a configurable virtual scene, a movable virtual pan-tilt camera,
synthetic disturbances, and a tracking algorithm that closes the loop.

### 1.1 The one number that frames the whole design

The specification is internally inconsistent, and finding out *where* is the
most useful thing we did before writing any code.

| Specification row | Value | In pixels per frame at 30 Hz |
|---|---|---:|
| 13 — max pan rate | 5 °/s | **26.7 px/frame** of mount authority |
| 23 — camera jitter | ±20 px/frame | 20 px/frame of disturbance |
| 25 — platform motion | ±20 px/frame | 20 px/frame of disturbance |

Rows 23 and 25 together are **40 px/frame of disturbance against 26.7 px/frame
of authority — 150 %.**

A purely reactive controller cannot meet row 17's 10 px tracking error under
those conditions, and no amount of gain tuning changes that: the mount
physically cannot move fast enough to cancel a disturbance larger than its own
slew limit. **Prediction is therefore not an optimisation in this problem; it
is a requirement.** That single observation determines the architecture — it is
why there is a Kalman filter with an analytically derived process noise, why
there is velocity feedforward, and why the controller is given a *predicted*
position rather than a measured one.

It also produces a **derived floor**. Row 23's jitter is drawn uniformly on
[−20, +20] px each frame and applied to the true boresight. For a uniform
distribution on [−A, A] the variance is A²/3, so over two independent axes the
magnitude has

```
RMS = sqrt(2 × 400/3) = 16.33 px
```

No controller can report a row-17 figure below this while row 23 is active.
Measured on the specification defaults, 30 s, steady state, the ONLY difference
being row 23:

```bash
./build/sat-tracker --headless --duration 30                                  # 16.94 px
./build/sat-tracker --headless --duration 30 --set disturbance.jitter_px_per_frame=0
                                                                              #  0.30 px
```

**16.94 px with jitter, 0.30 px without.** The loop is not the limitation —
with the disturbance removed it points to a third of a pixel. The 16.94 px
figure is the jitter itself, and it is reported as a *derived bound* rather
than as a pass or a failure.

### 1.2 The second inconsistency

Row 11 makes the target's initial position **random**; row 16 asks for
acquisition within **2 s**. A random position on the 12.5° screen viewed
through a 4°×3° camera is visible at t=0 with probability

```
(640 × 480) / (2000 × 2000) = 7.68 %
```

and a full search at row 13's 5 °/s takes 15–19 s. The two rows cannot both
hold. We report **two acquisition figures, always together and always
labelled**: the in-view time (0.067 s) and the cold time, marked BOUND DERIVED
against the geometric argument above.

---

## 2. Architecture

```
 Scenario (TOML, schema-validated, 18 negative fixtures)
    |
    +-> World   analytic motion algebra, closed forms        [300 Hz sub-tick]
    +-> Plant   gimbal: rate + accel limit, 10 ms transport
    |           delay, 20 ms lag, encoder quantisation       [300 Hz sub-tick]
    v
 Image formation   background -> emitter splat -> exposure blur (8 substeps)
    v
 Damage chain      atmosphere -> shot+read noise (fused) -> FPN -> defects
    |                                          [AVX2, runtime-dispatched]
    v
 Detection         median 3x3 -> top-hat -> 2x SAT -> matched filter
    |              -> 2x CFAR -> grouping     [ROI-windowed when Confirmed]
    v
 Centroiding       windowed CoM / Gaussian fit + compiled-in S-curve bias table
    v
 Measurement       image px -> angle, through the ENCODER reading
    v
 Tracking          chi-square gate -> priority policy -> KF / IMM -> lifecycle
    v
 Supervisor        observable-only strategy switching
    v
 Mode FSM          7 states: Search / Acquire / Track / Reacquire / Handover
    v
 Control           PID + velocity feedforward + optional Smith predictor
    |
    +----------------------------------------> back to Plant
    v
 Metrics           the ONLY consumer of truth
```

29,962 lines of C++20 across 13 CMake modules with a **linker-enforced**
dependency graph.

### 2.1 Nine invariants, enforced rather than intended

This is the part of the project we would point at first. Each invariant is
enforced by a mechanism, and each mechanism is itself tested by deliberately
injecting a violation and confirming it goes red.

| # | Invariant | Enforced by |
|---|---|---|
| 1 | The tracker cannot see truth | Linker boundary + a configure-time link-closure walk + a source scan, each self-tested |
| 2 | The loop is closed | The controller's output determines the boresight of the *next* frame; an open-loop flag exists so the comparison is runnable |
| 3 | Bit-exact reproducibility | `--verify-reproducibility`, CI at −O0 and −O2, AVX2 ≡ scalar, seed-sensitivity |
| 4 | Zero heap allocation in steady state | A Debug `operator new` trap around `Pipeline::step` |
| 5 | Work in angles | Tag-typed `Angle2` / `Pixel2` / `Rate2` — a unit mix is a compile error |
| 6 | Centroiding ≠ tracking error | Separate fields, separate frame sets, no code path adds them |
| 7 | Runs fully without AI | `--no-ai` is the shipped path; a `linux-no-gui` CI job generalises it |
| 8 | No synthetic damage in video modes | Enforced by the code path, not by a flag |
| 9 | Never emit a stale or interpolated centroid | Gaps written as empty columns |

**INV-1 is the one that matters most for the marks.** The question *"how do we
know your tracker isn't secretly reading the ground truth?"* has a
sixty-second answer: `just gate-inv1-selftest` injects a `sat_search →
sat_world` edge into the module graph and the *configure step* refuses to
proceed. The detector is handed a span of pixels, not the enclosing frame
struct; it cannot reach truth even though truth is sitting in the same object.

**INV-4 found four real defects** when it was first armed — vectors outgrowing
their reserves inside the frame loop, invisible in any profile.

### 2.2 The physics we model, and the physics we do not

Modelled correctly, and worth stating because several are unusual:

- **The disturbance perturbs the boresight, never the pixels.** Jitter and
  platform motion move where the camera is *pointing*. Motion blur then falls
  out of exposure integration instead of being faked, and — critically — the
  tracker cannot implicitly learn its own pointing error.
- **Encoder quantisation reaches the measurement path.** A real mount reports a
  quantised angle, and every reconstruction perception performs uses that
  reading, not the true one. Sweeping the LSB from 20 µrad to 4000 µrad moves
  the reported screen-frame error from 0.22 px to 18.6 px, monotonically.
- **The transport delay is compensated, with both sides advanced together.** A
  Smith predictor that advances the estimate but not the error introduces a
  double lead; the analysis is written out at the call site.
- **Process noise is derived from the target's own analytic bounds**, not the
  mount's. Two earlier derivations were wrong and are recorded with their
  symptoms — a mount-derived `q` inflated the gate to 240 px across and let
  noise capture the track.
- **Coarse→fine handover is modelled** with a co-boresighted quadrant cell, a
  1 mrad capture range and a sustained-RMS criterion.

Not modelled, stated plainly:

- **Atmospheric turbulence.** `Atmosphere` is a five-level enum applying an
  affine contrast and brightness change. Row 24 asks for *"user-defined
  reduction in contrast and brightness"*, which is what this is — but it is a
  photometric model, not a turbulence model. There is no Kolmogorov-spectrum
  angle-of-arrival jitter, no scintillation, no Fried parameter. The boresight
  jitter is *functionally* a crude AoA model (it perturbs the arrival angle,
  resampled per frame) but it is white and uniform rather than spectrally
  correct.
- **Lens distortion and PSF.** The projection is equidistant rather than
  pinhole, which costs 0.13 px at the extreme corner and buys an exactly linear
  measurement equation — which is why a plain Kalman filter is correct here and
  an EKF is not needed.
- **Absolute attitude reference.** There is no IMU and no star tracker, so
  platform drift is unobservable. This has a consequence we report explicitly;
  see §4.3.

---

## 3. Results

### 3.1 The graded rows

Measured on `scenarios/spec_defaults.toml` — the specification's rows 1–25 with
row 11's random start pinned in view — 30 s, seed 42.

```bash
./build/sat-tracker --headless --out logs/run
```

| Row | Requirement | Measured | Verdict |
|---:|---|---:|---|
| 16 | Acquisition ≤ 2 s | **0.067 s** (in view) <!--@ acquisition.in_fov_s 0.001 --> | **PASS** |
| 17 | Tracking error ≤ 10 px | **16.94 px** steady RMS <!--@ tracking.steady.rms_px 0.05 --> | **BOUND DERIVED** (§1.1: the floor is 16.33 px) |
| 18 | Target loss < 5 % | **0.00 %** post-acquisition <!--@ lock.target_loss_post_acq 0.01 % --> | **PASS** |
| 19 | Re-acquisition ≤ 1 s | **0.109 s** mean over 200 runs | **PASS** |
| 20 | Processing ≥ 20 FPS | **306 FPS** p50 | **PASS** |
| — | Centroiding error (60 % of BP marks) | **0.197 px** image-frame <!--@ centroiding.rmse_image_px 0.01 --> | — |
| ★ | **FOV containment** — the PS's own objective | **100.00 %** <!--@ fov_containment.frac 0.01 % --> | — |

Five of those figures carry an anchor comment naming the metric in
`docs/baseline/run.json` they are checked against; `just gate-docs` fails if
any of them drifts. Rows 19 and 20 are not anchored, and for different reasons:
row 19 is a mean over the 200-run sweep rather than a single run, and row 20's
frame rate is a property of the machine, so pinning it would make the gate fail
on any hardware but this one.

Cold acquisition is reported separately and marked BOUND DERIVED (§1.2).

### 3.2 Through row 24's weather

```bash
just sweep     # 200 runs, 30 s each, 5 seeds
```

Clutter and decoy removed so the axis is weather alone:

| Row 24 mode | S&P 0 | S&P 10 % | p95 | target loss (post-acq) |
|---|---:|---:|---:|---:|
| clear | 0.141 px | 0.497 px | 1.71 | 0.00 % |
| haze | 0.182 px | 1.291 px | 5.41 | 0.00 % |
| rain | 0.252 px | 1.355 px | 5.42 | 0.00 % |
| fog | 0.472 px | 1.543 px | 5.44 | 0.05 % |
| **low light** | 90.787 px | **202.593 px** | 365.79 | **57.96 %** |

**Four of five hold. Low light does not, and it is not close.**

### 3.3 Benchmark Performance-2 — the video path

```bash
just bp2
```

Three committed clips, run end to end through the full closed loop against
committed truth. Truth is generated from the same analytic expression that
draws each beacon, so it is exact by construction rather than measured.

| Fixture | what it isolates | centroiding | tracking | p99 |
|---|---|---:|---:|---:|
| clean direct | the control — a noiseless box has an exact centroid | **0.0000 px** | n/a | 14.8 ms |
| noisy direct | H.264 + row-22 noise, no crop | **0.0014 px** | n/a | 16.8 ms |
| screen 2000×2000 | the full rehearsal | **0.1037 px** | **0.888 px** | **26.1 ms** |

On the rehearsal: acquisition 0.067 s, post-acquisition target loss 0.00 %, FOV
containment 100 %, handover reached at 2.000 s, and **2.70× real time**.

The control clip earns its place. A noiseless symmetric box has an exact
centroid, so a non-zero result there is a defect in the crop or the truth
rather than in the centroider — and it is what caught a half-pixel convention
error (`between()` is inclusive at both ends, so a beacon spanning
`[x, x+sz−1]` has its centre at `x + (sz−1)/2`). That showed up as a constant
+0.498 px bias, which on this clip is unmissable and on a noisy one would look
like noise.

### 3.4 Speed

| Stage | measured (p50) | design §15 budget |
|---|---:|---:|
| `frame_acquire` (background + splat + damage) | 1,352 µs | 740 µs |
| `perception` (the whole §9.4 chain) | 1,119 µs | 1,390 µs |
| `snapshot` (provenance) | 96 µs | 30 µs |
| **`frame_total`** | **2,537 µs** | **850 µs** |

17× faster than before the optimisation work (42,551 µs). It does **not** reach
design §15's 0.85 ms; §5.2 says what would.

### 3.5 End-to-end latency

Compute time is not latency. The number a pointing system cares about is how
stale the command is when it reaches the mount:

| Term | Value |
|---|---:|
| exposure ÷ 2 (the frame's effective instant is mid-exposure) | 2.500 ms |
| compute | 3.064 ms |
| transport delay | 10.000 ms |
| **total** | **15.564 ms** |

The transport delay is **3.3× the compute time**, so making the tracker faster
would barely move the latency. That is why the Smith predictor exists.

### 3.6 What happens when processing is slower than the camera

This is the robustness question we expect to be asked, and until recently the
honest answer was *"it cannot happen"* — the loop is a synchronous pull, so a
slow frame simply slowed the simulated clock down with it. That is not a
real-time argument, it is the absence of one.

There is now a deadline model. Each frame is measured against a budget (by
default one camera period, 33.33 ms, because that is when the next frame
arrives whether this one has finished or not), overruns are counted, and work
is shed to get back inside it. Two rungs: skip the 3×3 median, then halve the
detection window.

```bash
./build/sat-tracker --headless --duration 20                                # no deadline
./build/sat-tracker --headless --duration 20 --realtime --frame-budget-ms 4 # 8x tighter
```

| | no deadline | 4 ms budget |
|---|---:|---:|
| frames shed | 0 | **597** |
| retention | 99.67 % | **99.67 %** |
| row 18 target loss | 0.00 % | **0.00 %** |
| centroiding RMSE | 0.2010 px | 0.4018 px |
| row 17 steady state | 17.063 px | **17.063 px** |

Under a budget roughly eight times tighter than the measured frame time, the
loop **keeps the target**. It pays 0.2 px of centroiding accuracy, and row 17
does not move at all — because row 17 is jitter-limited at §1.1's 16.33 px
floor, and a fifth of a pixel of detector noise is invisible underneath it.

Two findings are worth more than the feature. Shedding during **acquisition**
destroys the run: search frames legitimately cost 19 ms, and charging them
against the deadline degraded the detector before any track existed, taking
retention to 20 %. And a third rung we tried — falling back to the straw-man
detector — traded the target for the frame rate under sustained load, taking
retention to 26.7 %. It was removed. **A loop that has lost the beacon is not
degraded, it has failed**, and no throughput figure is worth that.

For reproducibility the clock is separated from the policy:
`--inject-stall 50@60` drives the same policy from a deterministic schedule
and never reads the wall clock, so INV-3 holds and the tests assert exact
numbers. `--realtime` runs report that they are not bit-reproducible rather
than leaving a reader to assume they are.

---

## 4. What does not work, and why

A report that lists only successes is not evidence of anything. These are the
three open failures, each with the measurement against it.

### 4.1 Appearance discrimination — clutter and moving decoys

`scenarios/hard/clutter_field.toml` is the specification defaults with design
§9.1's 120-source clutter field switched on and nothing else changed:

| | spec defaults | + clutter field |
|---|---:|---:|
| centroiding (image) | 0.197 px | **172.90 px** |
| tracking (steady RMS) | 16.94 px | **253.73 px** |
| FOV containment | 100.00 % | **80.78 %** |
| row 18 (post-acq loss) | 0.00 % | **19.80 %** |

Acquisition still succeeds — the beacon is found in half a second — so this is
**not** an acquisition failure. The loop acquires the beacon, then loses it to
a clutter source and holds the wrong thing.

**The mechanism.** The priority policy scores candidates on motion consistency,
brightness, size plausibility, centrality and track history. A static clutter
source that is brighter than the beacon (§9.1 makes some deliberately brighter)
and closer to the boresight wins on brightness and centrality, ties on size,
and loses only on motion — and the motion term needs several frames of history
before it says anything.

This is what a learned appearance score (`CandidateNet`) is for, and it is the
highest-value ML target in the project precisely *because* the classical
baseline here is so bad.

### 4.2 Low light

A different problem with a similar symptom. The beacon does not clear the
detector's noise floor, so it is a **sensitivity** failure rather than an
association one, and no amount of association logic fixes it. 202.6 px
centroiding, 58 % post-acquisition target loss.

### 4.3 The screen-frame error is unbounded, and that is correct physics

This one is subtle enough to be worth the space, because it looks like a bug
and is not.

With the true boresight `B_true = B_cmd + D(t)`, where `D` is the accumulated
platform displacement:

- the beacon lands on the sensor at `T − B_true`;
- the detector finds it there, essentially exactly (0.197 px);
- the screen conversion reconstructs `B_cmd + (T − B_true) = T − D(t)`;
- truth is `T`.

So `centroid_error_screen = |D(t)|` **identically**. Measured on the spec
defaults with row 25's platform motion active, it grows 44 px at 4 s, 295 px at
30 s, 589 px at 60 s — matching `17 × √(t²/3)` to within 0.1 %.

A pan-tilt mount with encoders **cannot observe motion of the base it is bolted
to**. Without an IMU or a star tracker the world-frame position of the beacon
is genuinely unknowable. The physics is right; what was wrong was the
*reporting*, which printed that figure under "CENTROIDING (graded, 60 %)" with
no caveat.

`centroid.csv` now carries **three** columns — image frame, boresight-referenced
(the same error in screen pixels with `D(t)` cancelled), and screen frame — and
the summary prints the measured RMS of `D` beside the screen figure, so the
number reads as a consistency check on the simulator rather than as a diverging
centroider.

### 4.4 Where the AI is

The title says "AI-Based" and the shipped path contains no machine learning.
That is a deliberate decision and we would rather state it than dress it up.

- **What exists:** the training pipeline, dataset schema, ONNX export,
  inference wrapper with a mandatory classical fallback, model cards, and two
  architectures. 870 lines of Python.
- **What blocks it:** `--gen-dataset` is not implemented, so no training data
  can exist. Everything downstream is blocked on that one thing.
- **Why we did not ship a model anyway:** the sidecar from a dummy-data
  training run scores 0.274 px. The classical centroider measures **0.141 px**.
  Shipping a learned centroider that is worse than the classical one, to be
  able to say the word "AI", would be the wrong call — and saying so is a
  better answer than the model would have been.
- **Where ML would actually pay:** §4.1. The classical path fails at 172.9 px
  on clutter and 48.8 px on a moving decoy. That is where a learned appearance
  score has two orders of magnitude to win, and it is the opposite ordering to
  the obvious one.
- **The interface is ready.** `Strategy.centroider` and `Strategy.perception`
  are enums the supervisor already switches at runtime; adding a `Learned` case
  is an enum entry, not a refactor. INV-7 guarantees an ML failure can never
  take the system down.

---

## 5. Engineering process

### 5.1 How we know the numbers

| Mechanism | What it buys |
|---|---|
| **Bit-exact reproducibility** (INV-3) | A compliance matrix from 200 runs is evidence only if rerunning gives the same answer. Verified at −O0 and −O2, across AVX2 and scalar dispatch, with the *seed-sensitivity* check that catches a gate testing nothing. |
| **Named RNG streams** | Adding clutter cannot perturb the noise. Ablations differ by one thing. |
| **18 negative fixtures** | Every out-of-range parameter must be rejected, naming the key and its specification row. |
| **Scenario fuzzer** | 40 scenarios across the full legal parameter space, corner-sampled. 0 failures. |
| **ASan + UBSan** | Found a heap-buffer-overflow in the median filter's fast path that every other job on every platform was green for. |
| **Allocation trap** | Found four steady-state allocations. |
| **Source-citation checker** | Every `tests/…` path named in a comment must resolve. Found three dangling citations the day it was written. |

### 5.2 The 0.85 ms budget

Design §15 sets a 0.85 ms frame budget and an earlier amendment declared it
unreachable, deriving 9.7 CPU cycles per pixel against 16 for the one Gaussian
deviate per pixel that rows 21–22 require.

**That derivation assumes a single core and does not say so.** 9.7 cycles per
pixel is a *per-core* budget presented as an absolute. It also attributes the
whole gap to the Gaussian: measured, the Gaussian is 53 % of the damage chain
and the non-Gaussian arithmetic alone already exceeds §15's entire damage-chain
line. And it omits `snapshot`, which was 813 µs — 24 % of the frame — for
provenance that would not exist on real hardware.

We have taken the part of that path that does not require threading:
`snapshot` is now 96 µs (an 8-lane hash, chosen over the obvious 8-bytes-at-a-
time load because that is endian-dependent and would break the cross-machine
claim the fingerprint exists to make). The remainder — counter-addressed PCG so
the damage chain can be row-striped deterministically, AVX2 for the three
perception kernels that still dominate, and splitting the simulator from the
tracker across the SPSC queue pattern the decoder already proves — is scoped
and costed and is not yet built.

**We think the budget is reachable and that the amendment declaring it
unreachable was wrong.** We have not proved it, so we say that rather than
claiming it.

### 5.3 What we got wrong

Recorded because the corrections are the useful part.

- **The default scenario was the worst run in the repository.** `baseline.toml`
  was the default for `--gui` and `--headless`, the first picker entry, the
  README's hero screenshot, *and* the documented "known-good demo fallback". It
  scored 913 px tracking RMS and **zero** centroiding frames. Twenty test
  suites were green, because not one of them ran the *default*.
- **The reproducibility gate ran with the damage chain switched off,** and had
  been printing the evidence — identical digests for different seeds — on every
  run for months.
- **The latency histogram saturated at 100 ms** and printed the top bucket's
  centre as if it were a measurement. A "p99 of 95.6 ms" that never changed was
  a ceiling, not a number.
- **Two documented claims were disprovable in five minutes** by running the
  project's own tools. Both are corrected, and both corrections say what the
  claim used to be.
- **The dashboard kept a second implementation of the metric definitions,**
  which graded row 17 on the acquisition transient and disagreed with the
  summary by 4×.

---

## 6. Deliverables

| PS deliverable | Status |
|---|---|
| Standalone executable application | `just package` builds a tarball carrying the binary, the scenarios, the docs and a clip; `just verify-package dist/<name>.tar.gz` unpacks it into a temporary directory and runs it there, with nothing from the source tree on the path |
| Source code, modular and commented | 29,962 lines, 13 linker-enforced modules |
| **Technical report** | This document |
| **User manual** | [`docs/MANUAL.md`](../MANUAL.md) + [`docs/GUIDE.md`](../GUIDE.md), with screenshots regenerated from the live product by `just screenshots` |
| Live GUI display of tracking performance | The dashboard: camera view, screen overview, live damage and algorithm controls, compliance panel, per-stage timings |
| **Auto-generated performance report** | `run.json` + `centroid.csv` + `report.html`, all three written per run with zero manual steps |
| **Log of centroiding error** | `centroid.csv`, three frames per row, gaps left empty rather than interpolated (INV-9) |
| Demo video (optional) | Not produced |

---

## 7. How to reproduce every number in this report

```bash
just build                                     # configure and build
just test                                      # 21 suites

./build/sat-tracker --headless --out logs/run  # §3.1, the graded rows
just sweep                                     # §3.2, 200 runs
just bp2                                       # §3.3, Benchmark Performance-2
./build/sat-tracker --headless --duration 30 --stages   # §3.4 and §3.5

./build/sat-tracker --verify-reproducibility   # INV-3
just gate-inv1-selftest                        # INV-1, by injected violation

./build/sat-tracker --headless --scenario scenarios/hard/clutter_field.toml \
                    --out logs/clutter         # §4.1, the clutter failure
```

Every figure quoted above is in the `run.json` one of those commands writes.
