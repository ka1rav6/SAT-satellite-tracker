# SIH PS 26169 — Critical Engineering Audit

**Subject:** SAT — Satellite Adaptive Tracker
**Problem Statement:** 26169, *Development of an AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals* (Dept. of Space / ISRO)
**Audit date:** 2026-09-21
**Commit audited:** `803733d`
**Audit machine:** Intel Core Ultra 7 256V, 8 cores, AVX2 (no AVX-512), Linux
**Scope note:** the ML component (Stage 11) is unimplemented by explicit decision and is **not** evaluated as if it existed. It is assessed only as an integration surface.

> The question this document answers throughout:
> **If I were an SIH judge trying to break this project during evaluation, how would I do it, and what evidence would the team need to prove the system actually solves PS 26169?**

This is an adversarial engineering audit, not a promotional document. Every claim below is either backed by a command that was run on this commit, or explicitly marked `UNVERIFIED`.

---

## 1. Executive Summary

### 1.1 What is genuinely strong

These are not concessions; they are measured facts and they are the project's real assets.

| Capability | Evidence (re-measured on this commit) |
|---|---|
| Sub-pixel centroiding | **0.19–0.21 px image-frame RMSE** on `compliance.toml`, stable across 4 s / 30 s / 60 s |
| Centroiding on a real, noisy MP4 | **0.0168 px RMSE** (640×480 direct), **0.0950 px RMSE** (2000×2000 screen mode) — measured by this audit with purpose-built clips |
| Closed-loop tracking on the video benchmark | **1.143 px RMS**, retention 98.89 %, handover reached at 2.0 s |
| Determinism | `--verify-reproducibility` → 10/10 bit-identical; seed changes the fingerprint; CI compares −O0 vs −O2 digests |
| Engineering discipline | 20 CTest suites green in 47 s; ASan+UBSan job; 5-config CI matrix incl. native MSVC; linker-enforced INV-1; armed allocation trap; scenario fuzzer (40 scenarios, 0 failures) |
| GUI | A genuinely competitive dashboard: live damage sliders, live algorithm toggles, compliance panel, per-stage timing, screen overview, detector ablation |
| Honesty culture | `issues_till_now.md` and the design amendments record retracted claims and failed hypotheses. This is rare and valuable. |

### 1.2 The five things that will cost marks

1. **`baseline.toml` — the default scenario for `--gui` and `--headless`, and the documented "known-good demo fallback" — is a total system failure.** 913 px tracking RMS, **zero** centroiding frames scored, 1778 false tracks/min, beacon never acquired. A judge who types `just gui` sees this. **P0.**
2. **The project's primary objective metric is not reported.** PS 26169 asks the system to "locate and **maintain the remote terminal within its camera FOV**". The fraction of run time the beacon was inside the FOV is never reported as a metric. Because `target_loss` is normalised over *in-FOV frames only*, a run where the beacon was visible for 4.4 % of the time reports **7.59 % target loss**. The denominator hides catastrophic failure. **P0.**
3. **The headline speed claim is false on the benchmark it is claimed for.** README and design §14.0d assert the excess is "the SIMULATOR, which does not exist in the video path that 30 % of the marks are scored on." Measured: the video path is **32.5 ms/frame (30.8 FPS)**, p95 87 ms, **p99 95.6 ms — below spec row 20's 20 FPS**, and runs at **1.04× real time**. The video path is 12× *slower* than the synthetic path, not faster. **P0.**
4. **The INV-3 reproducibility gate runs with the entire damage chain disabled.** `verify_repro.cpp` never enables the sensor model (`SensorModel::enabled_` defaults to `false`), so the AVX2 noise path — the single most fragile code for cross-machine bit-exactness — is never exercised by the gate that exists to protect it. **P1.**
5. **Two documented numeric claims are contradicted by the project's own tooling.** README: "Through every weather mode … centroiding stays at 0.14–3.4 px and target loss at 1.7 %" — `just sweep` reports lowlight at **237.3 px / 37.8 %**. And `issues_till_now.md` §2.4's "the policy correctly refuses to commit to any of them" is disproven by 1777 Confirmed frames on clutter. **P1.**

### 1.3 The 0.85 ms question

The user asked specifically for a solution to design §15's 0.85 ms frame budget, which §14.0d declares unreachable.

**§14.0d's proof is wrong.** It assumes single-core execution and never says so. Its own arithmetic — "9.7 cycles per pixel" — is a *per-core* budget silently treated as an absolute. The codebase already contains the hard prerequisite for the fix (PCG jump-ahead constants, `src/degrade/sensor_simd.cpp:32–50`). §13 of this document gives a costed path to **~790 µs single-frame** and **~475 µs observed tracker frame**, with a determinism proof that is stronger than the one in place today.

---

## 2. Problem Statement Interpretation

### 2.1 The problem, stated in the PS's own terms

Deploying FSOC between mobile platforms requires **pointing, acquisition and tracking (PAT)** of a narrow laser beam. PAT has two stages; this PS addresses only the first, **coarse alignment**, whose job is to *"first locate and maintain the remote terminal within its camera Field-of-View (FOV)."* Because real PAT hardware is expensive, the deliverable is a **software virtual testbed**: a configurable virtual scene, a movable virtual pan-tilt camera, synthetic disturbances, and a tracking algorithm that closes the loop.

The four required sub-functions, verbatim: **Observe** the environment · **Acquire and detect** the beacon · **Estimate** the position · **Continuously adjust** the pointing direction to maintain visibility.

### 2.2 Users and operating environment

**Intended users:** algorithm developers and students, not field operators. The PS says the platform exists "for algorithm development and learning". This matters: *explainability, configurability and instrumentation are first-class requirements*, not polish. The project understands this well — it is its strongest differentiator (§18).

**Operating environment:** a desktop application. The PS demands "a standalone executable application" and a "User Manual with description of installation". Judges will run a binary.

### 2.3 Required inputs / outputs

| | Required |
|---|---|
| **Inputs** | Scenario configuration (all parameters in rows 1–25, user-defined); **and**, for Benchmark-2, evaluator-supplied `.mp4` @30 fps covering a complete screen with noise and a moving beacon, which the software must accept **bypassing its PTZ camera** |
| **Outputs** | Live GUI display of tracking performance and statistics; an **automatically generated performance report** containing simulation duration, FPS, acquisition time, **average and maximum** tracking error, lock retention rate, processing time; a **log of centroiding error** |

### 2.4 The specification table (rows as this audit numbers them)

| Row | Parameter | Value | Status of the number |
|---:|---|---|---|
| 1 | Screen size (min.) | 2000 × 2000 px | hard minimum |
| 2 | Camera type | Monochrome FPA (colour optional) | |
| 3 | Camera resolution | 640 × 480 | user-definable |
| 4 | Camera FOV | default 4° × 3° | user-definable |
| 5 | Camera update rate | ≥ 30 Hz | hard minimum |
| 6 | Initial camera position | centre of screen | |
| 7 | Target type | beacon spot | |
| 8 | Number of targets | 1 mandatory, multiple optional | |
| 9 | Target shape | default square | |
| 10 | Target size | 5–20 × 5–20 px, default 10 | |
| 11 | Initial target location | default **random** | |
| 12 | Motion | **at least four**: straight line, circular, figure-8, random | |
| 13–14 | Max pan / tilt speed | 5–10 °/s, default 5 | |
| 15 | Update interval | ≥ 20 Hz | |
| **16** | **Acquisition time** | **≤ 2 s** | graded |
| **17** | **Tracking error** | **≤ 10 px** | graded |
| **18** | **Target loss** | **< 5 %** | graded |
| **19** | **Re-acquisition time** | **≤ 1 s** | graded |
| **20** | **Processing speed** | **≥ 20 FPS** | graded |
| 21 | Image noise | salt & pepper ~10 %, Gaussian, Poisson | user-selectable |
| 22 | Max noise std. dev. | 20 px | |
| 23 | Max camera jitter | ± 20 px/frame | |
| 24 | Atmospheric disturbance | clear, haze, fog, rain, low light | |
| 25 | Platform motion | ± 20 px/frame max; linear mandatory | |

### 2.5 Marks allocation — where the risk actually is

| Stage | % | What is graded | Project's exposure |
|---|---:|---|---|
| Functional Verification | 20 | all mandatory functions demonstrated in 10–15 min; operational success; **GUI** | **High risk from the default scenario (§16)**; GUI itself is strong |
| Benchmark Performance-1 | 30 | execution of *evaluator-supplied scenarios*; **log of centroiding error**; auto-generated performance logs | Medium — logs are excellent; the *screen-frame* centroid column is unbounded (§8.3) |
| Benchmark Performance-2 | 30 | *evaluator-supplied `.mp4`* @30 fps; centroiding error vs predefined values; RMSE, acquisition, re-acquisition, lock retention, FPS | **Accuracy excellent, FPS fails at p95/p99 (§1.2.3)**; no end-to-end fixture exists in-repo |
| Technical Evaluation | 20 | problem understanding, architecture, algorithm selection, **AI and computer vision**, innovation, documentation, Q&A | Strong on architecture/docs; **"AI" is the exposed flank** |

### 2.6 Implicit engineering requirements the PS does not spell out

1. **The disturbance can exceed mount authority.** Row 13 gives 5 °/s = 26.7 px/frame at 30 Hz; row 23 gives 20 px/frame of jitter and row 25 another 20. Rows 23+25 = 40 px/frame = **150 % of authority.** A purely reactive controller cannot meet row 17. The project derives this correctly (`src/degrade/disturbance.hpp:20–30`) and it is the intellectual core of the design.
2. **Rows 11 and 16 are mutually inconsistent.** A random initial target position on a 12.5° screen viewed through a 4°×3° camera gives P(visible at t=0) = 7.68 %; a full search at 5 °/s takes ~15–19 s against row 16's 2 s. The project identifies this and reports "cold acquisition" as a derived bound rather than pass/fail. **This is correct and defensible** — but see §16.1 for the demo consequence.
3. **Row 17 vs row 23.** With ±20 px/frame of jitter on the true boresight, the instantaneous pointing error is bounded below by the jitter itself. The project derives a 16.33 px floor and reports row 17 as "BOUND DERIVED". Verified by this audit: with `jitter=0`, tracking RMS drops to **3.357 px** and handover is reached at 2.2 s.
4. **Judges will supply data the team has never seen.** Both benchmark stages are held-out. Robustness to *unseen* parameterisation is therefore worth 60 % of the marks.

---

## 3. Requirement Traceability Matrix

Legend: **IMPL** implemented and verified by this audit · **PART** partially implemented · **CLAIM** claimed but not demonstrated end-to-end · **GAP** missing · **UNVERIFIED** could not be checked from available files.

### 3.1 Camera, target and motion parameters

| PS requirement | Where addressed | Status | Evidence (this audit) | Gap |
|---|---|---|---|---|
| Row 1 — screen ≥ 2000×2000 | `src/core/frames.hpp` `ScreenGeometry`; `scenarios/*.toml [world].canvas_px` | **IMPL** | GUI screen-overview panel renders the 2000×2000 canvas; schema enforces a minimum | — |
| Row 2 — monochrome FPA | `src/core/image.hpp`, 8-bit grey throughout; colour clip converted at decode | **IMPL** | `tests/video/clips/colour_640x480.mp4` handled | — |
| Row 3 — 640×480, user-defined | `CameraGeometry::make`; `[camera].resolution` | **IMPL** | direct-mode clip of 641×481 triggers a documented workspace rebuild (`pipeline.cpp:~500`) | — |
| Row 4 — FOV default 4°×3° | `CameraGeometry`, IFOV 109.083 µrad/px logged | **IMPL** | GUI "Derived" panel shows 109.083 µrad/px | Linear (equidistant) projection, not pinhole — documented, see §8.1 |
| Row 5 — ≥30 Hz | `[sim].camera_hz`, schema range 30–1000 | **IMPL** | all scenarios at 30 | **Bug:** blur samples platform rate at a hardcoded 30 Hz (§7.1 B-1) |
| Row 6 — initial camera at screen centre | `[camera].initial_pos_px = [999.5, 999.5]` | **IMPL** | — | — |
| Row 7 — beacon spot | `EmitterKind::Target` | **IMPL** | — | — |
| Row 8 — 1 target mandatory, multiple optional | `EmitterSoA`, decoys, clutter | **IMPL** | `decoy_swarm.toml` runs | Multi-*target* tracking (as opposed to multi-*object* discrimination) not demonstrated |
| Row 9 — shape, default square | `ShapeKind::{Square,Circle,Gaussian}` | **IMPL** | `src/camera/splat.cpp` separable splat per shape | — |
| Row 10 — size 5–20 px | `[target].size_px`, schema-bounded | **IMPL** | `tests/bad_configs/03,04` reject out-of-range | — |
| Row 11 — initial location, default random | `[target].initial_px = "random"` | **IMPL** | `baseline.toml` uses it | **This is what breaks the default scenario — §5.1** |
| Row 12 — ≥4 motion modes | `src/world/motion_factory.cpp` | **IMPL** | linear, circular, lissajous/figure-8, random (OU), spiral, sinusoidal | Exceeds requirement |
| Row 8 (edge behaviour) | `world/motion_component.cpp` triangle-wave fold | **IMPL** | closed in `issues_till_now.md` §2.0; verified by `just test-one motion` | — |
| Rows 13–15 — pan/tilt ≤5–10 °/s, ≥20 Hz | `src/plant/gimbal.hpp`, `GimbalParams::from_dps` | **IMPL** | rate + accel limits, 10 ms transport delay, 20 ms lag, encoder LSB, sub-tick integration at 300 Hz | Encoder model **bypassed in perception** — §7.1 B-2 |

### 3.2 Disturbances

| PS requirement | Where addressed | Status | Evidence | Gap |
|---|---|---|---|---|
| Row 21 — salt & pepper ~10 %, Gaussian, Poisson | `src/degrade/noise.cpp`, `sensor.cpp`, AVX2 in `sensor_simd.cpp` | **IMPL** | sweep axis exercises 0 and 0.10 | — |
| Row 22 — σ ≤ 20 grey levels | `[noise].gaussian_sigma`, schema-capped | **IMPL** | `tests/bad_configs/08` rejects >20 | — |
| Row 23 — jitter ±20 px/frame | `DisturbanceGenerator::offset`, uniform, resampled once per **camera** frame | **IMPL** | verified: `jitter=0` → 3.357 px RMS; `jitter=20` → 17.17 px | Correctly applied to the *boresight*, not to pixels — physically right |
| Row 24 — clear/haze/fog/rain/low light | `Atmosphere` enum, affine contrast/brightness | **IMPL** | all five in the sweep | Atmosphere is a **5-level enum, not a continuous severity**; no turbulence model — §8.2 |
| Row 25 — platform motion ±20 px/frame, linear mandatory | same motion algebra as targets, `Stream::PlatformMotion` | **IMPL** | `baseline.toml` uses linear (15, −8) px/s | — |

### 3.3 Graded performance rows — the ones that decide the marks

Measured on `scenarios/compliance.toml` (spec defaults, beacon in view at t=0), clutter/decoy removed, 60 s, this commit.

| Row | Requirement | Measured | Verdict | Note |
|---:|---|---|---|---|
| 16 | Acquisition ≤ 2 s | **0.067 s** (in-view) | **PASS** | Cold acquisition is geometrically impossible (§2.6.2); correctly reported as a derived bound |
| 17 | Tracking error ≤ 10 px | **17.17 px** RMS | **BOUND DERIVED** | 3.357 px with jitter off. The 16.33 px floor argument is sound and verified |
| 18 | Target loss < 5 % | **0.11 %** | **PASS** | …**but the denominator is in-FOV frames only — §9.2** |
| 19 | Re-acquisition ≤ 1 s | 0 episodes (clean) / **0.109 s** over the sweep | **PASS** | |
| 20 | Processing ≥ 20 FPS | synthetic **222 FPS** p50 · **video 30.8 FPS p50, 10.5 FPS p99** | **PASS synthetic / FAIL video at p99** | §12 |
| — | Centroiding error (60 % of BP marks) | image **0.19 px** · **screen 589 px @60 s** | **PASS / STRUCTURAL FAIL** | §8.3 |

### 3.4 Deliverables

| Deliverable | Status | Evidence | Gap |
|---|---|---|---|
| Standalone executable | **IMPL** | `dist/sat-tracker-*-linux-x86_64.tar.gz` exists; `just dist` builds it; CI produces a Windows artifact | Linux tarball only in-repo; **Windows binary not verified by this audit** (`UNVERIFIED`) |
| Source code, modular & commented | **IMPL** | 27.7 k LOC, 13 CMake modules, linker-enforced boundaries. Comment quality is exceptional | Some comments are **stale** (§15.2) |
| Technical report (10–15 pp) | **GAP** | `docs/report/.gitkeep` is empty | **Not started.** 20 % of marks depend partly on it |
| User manual | **IMPL** | `docs/MANUAL.md` (360 ll) + `docs/GUIDE.md` (416 ll) with 6 screenshots | `docs/manual/.gitkeep` empty — the *submitted* manual artefact does not exist as a document |
| Demo video (3–5 min, optional) | **GAP** | — | Optional |
| **Performance log (auto-generated)** | **IMPL** | `run.json` + `centroid.csv` + `report.html`, all three written per run | PS asks for **average** tracking error; project reports RMS/p95/max, **no mean** — §9.3 |

### 3.5 "AI-Based" — the title requirement

| PS text | Status |
|---|---|
| Title: "**AI-Based** Virtual Camera Tracking System" | **GAP** — no ML in the shipped path |
| Expected solution: "an **AI-assisted** camera tracking system" | **GAP** |
| Technical Evaluation criterion 4: "**AI and computer vision**" | **PART** — CV is strong; AI absent |
| Technical Report: "AI methods **(if used)**" | The PS's own hedge; this is the team's defence |

Per audit scope this is **not** scored as an implementation defect. It is recorded because it is the single most predictable Q&A attack (§17.4) and because the architecture must be judged on whether it can absorb the fix (§22).

---

## 4. Current System Architecture

### 4.1 The pipeline as built

```
 Scenario (TOML, schema-validated, 18 negative fixtures)
    |
    +-> World  (EmitterSoA; analytic motion algebra, §7.2 closed forms)     [300 Hz sub-tick]
    |
    +-> Plant  (gimbal: rate+accel limit, 10 ms delay, 20 ms lag, encoder)  [300 Hz sub-tick]
    |
    v
 Image formation   background -> emitter splat (separable) -> exposure blur (8 substeps)
    |
    v
 Damage chain      atmosphere affine -> shot+read noise (fused, 1 Gaussian) -> FPN -> defects -> quantise
    |                                                          [AVX2, runtime-dispatched]
    v
 Detection         median 3x3 -> top-hat (van Herk) -> 2x SAT -> matched filter -> 2x CFAR -> grouping
    |                                                          [ROI-windowed when Confirmed]
    v
 Centroiding       windowed CoM / Gaussian fit, + compiled-in S-curve bias table
    |
    v
 B16               image px -> angular measurement via COMMANDED boresight
    |
    v
 Tracking          chi-square gate -> priority policy (5 terms) -> KF / IMM(CV,CA,CT) -> lifecycle FSM
    |
    v
 Supervisor        observable-only Strategy switching (detector, centroider, CFAR k, gains, q)
    |
    v
 Mode FSM          7 states; Search / Acquire / Track / Reacquire / Handover
    |
    v
 Control           PID + velocity feedforward + optional Smith predictor, anti-windup, bumpless
    |
    +--------------------------------> back to Plant   [INV-2: the loop is closed]
    |
    v
 Metrics           the ONLY consumer of truth; centroid.csv / run.json / report.html
    |
    v
 Snapshot          triple-buffered; FNV-1a fingerprint (INV-3); GUI attach point
```

### 4.2 Stage-by-stage reality check

| Stage | Exists | Real / mocked / simplified | Note |
|---|---|---|---|
| Scenario | ✅ | Real | 23-field schema, typed, range-checked, 18 negative fixtures |
| World / motion | ✅ | **Real, and better than required** | Analytic closed forms give exact position *and* max-speed/max-accel bounds, which the Kalman `q` is derived from |
| Plant (gimbal) | ✅ | Real | Sub-tick integration at 300 Hz is the right call |
| Image formation | ✅ | **Simplified** | Linear projection, no lens distortion, no PSF beyond the shape splat, no vignetting |
| Damage chain | ✅ | Real | Physically ordered; one fused pass |
| Atmosphere | ⚠️ | **Simplified to an enum** | Affine contrast/brightness only. **No turbulence** — §8.2 |
| Detection | ✅ | Real, textbook-correct | CFAR on a matched-filter response, with an SNR gate |
| Centroiding | ✅ | **Real, and the project's best work** | Bias table generated by `--calibrate-centroid` from a measured S-curve |
| Tracking | ✅ | Real | KF + IMM(CV/CA/CT), NIS-based quality, 5-term priority |
| Prediction | ✅ | Real | `predict_position(horizon)`, Smith predictor for transport delay |
| Control | ✅ | Real | PID+FF, anti-windup, bumpless switching, saturation-aware |
| Search | ✅ | Real | Spiral / raster / probabilistic (with negative information) / camp-and-wait |
| Supervisor | ✅ | Real, observable-only | Rule table; the learned policy is out of scope |
| Metrics | ✅ | Real | Strict truth isolation |
| **Platform-rate estimation** | ❌ | **Deliberately absent** | Argued correct in `pipeline.hpp:405–429`; the cited test file **does not exist** — §7.1 B-3 |
| **ML** | ❌ | Out of scope | §22 |

### 4.3 Invariants — the architecture's load-bearing claims

| INV | Claim | Enforcement | Audit verdict |
|---|---|---|---|
| 1 | Tracker cannot see truth | linker boundary + configure-time closure walk + `check_source_invariants.py` + self-test by injected violation | **VERIFIED** — genuinely excellent |
| 2 | Loop is closed | `pipeline.cpp:562` applies last frame's `cmd_rate_` before rendering | **VERIFIED** |
| 3 | Bit-exact reproducibility | `--verify-reproducibility`, CI −O0 vs −O2 | **VERIFIED but the gate is hollow** — damage chain disabled (§7.1 A-1) |
| 4 | Zero heap allocation in steady state | Debug `operator new` trap around `Pipeline::step` | **VERIFIED**; the trap found 4 real defects |
| 5 | Work in angles | tag-typed `Angle2`/`Pixel2`/`Rate2` | **VERIFIED** — compiler-enforced |
| 6 | Centroiding ≠ tracking error | separate fields, separate reporting | **VERIFIED** |
| 7 | Runs fully without AI | `--no-ai` is the shipped path; `linux-no-gui` CI job | **VERIFIED** |
| 8 | No damage in video modes | `SyntheticSource` not called when `video_` is set | **VERIFIED** — enforced by code path |
| 9 | Never emit a stale/interpolated centroid | gaps written as empty columns | **VERIFIED** |

INV-1 and INV-4's enforcement mechanisms are stronger than anything I would expect at this level. They should be demonstrated live (§24).

---

## 5. Current Implementation Status

### 5.1 The headline: the default scenario is broken

`scenarios/baseline.toml` is simultaneously:

* the default for `--headless` (`src/app/headless.cpp:344`),
* the default for `--gui` (`src/app/main.cpp:162`),
* the first entry in the GUI's scenario picker (`src/gui/dashboard.cpp:293`),
* the source of the README's hero screenshot (`Justfile:753`),
* the `just smoke` / dist sanity scenario (`Justfile:345`),
* and, per `docs/DEMO.md:172`, **"the known-good fallback (CP 15.3)"**.

Reproduce:

```bash
./build/sat-tracker --headless --scenario scenarios/baseline.toml --out /tmp/base
```

Measured, 60 s, seed 42:

```
CENTROIDING  frames scored 0      <- no frame had both a detection and the beacon in view
TRACKING     RMS 913.054 px       <- spec row 17 is 10 px
LOCK         retention n/a        <- the beacon was never in view
             false tracks 1777.99 /min  (1777 of 1800 frames confirmed with no beacon in view)
HANDOVER     not reached
```

The cause is row 11's random initial position interacting with §10.5's search geometry — a known, *documented* tension. But two things make this a defect rather than a documented limitation:

1. The file's own header says *"This is also the known-good scenario preloaded as a fallback that CP 15.3 requires for the live demo."* It is not known-good. It is the worst scenario in the repository.
2. `issues_till_now.md` §2.4 asserts *"With 120 clutter sources the policy correctly refuses to commit to any of them."* **It does not refuse.** It confirms a track on clutter for 1777 of 1800 frames and drives the mount at it for the whole run.

Removing the clutter does not rescue it:

| Configuration | Frames scored | Tracking RMS | Re-acq mean | Target loss | Gimbal sat. |
|---|---:|---:|---:|---:|---:|
| `baseline` as shipped | **0** | 913.05 px | 0.200 s | n/a | 0.91 % |
| `baseline`, clutter = 0 | 76 | 508.36 px | 2.973 s | 32.05 % | 44.91 % |
| `baseline`, clutter = 0, decoy = 0 | 79 | 196.32 px | **11.078 s** | 7.59 % | 44.91 % |

In the last row the image-frame centroiding is a healthy **0.347 px** — the detector is fine. The beacon is simply in the FOV for **79 of 1800 frames (4.4 %)**. This is an *acquisition and retention* failure, not a perception one.

### 5.2 Status by subsystem

| Subsystem | Status | Confidence |
|---|---|---|
| Scenario / schema | **IMPLEMENTED** | High — 18 negative fixtures, fuzzer-validated |
| World & motion algebra | **IMPLEMENTED** | High |
| Image formation | **IMPLEMENTED** | High |
| Damage chain (scalar + AVX2) | **IMPLEMENTED** | Medium — AVX2 parity unit-tested but **not covered by the INV-3 gate** |
| Detection | **IMPLEMENTED** | High |
| Centroiding | **IMPLEMENTED** | High — best-in-class |
| Tracking (KF/IMM/priority) | **IMPLEMENTED** | High |
| Control | **IMPLEMENTED** | High |
| Search / acquisition | **PARTIALLY IMPLEMENTED** | **Low** — cannot acquire from cold in clutter within 60 s |
| Supervisor | **IMPLEMENTED** | Medium — +15 pts in fog, **0** against clutter (project's own measurement) |
| Metrics / logs | **IMPLEMENTED** | High, with a denominator defect (§9.2) |
| GUI | **IMPLEMENTED** | High, with a display-consistency defect (§16.3) |
| Video ingest | **IMPLEMENTED** | Medium — accuracy excellent, **throughput fails at p95/p99** |
| Reproducibility | **IMPLEMENTED** | Medium — gate does not cover the noise chain |
| ML | **NOT IMPLEMENTED** (by decision) | — |
| Technical report | **NOT STARTED** | — |

---

## 6. Verification of `issues_till_now.md`

`issues_till_now.md` is itself an audited artefact. It is, on the whole, **unusually honest** — but it contains one materially false claim, several unreproducible numbers, and one significant omission.

### 6.1 Section 1 — performance

| # | Claim | Verdict | What I measured | Severity |
|---|---|---|---|---|
| 1.1 | `frame_acquire` was one opaque line; `centroid` was mislabelled | **FIXED / CONFIRMED** | `--stages` now shows `background`/`splat`/`damage_chain` separately and `centroid` at 2.8 µs | — |
| 1.2 | `frame_total` p50 42,551 → **2,618 µs**; 382 FPS | **NOT REPRODUCIBLE ON THIS MACHINE** | **4,104.7 µs p50** / 222 FPS on `compliance.toml`, 6 s. Different CPU, but the number is quoted absolutely, with no machine stated | Low (methodology) |
| 1.3 | Stage table of open gaps | **PARTIALLY CONFIRMED** | Directionally right; magnitudes differ by ~1.6× on this CPU | Low |
| 1.3 | **Omission** | **INCORRECT BY OMISSION** | The table lists 12 stages but **omits `snapshot`, which is 27.1× over its budget (813 µs vs 30 µs) and is 20 % of the frame.** It also omits `frame_acquire` (2.5× over) | **Medium** |
| 1.3 | `perception` 1,275 µs "**inside its §15 scalar line**" | **MISLEADING — apples to oranges** | True only at **p50 with the ROI window active**. `--bench-kernels` runs the same kernels full-frame: median+top-hat+2×SAT+matched+2×CFAR+grouping = **9,533 µs**. The `--stages` **p95 is 18,938 µs, 13.6× the budget.** §15's budget is a full-frame budget | **High** |
| 1.4 | AVX2 still missing for van Herk / matched filter / CFAR | **CONFIRMED, open** | `--bench-kernels`: matched 18.7×, cfar 28.6×, top_hat 6.9× over | Medium |
| 1.5 | CP 7.3 "120 s in under 2 s wall time" still 9.5× over | **CONFIRMED** | 20 s sim in 3.04 s wall = 6.6× real time | Low |

### 6.2 Section 2 — accuracy and robustness

| # | Claim | Verdict | Evidence |
|---|---|---|---|
| 2.0 | Four defects found by running 30 s instead of 6 s | **CONFIRMED and closed** | Each has a mechanism and a test. This is the best material in the file |
| 2.1 | Motion-term development history | **CONFIRMED as recorded** | Not independently re-measured — `UNVERIFIED` in detail, but the final behaviour is observable |
| 2.2 | Four control cases were scoring a transient | **CONFIRMED** | Corroborated independently: on a jitter-free run the metric prints `RMS 3.357 px, p95 0.973 px` — **RMS > p95**, proving the RMS is transient-dominated (§9.4) |
| 2.3 | Three retracted claims | **CONFIRMED as retractions** | Good practice |
| 2.4 | "Acquisition from cold, in clutter … **the policy correctly refuses to commit to any of them**" | **INCORRECT** | 1777/1800 frames Confirmed, 1778 false tracks/min. It commits, holds, and drives the mount for the entire run |
| 2.4 | "the spiral has not swept back over the beacon within 30 s" | **CONFIRMED and worse** | Still not acquired at **60 s** |
| 2.4 | Moving decoy not separated | **CONFIRMED** | `decoy_swarm.toml`: 48.8 px tracking RMS, 33.8 px centroiding, max 316 px |
| 2.4 | Supervisor: +15 pts in fog, 0 against clutter | **UNVERIFIED** | `just cp123` not run in this audit |
| 2.4 | **"Low light. Target loss reaches 86 %"** | **NOT REPRODUCIBLE** | `compliance.toml` + lowlight + no clutter, 20 s → **3.17 % target loss** (PASS). The 4 s sweep cell → 37.8 %. Three different numbers; no reproducing command is given in the file |
| 2.4 | Low-SNR centroiding 3.64× the bound at SNR 13 | **UNVERIFIED** | Requires `--calibrate-centroid`; not re-run |
| 2.4 | Figure-8 acceleration lag 16.56 px | **UNVERIFIED** | Plausible: feedforward carries velocity only |
| 2.4 | Handover 0 % under row-23 jitter, closed as won't-fix | **CONFIRMED, and the argument holds** | `jitter=0` → handover reached at **2.2 s**, tracking RMS **3.357 px**. The derivation is sound |

### 6.3 Section 3 — deliverables

| # | Claim | Verdict |
|---|---|---|
| 3.1 | "every §12 panel complete" | **CONFIRMED** — IMM, strategy, FSM and hypotheses panels all present in the captured screenshot |
| 3.1 | "Documentation for a first-time user" closed | **CONFIRMED** — `docs/GUIDE.md`, 6 screenshots, regenerated by `just screenshots` |
| 3.1 | "Everything in scope is complete" | **INCORRECT** — the **Technical Report** is a mandatory PS deliverable, is not built (`docs/report/.gitkeep`), and is not listed anywhere in this file as open |
| 3.2 | ML out of scope | **CONFIRMED** — but see §6.5 |

### 6.4 Section 4 — closed items

All spot-checked items hold. `--fuzz-scenarios 40` passes (9,411 frames, 0 failures); `scenarios/adversarial/` holds six scenarios; the allocation trap is armed in Debug and `tests/robustness/test_alloc.cpp` passes.

### 6.5 What `issues_till_now.md` does not say

1. **`baseline.toml` — the default scenario — fails totally.** §2.4 mentions cold acquisition in clutter as a *Stage 13 sub-problem*. It never says "the file we ship as the demo fallback scores zero on the graded metric".
2. **The `snapshot` stage is 27× over budget and is in the frame in headless mode.**
3. **The INV-3 gate does not exercise the damage chain.**
4. **The video path fails row 20 at p95/p99.**
5. **The Technical Report deliverable does not exist.**
6. **`models/centroidnet_v1.json` exists and claims `best_val_rmse_px: 0.2744`** while the same file asserts ML is "not built". (The *model card* `docs/models/centroidnet_v1.md` is correctly labelled "PRELIMINARY — trained on synthetic dummy data"; the JSON carries no such disclaimer and would be read by a judge or a script as a result. It is also *worse* than the classical centroider's measured 0.19 px.)

---

## 7. Independently Discovered Bugs

Not present in `issues_till_now.md`. Each is stated with the file, the line, the evidence, and the consequence.

### 7.1 Confirmed defects

#### A-1 — The INV-3 reproducibility gate runs with the damage chain disabled · **P1**

`src/app/verify_repro.cpp:47–58` builds a bare `PipelineConfig` and never enables the sensor model. `src/degrade/sensor.hpp:84`: `bool enabled_ = false;`. Therefore `--verify-reproducibility` exercises a pipeline with **no Gaussian noise, no Poisson, no salt & pepper, no fixed-pattern, no defects, no jitter, no platform motion, no atmosphere**.

Confirming evidence — the seed does not change the digest for 4 of the 5 built-in scenarios:

```
static   1  IDENTICAL  6e5ea6926c621cbf
static   2  IDENTICAL  6e5ea6926c621cbf     <- same digest, different seed
linear   1  IDENTICAL  bf817002c3fc587c
linear   2  IDENTICAL  bf817002c3fc587c     <- same
```

Only `multi` differs, because its seed drives build-time clutter *layout*, not per-frame randomness. (The seeding mechanism itself is fine: real scenarios do respond — `compliance.toml` seed 42 → `5563255918112396865`, seed 43 → `9765698476702505215`.)

**Why it matters.** The AVX2 damage chain is **runtime-dispatched** (`src/degrade/sensor_simd.cpp`). Runtime dispatch means different machines execute different code. That is precisely the risk INV-3 exists to catch, and the gate never runs it. The CI's −O0/−O2 comparison inherits the same hole, and both jobs run on the same runner, so AVX2-vs-scalar divergence is untested end-to-end.

**Fix.** Add noise, jitter, platform motion and a non-clear atmosphere to the five built-in scenarios; add a sixth that forces the scalar path (`SAT_NO_AVX2=1` or equivalent) and assert its digest equals the AVX2 one. Extend the CI matrix to compare digests across the scalar and AVX2 dispatch.

#### A-2 — `snapshot` costs 813 µs per frame in headless mode · **P1**

`src/engine/pipeline.cpp:1145` — `snap.set_preview(frame.pixels)` copies 307,200 bytes per frame; `fingerprint(snap)` then hashes the same bytes with byte-at-a-time FNV-1a. Measured `--stages`: **813.1 µs p50, against a §15 budget of 30 µs — 27.1×**.

It is on by default in headless (`headless.hpp:62 bool fingerprint = true;`), so `just stages`, `just headless` and every quoted FPS number include it:

| | p50 | FPS |
|---|---:|---:|
| default (fingerprint on) | 4.491 ms | 222.7 |
| `--bench` (fingerprint off) | 3.429 ms | 291.6 |

**24 % of the frame** is a memcpy + hash for provenance that would never run on a real camera. The copy is unnecessary — the fingerprint can hash `frame.pixels` in place — and the preview is only needed when a GUI consumer is attached.

#### A-3 — Blur samples the platform rate at a hardcoded 30 Hz · **P2**

`src/engine/pipeline.cpp:659`:

```cpp
const Rate2 pr = source_.disturbance().platform_rate(frame_ / 30.0);
```

`camera_hz` is schema-legal from **30 to 1000** (`src/scenario/schema.cpp:31`). At `camera_hz = 60`, the platform rate used for the exposure smear is sampled at **twice the true elapsed time**, so the motion-blur direction and magnitude are wrong for any periodic platform motion (circular, figure-8, sinusoidal). `clk.camera_dt()` is in scope on the same function (`pipeline.cpp:~549`). Every shipped scenario uses 30, so this is latent — until a judge sets row 5 above its minimum, which the PS explicitly permits.

**Fix.** `source_.disturbance().platform_rate(frame_ * clk.camera_dt())`.

#### A-4 — Perception reconstructs measurements through the *true* gimbal angle, not the encoder · **P1**

`src/engine/pipeline.cpp:648`:

```cpp
const Angle2 commanded = video_ ? gimbal_.position() : gimbal_.true_position();
```

`position()` is the quantised encoder reading; `true_position()` is the exact mount angle. `src/plant/gimbal.hpp:156–158` states the rule explicitly: *"position() is what the controller is allowed to read: a real encoder reports a quantised angle. true_position() is what the metrics use. Mixing them up is the bug §10.3 exists to make impossible."* Design §14 CP 4.10's acceptance criterion is *"the controller reads only the quantised value"*.

The synthetic path violates it for the B16 measurement reconstruction — and is **inconsistent with the video path**, which correctly uses `position()`.

Demonstrated by sweeping the encoder LSB on `compliance.toml`, 10 s:

| `encoder_lsb_urad` | in pixels | screen RMSE | image RMSE | tracking RMS |
|---:|---:|---:|---:|---:|
| 20 | 0.18 px | 99.3061 | 0.2028 | 17.585 |
| 500 | 4.58 px | 99.3086 | 0.1881 | 17.613 |
| 4000 | **36.67 px** | **99.3060** | 0.2038 | 20.383 |

**A 36.67 px encoder quantisation — 3.7× the entire row-17 budget — changes the reported centroiding error by zero.** The encoder model reaches only the control feedback path; the measurement path is immune to it. The reported measurement accuracy is therefore optimistic by the encoder error, and "what happens if your encoder is coarse?" gets a misleadingly good answer.

*Mitigating:* the magnitude at the default 20 µrad LSB is ~0.05 px, and `pipeline.cpp:~720` already budgets `encoder` into the measurement noise `R`, so the filter is conservative. The defect is one of **correctness of the model**, not of current numbers.

#### A-5 — The build provenance hash is stale · **P2**

`CMakeLists.txt:64–80` stamps `SAT_GIT_HASH` with `execute_process` at **configure** time. It is not re-evaluated on rebuild.

```
$ git rev-parse --short=7 HEAD        -> 803733d
$ ./build/sat-tracker --version       -> SAT ... 0.1.0+d563657     (two commits stale)
$ run.json  .provenance.build         -> d563657
```

The project's own justification for the field (`CMakeLists.txt:65`) is *"Every run.json and centroid.csv header carries the commit it was produced by… Without it a benchmark number cannot be traced to code."* Right now it carries the wrong commit. Any benchmark submitted from this tree is mis-attributed.

**Fix.** Move the stamp into a `add_custom_target` / `configure_file` driven by a build-time `cmake -P` script, or add `CMAKE_CONFIGURE_DEPENDS` on `.git/HEAD` and `.git/index`.

#### A-6 — `--set` rejects string values without a usable error · **P2**

```
$ ./build/sat-tracker --headless --set atmosphere.mode=lowlight ...
sweep: override 'atmosphere.mode = lowlight' is not valid TOML: Error while parsing value: could not determine value type
```

Two problems: the message says `sweep:` during a `--headless` run, and it never says *"string values must be quoted: --set 'atmosphere.mode=\"lowlight\"'"*. This is exactly the command a judge would type to change the weather live. **Fix:** auto-quote a bare token that fails to parse as a TOML scalar and matches the schema's enum for that key; correct the prefix.

#### A-7 — A cited evidence file does not exist · **P3**

`src/engine/pipeline.hpp:422` grounds the platform-drift argument in `tests/loop/test_feedforward.cpp`. `tests/loop/` contains only `test_closed_loop.cpp`, `test_controller.cpp`, `test_gimbal.cpp`. The actual measurement appears to live in `tests/control/test_stage10.cpp:659`. The argument is sound; its citation is dangling. Given that this project's credibility rests on citations being checkable, this matters more than usual.

#### A-8 — A load-bearing comment is factually stale · **P3**

`src/gui/dashboard.cpp:177`: *"because the current detector is the Stage 1 straw man, and at row 21's 10 % impulse noise it fails outright"*. The default detector has been `Classical` since Stage 4; the screenshot shows "classical (§9.4)" selected. The *behaviour* (open clean, dial damage up) is a good demo decision; the stated reason is no longer true.

### 7.2 Non-defects — things that look wrong and are not

Recorded so a reviewer does not re-litigate them.

* **`platform_rate_est_` is never set outside a test.** Deliberate, and correct. `pipeline.hpp:405–429` proves the drift is already cancelled by reconstructing through the commanded boresight; subtracting an estimate would remove it twice. (Only the citation is broken — A-7.)
* **The projection is linear, not pinhole.** Documented at `core/frames.hpp:30–48` with the tradeoff stated (0.13 px at the extreme corner) and the benefit (a linear measurement equation, so a plain KF is exactly right). Defensible; see §8.1 for the Q&A framing.
* **`RMS 3.357 px` with `p95 0.973 px`.** Mathematically consistent for a heavy-tailed distribution; the RMS is dominated by the acquisition transient. But it *reads* as an error — see §9.4.
* **Elevation points down.** Documented at `core/frames.hpp:50–57`; consistent through every frame; removes sign flips.

---

## 8. Physics / FSOC Audit

This is not a generic CV project and must not be judged as one. The question is whether the software models the *pointing* problem.

### 8.1 Modelled correctly

| Aspect | Verdict | Evidence |
|---|---|---|
| Boresight-referenced disturbance | **Correct, and the single best physics decision in the project** | Jitter and platform motion perturb the **true boresight**, never the pixels (`degrade/disturbance.hpp:1–30`). This means motion blur falls out of exposure integration instead of being faked, *and* the tracker cannot implicitly learn its own pointing error |
| Pixel ↔ angle conversion | Correct and exact | `CameraGeometry::unproject/project`; IFOV = FOV/width (not width−1) — right, because FOV spans the full sensor; principal point at (w−1)/2 — right, because coordinates name pixel centres |
| Angular state (INV-5) | Correct | All tracker state in µrad; tag types make a pixel/angle mix a compile error |
| Gimbal dynamics | Correct | Rate limit, acceleration limit, 10 ms transport delay, 20 ms first-order lag, encoder LSB, resonance hook; integrated at 300 Hz sub-ticks so the acceleration limit is observable |
| Transport-delay compensation | Correct | Smith predictor, with both sides of the error advanced together — the CP 10.1 double-lead analysis in `pipeline.cpp:~990` is textbook-correct and genuinely subtle |
| Disturbance ≥ authority | Correct, and derived | 26.7 px/frame authority vs 40 px/frame combined disturbance. This is *the* insight of the design |
| Process noise from analytic bounds | Correct, and better than typical | `q` and the velocity prior are derived from the target's own closed-form max acceleration/speed, not from the mount's. The two retracted derivations recorded at `pipeline.cpp:62–115` are exactly the mistakes most teams will ship |
| Exposure smear | Correct | 8 sub-steps across a 5 ms exposure; gimbal + platform rate, jitter deliberately excluded (jitter is per-frame, not intra-frame) |
| Acquisition vs tracking distinction | Correct | 7-state mode FSM; separate acquisition/re-acquisition metrics |
| Coarse vs fine handover | **Modelled, which is rare** | A co-boresighted quadrant-cell sensor with a 1 mrad capture range and a sustained-RMS criterion. This is a real FSOC concept and few teams will have it |

### 8.2 Not modelled — and what it costs

| Missing physics | Status | Consequence | Judge risk |
|---|---|---|---|
| **Atmospheric turbulence** | **GAP** | `Atmosphere` is a 5-value enum applying an **affine contrast/brightness change** only. There is no angle-of-arrival jitter, no scintillation (irradiance fluctuation), no beam wander, no Fried parameter, no Greenwood frequency, no C²ₙ | **High.** The PS background is *literally about* atmospheric propagation. "Fog" that only dims the image is a *photometric* model, not a *turbulence* model. Row 24 says "User-defined **reduction in contrast and brightness**", which is the team's defence — but a Dept. of Space evaluator will ask about AoA jitter and scintillation |
| **Lens distortion / PSF** | Simplified | Beacon rendered as a geometric shape with exact-coverage anti-aliasing; no Airy pattern, no defocus, no radial distortion | Medium — ask "what if the evaluator's clip has a defocused blob?" Mitigant: the Gaussian shape kind partially covers it |
| **Vignetting / flat-field** | GAP | Fixed-pattern noise exists (gain+offset) but no radial falloff | Low |
| **Rolling shutter** | GAP | Global shutter assumed | Low — FPA implies global shutter |
| **Wavelength / photometry** | Simplified | Intensity is in grey levels, not W/m² or photons; "SNR" is contrast/σ, not a radiometric SNR | Medium — the shot-noise model is therefore qualitative, not calibrated |
| **Absolute attitude reference** | **GAP — and it is architectural** | No IMU, no star tracker. Platform drift is unobservable, so the world-frame estimate drifts without bound. See §8.3 | **High** — this is the finding below |

**Note on turbulence — an important nuance the team should pre-empt.** Because jitter is applied at the boresight and is resampled once per camera frame with a uniform ±20 px bound, it is *functionally* a crude angle-of-arrival jitter model. That is a legitimate defence, but it is white (no Kolmogorov temporal spectrum), uniform (not Gaussian), and identical in both axes. Adding a proper AoA model is cheap (§21 Phase 3) and would convert a weakness into a differentiator.

### 8.3 The screen-frame centroid error is unbounded — and it is a graded column

This is the deepest issue in the project, and it is a *consequence* of correct physics, not a coding bug.

**The algebra.** With true boresight `B_true = B_cmd + D(t)` where `D` is the platform displacement:

* the beacon lands on the sensor at `T − B_true`;
* the detector finds it there (image RMSE 0.19 px, so this step is near-exact);
* `pipeline.cpp:~700` reconstructs `detection_screen = B_cmd + (T − B_true) = T − D(t)`;
* truth is `T`;
* therefore `centroid_error_screen = |D(t)|` — **identically the accumulated platform displacement.**

**The measurement.** `compliance.toml`, clutter and decoy removed, platform motion at its shipped linear (15, −8) px/s:

| Duration | screen RMSE | bias (signed) | image RMSE |
|---:|---:|---|---:|
| 4 s | 44.23 px | (−30.63, +16.53) | 0.2146 |
| 30 s | 294.64 px | (−224.83, +119.89) | 0.1972 |
| 60 s | **589.48 px** | (−449.94, +240.64) | 0.1926 |

This matches the prediction exactly. Mean of `D` over [0, 60] is (450, −240) → predicted bias (−450, +240); measured (−449.94, +240.64). Predicted RMSE = 17 × √(60²/3) = 588.9 px; measured 589.48 px. **The screen-frame "centroiding error" is a measurement of the disturbance, not of the algorithm.**

**Why this matters for marks.** Benchmark Performance-1 grades *"Log of Centroiding error"*. `centroid.csv` carries both frames, and the screen-frame column is the one a judge will naturally read because it is in the canvas coordinates the scenario is written in. On a clean 60 s run in clear air with nothing else in the frame, that column reads **589 px**.

**Why it is not simply a bug.** A pan-tilt mount with encoders cannot observe motion of the *base it is bolted to*. Without an IMU or a star tracker, the world-frame position of the beacon is genuinely unknowable to this system. The physics is right. The **reporting** is wrong.

**The fix is presentational and architectural, and both halves are needed:**

1. **Presentational (P0, cheap).** Report a third column, `centroid_error_boresight_relative`, and make the summary block say in one line: *"screen-frame error = image-frame error + accumulated pointing error; with row 25 platform motion active it grows as ∫|D|dt and is not a detector metric."* Right now `INV-6` is asserted in a comment but the summary prints `screen RMSE 589.48 px` under the heading **"CENTROIDING (graded, 60 %)"** with no such caveat.
2. **Architectural (P2, and it is a differentiator).** Add an optional **attitude sensor** — a noisy rate gyro, or a "star-tracker fix every N seconds" — as a scenario-configurable input, and estimate `D` in the filter. Then the screen-frame error becomes bounded, the system can *report* absolute pointing, and the team has a demonstrable answer to "how would you deploy this?" (§18.3).

### 8.4 Latency, timing and synchronisation

| Aspect | Verdict |
|---|---|
| Control-loop frequency | Correct — `control_hz` separate from `camera_hz`, both dividing `truth_hz`; schema *enforces* exact divisibility (`schema.cpp:314`) |
| Sub-tick plant integration | Correct — 300 Hz, 10 sub-ticks/frame |
| Frame timestamps | Correct — video mode uses **container timestamps**, not index/fps, and ships a VFR clip to prove it |
| Transport delay | Modelled (10 ms) and compensated (Smith) |
| **End-to-end latency** | **NOT MEASURED** — see §9.1. `frame_total` is compute time, not sensor-to-command latency. The two differ by the exposure, the decode queue depth and the control sub-tick phase |
| **Frame-rate ≠ processing-rate coupling** | **NOT MODELLED** — see §10.4 |

---

## 9. Metrics & Evaluation Audit

### 9.1 Coverage against the PS's Performance Log deliverable

| Metric | Defined | Measured | Logged | Visualised | Reproducible | Baseline-compared |
|---|:-:|:-:|:-:|:-:|:-:|:-:|
| Simulation duration | ✅ | ✅ | ✅ `run.json` | ✅ | ✅ | n/a |
| FPS | ✅ | ✅ p50/p95/p99 | ✅ | ✅ | ✅ | n/a |
| Acquisition time | ✅ | ✅ in-view **and** cold | ✅ | ✅ | ✅ | n/a |
| **Average tracking error** | ⚠️ | **RMS, p95, max — no mean** | ✅ | ✅ | ✅ | ✅ (ablations) |
| Max tracking error | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| Lock retention rate | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| Processing time | ✅ | ✅ per-stage p50/p95/p99 | ✅ | ✅ (GUI table) | ✅ | ✅ (§15 budget) |
| Re-acquisition time | ✅ | ✅ | ✅ | ✅ | ✅ | n/a |
| Centroiding error | ✅ | ✅ image **and** screen | ✅ `centroid.csv` | ✅ | ✅ | ✅ |
| False-alarm rate | ✅ | ✅ /min | ✅ | ✅ | ✅ | ✅ |
| Gimbal saturation | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| **FOV containment (time on target)** | ❌ | ❌ | ❌ | ❌ | — | — |
| **End-to-end latency** | ❌ | ❌ | ❌ | ❌ | — | — |
| **CPU / memory usage** | ❌ | ❌ | ❌ | ❌ | — | — |
| False-negative rate | ⚠️ | implicit in target loss | ⚠️ | ❌ | — | — |

This is, with the three gaps below, **a better metric suite than the PS asks for**. It is a genuine strength.

### 9.2 The denominator defect — the most consequential metric problem · **P0**

`src/metrics/collector.cpp:196–199`:

```cpp
m.lock_retention_rate = frames_in_fov_ ? (frames_held_in_fov_ / frames_in_fov_) : 0.0;
m.target_loss_frac    = frames_in_fov_ ? (1.0 - m.lock_retention_rate) : 0.0;
```

Retention is normalised over **in-FOV frames only**. The reasoning given (`collector.cpp:39–43`) is defensible in isolation: don't penalise the system for frames where the beacon genuinely is not there to be seen. But it produces this:

```
baseline.toml, clutter=0, decoy=0, 60 s (1800 frames)
  retention    92.41 %   (73 held / 79 in-FOV frames)
  target loss   7.59 %   (row 18, < 5 %)
```

The beacon was inside the FOV for **79 of 1800 frames — 4.4 % of the run**. The system spent 95.6 % of the run pointing at empty sky. The metric reports **7.59 % target loss**.

**This directly contradicts the PS's stated objective**, which is to *"locate and maintain the remote terminal within its camera Field-of-View"*. Maintaining FOV containment **is** the deliverable, and the fraction of run time achieved is not reported as a metric anywhere — it is recoverable only by dividing two numbers inside a parenthetical.

**Fix (P0, one afternoon).** Add a first-class metric and print it above retention:

```
FOV CONTAINMENT  (PS coarse-alignment objective)
  time on target      4.39 %   (79 / 1800 frames with the beacon inside the FOV)
  post-acquisition    —        (excluding the initial search)
```

Report `target_loss` twice — once over in-FOV frames (the current, detector-centric reading) and once over post-acquisition frames (the system-centric reading). Grade row 18 against the second.

### 9.3 PS asks for "average", the project reports RMS · **P2**

BP-2 grades *"Comparison of Centroiding error with predefined error values"* and *"RMSE, acquisition and re-acquisition time, Lock retention rate, FPS"*. The Performance Log deliverable separately asks for *"**average** and maximum tracking error"*. The project reports RMS, p95 and max — never the arithmetic mean. For a heavy-tailed distribution these differ substantially (see §9.4). **Fix:** add `mean_px` alongside `rms_px` in `run.json` and the summary. Trivial, and it closes a literal deliverable gap.

### 9.4 RMS is transient-dominated and there is no steady-state figure · **P1**

Observed on a jitter-free 20 s `compliance.toml` run:

```
TRACKING  RMS 3.357 px   p95 0.973   max 45.886
```

**RMS (3.357) > p95 (0.973).** Arithmetically consistent — a handful of 45 px acquisition-transient samples dominate the sum of squares — but it reads as a bug, and a judge will say so. The project already learned this lesson once (`issues_till_now.md` §2.2: "Four Stage 10 control cases were scoring a transient") and fixed it by *lengthening runs*, not by *separating the transient*.

**Fix:** report tracking error twice — `acquisition transient (first N frames after lock)` and `steady state (the rest)` — and grade row 17 on the steady-state figure, stating the split. This is both more honest and more favourable.

### 9.5 Where the evaluation is cherry-picked

The README's headline table is labelled *"On clear air with nothing else in the frame — 200 runs, `just sweep`"*. That labelling is honest as far as it goes. What it omits:

1. **Each sweep run is 4.0 seconds long** (`scenarios/sweeps/weather.toml:29`). 120 frames. The project's own `issues_till_now.md` §2.0 is a case study in how six-second runs hid four independent defects; the compliance sweep still runs at four.
2. **The quoted cell is 1 of 40.** The full matrix run in this audit: **106 PASS, 69 FAIL, 82 BOUND DERIVED**.
3. **Duration sensitivity is not reported.** §8.3 shows the graded screen-frame metric grows 44 → 295 → 589 px from 4 s → 30 s → 60 s. A 4 s sweep reports the most flattering point on a monotone curve.

### 9.6 A README claim contradicted by the project's own sweep · **P1**

> README: *"Through every weather mode the specification lists, with row 21's 10 % impulse noise and row 22's read noise at the cap, centroiding stays at **0.14–3.4 px** and target loss at **1.7 %**."*

`just sweep`, clutter-free / decoy-free cells, salt & pepper = 0.10:

| Weather (row 24) | Centroiding (image) | p95 | Target loss | Sweep verdict |
|---|---:|---:|---:|---|
| clear | 0.200 px | 0.210 | 1.667 % | PASS |
| haze | 3.172 px | 14.804 | 1.667 % | **FAIL** |
| fog | 3.437 px | 14.879 | 1.833 % | **FAIL** |
| rain | 3.233 px | 14.805 | 1.667 % | **FAIL** |
| **lowlight** | **237.321 px** | **366.628** | **37.769 %** | **FAIL** |

Two separate problems:

* **"Every weather mode" is false.** Low light is one of row 24's five modes and it is 237 px / 37.8 %, not "0.14–3.4 px / 1.7 %". (The README *does* mention low light three paragraphs later as a non-passing condition — so the document contradicts itself rather than concealing.)
* **The quoted range uses means where the project's own tool grades p95.** Haze, fog and rain are all marked **FAIL** by `--sweep` at p95 ≈ 14.8 px, while the README's 3.4 px upper bound makes them read as passing.

**Fix:** restate as *"Through four of row 24's five weather modes … mean 0.14–3.4 px (p95 up to 14.9 px, which the compliance matrix grades FAIL); low light is a known sensitivity failure at 237 px."*

---

## 10. Robustness Audit

Assume a judge is deliberately trying to break the system. For each case: **what does the system actually do**, not what it should do.

### 10.1 Target-related

| Attack | Actual behaviour | Verdict |
|---|---|---|
| Target absent from t=0 | `beacon_late_640x480.mp4` handled; FSM stays in Search; no false lock in clean air | **PASS** |
| Target appears suddenly | Detected within 2 frames (in-view acquisition 0.067 s) | **PASS** |
| Target disappears | §7.4 `occlude_target` is an *interval*, correctly (edge-triggered occlusion "never ends" — the code says so at `pipeline.cpp:~600`). Confirmed → Coasting → Search | **PASS** |
| Target moves abruptly | IMM (CV/CA/CT) plus NIS-gated quality tests. Figure-8 leaves a **16.56 px** acceleration residual (feedforward carries velocity, not acceleration) | **PARTIAL** |
| Target leaves FOV | `beacon_exits_640x480.mp4` handled; retention denominator excludes out-of-FOV frames (§9.2) | **PASS mechanically, metric misleads** |
| Multiple targets | Multiple emitters render and are detected; only one is tracked | **PARTIAL** — PS row 8 makes this optional |
| **Moving decoy** | **FAILS.** `decoy_swarm.toml`: 48.8 px tracking RMS, 33.8 px centroiding, **max 316 px**. The motion term cannot discriminate because the decoy genuinely moves | **FAIL — known, owned by ML** |
| **120 static clutter sources** | **FAILS.** Locks onto clutter and holds it for the whole run (§5.1) | **FAIL — known, misreported in the issues log** |
| Low-contrast target | `lowlight`: 237 px centroiding, 37.8 % loss at 4 s; 3.17 % loss at 20 s. High variance | **FAIL** |
| Target intensity changes | `strobing_target.toml` exists and runs | **UNVERIFIED** (not scored in this audit) |
| Target larger than FOV | `beacon_larger_than_fov.toml` exists | **UNVERIFIED** |

### 10.2 Camera-related

| Attack | Actual behaviour | Verdict |
|---|---|---|
| Camera translation/rotation | Modelled as boresight motion; the mount follows | **PASS** |
| Camera jitter ±20 px/frame (row 23) | Tolerated; tracking settles to 17.17 px vs a derived 16.33 px floor. Handover fails, correctly and with a derivation | **PASS (bound derived)** |
| Motion blur | 8-substep exposure integration | **PASS** — but see A-3 for `camera_hz ≠ 30` |
| Exposure changes | `[camera].exposure_ms` configurable | **PASS** |
| Resolution changes | Direct mode rebuilds the entire workspace; `odd_641x481.mp4` is a committed fixture | **PASS — genuinely good** |
| Frame drops | Decoder **blocks rather than drops**, deliberately, to preserve INV-3 | **PASS for repeatability, fails as a real-time model** (§10.4) |
| Variable FPS | `vfr_640x480.mp4`; container timestamps used | **PASS** |
| Camera rate above 30 Hz | Blur samples platform rate at a hardcoded 30 Hz (A-3) | **BUG** |

### 10.3 Environment-related

| Attack | Actual behaviour | Verdict |
|---|---|---|
| Gaussian noise σ ≤ 20 | Handled; 0.2 px centroiding | **PASS** |
| Salt & pepper 10 % | Handled by the median stage; degrades haze/fog/rain p95 to ~14.8 px | **PARTIAL** |
| Poisson / shot noise | Modelled, variance-summed with read noise in one draw | **PASS** |
| **Atmospheric turbulence** | **Not modelled** — affine contrast/brightness only (§8.2) | **GAP** |
| Background interference | 120-source clutter field, some **brighter than the beacon** by design | **FAIL** (see 10.1) |
| Illumination variation | Atmosphere enum; `all_weather_churn.toml` + §7.4 `set_atmosphere` events | **PASS** |
| Compound disturbances | The sweep crosses weather × impulse × clutter × decoy | **PASS as a method** |

### 10.4 System-related — the weakest column

| Attack | Actual behaviour | Verdict |
|---|---|---|
| CPU overload | **Untested and unhandled.** No load-shedding, no adaptive ROI-on-overrun, no frame skipping | **GAP** |
| **Processing slower than camera FPS** | **Cannot happen by construction, which is itself the defect.** The pipeline is a synchronous pull loop: `source_.next()` / `video_->next()` blocks. If processing is slow the *simulated clock* slows with it. There is no wall-clock deadline anywhere | **GAP — P1** |
| Dropped / delayed frames | Decoder blocks; no drop path exists | **GAP** |
| Initialization failure | Schema rejects 18 bad configs cleanly; missing workspace falls back to the straw-man detector rather than reading past a buffer | **PASS** |
| Tracker divergence | NIS EMA + hit-ratio quality tests kill a noise-fed track; reachability cap on the gate | **PASS — well done** |
| Lost target / reacquisition | Search re-centres on last known position and restarts; 0.109 s mean over the sweep | **PASS** |
| Restart / recovery | GUI `Reset` button; scenario reload rebuilds the world | **PASS** |
| Crash / hang / NaN | `--fuzz-scenarios 40 --seed 4242` → 9,411 frames, 0 failures. ASan+UBSan CI job | **PASS — strong** |
| Memory exhaustion | Arena reserved up front; blob table bounded at 16,384 with deterministic truncation and an overflow report | **PASS — strong** |

**The one that will be asked about.** "What happens when your processing is slower than the camera?" The honest current answer is *"the simulation slows down with it"*, because there is no real-time clock in the loop. For a **virtual** testbed that is a legitimate design choice, and the team should say so plainly — but they must then show the *measurement* that makes it moot: p99 frame time against the 33.3 ms budget. On the video path today that measurement **fails** (95.6 ms p99). See §12.

---

## 11. Experimental Methodology Audit

### 11.1 What is genuinely rigorous

* **Deterministic seeding** with named, non-interfering RNG streams (`Stream::Jitter`, `Stream::PlatformMotion`, `Stream::ClutterLayout`, …). Adding clutter cannot perturb the noise.
* **Multi-seed sweeps** with p95 reporting, not single runs — the sweep file's own header argues this correctly.
* **Isolated axes.** Clutter and decoy are *separate* sweep axes because "the first sweep set static_sources to 0 and left the decoy on, and the clutter-free rows still showed 31 px — all of it the decoy, and invisible as such until it got its own axis." This is real experimental hygiene.
* **Ablations that are one flag apart** — `--set control.k_ff=0`, detector straw-man toggle, IMM on/off, supervisor on/off.
* **Negative fixtures** — 18 bad configs that must be rejected.
* **Fuzzing** across every parameter's legal range with deliberate corner sampling.
* **Retracted hypotheses recorded** rather than quietly re-tuned.

### 11.2 Where it fails

| Problem | Evidence | Severity |
|---|---|---|
| **Sweep runs are 4 s** | `sweeps/weather.toml:29 duration_s = 4.0`. The graded metric changes by 13× between 4 s and 60 s (§8.3) | **P1** |
| **No train/test separation for parameters** | Every reported number comes from scenarios the team wrote. Both benchmark stages are **held-out**. There is no "unseen scenario" protocol | **P1** |
| **Numbers are machine-anonymous** | `issues_till_now.md` §1.2 quotes "2,618 µs" and "382 FPS" with no CPU, clock, or core count. Re-measured here: 4,105 µs / 222 FPS. Neither is wrong; the record is incomplete | **P2** |
| **No statistical reporting beyond p95** | No confidence intervals, no seed-to-seed variance, no n stated per cell in `run.json` | **P2** |
| **Worst-case not separated from transient** | §9.4 | **P1** |
| **Open issues lack reproducing commands** | The file's own rule ("Every open item carries a measurement") is honoured, but the *command* is usually absent. "Low light target loss 86 %" could not be reproduced at any duration tried (3.17 % at 20 s, 37.8 % at 4 s) | **P1** |
| **No end-to-end video regression** | §12.2 | **P0** |

### 11.3 Could another team reproduce the reported results?

| Claim | Reproducible? | Why / why not |
|---|---|---|
| Determinism | **Yes** | `--verify-reproducibility` — though it tests less than it claims (A-1) |
| Sweep compliance matrix | **Yes** | `just sweep`; ran cleanly here in ~3 min on 8 jobs |
| "0.143 px RMSE" | **Yes** | Exactly reproduced (clear, sp=0, clutter=0, decoy=0 cell) |
| "2,618 µs / 382 FPS" | **No** | No machine recorded; got 4,105 µs / 222 FPS |
| "Low light target loss 86 %" | **No** | No command; three different numbers obtainable |
| "Supervisor +15 pts in fog, 0 vs clutter" | **UNVERIFIED** | `just cp123` exists but was not run here |
| BP-2 end-to-end accuracy | **No fixture existed** | This audit had to *construct* one (§12.2) |

---

## 12. Video Path Audit — Benchmark Performance-2 (30 % of marks)

### 12.1 The throughput claim is false

README and `docs/SAT-DESIGN.md` §14.0d both assert:

> *"All of the excess is the SIMULATOR, which does not exist in the video path that 30 % of the marks are scored on."*

Measured on this commit:

| Path | p50 | p95 | p99 | FPS (p50) | FPS (p99) | Wall vs real time |
|---|---:|---:|---:|---:|---:|---:|
| Synthetic (`compliance.toml`) | 4.49 ms | 4.91 ms | 24.8 ms | 222.7 | 40.3 | 0.15× |
| **Video, screen mode, 2000×2000** | **32.49 ms** | **35.55 ms** | **95.60 ms** | **30.8** | **10.5** | **0.96×** |
| Video, screen mode, high-bitrate clip | 22.67 ms | 87.38 ms | 95.60 ms | 44.1 | 10.5 | **1.04×** |

**The video path is ~12× slower than the synthetic path, not faster.** The p99 of 95.6 ms is **10.5 FPS — below spec row 20's 20 FPS minimum.** And at 1.04× real time on a 30 fps clip, the system is not keeping up with the input.

**Root cause: decode is deliberately single-threaded.** `src/engine/decode_thread.cpp:30–57` forces `OPENCV_FFMPEG_THREADS=1` and `CAP_PROP_N_THREADS=1`, for two stated reasons:

1. *"FFmpeg's multi-threaded H.264 decoder deadlocked on the corrupt clip … all four worker threads parked in `futex_do_wait`."* — **Real and serious.** But the correct fix is a watchdog on the corrupt clip, not disabling threading for every clip.
2. *"Its frame-slicing also makes output depend on thread count."* — **Technically incorrect for conforming H.264.** FFmpeg's frame- and slice-threaded H.264 decode produces bit-identical output to the single-threaded decoder for conforming streams; that is a conformance requirement of the codec. And the project already owns the instrument to *prove* it: the INV-3 fingerprint gate.

Note also that CP 8.1's own acceptance criterion — *"a 2000×2000 clip decodes at >60 fps"* (`decode_thread.hpp:4`) — **is not met**, and the class exposes `consumer_waits()` specifically so this can be checked, but nothing reports it.

**Fix (P0):** enable multithreaded decode behind a watchdog + a `--decode-threads` flag defaulting to `min(4, hw_concurrency)`, keep `1` as the fallback for a clip that stalls, and add the scalar/threaded digest comparison to the INV-3 gate. Expected: 32 ms → ~8 ms/frame, p99 comfortably inside 33 ms.

### 12.2 No end-to-end BP-2 fixture exists · **P0**

The single most heavily graded capability — *"Comparison of Centroiding error with predefined error values"* on an evaluator-supplied MP4 — has **no end-to-end measured number anywhere in the repository**:

* `just video <file>` does **not** pass `--truth`, so nothing is scored.
* **No truth CSV is committed for any clip** (`find . -name "*.csv"` returns only control traces).
* The CP 8.7 self-scoring test (`tests/video/test_video_source.cpp:379`) deliberately uses *"a plain intensity-weighted centroid … rather than the perception pipeline"*, runs the **source alone** (not the closed loop), and only in **direct** mode.

So the project can say "the decoder works" and "the perception pipeline works", but has never demonstrated **clip in → full pipeline → `centroid.csv` → RMSE vs truth**.

### 12.3 The number the project is missing — measured here

This audit constructed the missing fixtures. **Everything below is new evidence the team currently does not have.**

**(a) The committed fixture is worthless as a benchmark.** `direct_640x480_30fps.mp4` is a noiseless white box on black; the full pipeline scores **0.0003 px RMSE**. A perfectly symmetric noiseless box has an exact centroid — this measures nothing.

**(b) Noisy direct mode** — 640×480, 10 px beacon, linear motion, ffmpeg `noise=alls=25:allf=t`, CRF 18, 120 frames:

```
CENTROIDING   image RMSE 0.0168 px   p95 0.0301   max 0.0494
LOCK          retention 98.33 %      target loss 1.67 %
SPEED         p50 2.187 ms (457 FPS) | p95 17.3 ms | p99 20.7 ms
```

**(c) Full BP-2 rehearsal** — 2000×2000 screen mode, 10 px beacon, `noise=alls=18`, CRF 20, 180 frames, PTZ loop engaged:

```
CENTROIDING   screen RMSE 0.0950 px  p95 0.1596   max 0.1883
TRACKING      RMS 1.143 px           p95 1.755    max 2.480     <- row 17 (10 px): PASS
ACQUISITION   0.067 s                                            <- row 16 (2 s):  PASS
LOCK          retention 98.89 %      target loss 1.11 %          <- row 18 (5 %):  PASS
HANDOVER      reached at 2.000 s
SPEED         p50 22.67 ms (44.1 FPS) | p95 87.38 ms (11.4 FPS) | p99 95.60 ms
              wall 5.72 s for 5.97 s simulated (1.04x real time) <- row 20 (20 FPS): FAIL at p95
```

**Read this carefully — it is the headline of the whole audit.** On the benchmark worth 30 % of the marks, **accuracy is excellent and would score near the top**, while **throughput fails the spec at p95 and p99**. The team's *actual* BP-2 risk is the opposite of what the documentation says it is.

These three clips and their truth CSVs should be committed and wired into CI immediately (§21 Phase 1).

---

## 13. The 0.85 ms Frame Budget — Why §14.0d Is Wrong, and How to Reach It

The user asked specifically for a solution. Here it is.

### 13.1 What §14.0d actually proves

> 0.85 ms at 3.5 GHz over 640×480 = 2,975,000 cycles ÷ 307,200 px = **9.7 cycles per pixel for the entire frame**. One Gaussian deviate per pixel costs **16 cycles**. Therefore unreachable.

The arithmetic is right. **The scope is wrong in three independent ways.**

**(1) It silently assumes one core.** "9.7 cycles per pixel" is a *per-core* budget presented as an absolute. §15 never says single-threaded. Worse, the design's own decision 11 (`SAT-DESIGN.md:2415`) rejects threading on the grounds *"The frame costs 0.85 ms. Only decode justifies a second thread"* — the budget is used to reject the means of meeting the budget. That is circular. On 8 cores the aggregate budget is **77.6 cycles/px**, against a measured damage chain of 16.

**(2) It attributes the whole gap to the Gaussian.** `--bench-kernels` decomposes the damage chain:

| Component | µs (full frame, this CPU) | Share |
|---|---:|---:|
| `damage_chain` total | 1711 | 100 % |
| …minus the Gaussian (`no gaussian`) | 797 | 47 % |
| …copy only | 66 | 4 % |
| **⇒ Gaussian draw** | **914** | **53 %** |
| **⇒ non-Gaussian arithmetic** | **731** | **43 %** |

**The non-Gaussian arithmetic alone (731 µs) already exceeds §15's entire 550 µs damage-chain line.** The per-pixel `sqrt` for the shot+read variance sum is the obvious target and §14.0d never mentions it.

**(3) It ignores 20 % of the frame that is not physics at all.** The `snapshot` stage is **813 µs** (A-2) and is not in §14.0d's table, not in `issues_till_now.md` §1.3's gap table, and is pure memcpy + byte-at-a-time hash.

### 13.2 The costed path

Baseline: `--stages` on `compliance.toml`, 6 s, this machine → `frame_total` p50 **4,104.7 µs**.

| # | Change | Stage | Now | After | Saving | Risk |
|---:|---|---|---:|---:|---:|---|
| 1 | Hash `frame.pixels` in place with a word-at-a-time (8 B/iter) hash; copy the preview **only** when a GUI consumer is attached | `snapshot` | 813 | **15** | **798** | None — pure removal of a copy |
| 2 | Generalise the existing `pcg_jump` (`sensor_simd.cpp:32–50`) from M⁸ to M^w → **random access** into the stream at any pixel index | `damage_chain` | — | — | — | Low — reuses a solved mechanism |
| 3 | With (2), row-stripe the damage chain across 4 worker threads | `damage_chain` | 1526 | 400 | 1126 | Low — bit-identical *by construction* |
| 4 | Replace the per-pixel `sqrt(a·I + b)` with `rsqrt`+1 Newton step, or a 256-entry LUT when the atmosphere is frame-constant | `damage_chain` | 400 | **230** | 170 | Medium — needs a parity test |
| 5 | Row-stripe `splat` + `background` on the same pool | `splat`,`background` | 270 | **90** | 180 | Low |
| 6 | AVX2 for the van Herk opening, the matched filter and CFAR (already scoped in `issues_till_now.md` §1.4) | `perception` | 1395 | **450** | 945 | Medium — parity tests required |
| 7 | Split the full-frame ROI refresh into N row-bands, one band per frame | `perception` p95 | 18,938 | ~1,500 | — | Low — removes the 13.6× tail |
| | **Total** | | **4,105** | **≈ 790 µs** | | |

**790 µs < 850 µs.** The budget is met, single-frame, on this laptop.

### 13.3 The stronger version — pipeline the simulator

The deeper point is that §15's budget is about *the tracker's real-time behaviour*, and the simulator is not part of the deliverable system. Split them across the existing SPSC-queue pattern already proven by `DecodeThread`: produce frame *N+1* while the tracker processes frame *N*.

| Half | Contents | Cost after §13.2 |
|---|---|---:|
| **Simulator** (4 threads) | world, splat, background, damage chain | ≈ 320 µs |
| **Tracker** (1 thread) | perception, centroid, tracking, supervisor, FSM, control, metrics, snapshot | ≈ 475 µs |

Observed frame period = `max(320, 475)` = **≈ 475 µs**, and the number §15 actually cares about — *the tracker's frame* — is **475 µs, 1.8× inside budget**.

### 13.4 Why this does not break INV-3

This is the part that makes the recommendation safe, and it is why item (2) comes first.

* A counter-based address into the PCG stream makes the noise at pixel *i* a **pure function of (stream, base + i)**. Evaluation order, thread count and scheduling become irrelevant *by construction* — not by convention, and not by a lock.
* The project has already built the hard half: `pcg_jump(inc)` computes the multiply/add constants for 1…8 steps. Brown's jump-ahead generalises to arbitrary *k* in O(log k); precomputing one jump per **row** costs 480 jumps of ~10 operations each, i.e. nothing.
* The sim→track handoff is an ordered, bounded, blocking SPSC queue — structurally identical to `DecodeThread`, whose determinism argument is already written out at `decode_thread.hpp:14–30`.

### 13.5 How to prove it — the validation that should be demonstrated to judges

1. Fix A-1 first: make `--verify-reproducibility` enable the damage chain, jitter, platform motion and a non-clear atmosphere. *Without this the proof below is vacuous.*
2. Add `--sim-threads N`. Assert that the frame fingerprint digest is **identical for N ∈ {1, 2, 4, 8}**, and identical to the pre-change single-threaded digest.
3. Assert scalar-dispatch and AVX2-dispatch digests are identical.
4. Report both numbers in the summary block and in `run.json`:

   ```
   SPEED   tracker frame   p50 0.47 ms   p95 0.61 ms   (design §15 budget 0.85 ms)  PASS
           simulator frame p50 0.32 ms   (4 threads, off the critical path)
           end-to-end      p50 0.49 ms   =  2040 FPS
   ```

5. Retract §14.0d and replace it with §14.0f: *"the budget is reachable; it was a per-core budget being read as an absolute."* **Retracting your own amendment on the basis of a measurement is exactly the behaviour this project's culture is built on, and it is a strong story in the Q&A.**

### 13.6 And the video path

Item (8), separate from the frame budget but the same theme: enable threaded decode behind a watchdog (§12.1). Expected 32 ms → ~8 ms/frame, taking BP-2 from *failing row 20 at p95* to ~125 FPS.

---

## 14. Baseline Analysis

### 14.1 What the project actually compares against

| Baseline | Implemented | Measured against | Verdict |
|---|:-:|---|---|
| Brightest-pixel / straw-man detector | ✅ | CP 4.11 ablation, live-switchable in the GUI | **Excellent** — this is the right baseline and it is *runnable in the demo* |
| Plain CV Kalman vs IMM | ✅ | `tests/control/test_stage10.cpp:1099` asserts `imm.rms_px < 0.85 × cv.rms_px` | **Good** — a quantitative superiority claim, asserted in CI |
| Feedforward on/off (`k_ff`) | ✅ | Swept; the *sign* of the residual is asserted rather than the magnitude | **Good, and subtle** |
| Smith predictor on/off | ✅ | CP 10.4, with a deliberately-wrong plant model | **Good** |
| Supervisor on/off | ✅ | `just cp123`: +15 pts fog, 0 vs clutter | **Good, honest** |
| Priority policy on/off | ✅ | `tracking.priority = false` escape hatch | **Good** |
| Search strategies (4) | ✅ | Strategy benchmark | **Good** |
| **Classical centroid vs Gaussian fit vs bias-corrected CoM** | ✅ | `--calibrate-centroid` S-curve | **Good** |
| **Template matching / optical flow / correlation tracker** | ❌ | — | **GAP** — no *tracking-algorithm* baseline from the wider literature |
| **OpenCV built-in trackers (CSRT, KCF, MOSSE)** | ❌ | — | **GAP** — OpenCV is already a dependency; this is nearly free |

### 14.2 The gap that matters

The project's ablations answer *"does each of our components help?"* — well. They do not answer *"is your architecture necessary at all?"*

A judge will ask: **"Why not just run OpenCV's CSRT tracker on the frame?"** The honest answers exist and are strong, but none is currently *measured*:

1. CSRT/KCF are **appearance** trackers. The beacon is a 10 px featureless blob — there is no texture to correlate. They will fail outright.
2. They produce a **bounding box**, not a sub-pixel centroid. BP-1/BP-2 grade sub-pixel centroiding; a box-centre quantises to ~0.5 px, **2.6× worse than this project's measured 0.19 px**.
3. They have **no state estimate**, so there is no velocity to feed forward — and §2.6.1 shows a reactive loop cannot meet row 17 when the disturbance is 150 % of mount authority.
4. They have **no measurement covariance**, so there is no association gate, so clutter captures them immediately.

**Recommendation (P2, high value/effort ratio).** Add a fourth detector arm, `--detector opencv-csrt` (OpenCV is already linked for video), and publish a four-row table: *straw-man · CSRT · classical · classical+priority*, scored on centroiding RMSE, target loss, and frame time. This converts four assertions into four measurements and is the single cheapest way to answer "why is your complexity justified?"

### 14.3 Complexity added without a demonstrated benefit

Being fair: most of this project's complexity *is* justified by a measurement. Two items are not:

| Component | Justification status |
|---|---|
| **Smith predictor** | Plumbed, tested, and **off in every shipped scenario** (`control.smith` defaults false). Its benefit is demonstrated only in a unit test, never in the compliance matrix |
| **IMM (CV/CA/CT)** | Asserted better than CV in one test. But the figure-8 residual is **16.56 px** — the case IMM's CA mode exists for — so its practical benefit on the graded metric is unshown |
| Supervisor | +15 pts in fog is a real, measured benefit. **Keep** |
| Priority policy | Turned a total clutter failure into … a different total clutter failure (§5.1). **Its measured benefit is currently zero on the scenario it was written for** |

---

## 15. Code & Architecture Audit + Documentation Audit

### 15.1 Code quality

| Dimension | Assessment |
|---|---|
| **Architecture** | **Excellent.** 13 CMake modules with a linker-enforced dependency graph; INV-1 enforced by three independent mechanisms, each self-tested by injecting a violation |
| **Modularity / separation** | **Excellent.** Perception cannot reach truth; the detector is handed `frame.pixels` and nothing else |
| **Naming** | **Excellent.** Tag-typed `Angle2`/`Pixel2`/`Rate2` make unit confusion a compile error |
| **Error handling** | **Good.** `Result<T>`/`Status`; no exceptions in the hot path; schema errors name the offending key and its legal range |
| **Logging** | **Good** for artefacts. **Weak** for diagnosis — no structured run log, no event log for "why did the track drop at frame 412?" |
| **Configuration** | **Excellent.** One typed schema, 18 negative fixtures, `--set` overrides validated by the same schema (modulo A-6) |
| **Reproducibility** | **Good mechanism, hollow gate** (A-1) |
| **Testability / tests** | **Excellent.** 20 suites, 47 s, plus fuzzing, sanitizers, negative fixtures, and 14 awkward video clips |
| **Performance** | **Good and improving.** 16× achieved. Remaining work is scoped (§13) |
| **Resource management** | **Excellent.** Arena allocation, bounded tables with deterministic truncation, armed allocation trap |
| **Numerical stability** | **Good.** `-ffp-contract=off`, `-fno-fast-math`, `/fp:precise`; squared-distance gating avoids `sqrt` |
| **Thread safety** | **Good** — only one extra thread, with a written determinism argument |
| **Determinism** | **Good** (modulo A-1) |
| **Maintainability** | **Good**, with a caveat: the comment density is very high and some comments have gone stale (A-7, A-8). A comment that cites a non-existent file is worse than no comment in a project whose credibility rests on checkable citations |

### 15.2 Debt ledger

| Kind | Items |
|---|---|
| **Technical debt** | A-3 hardcoded 30 Hz · A-4 encoder bypass · A-5 stale build hash · A-6 `--set` string handling · AVX2 missing on 3 kernels · no ROI-refresh amortisation (perception p95 13.6× budget) |
| **Architectural debt** | No absolute attitude reference (§8.3) · no real-time clock / deadline model (§10.4) · sim and tracker not separable (§13.3) · single-threaded decode (§12.1) · no ONNX runtime in the C++ dependency graph (§22) |
| **Demo debt** | `baseline.toml` is the default *and* broken (§5.1) · GUI opens clean while its own spec table claims full-spec noise (§16.3) · `--set` string bug on the most likely live command |
| **Research debt** | No turbulence model · no external tracking baseline (§14.2) · decoy and clutter discrimination unsolved and deferred to unimplemented ML |
| **Documentation debt** | Technical Report not started (**mandatory deliverable**) · README weather claim contradicted by `just sweep` (§9.6) · `issues_till_now.md` claims disproven (§6.2) and omissions (§6.5) · numbers quoted without a machine |

### 15.3 Documentation audit

| Claim | Where | Implementation demonstrates? |
|---|---|---|
| "Stages 0–10 and 12–15 complete. All five ★ gates pass." | README | **Mostly.** Stage 13 (acquisition) does not meet its own goal on the default scenario |
| "0.143 px RMSE / 1.67 % loss / 0.067 s / 251 FPS" | README | **Yes**, and exactly reproduced — for **1 of 40** sweep cells, at 4 s |
| "Through every weather mode … 0.14–3.4 px and 1.7 %" | README | **No** — §9.6 |
| "the frame is 2.62 ms — 382 FPS" | README | **Not on this machine** (4.10 ms / 222 FPS), and it **includes an 813 µs fingerprint copy** |
| "All of the excess is the SIMULATOR, which does not exist in the video path" | README, §14.0d | **No — the video path is 12× slower** (§12.1) |
| "0.85 ms … is not reachable" | README, §14.0d | **The argument is unsound** (§13.1) |
| "Two conditions do not pass, and they are the same condition twice" | README | **Understated** — the default scenario also fails, and low light is a third condition |
| "known-good fallback (CP 15.3)" | `docs/DEMO.md:172` | **No** — §5.1 |
| "every §12 panel complete" | `issues_till_now.md` | **Yes** — verified in the captured screenshot |
| "Everything in scope is complete" | `issues_till_now.md` §3 | **No** — the Technical Report is a mandatory PS deliverable and is not started |
| Nine invariants | README | **8 fully verified, INV-3's gate is hollow** (A-1) |

**What the doc tooling does and does not check.** `tools/check_docs.py` passes and verifies *"every recipe, path, link and cross-reference in 9 documents resolves"* — link integrity. It does **not** check numeric claims. Every contradiction above is a *number*, which is precisely the class the tool cannot see. **Recommendation:** extend it to parse the claim tables in README/RESULTS and re-derive each figure from a committed `run.json`, failing CI on drift. That turns the documentation from *asserted* into *generated*, which is a differentiator in its own right.

---

## 16. Demo-Risk Analysis

### 16.1 Risks, ranked by probability × damage

| # | Risk | Probability | Damage | Mitigation |
|---:|---|---|---|---|
| 1 | **Judge types `just gui` or `./sat-tracker --gui` and sees the broken default** | **Certain** — it is the default | **Severe** — 913 px error, 1778 false tracks/min, red compliance panel, in the first 10 seconds | Change the default to `compliance.toml`; keep `baseline.toml` as an explicitly-labelled "hard case" |
| 2 | **Judge runs the video benchmark and sees 30 FPS / 10.5 FPS p99** | **High** — BP-2 is 30 % of marks | **High** — a literal row-20 failure | §12.1 threaded decode |
| 3 | **Judge asks for the Technical Report** | **Certain** — mandatory deliverable | **High** | Write it |
| 4 | **Judge types `--set atmosphere.mode=fog` live and it errors** | **Medium** | **Medium** — looks broken on stage | A-6 |
| 5 | GUI shows "Centroiding RMSE 0.084 px" beside "row 21 salt & pepper 10 %" while running **clean** | **High** | **High** — reads as an unsupported claim made by the product itself | §16.3 |
| 6 | Judge asks "what is your acquisition time from a cold start?" | **Certain** | Medium | Answer is prepared and correct (geometric bound); **rehearse it** |
| 7 | Judge adds clutter with the GUI slider | **High** — the slider is right there | **High** — the system fails visibly | Pre-empt it: *show* the failure yourself, with the ablation, and name it as the ML problem |
| 8 | Judge asks about turbulence | **High** — ISRO panel | Medium | §8.2; ship an AoA model |
| 9 | Long startup / build on the demo machine | Low | Medium | `dist/` tarball exists; `just smoke` is 3 s |
| 10 | Non-determinism on stage | Very low | Low | INV-3 is genuinely solid |

### 16.2 What is already demo-safe

* No network requirement, no GPU requirement, no model download.
* `SAT_WITH_GUI=AUTO` and a `linux-no-gui` CI job — the app runs headless if GLFW is missing.
* `--has-video` probes decoder availability and exits cleanly.
* Scenario presets exist; Pause / Step / Reset all work.
* Screenshots regenerate from the real dashboard via `--gui --shot`, so figures cannot drift from the product. **This is a very good idea and worth saying out loud to judges.**
* Startup is instant; `just smoke` completes in ~3 s.

### 16.3 The GUI consistency defect · **P1**

`src/gui/dashboard.cpp:181` opens on the **Clean** preset: gaussian σ → 0, salt & pepper → 0, Poisson off, defects off, clutter 0/0. The *decision* is good (start clean, dial damage up — it is also §14.1's running order). The *presentation* is not:

* the same window's spec table (`dashboard.cpp:1258`) reads rows 21/22/23 **from the scenario file**, so it prints "salt & pepper 10 %", "read noise sigma 20", "jitter 20 px/frame";
* the clutter row of the *same table* reads **live** ("0 sources, 0 decoy");
* the compliance panel then prints **"Centroiding RMSE (image) 0.084 px"**.

A judge reading that window concludes the system achieves 0.084 px under 10 % impulse noise and σ=20 read noise. **It does not** — measured under those conditions it is 0.200 px, and at p95 across haze/fog/rain it is ~14.8 px.

**Fix (small, high-value):** make every row of the spec table read the **live** chain, and put a persistent banner over the damage group — `DAMAGE OVERRIDDEN: CLEAN — click "Full spec" to restore the scenario` — so the state is never ambiguous.

---

## 17. SIH Judge Attack Questions

For each: can the project answer it **today**, with evidence?

### 17.1 Problem understanding

| Question | Can answer? | Evidence / gap |
|---|---|---|
| What is the real-world problem? | **Yes, excellently** | Coarse alignment of a narrow FSOC beam; the design opens with it |
| Why is it difficult? | **Yes — best answer in the project** | Disturbance (40 px/frame) exceeds mount authority (26.7 px/frame) = 150 %. Therefore reactive control cannot work; prediction is mandatory. **Lead with this** |
| Why isn't a conventional tracker sufficient? | **Argued, not measured** | §14.2 — no CSRT/KCF comparison exists |
| What part of the problem are you solving? | **Yes** | Coarse alignment only; handover to fine pointing is explicitly modelled with a quadrant cell |

### 17.2 Technical depth

| Question | Can answer? | Evidence / gap |
|---|---|---|
| What happens when the target disappears? | **Yes** | Confirmed → Coasting → Search; search re-centres on last known position; NIS/hit-ratio quality tests kill a noise-fed track |
| What happens when the camera moves? | **Yes** | Disturbance is applied at the boresight; the tracker only ever sees the commanded angle |
| How do you separate target motion from camera motion? | **Yes, and it is subtle** | Reconstructing through the commanded boresight cancels platform drift algebraically; the priority policy's motion term uses a **per-frame median over a 32-frame sliding window** with an ego-motion subtraction. `issues_till_now.md` §2.1 records three wrong versions first — **this is a great Q&A story** |
| How do you quantify pointing error? | **Yes** | `tracking_error_px` = \|true boresight − true beacon\| in screen px, separate from centroiding (INV-6) |
| What happens under turbulence? | **Weak** | Only affine contrast/brightness. Jitter is a crude AoA proxy. **§8.2 — fix before the event** |
| What happens when the tracker loses lock? | **Yes** | Re-acquisition measured at 0.109 s mean over 200 runs |
| What happens when FPS drops? | **No** | §10.4 — no deadline model. Honest answer: "it is a virtual testbed; the sim clock slows." Then show p99 — which **fails on video today** |
| What happens when the system is overloaded? | **No** | No load-shedding path |
| Why is your `q` that value? | **Yes, outstandingly** | Derived from the target's analytic max acceleration; two wrong derivations are recorded with their symptoms |
| Why a linear Kalman filter and not an EKF? | **Yes** | The equidistant projection makes the measurement equation exactly linear |

### 17.3 Validation

| Question | Can answer? | Evidence / gap |
|---|---|---|
| What is your baseline? | **Partial** | Internal ablations yes; external tracker no |
| What is your worst case? | **Partial** | `max` is reported, but it is transient-dominated (§9.4) and there is no named worst-case scenario |
| How many experiments? | **Yes** | 200-run sweep, 40 cells × 5 seeds; 40-scenario fuzz; 20 CI suites |
| Maximum tolerated disturbance? | **Yes** | Swept: row 17 lost at 600 px/s drift, saturation at 800; asserted as an **ordering**, not constants |
| Acquisition time? | **Yes, both figures, correctly separated** | 0.067 s in-view; cold is a derived geometric bound |
| Tracking error? | **Yes, with the floor derivation** | 17.17 px with row-23 jitter; 3.357 px without |
| Lock retention? | **Yes — but the denominator misleads** | §9.2 |
| Can you reproduce it? | **Yes** | `--verify-reproducibility`, CI −O0/−O2 — but the gate is hollow (A-1) |

### 17.4 Real-world deployment — the weakest section

| Question | Can answer? | Gap |
|---|---|---|
| What hardware would this run on? | **No** | No target platform named, no CPU/memory budget, no embedded story |
| What is your latency budget? | **Partial** | Compute time is measured; **sensor-to-command latency is not** (§8.4) |
| What sensor assumptions? | **Partial** | Mono FPA, global shutter, linear projection, no radiometric calibration. Not collected in one place |
| What if camera characteristics differ? | **Partial** | FOV/resolution are configurable; **no lens distortion, no PSF, no intrinsics calibration path** |
| What happens when simulation assumptions don't hold? | **No** | No sim-to-real gap analysis exists |
| **"Where is the AI?"** | **No** | §3.5. Prepare a precise answer: what is built, what the interface is, what data it needs, why it was deferred, and what it will buy (the decoy/clutter case, with the 48.8 px measurement as the motivating number) |

### 17.5 Engineering

| Question | Can answer? | Evidence |
|---|---|---|
| Why this architecture? | **Yes, excellently** | Every module has a written rationale; decisions are numbered |
| What if a module fails? | **Partial** | Perception falls back to the straw man; supervisor degrades gracefully. No general fault model |
| How are parameters configured? | **Yes** | One typed schema, `--set` overrides, 18 negative fixtures |
| How are experiments reproduced? | **Yes** | `Justfile` recipe per figure; `check_docs.py` verifies every recipe exists |
| How are failures logged? | **Weak** | `run.json` carries outcomes, not causes. No event log |
| How would you debug a failed run? | **Partial** | `--trace` writes a per-frame control trace (contains truth, never graded). Good, but there is no "why did the track drop" narrative |

---

## 18. Differentiation / USP Audit

### 18.1 Is this a generic "AI + CV + tracking" project? **No.**

Most SIH entries for this PS will be: OpenCV + YOLO-ish detector + a bounding box + a PID on the box centre, in Python, with a Tkinter window. This project is categorically different, and the differences are real and measurable.

| Differentiator | Demonstrable today? | Strength |
|---|---|---|
| **Enforced invariants** — INV-1 (no truth leak) enforced by the **linker**, a configure-time closure walk, *and* a source scan, each self-tested by injecting a violation | **Yes** — `just` targets exist and fail loudly | **Very high.** Almost no team will have this. It is the answer to "how do we know you're not cheating?" |
| **Bit-exact reproducibility as a CI gate** | **Yes** (fix A-1 first) | **High** |
| **Zero steady-state heap allocation**, trapped in Debug — and it **found four real defects** | **Yes** | **High** — this is embedded-grade discipline |
| **Analytic motion algebra** giving exact position *and* the max-accel/max-speed bounds the filter's `q` is derived from | **Yes** | **High** — deeply non-obvious |
| **Measured centroid S-curve with a compiled-in bias table** | **Yes** — `--calibrate-centroid` | **High** — 0.19 px is a serious number |
| **The disturbance-exceeds-authority derivation** | **Yes** | **High** — it reframes the whole problem |
| **Coarse→fine handover modelled with a quadrant cell** | **Yes** | **High** — genuine FSOC domain knowledge |
| **Self-adaptive supervisor** switching detector/centroider/CFAR-k/gains on **observables only** | **Yes**, +15 pts in fog | **Medium-high** |
| **A failure ledger with retracted claims** | **Yes** | **High** in Q&A — it signals engineering maturity |
| **Screenshots regenerated from the live product** | **Yes** | **Medium** |
| **Scenario fuzzer over the full legal parameter space** | **Yes** | **Medium-high** |

### 18.2 What is *not* differentiated

* The detection chain (median → top-hat → matched filter → CFAR → grouping) is **textbook**. Well executed, not novel.
* The tracker (KF/IMM, chi-square gating) is textbook.
* The control law (PID + FF + Smith) is textbook.

That is fine — **the differentiation is the rigour, not the algorithms** — but the team must say it that way. Claiming algorithmic novelty would invite an easy rebuttal.

### 18.3 The strongest available differentiator is not yet built

**"Digital twin with a provable evaluation harness."** The pieces are 80 % present; three additions complete it and each is demonstrable:

1. **An attitude-sensor model** (noisy gyro + periodic absolute fix) making platform drift observable (§8.3). Turns an unbounded metric into a bounded one and gives a deployment answer.
2. **A turbulence model** — Kolmogorov-spectrum angle-of-arrival jitter with a configurable Fried parameter, plus log-normal scintillation (§8.2). Converts the most likely ISRO-panel attack into a demo highlight.
3. **Documentation generated from measurements** rather than asserted (§15.3), so every number in the README is regenerated by CI and cannot drift.

Together those say: *"this is not a tracker demo, it is a **validated testbed** for developing PAT algorithms"* — which is literally what the PS asks for ("A software based virtual camera tracking provides an inexpensive and accessible platform for **algorithm development and learning**").

---

## 19. P0 / P1 / P2 / P3 Issues

### P0 — Blocking

| ID | Issue | § |
|---|---|---|
| **P0-1** | `baseline.toml` — the default for `--gui`/`--headless` and the documented demo fallback — fails totally (913 px, 0 frames scored, 1778 false tracks/min) | 5.1 |
| **P0-2** | FOV containment — the PS's actual objective — is not a reported metric; `target_loss` normalises over in-FOV frames and hides a 95.6 %-of-run failure as "7.59 %" | 9.2 |
| **P0-3** | Video path fails spec row 20 at p95/p99 (11.4 / 10.5 FPS) and runs at 1.04× real time; the README claims the opposite | 12.1 |
| **P0-4** | No end-to-end BP-2 fixture: no truth CSV, no scored clip-in→centroid.csv-out test. 30 % of marks unmeasured | 12.2 |
| **P0-5** | Technical Report (mandatory PS deliverable) not started | 3.4 |

### P1 — Critical

| ID | Issue | § |
|---|---|---|
| **P1-1** | INV-3 gate runs with the damage chain, jitter, platform motion and atmosphere all disabled; the AVX2 path is never exercised by it | 7.1 A-1 |
| **P1-2** | `snapshot` = 813 µs (27× budget, 24 % of the frame) in headless; absent from every performance table | 7.1 A-2 |
| **P1-3** | Perception reconstructs through `true_position()` not the encoder; a 36.67 px LSB changes the result by zero | 7.1 A-4 |
| **P1-4** | README weather claim contradicted by `just sweep` (lowlight 237 px / 37.8 %); means quoted where the tool grades p95 | 9.6 |
| **P1-5** | `issues_till_now.md` §2.4 "the policy correctly refuses to commit" is false (1777 Confirmed frames on clutter) | 6.2 |
| **P1-6** | Screen-frame centroiding error grows without bound (44→295→589 px) and is printed under "CENTROIDING (graded, 60 %)" with no caveat | 8.3 |
| **P1-7** | Perception p95 is 18,938 µs — 13.6× budget — because the ROI refresh frame pays the full-frame cost in one frame | 6.1, 13.2 |
| **P1-8** | GUI opens clean while its own spec table claims full-spec noise beside a 0.084 px result | 16.3 |
| **P1-9** | RMS is transient-dominated (RMS 3.357 > p95 0.973); no steady-state figure | 9.4 |
| **P1-10** | No real-time deadline model; "processing slower than camera" cannot occur and is untested | 10.4 |
| **P1-11** | Sweep runs 4 s per cell while the graded metric changes 13× between 4 s and 60 s | 9.5, 11.2 |

### P2 — Important

| ID | Issue | § |
|---|---|---|
| **P2-1** | No turbulence model (no AoA jitter, no scintillation) | 8.2 |
| **P2-2** | No external tracking baseline (CSRT/KCF/template) | 14.2 |
| **P2-3** | No absolute attitude reference → world-frame estimate drifts unboundedly | 8.3 |
| **P2-4** | Blur samples platform rate at a hardcoded 30 Hz | 7.1 A-3 |
| **P2-5** | Build provenance hash stale (`d563657` vs HEAD `803733d`) | 7.1 A-5 |
| **P2-6** | `--set` rejects unquoted strings with a misleading, mis-prefixed error | 7.1 A-6 |
| **P2-7** | PS asks for **average** tracking error; only RMS/p95/max are reported | 9.3 |
| **P2-8** | No CPU/memory usage in the performance log | 9.1 |
| **P2-9** | End-to-end latency never measured (only compute time) | 8.4 |
| **P2-10** | `models/centroidnet_v1.json` claims a 0.274 px val RMSE with no disclaimer, while ML is declared unbuilt | 6.5 |
| **P2-11** | Smith predictor and IMM have no demonstrated benefit on the graded metric | 14.3 |
| **P2-12** | No structured failure/event log for post-hoc diagnosis | 15.1 |

### P3 — Polish

| ID | Issue | § |
|---|---|---|
| **P3-1** | `pipeline.hpp:422` cites a non-existent test file | 7.1 A-7 |
| **P3-2** | `dashboard.cpp:177` comment factually stale (straw man vs classical) | 7.1 A-8 |
| **P3-3** | `run.json` writes `world.edge_behaviour = 0` as an integer, not a name | 5 |
| **P3-4** | `check_docs.py` verifies links but never numbers | 15.3 |
| **P3-5** | Performance numbers quoted without a machine specification | 11.2 |
| **P3-6** | `docs/manual/` and `docs/report/` are empty `.gitkeep` directories | 3.4 |
| **P3-7** | `issues_till_now.md:267` says the camera sees "9.8 %" of the screen; every other document and the arithmetic say **7.68 %** ((640x480)/(2000x2000)) | 15.3 |

---

## 20. Detailed Fix Recommendations

Full template for the P0s and the most consequential P1s. The rest are specified in §19 and §21.

### P0-1 — The default scenario is a total failure

* **Problem.** `baseline.toml` is the default for `--gui` and `--headless`, the first picker entry, the README hero screenshot, and `docs/DEMO.md`'s "known-good fallback". It scores 0 centroiding frames, 913 px tracking RMS and 1778 false tracks/min.
* **Why it matters.** It is the first thing a judge sees. Functional Verification is 20 % and criterion 2 is literally "Operational success".
* **Evidence.** §5.1 table; `docs/DEMO.md:172`; `src/app/main.cpp:162`; `src/app/headless.cpp:344`.
* **Severity.** Critical. **Impact on SIH:** direct loss on Functional Verification and on first impression for everything after.
* **Fix.** Make `compliance.toml` the default for `--gui` and `--headless`. Rename `baseline.toml` → `scenarios/hard/cold_start_in_clutter.toml`, keep it in the picker under a "Hard cases" group, and put its measured failure *in its own header*. Add `scenarios/spec_defaults.toml` = today's `baseline.toml` with `initial_px` fixed in view, as the literal "specification defaults" reference.
* **Implementation.** `main.cpp:162`, `headless.cpp:344`, `dashboard.cpp:293`, `Justfile:345,753`, `docs/DEMO.md:172`, README.
* **Validation.** `just smoke` and `just headless` with no arguments must report row-16/18/19/20 PASS and non-zero centroiding frames. Add a CI assertion that the **default** scenario scores > 0 centroiding frames — this class of defect can then never recur.
* **Priority. P0.**

### P0-2 — Report FOV containment as a first-class metric

* **Problem.** Retention and target loss are normalised over in-FOV frames, so a run with the beacon visible 4.4 % of the time reports 7.59 % target loss.
* **Why it matters.** PS 26169's objective is to *"locate and maintain the remote terminal within its camera FOV"*. That quantity is not reported.
* **Evidence.** `src/metrics/collector.cpp:196–199`; §9.2's measurement.
* **Severity.** Critical (metric validity). **Impact on SIH:** a judge who spots it concludes the metrics flatter the system — which poisons every other number.
* **Fix.** Add to `Metrics`, `run.json` and the summary:
  * `fov_containment_frac = frames_in_fov / frames_total`
  * `fov_containment_post_acq = frames_in_fov / frames_after_first_lock`
  * `target_loss_post_acq = 1 − frames_held / frames_after_first_lock`
  Print FOV containment **above** retention. Grade row 18 on `target_loss_post_acq`, stating the definition inline.
* **Implementation.** `src/metrics/collector.{hpp,cpp}`, `run_report.cpp`, `docs/METRICS.md`, GUI compliance panel.
* **Validation.** On `baseline.toml` the summary must read `time on target 4.39 %`. Add a test asserting that a run with the beacon out of FOV for >50 % of frames cannot report <5 % target loss on the post-acquisition definition.
* **Priority. P0.**

### P0-3 / P0-4 — Make BP-2 measurable, then make it fast

* **Problem.** No scored end-to-end video fixture; and the video path fails row 20 at p95/p99.
* **Evidence.** §12.1, §12.2, §12.3.
* **Fix, in order.**
  1. Commit the three fixtures from §12.3 (noiseless direct, noisy direct, 2000×2000 noisy screen) with their truth CSVs, generated by `tools/make_test_videos.sh` from analytic expressions so truth is exact by construction.
  2. Add `just bp2` running all three through the **full pipeline** with `--truth`, printing the graded table.
  3. Add a CI regression asserting screen-mode RMSE < 0.2 px, retention > 95 %, **and p99 frame time < 33.3 ms**.
  4. Enable threaded decode behind a `--decode-threads` flag and a watchdog; keep `1` as the fallback for a clip that stalls; add the corrupt clip to the watchdog test.
* **Validation.** `just bp2` green, with p99 < 33.3 ms. Fingerprint digest identical at 1 and 4 decode threads.
* **Priority. P0.**

### P1-1 — Make the INV-3 gate exercise what it claims to protect

* **Problem.** The reproducibility gate runs with the damage chain disabled, so the AVX2 path is never covered.
* **Evidence.** `src/app/verify_repro.cpp:47–58`; `src/degrade/sensor.hpp:84`; identical digests across seeds for 4 of 5 scenarios.
* **Fix.** Enable the sensor model, jitter, platform motion and a non-clear atmosphere in the five built-in scenarios; add a sixth with the scalar dispatch forced; assert scalar ≡ AVX2; extend CI to compare digests across dispatch paths as well as across −O0/−O2.
* **Validation.** Seeds must now produce *different* digests per scenario (proving noise is live), and repeats must still be identical. Deliberately break one AVX2 lane and confirm the gate goes red.
* **Priority. P1** — and it is a **prerequisite** for the §13 threading work.

### P1-2 — Remove the snapshot copy from the frame

* **Fix.** Hash `frame.pixels` directly with a word-at-a-time hash; copy `preview` only when `snapshots_.has_reader()`. Add `snapshot` and `frame_acquire` to `issues_till_now.md` §1.3's gap table.
* **Validation.** `--stages` shows `snapshot` < 30 µs; the frame fingerprint is **unchanged** (hashing the same bytes from a different buffer must produce the same digest — assert it).
* **Priority. P1.**

### P1-6 — Report the screen-frame error honestly

* **Fix.** Add a `centroid_error_boresight_px` column; in the summary, print the screen-frame figure with the one-line explanation from §8.3 and a derived expectation `E[|D|]` so a reader can see the measured value matches the disturbance. Update `docs/METRICS.md` and the GUI plot legend.
* **Validation.** On `compliance.toml` at 60 s the summary must show screen RMSE ≈ E[|D|] = 589 px **and say so**, so the number reads as a *consistency check on the simulator* rather than a detector failure.
* **Priority. P1.**

### P1-7 — Amortise the ROI refresh

* **Fix.** Replace the "every Nth frame, process the whole frame" refresh with "every frame, process the ROI plus one of N row-bands, rotating". Same total coverage, `1/N` of the peak cost.
* **Validation.** `--stages` p95/p50 ratio for `perception` drops below 2×; target re-detection after an induced dropout is unchanged.
* **Priority. P1.**

---

## 21. Improvement Roadmap

### Phase 1 — Correctness *(≈ 3 days; nothing else should start first)*

| Task | Files | Outcome | Validation |
|---|---|---|---|
| Swap the default scenario; reorganise `scenarios/hard/` | `main.cpp:162`, `headless.cpp:344`, `dashboard.cpp:293`, `Justfile`, `DEMO.md`, README | `just gui` opens on a passing run | CI: default scenario scores >0 centroiding frames |
| FOV containment metric | `collector.{hpp,cpp}`, `run_report.cpp`, `METRICS.md`, GUI | The PS's objective is reported | Test: >50 % out-of-FOV ⇒ cannot report <5 % loss |
| Fix A-3 (hardcoded 30 Hz) | `pipeline.cpp:659` | Correct blur at any `camera_hz` | Run `camera_hz=60` with circular platform motion; blur direction matches the analytic rate |
| Fix A-4 (encoder bypass) | `pipeline.cpp:648` | Encoder model actually applies | Sweep LSB 20→4000 µrad; centroiding error must now **grow** monotonically |
| Fix A-5 (stale hash), A-6 (`--set` strings), A-7, A-8 | `CMakeLists.txt`, `sweep_spec.cpp`, comments | Provenance correct; live demo commands work | `--version` == `git rev-parse` after a rebuild with no reconfigure |
| Correct the README/issues claims | README, `issues_till_now.md` | Documentation matches measurement | `just sweep` output reproduces every quoted figure |

### Phase 2 — Architecture *(≈ 5 days)*

| Task | Files | Outcome | Validation |
|---|---|---|---|
| Harden the INV-3 gate (P1-1) | `verify_repro.cpp`, `reproducibility.yml` | The gate covers the noise chain and both dispatch paths | Digests differ by seed; scalar ≡ AVX2 |
| Counter-based PCG addressing | `core/rng.hpp`, `degrade/sensor_simd.cpp` | Noise is a pure function of (stream, index) | Bit-identical to the sequential stream over 10⁶ draws |
| Split sim / tracker across an SPSC queue; `--sim-threads N` | `engine/pipeline.cpp`, new `engine/sim_thread.*` | Simulator off the critical path | Digest identical for N ∈ {1,2,4,8} |
| Remove the snapshot copy (P1-2) | `engine/snapshot.hpp`, `pipeline.cpp:1145` | −798 µs | Fingerprint unchanged |
| Threaded decode + watchdog | `engine/decode_thread.cpp` | BP-2 32 ms → ~8 ms | Digest identical at 1 and 4 threads; corrupt clip does not hang |

### Phase 3 — Robustness *(≈ 5 days)*

| Task | Detail | Validation |
|---|---|---|
| **Turbulence model** | Kolmogorov-spectrum AoA jitter with configurable Fried parameter r₀; log-normal scintillation with configurable σ²ᵢ; both as a new `[atmosphere.turbulence]` block | At r₀ = 5 cm, AoA PSD must follow f^(−11/3) over the resolved band, asserted by a test |
| **Real-time deadline model** | `--realtime` mode: a wall-clock budget per frame; on overrun, drop the frame and **record it**; report `deadline_misses` | Inject a 50 ms stall; assert the run completes, reports the miss, and re-acquires |
| **Load shedding** | On predicted overrun, shrink the ROI / skip the median stage / fall back to the straw man, and log the decision | Under an artificial 4× slowdown, row 20 still holds and the supervisor timeline shows the decisions |
| **Attitude sensor (optional)** | `[platform.gyro]` with bias + random walk, plus a periodic absolute fix; estimate `D` in the filter | Screen-frame centroiding error becomes **bounded** at 60 s instead of 589 px |
| **Named worst cases** | Promote the six adversarial scenarios into the graded sweep as their own axis | The compliance matrix includes an "adversarial" block |

Concretely, replacing "improve robustness": *inject camera rotation of 0.5–3.0° at 2–15 Hz across 5 amplitudes × 4 frequencies; measure tracking RMS in the 2 s before and the 2 s after onset; record time-to-reacquire; report the distribution over 20 deterministic seeds per cell (400 runs) as p50/p95/max with the seed list committed.*

### Phase 4 — Evaluation *(≈ 4 days)*

| Task | Validation |
|---|---|
| Commit the three BP-2 fixtures + truth; add `just bp2` and its CI regression (P0-4) | Green, p99 < 33.3 ms |
| Raise sweep duration 4 s → 30 s; re-baseline the compliance matrix | Every README figure regenerated from the new matrix |
| Separate transient from steady state in every error metric (P1-9) | RMS ≥ p95 can no longer occur without an explicit "transient" label |
| Add `mean_px` (P2-7), `deadline_misses`, CPU/peak-RSS to `run.json` (P2-8) | PS Performance-Log deliverable fully satisfied |
| Add CSRT/KCF/template baselines as detector arms (P2-2) | Four-row comparison table in `RESULTS.md`, regenerated by CI |
| Make `check_docs.py` verify **numbers** against a committed `run.json` (P3-4) | CI fails on documentation drift |

### Phase 5 — ML integration *(≈ 7 days; see §22)*

### Phase 6 — Differentiation *(≈ 4 days)*

Turbulence demo · attitude-sensor demo · generated documentation · the invariant self-tests promoted into a demo step (§24 step 7).

### Phase 7 — Demo engineering *(≈ 2 days)*

Fix P1-8 (GUI consistency banner + live spec table) · add a scripted `just demo` that walks the §24 sequence with pauses · add a one-key "restore spec defaults" in the GUI · rehearse §17's answers · pre-record a fallback video.

### Phase 8 — Final SIH hardening *(≈ 3 days)*

Write the Technical Report (P0-5) · produce the User Manual as a submitted PDF · build and **test** the Windows binary on a clean machine · produce the 3–5 min demo video · dry-run on the demo hardware with no network.

---

## 22. ML Integration Requirements

Assessed as an interface, not as an implementation.

### 22.1 What exists today

| Artefact | State |
|---|---|
| `ml/` — datasets, training loops, export, evaluation, parity tests (870 LOC Python) | Scaffolding complete and coherent |
| `ml/models/{centroid,candidate}.py` | Architectures defined |
| `ml/inference.py` | ONNX session **with a mandatory classical fallback** on load failure — INV-7-shaped, correct |
| `docs/SAT-ML.md` (1,026 ll) | Companion spec |
| `docs/models/*.md` | Model cards, **correctly labelled** "PRELIMINARY — trained on synthetic dummy data" |
| `models/centroidnet_v1.json` | Training metrics from **dummy shards**, **no disclaimer in the file** (P2-10) |
| `Stream::DatasetSampling` reserved in `core/rng.hpp:232` | Good foresight |
| `tools/make_dummy_shards.py` | Fixture generator that explicitly refuses to impersonate the real one |

### 22.2 Blocking prerequisites

| # | Requirement | Status | Note |
|---:|---|---|---|
| 1 | **`--gen-dataset`** | **NOT IMPLEMENTED** (`main.cpp:246` lists it as "still to come") | **The hard blocker.** No training data can exist without it |
| 2 | **ONNX Runtime in the C++ dependency graph** | **ABSENT** from `vcpkg.json` | Adds a dependency to a build that is currently clean on 5 CI configs; needs a `SAT_WITH_ML=OFF` path mirroring `SAT_WITH_GUI` |
| 3 | **Split discipline by run, not by frame** | Designed (`verify_split_disjoint`) | Frames within a run are heavily correlated; a frame-level split would leak. The design already gets this right |
| 4 | **Determinism of inference** | **UNRESOLVED** | ONNX Runtime is not bit-exact across versions/providers. `SAT-DESIGN.md` §11.4's answer — hash *discrete decisions* at a declared tolerance, not raw outputs — is the right one and must be implemented before ML lands, or INV-3 breaks |
| 5 | **Latency budget** | Declared 0.02 ms/frame for CentroidNet; **not measured** | At 236,306 parameters on a 15×15 patch this is plausible for one call/frame; `CandidateNet` at N calls/frame is the risk |

### 22.3 What the current architecture gets right for ML

* **The seams already exist.** `Strategy.centroider` and `Strategy.perception` are enums the supervisor already switches at runtime; adding `Learned` is an enum case, not a refactor.
* **INV-7 is enforced by CI** (`linux-no-gui` generalises the pattern), so an ML failure can never take the system down.
* **The classical fallback is already the shipped path**, so ML is strictly additive.
* **Detections carry `snr`, `size_est_px`, `area_px`, `centroid_sigma_est`** — exactly the scalars `CentroidNet`/`CandidateNet` consume.
* **The arena and INV-4** mean an ML buffer must be pre-allocated, which is the right constraint.
* **Priority scoring is already a weighted score in `[0,1]`**, so `CandidateNet` can replace one term without touching the policy.

### 22.4 What would make ML integration hard — fix these first

| Risk | Detail | Mitigation |
|---|---|---|
| **INV-3 vs ONNX** | Runtime dispatch inside ORT will break bit-exactness | Implement §11.4's tolerance-hashing **before** ML lands; make the fingerprint hash the discrete decision (which candidate was chosen) plus the centroid quantised to a micro-pixel |
| **INV-4 vs ORT** | ORT allocates per `Run()` | Pre-allocate IO bindings; extend the Debug trap over the inference call |
| **The frame budget** | §13's budget has no ML line. `CandidateNet` at N candidates/frame is unbounded | Cap candidates before inference (`max_candidates` already exists); add an explicit `Stage::Inference` with a budget |
| **`--gen-dataset` must not leak truth** | It writes labels, so it is the one place INV-1 is deliberately crossed | Put it in its own module that the tracker cannot link, exactly as `metrics` is; extend `check_source_invariants.py` to cover it |
| **The ML would currently be a regression** | `models/centroidnet_v1.json` claims 0.274 px val RMSE; the classical centroider **measures 0.19 px** | Target `CandidateNet` (the decoy/clutter problem, where classical fails at 48.8 px) **first**, not `CentroidNet`. This is the highest-value ordering and it is the opposite of the current file ordering |

### 22.5 Recommended ML ordering

1. `--gen-dataset` (unblocks everything; also useful on its own as a data-generation demo).
2. **`CandidateNet` first** — it addresses the two real open failures (moving decoy, 120-source clutter) where classical measures 48.8 px and total lock loss. This is where ML can show a 100× improvement.
3. `CentroidNet` second, and only if it beats 0.19 px on held-out data. If it does not, **say so and ship the classical one** — that is a *stronger* Technical-Evaluation story than a worse learned model.
4. Keep `MotionNet` and the learned `StrategyPolicy` out of scope; say so explicitly.

---

## 23. SIH-Ready Architecture

The recommended target architecture, given what exists today. **Bold** marks additions or changes.

```
                        ┌──────────────────────────────────────────┐
  Scenario (TOML) ─────►│  SIMULATOR  — N worker threads           │
  or MP4 ──────┐        │                                          │
               │        │  World (analytic motion)                 │
               │        │  Plant (gimbal: rate/accel/delay/lag)    │
               │        │  Image formation (splat + exposure blur) │
               │        │  Damage chain  **counter-addressed RNG** │
               │        │  **Turbulence: AoA jitter + scintillation**
               │        │  **Attitude sensor: gyro + periodic fix**│
               │        └───────────────┬──────────────────────────┘
               │                        │  **SPSC frame queue (ordered, bounded)**
               │                        │  produce N+1 while tracking N
  ┌────────────▼──────────┐             │
  │ VIDEO SOURCE          │             │
  │ **threaded decode**   ├─────────────┤
  │ + watchdog            │             │
  │ INV-8: no damage      │             │
  └───────────────────────┘             │
                                        ▼
                        ┌──────────────────────────────────────────┐
                        │  TRACKER  — 1 thread, the §15 budget     │
                        │                                          │
                        │  Detection (**AVX2 van Herk/MF/CFAR**)   │
                        │    **ROI + rotating row-band refresh**   │
                        │  Centroid (classical  ⇄ **CentroidNet**) │
                        │  B16: image px → angle via **encoder**   │
                        │  Gate → Priority (⇄ **CandidateNet**)    │
                        │  KF / IMM  + **platform-drift state**    │
                        │  Supervisor (observable-only)            │
                        │  Mode FSM → Control (PID+FF+Smith)       │
                        └───────────────┬──────────────────────────┘
                                        │ INV-2: closed loop
                                        ▼
                        ┌──────────────────────────────────────────┐
                        │  METRICS  — the only truth consumer      │
                        │  **FOV containment** (PS objective)      │
                        │  centroiding: image / screen / **bore**  │
                        │  tracking: **transient vs steady state** │
                        │  **deadline misses, CPU, peak RSS**      │
                        │  run.json · centroid.csv · report.html   │
                        └───────────────┬──────────────────────────┘
                                        ▼
                        GUI dashboard  ·  **generated documentation**
```

Five structural changes, each independently justified:

1. **Sim/tracker split across an SPSC queue.** Makes the §15 budget meaningful (it is a *tracker* budget), takes the simulator off the critical path, and reaches 0.85 ms (§13.3).
2. **Counter-addressed RNG.** Makes the above deterministic *by construction* rather than by locking, and unlocks row-striped parallelism (§13.4).
3. **Platform-drift state + optional attitude sensor.** Turns the unbounded screen-frame error into a bounded one and supplies the deployment answer (§8.3).
4. **Turbulence in the simulator.** Closes the largest physics gap against an ISRO panel (§8.2).
5. **ML at two existing enum seams** (`Strategy.centroider`, priority term), never as a mandatory path (§22.3).

---

## 24. Recommended Demo Flow

A 12-minute sequence engineered so the judge's own attack becomes a scripted beat. Every step is a command that exists today (except where marked ⚠, which Phase 1/7 must deliver).

| # | Min | Step | Command | What the judge sees | Why |
|---:|---:|---|---|---|---|
| 1 | 0:00 | **Frame the problem in one number** | slide/whiteboard | Mount authority 26.7 px/frame; row 23+25 disturbance 40 px/frame = **150 %**. "A reactive loop *cannot* meet row 17. That is why this is a prediction problem." | Establishes depth in 30 s. Strongest opening available |
| 2 | 0:30 | **Normal case** ⚠ | `just gui` (default → `compliance.toml`) | Beacon acquired in 0.067 s; compliance panel green; live camera + 2000×2000 screen overview | Operational success, immediately |
| 3 | 2:00 | **Dial the damage up, live** | GUI: `Clean → Sensor noise → Full spec`; drag σ to 20, S&P to 10 % ⚠ *(banner shows the override)* | Image degrades visibly; centroiding error plot climbs on a log axis; lock holds | Shows row 21/22 handled, and that the numbers respond to conditions |
| 4 | 3:30 | **Disturbance** | GUI jitter slider to 20 px/frame | Tracking-error plot crosses the red 10 px line and **stays near 17 px** | Pre-empt row 17: "this is the derived 16.33 px jitter floor, not a control failure" — then set jitter 0 and show **3.357 px**. Turns a FAIL into a derivation |
| 5 | 5:00 | **Degradation and recovery** | GUI: occlude / `Step` through a dropout | Confirmed → Coasting → Search → Reacquire; re-acq time printed | The PS's "maintain visibility" loop, visible |
| 6 | 6:00 | **Break it on purpose** | Clutter slider 0 → 120 | The system loses the beacon and locks clutter | **Show your own failure before the judge finds it.** Name it: "this is the appearance-discrimination problem; it is what CandidateNet exists for, and here is the 48.8 px measurement" |
| 7 | 7:00 | **Prove you are not cheating** | `just gate-inv1-selftest` (injects a violation, the configure step goes red) | The linker rejects a tracker that touches truth | **The single most differentiating 60 seconds available.** No other team will have this |
| 8 | 8:00 | **Prove it is repeatable** | `./sat-tracker --verify-reproducibility` ⚠ *(with damage enabled)* | 10/10 bit-identical digests | Underwrites every number quoted |
| 9 | 9:00 | **The video benchmark (BP-2)** ⚠ | `just bp2` | Screen-mode RMSE **0.095 px**, tracking **1.14 px**, retention 98.9 %, handover at 2.0 s, p99 < 33 ms | This is 30 % of the marks; rehearse it on the demo hardware |
| 10 | 10:30 | **The auto-generated log** | open `logs/run.json` + `report.html` | Every PS Performance-Log field, plus FOV containment and per-stage timing | Deliverable satisfied, on screen |
| 11 | 11:30 | **The failure ledger** | open `issues_till_now.md` | Retracted claims, measured failures, owners | Closes on engineering maturity, and inoculates the Q&A |

**Rules for the demo.** Never run `baseline.toml` unless you are deliberately at step 6. Never let a judge be the first to touch the clutter slider. Have the pre-recorded video ready. Run from the `dist/` tarball on the demo machine, with no network.

---

## 25. Final Engineering Backlog

Ordered by technical dependency and SIH risk, not by size.

```
P0
 1.  Swap the default scenario (compliance.toml) and reorganise scenarios/hard/     [P0-1]
 2.  FOV-containment metric + post-acquisition target loss                          [P0-2]
 3.  Commit BP-2 fixtures + truth CSVs; add `just bp2` and its CI regression        [P0-4]
 4.  Threaded video decode behind a watchdog; p99 < 33.3 ms on BP-2                 [P0-3]
 5.  Write the Technical Report (10-15 pp) — mandatory deliverable                  [P0-5]

P1
 6.  Harden the INV-3 gate: enable damage/jitter/platform/atmosphere; scalar≡AVX2   [P1-1]
       (prerequisite for 8, 9, 10)
 7.  Correct the README weather claim and the issues_till_now claims                [P1-4, P1-5]
 8.  Remove the snapshot copy from the frame (-798 us)                              [P1-2]
 9.  Counter-addressed PCG; row-stripe damage chain + splat + background            [§13.2/3/5]
10.  Split sim and tracker across an SPSC queue; --sim-threads N                    [§13.3]
11.  AVX2 for van Herk / matched filter / CFAR                                      [§13.2/6]
12.  Amortise the ROI refresh into rotating row-bands (kills the 13.6x p95)         [P1-7]
13.  Report screen-frame error with its derivation; add the boresight column        [P1-6]
14.  Separate transient from steady state in every error metric                     [P1-9]
15.  Fix A-4: reconstruct through the encoder, not true_position()                  [P1-3]
16.  GUI: live spec table + "DAMAGE OVERRIDDEN" banner                              [P1-8]
17.  Raise sweep duration 4 s -> 30 s; re-baseline every quoted number              [P1-11]
18.  Real-time deadline mode + load shedding + deadline_misses                      [P1-10]

P2
19.  Turbulence: Kolmogorov AoA jitter + log-normal scintillation                   [P2-1]
20.  Attitude sensor (gyro + periodic fix) and platform-drift state                 [P2-3]
21.  CSRT / KCF / template baseline arms; four-row comparison table                 [P2-2]
22.  Fix A-3 (hardcoded 30 Hz), A-5 (stale hash), A-6 (--set strings)               [P2-4/5/6]
23.  Add mean_px, CPU, peak RSS, end-to-end latency to run.json                     [P2-7/8/9]
24.  Disclaimer in models/centroidnet_v1.json; or delete it until real              [P2-10]
25.  Structured event log for post-hoc failure diagnosis                            [P2-12]
26.  Demonstrate Smith and IMM benefit on the graded metric, or default them off    [P2-11]

P3
27.  Fix the dangling test citation (A-7) and the stale GUI comment (A-8)           [P3-1/2]
28.  check_docs.py verifies NUMBERS against a committed run.json                    [P3-4]
29.  run.json: edge_behaviour as a name, not an integer                             [P3-3]
30.  Record the machine spec beside every quoted performance figure                 [P3-5]
31.  Produce the User Manual and Technical Report as submitted PDFs                 [P3-6]
32.  3-5 minute demo video (optional deliverable)
```

**Critical path to a defensible submission:** 1 → 2 → 3 → 4 → 5 → 6 → 7. Items 1–7 are roughly one week and remove every P0 plus the two P1s that damage credibility. Items 8–12 are the 0.85 ms work and depend on 6.

---

## 26. Final Judge Assessment

### 26.1 Dimensional assessment

| Dimension | Current evidence | Major gap | Required improvement |
|---|---|---|---|
| **Problem alignment** | Every scenario key is annotated with its spec row; rows 1–25 traced; the disturbance-vs-authority derivation shows genuine understanding | The PS's stated objective — **FOV containment** — is not a reported metric. "AI-Based" in the title is unmet | Add FOV containment (P0-2); prepare a precise AI answer |
| **Functional completeness** | All mandatory functions present: configurable world, ≥4 motion modes (6 built), movable virtual camera, auto detection, CV tracking, camera control, all 5 disturbance classes, real-time stats | Technical Report not started (**mandatory**); `--gen-dataset` absent | Write the report; ship the manual as a document |
| **Technical correctness** | Coordinate frames exact and compiler-enforced; `q` derived from analytic bounds; Smith/feedforward double-lead analysis correct; INV-1 linker-enforced | A-4 encoder bypass; A-3 hardcoded 30 Hz; screen-frame metric unbounded | Fix A-3/A-4; report the screen-frame derivation |
| **Robustness** | Fuzzer 0 failures/9,411 frames; ASan+UBSan; bounded tables; 14 awkward video clips; graceful schema rejection | **Fails on clutter and on a moving decoy**; low light fails; no overload/deadline model | ML for discrimination (Phase 5); deadline mode (Phase 3) |
| **Evaluation** | 200-run multi-seed sweep with p95s; isolated axes; ablations one flag apart; retracted claims recorded | 4 s runs; no held-out protocol; **no end-to-end BP-2 number**; transient-dominated RMS | Phase 4 in full |
| **Real-world relevance** | Correct plant model; encoder/delay/lag; coarse→fine handover with a quadrant cell | **No turbulence**; no attitude reference; no target hardware; no sim-to-real analysis | Phase 3 |
| **Architecture** | 13 linker-enforced modules; 9 invariants with self-tested mechanisms; arena allocation; clean dependency graph | Sim and tracker inseparable; single-threaded by a circular argument; decode single-threaded | §13.3, §23 |
| **Differentiation** | Enforced invariants, bit-exactness, zero-allocation, analytic motion algebra, measured S-curve bias table, failure ledger | Algorithms themselves are textbook; the "digital twin" story is 80 % built but unstated | §18.3 |
| **Demonstrability** | Genuinely strong GUI: live damage, live algorithm toggles, ablation switch, compliance panel, stage timings | **The default scenario is the worst run in the repo**; GUI misstates its own conditions | P0-1, P1-8, Phase 7 |
| **Reproducibility** | INV-3 gate, CI −O0/−O2, named RNG streams, `Justfile` recipe per figure, `check_docs.py` | **The gate excludes the noise chain**; numbers lack a machine; open issues lack commands | P1-1; record machines; add commands |

### 26.2 Current project state — objectively

This is a **high-quality classical FSOC coarse-alignment simulator and tracker**, written in ~27.7 k lines of well-structured, exceptionally well-commented C++20, with an engineering process (enforced invariants, bit-exact reproducibility, sanitizers, fuzzing, a multi-platform CI matrix, an allocation trap, a failure ledger) that is materially above what this competition normally sees.

Its **detection and centroiding are genuinely excellent** — 0.19 px in simulation and 0.095 px on a real 2000×2000 noisy MP4 through the full closed loop, with a compiled-in bias table generated from a measured S-curve.

It **fails in three places**: appearance discrimination (clutter and moving decoys), low light, and throughput on the video benchmark. The first is correctly identified as the ML problem and deferred. The second and third are not yet acknowledged.

Its **documentation is extensive and mostly accurate, but overstates in exactly the places a judge will check**: the frame budget, the weather sweep, and the video path's speed.

And its **default entry point — the thing that runs when you type the obvious command — is its worst-performing configuration.**

### 26.3 Biggest risks

1. The demo opens on a broken scenario (P0-1). Certain to occur; the highest-damage, lowest-cost defect in the project.
2. The video benchmark, worth 30 %, fails row 20 at p95/p99 while the documentation claims it is the fast path (P0-3).
3. The Technical Report — a mandatory deliverable — does not exist (P0-5).
4. Two documented claims are disprovable in under five minutes by anyone running the project's own tools (P1-4, P1-5). In a project whose entire credibility rests on "every number is measured", a disproven number costs more than the number was worth.
5. The primary objective metric is not reported, and its stand-in flatters a 95.6 %-of-run failure into "7.59 % target loss" (P0-2).
6. "Where is the AI?" in a PS titled "AI-Based", with no rehearsed answer.

### 26.4 Biggest opportunities

1. **Reach 0.85 ms and retract §14.0d** (§13). Turns the project's most prominent admitted failure into a measured success *and* a story about intellectual honesty. ~3 days on the critical path after P1-1.
2. **Publish the BP-2 numbers** (§12.3). They are excellent — 0.095 px, 1.14 px tracking, 98.9 % retention — and the team does not currently know them.
3. **Demonstrate INV-1's linker enforcement live** (§24 step 7). Sixty seconds, no other team will have it.
4. **Add turbulence** (§8.2). Converts the most likely ISRO-panel attack into a highlight.
5. **Add the FOV-containment metric** (§9.2). Aligns the scoreboard with the PS's own sentence.

### 26.5 What a harsh judge would attack first

In order:

1. `./sat-tracker --gui` → "your own default scenario never finds the target."
2. Drag the clutter slider to 120 → "it locks onto a rock."
3. `just video <my clip>` → "you claim 382 FPS; I measure 30, and your p99 is 10."
4. "Your README says every weather mode is 0.14–3.4 px. Your own sweep says low light is 237 px."
5. "Your target loss is 7.59 % on a run where the beacon was visible 4 % of the time. Explain the denominator."
6. "Your problem statement is about atmospheric propagation. Where is turbulence? Where is scintillation? What is your r₀?"
7. "Your screen-frame centroiding error is 589 px and grows with run length. Is your centroider diverging?"
8. "The title says AI-Based."
9. "Your reproducibility gate runs with the noise turned off — so what exactly does it prove about your AVX2 path?"
10. "Show me the Technical Report."

Answers 5, 6, 7 and 9 are all *available and good* — the team simply has not prepared them. Answers 1–4 and 10 require the work in §25.

### 26.6 Minimum viable SIH-ready state

The project should not be considered competition-ready until **all** of these hold:

1. `./sat-tracker --gui` and `./sat-tracker --headless`, with no arguments, produce a run that **passes rows 16, 18, 19, 20** and scores > 0 centroiding frames — asserted in CI.
2. `just bp2` exists, runs three committed clips end-to-end against committed truth, and reports **p99 frame time < 33.3 ms** alongside the accuracy table.
3. **FOV containment** is a reported metric, and row 18 is graded post-acquisition.
4. `--verify-reproducibility` runs **with the damage chain, jitter, platform motion and a non-clear atmosphere enabled**, and asserts scalar ≡ AVX2.
5. Every number in the README and `issues_till_now.md` is reproduced by a committed command **on a stated machine**, verified in CI.
6. The **Technical Report** and **User Manual** exist as submitted documents.
7. The demo sequence in §24 has been rehearsed end-to-end on the demo hardware, offline, from the `dist/` tarball.
8. Prepared, rehearsed answers exist for all ten attacks in §26.5.

Items 1–4 and 6 are the hard gates. Everything in §13 (the 0.85 ms work) is upside, not a gate — but it is the highest-value upside available, and item 4 is its prerequisite.

---

## Appendix — Unverified Claims and Assumptions

### A.1 Claims this audit could not verify

| Claim | Source | Why unverified |
|---|---|---|
| Supervisor: +15 pts lock retention in fog, 0 against clutter | `issues_till_now.md` §2.4 | `just cp123` not executed |
| Low-SNR centroiding 3.64× the theoretical bound at SNR 13 | `issues_till_now.md` §2.4 | Requires `--calibrate-centroid` sweep |
| Figure-8 acceleration lag 16.56 px | `issues_till_now.md` §2.4 | Not re-measured; mechanism (velocity-only feedforward) is plausible |
| "Row 17 lost at 600 px/s drift, saturation at 800" | `issues_till_now.md` §2.3 | Sweep not re-run |
| Windows MSVC binary builds and runs | CI `build.yml` | No Windows machine available to this audit |
| Cross-machine digest agreement | `reproducibility.yml` | Single machine available; and see A-1 |
| The motion-term development history (§2.1's three failed versions) | `issues_till_now.md` §2.1 | Historical; final behaviour is observable, the intermediate measurements are not |
| `strobing_target.toml`, `beacon_larger_than_fov.toml`, `edge_camper.toml`, `target_faster_than_mount.toml`, `all_weather_churn.toml` behaviour | `scenarios/adversarial/` | Only `decoy_swarm.toml` was scored here |
| AVX2 damage chain parity with scalar | `issues_till_now.md` §1.4 | Unit test exists and passes; **not covered by the INV-3 gate** (A-1), and no non-AVX2 machine available |

### A.2 Assumptions this audit made

1. **`docs/problem_statement.pdf` is the authoritative PS text.** Row numbering in this document is the audit's own, derived from the PDF's table order; the PDF's own "Sr. No." column restarts at 2 in the Disturbances block, so rows 21–25 here correspond to the PDF's disturbance rows 1–5 plus the image-noise row.
2. **Judges will read the screen-frame centroid column.** If they use the image-frame column, §8.3's severity drops from P1 to P3. This is worth clarifying with the organisers if possible.
3. **Benchmark-1 scenarios will be TOML-shaped parameter sets** run in the team's own simulator. If they are supplied as video, BP-1 collapses into BP-2 and §12 becomes 60 % of the marks rather than 30 %.
4. **Performance figures are machine-specific.** All timings here are from an Intel Core Ultra 7 256V (8 cores, AVX2, no AVX-512) at a low clock state. Relative comparisons hold; absolute microsecond figures will differ from the team's own by ~1.6×.
5. **The audit's BP-2 clips are representative.** They were built with ffmpeg `geq`/`drawbox` + `noise` to mirror the PS's description ("covering a complete screen with noise and moving beacon spot"). Real evaluator clips may include compression artefacts, non-square beacons, or multiple objects.

### A.3 Commands used to produce the evidence in this document

```bash
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure -j$(nproc)

./build/sat-tracker --headless --scenario scenarios/baseline.toml   --out /tmp/base
./build/sat-tracker --headless --scenario scenarios/compliance.toml --duration {4,10,20,30,60} --out /tmp/cmp
./build/sat-tracker --headless --scenario scenarios/compliance.toml --set clutter.static_sources=0 \
                    --set clutter.decoy_beacons=0 --duration 60 --out /tmp/clean
./build/sat-tracker --headless --scenario scenarios/compliance.toml --set gimbal.encoder_lsb_urad={20,500,4000} ...
./build/sat-tracker --headless --scenario scenarios/compliance.toml --duration 6 --stages --out /tmp/st
./build/sat-tracker --headless --scenario scenarios/compliance.toml --duration 20 --bench
./build/sat-tracker --bench-kernels
./build/sat-tracker --sweep scenarios/sweeps/weather.toml --out /tmp/sweep --jobs 8
./build/sat-tracker --verify-reproducibility --seeds 2 --duration 2
./build/sat-tracker --fuzz-scenarios 40 --seed 4242 --duration 1.5
./build/sat-tracker --gui --scenario scenarios/compliance.toml --shot /tmp/shot.png --shot-after 60
./build/sat-tracker --video tests/video/clips/screen_2000x2000_30fps.mp4 --out /tmp/v1
python3 tools/check_docs.py ; python3 tools/check_source_invariants.py

# BP-2 fixtures constructed by this audit (§12.3) — these should be committed
ffmpeg -f lavfi -i "color=c=black:s=640x480:r=30:d=4" \
  -vf "geq=lum='if(between(X,100+2*N,109+2*N)*between(Y,150+N,159+N),200,20)':cb=128:cr=128,\
noise=alls=25:allf=t,format=yuv420p" -c:v libx264 -crf 18 noisy_direct.mp4
ffmpeg -f lavfi -i "color=c=black:s=2000x2000:r=30:d=6" \
  -vf "geq=lum='if(between(X,990+3*N,999+3*N)*between(Y,990+1.5*N,999+1.5*N),235,12)':cb=128:cr=128,\
noise=alls=18:allf=t" -c:v libx264 -pix_fmt yuv420p -preset veryfast -crf 20 bp2_screen.mp4
./build/sat-tracker --video bp2_screen.mp4 --truth bp2_truth.csv --out /tmp/v4
```

---

*End of audit. Findings are stated against commit `803733d`. Every measurement in this document was produced by one of the commands in A.3 on the machine described in A.2.4.*
