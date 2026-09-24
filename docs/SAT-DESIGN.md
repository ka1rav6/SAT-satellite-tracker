# SAT — Master Design Specification
### Self-Adaptive beacon Tracking · SIH PS 26169
### AI-Based Virtual Camera Tracking for Coarse Alignment of Mobile FSOC Terminals
### Organisation: Department of Space / ISRO · Category: Software

---

# 0. HOW TO USE THIS DOCUMENT

**This document is the single source of truth for the project.** It is written to be worked through
by a coding agent or a developer, checkpoint by checkpoint.

## Working rules

1. **Work one checkpoint at a time**, in the order given in §14. Do not jump ahead.
2. **Every checkpoint has an acceptance test.** Write the test first, then the code, then run it.
3. **Do not proceed past a checkpoint until its acceptance test passes.** Not "mostly works."
4. **Four checkpoints are hard gates** (marked ★ GATE). If a gate fails, stop all other work and fix
   it.
5. **Never break the invariants in §2.** They are not style preferences; violating them invalidates
   the project's core claims.
6. **After every checkpoint, run the full test suite.** `ctest --output-on-failure`.
7. If a design decision in this document turns out to be wrong when you reach it, **say so and
   explain why** rather than silently deviating.

## Companion document

Model training, dataset generation and OpenCV usage are specified separately in **`SAT-ML.md`**.
This document specifies the C++ *integration contract* for models (§11); that document specifies how
they are built.

---

# 1. WHAT IS BEING BUILT

## 1.1 One paragraph

Two devices want to exchange data over a laser link. A laser beam is extremely narrow, so before any
data flows, one terminal must find the other and keep pointing at it while both platforms move and
vibrate. Finding it is called **coarse alignment**. This project simulates the whole coarse-alignment
problem in software: a virtual scene, a virtual pan-tilt camera, a moving beacon, deliberate image
degradation, and the detection-tracking-control algorithm that finds and follows the beacon. It must
also do all of this on externally supplied MP4 video.

## 1.2 The mechanics

- A **screen** of 2000 × 2000 pixels (default) containing a moving **beacon** (10 × 10 px default)
  plus clutter.
- A **virtual camera** seeing a 640 × 480 window of that screen — 7.68% of it.
- Every frame the image is degraded: 10% salt-and-pepper, Gaussian and Poisson noise, contrast loss
  from simulated weather, and boresight displacement of up to ±20 px/frame from jitter and platform
  motion.
- The software finds the beacon, reports its **exact sub-pixel centre**, and commands the camera.
- The camera obeys a rate limit of 5–10 °/s.
- Loop rate 30 Hz minimum.

## 1.3 The central difficulty

| Quantity | Value |
|---|---|
| Camera authority at 5 °/s, 30 Hz | 26.7 px/frame |
| Worst-case disturbance (jitter + platform) | ±40 px/frame |

**The disturbance can exceed the actuator's authority.** Reactive control therefore cannot meet the
10-pixel error budget; the system must predict. This drives the design of §9 and §10.

## 1.4 Derived constants (compute once, log at startup)

```
IFOV                        = 4° / 640            = 0.00625 °/px = 109.08 µrad/px
Screen angular extent       = 2000 × 0.00625°     = 12.5° × 12.5°
FOV fraction of screen      = (640×480)/(2000×2000) = 7.68%
Max pan @ 5 °/s             = 800 px/s            = 26.7 px/frame @ 30 Hz
Max pan @ 10 °/s            = 1600 px/s           = 53.3 px/frame
Jitter 20 px/frame          = 3.75 °/s            = 75% of default authority
Tracking error budget 10 px = 1.09 mrad
Salt-and-pepper corrupted   = 30,720 px           = 307:1 vs the 100-px beacon
Exhaustive raster of screen @ 5 °/s ≈ 12.5–15 s   (vs a 2 s acquisition spec — see §10.5)
```

---

# 2. INVARIANTS — NEVER VIOLATE THESE

These are enforced by the build system and CI, not by convention.

### INV-1 — Perception, tracking and control cannot access ground truth

The CMake targets `perception`, `ai`, `tracking`, `search`, `control` and `plant` **must not link**
`world`. Any attempt to read the simulator's true beacon position from those modules is a link error.

*Why:* the entire project is meaningless if the tracker knows the answer.

### INV-2 — The loop is closed

The controller's output determines the boresight at which the next frame is produced. There must be
no code path where frames are produced independently of the control output (except `video_direct`
mode, §8.3, where pointing is explicitly disabled).

### INV-3 — Bit-exact reproducibility

For fixed `(scenario, seed, build)` and `--no-ai`, the sequence of frame hashes must be identical on
any machine, at any load, headless or windowed.

Concretely:
- No `std::chrono` or any wall-clock read in the simulation path.
- No `rand()`, no unseeded generators. Every draw names a `Stream`.
- Compile with `/fp:precise` (MSVC) or `-ffp-contract=off -fno-fast-math` (GCC/Clang).
- Never iterate `std::unordered_map`/`set` in the simulation path.
- Never `std::sort` on a key that can tie — use `std::stable_sort` or add an index tiebreaker.
- No parallelism *inside* a run. Parallelism is across runs, via processes.

### INV-4 — Zero heap allocation in steady state

After scenario load, no allocation occurs during frame processing. Debug builds install a global
`operator new` trap that asserts if called while `g_in_frame` is true.

### INV-5 — Work in angles, not pixels

All internal state is in microradians. Pixels exist only at the sensor boundary (`project` /
`unproject`) and in the exported logs. Never store a tracker state in pixels.

### INV-6 — Centroiding error and tracking error are distinct

```
centroiding_error = |reported beacon centre − true beacon centre|   ← graded, 60% of marks
tracking_error    = |camera boresight − true beacon angle|          ← spec row 17, ≤ 10 px
```

They are computed separately, logged separately, plotted separately, and reported separately. Never
conflate them.

### INV-7 — The system must run fully without AI

`--no-ai` must pass the complete compliance matrix. Every model has a classical fallback. A missing
or corrupt model file logs a warning and falls back; it never crashes.

### INV-8 — No damage is added in video modes

When `input.mode != synthetic`, the noise and atmosphere generators are forcibly disabled. The
degradation is already in the supplied footage; adding more corrupts the benchmark.

### INV-9 — Never emit a stale or interpolated centroid

If no detection occurred on a frame, the centroid columns in the log are **blank**. Never write the
last known value, never write the prediction.

---

# 3. GRADED REQUIREMENTS

## 3.1 Evaluation weighting (from the official PS)

| Stage | Marks | Criteria |
|---|---|---|
| Functional Verification | **20%** | 10–15 min live demo. All mandatory functions · Operational success · **GUI** |
| Benchmark Performance-1 | **30%** | Scenarios supplied by evaluators. Execution · **Log of centroiding error** · **Automatically generated performance logs** |
| Benchmark Performance-2 | **30%** | **MP4 files supplied by evaluators**, 30 fps, covering a complete screen. Software must bypass its PTZ camera and take the video as input. Centroiding error vs predefined values · RMSE, acquisition and re-acquisition time, lock retention, FPS |
| Technical Evaluation | **20%** | Problem understanding · Architecture · Algorithm selection · **AI and computer vision** · Innovation · Documentation · Q&A |

**Implications that drive the plan:**
- 80% of marks are the software running; 20% is presentation.
- Centroiding error is named in both 30% stages → §9 is a first-class subsystem.
- Video ingest is worth as much as the entire simulation benchmark → §8 is a first-class subsystem.
- The evaluators supply the scenarios → robustness to untried parameter combinations matters
  (CP 14.1 fuzzing).
- The GUI is explicitly graded → §12 is a deliverable, not polish.

## 3.2 Specification parameter table (all 25 rows)

| Row | Parameter | Value | TOML key |
|---|---|---|---|
| 1 | Screen size (min) | 2000 × 2000 px | `world.canvas_px` |
| 2 | Camera type | Monochrome, FPA (colour optional) | `camera.type` |
| 3 | Camera resolution | 640 × 480 (user-defined) | `camera.resolution` |
| 4 | Camera FOV | User-defined, default 4° × 3° | `camera.fov_deg` |
| 5 | Camera update rate | ≥ 30 Hz | `sim.camera_hz` |
| 6 | Initial camera position | Centre of screen | `camera.initial_pos_px` |
| 7 | Target type | Beacon spot | `target.kind` |
| 8 | Number of targets | 1 mandatory, multiple optional | `[[target]]` count |
| 9 | Target shape | User-defined, default square | `target.shape.type` |
| 10 | Target size | 5–20 px, default 10 × 10 | `target.size_px` |
| 11 | Initial target location | User-defined, default random | `target.initial_px` |
| 12 | Motion | ≥4: line, circular, figure-8, random. Optional: spiral, sinusoidal, user-defined | `[[target.motion]]` |
| 13 | Max pan speed | 5–10 °/s, default 5 | `gimbal.max_pan_dps` |
| 14 | Max tilt speed | 5–10 °/s, default 5 | `gimbal.max_tilt_dps` |
| 15 | Update interval | ≥ 20 Hz | `sim.control_hz` |
| 16 | **Acquisition time** | **≤ 2 s** | `requirements.acquisition_s` |
| 17 | **Tracking error** | **≤ 10 px** | `requirements.tracking_error_px` |
| 18 | **Target loss** | **< 5%** | `requirements.target_loss_frac` |
| 19 | **Re-acquisition time** | **≤ 1 s** | `requirements.reacquisition_s` |
| 20 | **Processing speed** | **≥ 20 FPS** | `requirements.min_fps` |
| 21 | Image noise | Salt & pepper (~10%), Gaussian, Poisson — selectable | `noise.*` |
| 22 | Max noise std deviation | 20 (grey levels — documented interpretation) | `noise.gaussian_sigma` |
| 23 | Max camera jitter | ±20 px/frame | `disturbance.jitter_px_per_frame` |
| 24 | Atmospheric disturbance | Clear, Haze, Fog, Rain, Low light | `atmosphere.mode` |
| 25 | Platform motion | ±20 px/frame. Linear mandatory; circular, random, spiral, figure-8 optional | `[[disturbance.platform]]` |

---

# 4. TECHNOLOGY STACK

| Layer | Choice | Notes |
|---|---|---|
| Language | C++20 | |
| Build | CMake ≥ 3.20 + vcpkg (manifest, **pinned baseline**) | |
| Windows binary | GitHub Actions `windows-latest`, MSVC 2022 | Native build, not cross-compiled |
| Linux binary | GitHub Actions `ubuntu-latest`, GCC 13 | |
| Window / input | GLFW 3.4 | |
| OpenGL | glad, GL 3.3 core | Display only; no compute shaders |
| GUI | Dear ImGui (docking) + ImPlot | |
| Linear algebra | Eigen 3.4, fixed-size matrices | Kalman / IMM |
| Vision + video | OpenCV 4.x with `videoio` + `ffmpeg` | **Bounded use — §4.2** |
| ML inference | ONNX Runtime 1.17+ | §11 |
| Config | toml++ | The only configuration language |
| Image loading | stb_image | Custom shape masks |
| Logging | spdlog | |
| JSON | nlohmann/json | |
| Testing | doctest | |
| Profiling | Tracy | Dev builds only |

## 4.1 Explicitly rejected

| Rejected | Reason |
|---|---|
| **ReactPhysics3D** | Nothing collides. The gimbal is a rate/acceleration-limited second-order plant with a transport delay — 120 lines you must own for INV-3 and for applying saturation inside the integration step. |
| **EnTT** | Fewer than 1000 homogeneous entities. Parallel arrays (SoA) are faster and simpler. Revisit only above ~2000. |
| **Lua / any scripting language** | The three "user-defined" spec rows are covered by the declarative motion algebra (§7.2), image masks (§7.3) and event arrays (§7.4). Declarative wins on validation, reproducibility and analytic velocity. |
| **YOLO / any object detector** | The beacon is a uniform square with no texture or shape variety. A matched filter is the provably optimal detector for a known shape in additive noise and ~1000× faster. State this reasoning in the report — it is evidence of judgement under "Selection of Algorithms". |

## 4.2 OpenCV boundary — enforce this

| Use OpenCV for | Do NOT use OpenCV for |
|---|---|
| `cv::VideoCapture` — MP4 decode (§8) | Anything in the per-frame hot loop |
| `cv::imread`/`imwrite` — masks, debug dumps | Median filter — own implementation |
| `cv::connectedComponentsWithStats` | Morphology — own implementation |
| **Test oracle** for every own kernel | Box filtering / CFAR (OpenCV has no CFAR) |
| Offline dataset tooling | |

Rule: **anything that runs 30×/second and touches every pixel, we write.**

*Why:* `cv::Mat` allocates (violates INV-4), OpenCV's runtime SIMD dispatch can differ across machines
(risks INV-3), and generic type dispatch inflates p99 latency.

---

# 5. REPOSITORY LAYOUT

```
sat-tracker/
├── CMakeLists.txt
├── vcpkg.json
├── README.md
├── QUICKSTART.md
├── .github/workflows/
│   ├── build.yml                 # Windows + Linux, tests, artifacts
│   ├── reproducibility.yml       # INV-3 enforcement
│   ├── nightly-compliance.yml    # full sweep, matrix diff
│   └── fuzz.yml                  # CP 14.1
├── cmake/
│   └── modules.cmake             # target dependency graph, enforces INV-1
├── docs/
│   ├── INVARIANTS.md             # copy of §2
│   ├── METRICS.md                # copy of §13.1
│   ├── LOG_FORMAT.md             # copy of §13.2
│   ├── adr/                      # architecture decision records
│   ├── models/                   # model cards (see SAT-ML.md)
│   ├── report/                   # technical report sources
│   └── manual/                   # user manual sources
├── src/
│   ├── core/
│   │   ├── units.hpp             # Urad, Angle2, Pixel2, Rate2
│   │   ├── frames.hpp            # CameraGeometry, project/unproject
│   │   ├── time.hpp              # Clock
│   │   ├── rng.hpp               # Pcg32, Stream, RngSet
│   │   ├── arena.hpp             # Arena, ScopedArena
│   │   ├── ring.hpp              # Ring<T,N>
│   │   ├── triple_buffer.hpp     # TripleBuffer<T>
│   │   ├── hash.hpp              # fnv1a, snapshot_hash
│   │   ├── result.hpp            # Result<T> error type
│   │   └── profile.hpp           # SAT_ZONE macro, ScopedTimer
│   ├── engine/
│   │   ├── frame_source.hpp      # IFrameSource, SourceFrame   ← CP 1.1
│   │   ├── synthetic_source.hpp/.cpp
│   │   ├── video_source.hpp/.cpp # VideoScreenSource, VideoDirectSource
│   │   ├── decode_thread.hpp/.cpp
│   │   └── pipeline.hpp/.cpp     # the per-frame orchestrator (§6)
│   ├── world/
│   │   ├── world.hpp/.cpp
│   │   ├── emitters.hpp          # EmitterSoA
│   │   ├── motion_component.hpp  # IMotionComponent
│   │   ├── components/           # linear, circular, lissajous, ou, spiral,
│   │   │                         # sinusoid, waypoints, constant, accel
│   │   ├── composite_motion.hpp/.cpp
│   │   ├── background.hpp/.cpp
│   │   └── shape_mask.hpp/.cpp
│   ├── scenario/
│   │   ├── scenario.hpp          # the parsed config struct
│   │   ├── schema.hpp/.cpp       # FieldSpec table, validation
│   │   ├── toml_loader.cpp
│   │   └── events.hpp/.cpp
│   ├── camera/
│   │   ├── camera.hpp/.cpp
│   │   ├── splat.hpp/.cpp
│   │   └── coverage.hpp          # overlap_1d, gauss_1d
│   ├── degrade/
│   │   ├── sensor.hpp/.cpp       # the full chain
│   │   ├── atmosphere.hpp
│   │   ├── noise.hpp/.cpp
│   │   └── disturbance.hpp/.cpp  # jitter + platform
│   ├── perception/
│   │   ├── pipeline.hpp/.cpp     # IPerception implementations
│   │   ├── median.hpp/.cpp
│   │   ├── morphology.hpp/.cpp   # van Herk
│   │   ├── sat.hpp/.cpp          # summed-area tables
│   │   ├── matched.hpp/.cpp
│   │   ├── cfar.hpp/.cpp
│   │   ├── grouping.hpp/.cpp     # run-length + union-find
│   │   ├── centroid/
│   │   │   ├── estimators.hpp/.cpp  # 4 estimators
│   │   │   ├── bias_table.hpp/.cpp  # S-curve correction
│   │   │   └── uncertainty.hpp
│   │   └── simd/                 # avx2 kernels + scalar fallbacks
│   ├── ai/
│   │   ├── onnx_model.hpp/.cpp   # thin ORT wrapper
│   │   ├── centroid_net.hpp/.cpp
│   │   ├── candidate_net.hpp/.cpp
│   │   ├── recovery_net.hpp/.cpp
│   │   ├── motion_net.hpp/.cpp
│   │   ├── strategy_policy.hpp/.cpp
│   │   └── features.hpp/.cpp     # Conditions extraction
│   ├── tracking/
│   │   ├── kalman.hpp/.cpp
│   │   ├── imm.hpp/.cpp
│   │   ├── track.hpp
│   │   ├── association.hpp/.cpp
│   │   └── lifecycle.cpp
│   ├── search/
│   │   ├── prob_grid.hpp/.cpp
│   │   └── strategies.hpp/.cpp
│   ├── control/
│   │   ├── controller.hpp/.cpp
│   │   ├── mode_fsm.hpp/.cpp
│   │   ├── sat_supervisor.hpp/.cpp
│   │   └── smith.hpp
│   ├── plant/
│   │   └── gimbal.hpp/.cpp
│   ├── metrics/
│   │   ├── collector.hpp/.cpp
│   │   ├── compliance.hpp/.cpp
│   │   ├── centroid_log.hpp/.cpp
│   │   ├── report_json.cpp
│   │   └── report_html.cpp
│   ├── gui/
│   │   ├── dashboard.hpp/.cpp
│   │   ├── panels/
│   │   └── overlays.cpp
│   └── app/
│       ├── main.cpp
│       ├── cli.hpp/.cpp
│       ├── headless.cpp
│       ├── sweep.cpp
│       ├── dataset.cpp           # --gen-dataset (see SAT-ML.md)
│       ├── fuzz.cpp
│       └── bench.cpp
├── scenarios/
│   ├── spec_defaults.toml
│   ├── compliance.toml
│   ├── hard/               # named failures, each isolating one cause
│   ├── fog_figure8.toml
│   ├── maxnoise_random.toml
│   ├── decoy_beacon.toml
│   ├── saturating_platform.toml
│   ├── video_screen.toml
│   ├── video_direct.toml
│   ├── sweep_full.toml
│   ├── strategy_sweep.toml
│   ├── adversarial/
│   ├── shapes/                   # PNG masks
│   └── datasets/
├── models/                       # shipped .onnx files
├── tests/
│   ├── kernels/  centroid/  filters/  frames/  motion/
│   ├── schema/  bad_configs/  repro/  video/  ai/  compliance/
├── ml/                           # see SAT-ML.md
└── tools/
    ├── report.py  plots.py  make_test_videos.sh
```

## 5.1 CMake target graph (enforces INV-1)

```cmake
# cmake/modules.cmake
add_library(sat_core       ...)                                   # no deps
add_library(sat_world      ...)  target_link_libraries(sat_world      PUBLIC sat_core)
add_library(sat_scenario   ...)  target_link_libraries(sat_scenario   PUBLIC sat_core)
add_library(sat_camera     ...)  target_link_libraries(sat_camera     PUBLIC sat_core sat_world)
add_library(sat_degrade    ...)  target_link_libraries(sat_degrade    PUBLIC sat_core)

# ── INV-1 BOUNDARY: nothing below may link sat_world ──
add_library(sat_perception ...)  target_link_libraries(sat_perception PUBLIC sat_core)
add_library(sat_ai         ...)  target_link_libraries(sat_ai         PUBLIC sat_core onnxruntime)
add_library(sat_tracking   ...)  target_link_libraries(sat_tracking   PUBLIC sat_core Eigen3::Eigen)
add_library(sat_search     ...)  target_link_libraries(sat_search     PUBLIC sat_core)
add_library(sat_plant      ...)  target_link_libraries(sat_plant      PUBLIC sat_core)
add_library(sat_control    ...)  target_link_libraries(sat_control    PUBLIC sat_core sat_tracking
                                                                              sat_search sat_plant)
# ── end boundary ──

add_library(sat_engine     ...)  target_link_libraries(sat_engine     PUBLIC sat_core sat_world
                                   sat_camera sat_degrade sat_perception sat_tracking sat_control
                                   sat_ai opencv_videoio)
add_library(sat_metrics    ...)  target_link_libraries(sat_metrics    PUBLIC sat_core sat_world)
add_library(sat_gui        ...)  target_link_libraries(sat_gui        PUBLIC sat_core imgui implot)
add_executable(sat-tracker ...)
```

Add a CI check that greps `perception/`, `ai/`, `tracking/`, `search/`, `control/`, `plant/` for
`#include "world/` and fails if found.

---

# 6. THE RUNTIME PIPELINE

## 6.1 Startup sequence (once, on START)

| # | Step | Detail |
|---|---|---|
| A1 | Parse and validate TOML | Schema check, spec-row-citing errors. Abort cleanly on failure. |
| A2 | Resolve input mode | `synthetic` / `video_screen` / `video_direct`. If video: open, probe resolution/fps/codec, auto-detect mode, re-derive clock divisors. |
| A3 | Seed RNG streams | `RngSet::seed_all(master_seed)`. All streams seeded even if unused. |
| A4 | Allocate arenas | persistent 24 MB, frame 4 MB, video ring 48 MB, report 8 MB. **Last allocation of the run.** |
| A5 | Build world | *Synthetic only.* Emitters, compiled motion stacks, shape masks, clutter, decoys, initial target position from `Stream::TargetInit`. |
| A6 | Load ONNX models | 4 sessions, threads pinned to 1. Missing file + `fallback_on_fail` → warn, mark classical-only. Never fatal. |
| A7 | Load bias tables | S-curve coefficients, compiled into the binary. |
| A8 | Init subsystems | Perception windows from size hint; Kalman/IMM reset; probability grid uniform; gimbal at `camera.initial_pos_px`; FSM = `Idle`. |
| A9 | Open logs | Write `centroid.csv` header immediately (§13.2). |
| A10 | Start decode thread | *Video only.* Pre-fill the ring buffer. |
| A11 | Latch start tick | FSM `Idle → Search`. |

## 6.2 Per-frame sequence

**B1–B3, ×10 per camera frame (300 Hz truth rate):**

| # | Step |
|---|---|
| B1 | Advance world — evaluate each emitter's `CompositeMotion` at `t`; advance stochastic components |
| B2 | Advance disturbances — jitter sample, platform component; both perturb the **true** boresight |
| B3 | Step gimbal — push commanded rate into delay line, read delayed value, clamp rate and acceleration, integrate (midpoint), count saturation |

**B4 — frame acquisition. THE ONLY PLACE THE MODES DIFFER.**

| Mode | Behaviour |
|---|---|
| `synthetic` | View AABB from true boresight → `query_visible` → render background into viewport → splat each emitter with exact coverage across `blur_substeps`, interpolating boresight start→end → float radiance → damage chain → `uint8`. Emit `FrameTruth`. |
| `video_screen` | Pop decoded frame from ring → bicubic crop 640×480 at true boresight → `uint8`. **No damage added** (INV-8). Truth only if a truth CSV was supplied. |
| `video_direct` | Pop decoded frame → pass whole frame through. Boresight fixed; `supports_pointing() == false`. |

**B5 — a `SourceFrame` exists. Everything below is identical code for all three modes.**

| # | Stage | Step |
|---|---|---|
| B6 | Perception | Median 3×3 |
| B7 | | Top-hat background removal (van Herk) |
| B8 | | Build both summed-area tables (integer) |
| B9 | | Multi-scale matched filter, 6 scales |
| B10 | | CFAR threshold → sparse mask |
| B11 | | Run-length extraction + union-find → blobs with moments |
| B12 | | Shape/area/fill/aspect gate → ~5–20 candidates |
| B13 | | Centroid per candidate (estimator chosen by SAT) + S-curve bias correction + uncertainty |
| B14 | | `CandidateNet` on each patch; `CentroidNet` refinement if selected |
| B15 | | *Conditional:* zero candidates AND mode ∈ {Search, Reacquire} AND ≥100 ms since last → `RecoveryNet` |
| B16 | Tracking | Unproject to angles, add commanded boresight → world angular frame |
| B17 | | Gate against predictions (Mahalanobis d² < 9.21) |
| B18 | | Associate (nearest neighbour, or Hungarian if multi-target) |
| B19 | | IMM predict + update; `MotionNet` forecast in parallel |
| B20 | | Lifecycle transitions (Tentative/Confirmed/Coasting/Deleted) |
| B21 | | Apply priority policy if multi-target |
| B22 | SAT + control | Extract the 12-feature `Conditions` vector |
| B23 | | EMA smooth; if dwell ≥ 30 frames, run `StrategyPolicy` (or rule table); switch bumplessly |
| B24 | | Mode FSM transition |
| B25 | | Compute aim point — `prob_grid.best_look()` in Search, predicted position in Track |
| B26 | | Controller: PID + velocity feedforward + platform cancellation; clamp; anti-windup |
| B27 | | Emit commanded rate → **feeds back into B3 next tick** (INV-2) |
| B28 | Metrics | Compute centroiding error and tracking error (needs truth) |
| B29 | | Append `centroid.csv` row; update running stats and latency histograms |
| B30 | | Publish snapshot to triple buffer |
| B31 | Display | *Separate thread, async.* Read newest snapshot → upload texture → draw panels → swap |

**Detection-miss branch (between B15 and B16):**

```
detection found?
  ├── yes → associate, update filter, reset miss counter
  └── no  → coast on prediction, widen search gate, increment miss counter
            15 consecutive misses → delete track → FSM back to Search
```

## 6.3 Shutdown

Duration reached, video EOF, or user stop → drain and join decode thread → close video → finalise
percentile stats → flush `centroid.csv` → write `run.json` → generate `report.html` and
`compliance.txt` → release arenas → close ONNX sessions → GUI shows summary.

---

# 7. CONFIGURATION

## 7.1 Complete scenario schema

```toml
[meta]
name        = "fog_figure8"
description = "Figure-8 beacon under fog with maximum noise"

[input]
mode = "synthetic"                  # synthetic | video_screen | video_direct
# video_file      = "bench/clip_03.mp4"
# video_truth_csv = "bench/clip_03_truth.csv"

[sim]
truth_hz   = 300                    # must be divisible by camera_hz and control_hz
camera_hz  = 30                     # row 5, min 30
control_hz = 30                     # row 15, min 20
duration_s = 120
seed       = 42

[world]                             # rows 1, 6
canvas_px      = [2000, 2000]
edge_behaviour = "bounce"           # bounce | wrap | exit

[camera]                            # rows 2-6
type           = "mono"             # mono | colour
resolution     = [640, 480]
fov_deg        = [4.0, 3.0]
exposure_ms    = 5.0
blur_substeps  = 8
initial_pos_px = [1000, 1000]

[target]                            # rows 7-11
kind       = "target"
intensity  = 120.0
size_px    = 10                     # row 10: 5-20
initial_px = "random"               # row 11: "random" | [x, y]
shape.type = "square"               # row 9: square | circle | gaussian | mask
# shape.mask_file = "shapes/cross.png"

[[target.motion]]                   # row 12 — stack components, see §7.2
kind         = "lissajous"
amplitude_px = [700.0, 350.0]
freq_ratio   = 2.0
period_s     = 24.0

[gimbal]                            # rows 13-15
max_pan_dps      = 5.0              # row 13: 5-10
max_tilt_dps     = 5.0              # row 14: 5-10
max_accel_dps2   = 50.0
time_constant_s  = 0.020
latency_s        = 0.010
encoder_lsb_urad = 20.0
resonance_hz     = 0.0              # 0 = disabled

[noise]                             # rows 21-22
poisson        = true
gaussian_sigma = 20.0               # row 22, grey levels
salt_pepper    = 0.10               # row 21, ~10%
hot_pixels     = 40

[atmosphere]                        # row 24
mode = "fog"                        # clear | haze | rain | fog | lowlight

[disturbance]                       # rows 23, 25
jitter_px_per_frame = 20.0          # row 23

[[disturbance.platform]]            # row 25 — same component system as target motion
kind          = "linear"
velocity_px_s = [15.0, -8.0]

[clutter]
static_sources = 120
decoy_beacons  = 1

[ai]
enabled          = true
candidate_net    = "models/candidate_v3.onnx"
centroid_net     = "models/centroid_v2.onnx"
motion_net       = "models/motion_v2.onnx"
recovery_net     = "models/recovery_v1.onnx"
strategy_policy  = "models/strategy_v1.onnx"
fallback_on_fail = true

[logging]
centroid_csv = "logs/centroid.csv"
metrics_json = "logs/run.json"
report_html  = "logs/report.html"

[requirements]                      # rows 16-20
acquisition_s     = 2.0
tracking_error_px = 10.0
target_loss_frac  = 0.05
reacquisition_s   = 1.0
min_fps           = 20.0

[[event]]                           # optional timeline
t_s = 8.0 ; action = "set_atmosphere" ; mode = "fog" ; ramp_s = 2.0
```

## 7.2 The motion algebra

Motion is a **sum of components**. Each has an analytic position AND velocity.

```
position(t) = Σ component_i(t)
velocity(t) = Σ component_i'(t)
```

| `kind` | Parameters | Position | Velocity |
|---|---|---|---|
| `constant` | `offset_px` | `p₀` | `0` |
| `linear` | `velocity_px_s` | `v·t` | `v` |
| `accel` | `accel_px_s2` | `½a·t²` | `a·t` |
| `sinusoid` | `axis, amplitude_px, period_s, phase_deg` | `A sin(ωt+φ)` | `Aω cos(ωt+φ)` |
| `circular` | `radius_px, period_s, phase_deg` | `r(cos ωt, sin ωt)` | `rω(−sin ωt, cos ωt)` |
| `lissajous` | `amplitude_px[2], freq_ratio, phase_deg` | `(A sin ωt, B sin nωt)` | `(Aω cos ωt, Bnω cos nωt)` |
| `spiral` | `r0_px, growth_px_s, period_s` | `(r₀+kt)(cos ωt, sin ωt)` | product rule |
| `ou_noise` | `sigma_px_s, tau_s` | OU process | filter state |
| `waypoints` | `points=[[t,x,y],…], interp` | Catmull-Rom | spline derivative |

**Spec row 12 mapping:**

| Required motion | Expressed as |
|---|---|
| Straight line | `linear` |
| Circular | `circular` |
| Figure of 8 | `lissajous` with `freq_ratio = 2.0` |
| Random | `ou_noise` |
| Spiral (opt) | `spiral` |
| Sinusoidal (opt) | `linear` + `sinusoid` stacked |
| User-defined (opt) | `waypoints`, or any stack |

The same components drive `[[disturbance.platform]]` (row 25).

```cpp
// world/motion_component.hpp
struct MotionState { double x, y, vx, vy; };

class IMotionComponent {
public:
    virtual ~IMotionComponent() = default;
    virtual MotionState eval(double t_s) const = 0;
    virtual bool is_stochastic() const { return false; }
    virtual void advance(double dt, Pcg32&) {}
    virtual const char* kind() const = 0;
};

class CompositeMotion {
    std::vector<std::unique_ptr<IMotionComponent>> parts_;
public:
    MotionState eval(double t_s) const {
        MotionState s{};
        for (const auto& p : parts_) {
            const auto c = p->eval(t_s);
            s.x += c.x; s.y += c.y; s.vx += c.vx; s.vy += c.vy;
        }
        return s;
    }
    void advance(double dt, Pcg32& rng) {
        for (auto& p : parts_) if (p->is_stochastic()) p->advance(dt, rng);
    }
};
```

**OU noise must use the exact discretisation**, not Euler — it must behave identically at any truth
rate:

```cpp
void OuComponent::advance(double dt, Pcg32& rng) {
    const double a = std::exp(-dt / tau_);
    const double s = sigma_ * std::sqrt(1.0 - a * a);
    vx_ = a * vx_ + s * rng.next_normal();
    vy_ = a * vy_ + s * rng.next_normal();
    x_ += vx_ * dt;  y_ += vy_ * dt;
}
```

## 7.3 Custom shapes

`shape.type = "mask"` + `shape.mask_file = "shapes/cross.png"`. Loaded with stb_image at scenario
start, resampled to `size_px × size_px` with 4×4 supersampling, stored as a `uint8` coverage mask.
**Loaded once; never touched during the run.**

## 7.4 Events

```toml
[[event]]
t_s = 8.0 ; action = "set_atmosphere" ; mode = "fog" ; ramp_s = 2.0
[[event]]
t_s = 12.0 ; action = "occlude_target" ; duration_s = 0.4
[[event]]
t_s = 30.0 ; action = "spawn_decoy" ; offset_px = [60.0, -40.0]
[[event]]
t_s = 45.0 ; action = "platform_gust" ; magnitude_px = 35.0 ; duration_s = 1.5
```

`action` is a validated enum. New actions require a C++ change — correct, since actions are behaviour.

## 7.5 Validation

```cpp
// scenario/schema.hpp
enum class ValueKind { Int, Float, Bool, String, Array2, ArrayN, Table };

struct FieldSpec {
    const char* path;        // "gimbal.max_pan_dps"
    ValueKind   kind;
    bool        required;
    double      min, max;    // bounds from the spec table
    const char* spec_row;    // "row 13"
};

extern const std::array<FieldSpec, N> kSchema;
Result<Scenario> load_scenario(const std::filesystem::path&);
```

Error format — **must cite the spec row**:

```
scenarios/bad.toml:41: gimbal.max_pan_dps = 14.0 is outside the permitted
  range [5.0, 10.0] (specification row 13).
```

---

# 8. FRAME SOURCES

## 8.1 The interface — define at CP 1.1

```cpp
// engine/frame_source.hpp
struct FrameTruth {
    int64_t tick;
    Angle2  boresight_true;
    struct T {
        uint32_t id;
        Pixel2   image_pos;     // sub-pixel, in camera coordinates
        Pixel2   screen_pos;    // sub-pixel, in screen coordinates
        Angle2   world_ang;
        Rate2    world_rate;
        bool     in_fov;
    };
    std::array<T, kMaxTargets> targets;
    uint8_t n;
};

struct SourceFrame {
    std::span<const uint8_t> pixels;
    int      width, height;
    int64_t  frame_index;
    double   timestamp_s;
    Angle2   commanded_boresight;   // what the controller asked for
    bool     has_truth;
    FrameTruth truth;               // metrics only — INV-1 forbids perception access
};

struct FrameGeometry { int w, h; double fps; int screen_w, screen_h; };

class IFrameSource {
public:
    virtual ~IFrameSource() = default;
    virtual bool next(Angle2 commanded_bore, SourceFrame& out) = 0;
    virtual FrameGeometry geometry() const = 0;
    virtual bool supports_pointing() const = 0;
    virtual const char* name() const = 0;
};
```

## 8.2 `SyntheticSource`

Renders the world, applies the damage chain, emits truth. See §6.2 B4 and §9.1–9.3.

## 8.3 Video sources

**Requirement (BP-2, 30%):** *"Each team will be given a few video files (.mp4) @30 fps, covering a
complete screen with noise and moving beacon spot. The software needs to bypass its PTZ camera and
take this video as an input to the coarse pointing system."*

The wording is ambiguous. **Implement both readings** — ~200 lines, removes all risk.

| Mode | Reading | Pointing |
|---|---|---|
| `video_screen` | The video is the 2000×2000 screen; the PTZ crops a 640×480 viewport and pans | Active |
| `video_direct` | The video is the camera feed itself; no cropping | Disabled |

**Auto-detect:** if video resolution ≫ camera resolution → `video_screen`; if ≈ camera resolution →
`video_direct`. Log the choice. Config overrides.

```cpp
class VideoScreenSource : public IFrameSource {
    cv::VideoCapture      cap_;
    DecodeThread          decoder_;       // fills a Ring<DecodedFrame, 8>
    CameraGeometry        geo_;
    int screen_w_, screen_h_;
    double fps_;
public:
    static Result<std::unique_ptr<VideoScreenSource>> open(const std::filesystem::path&,
                                                           const Scenario&);
    bool next(Angle2 commanded_bore, SourceFrame& out) override;   // bicubic crop
    bool supports_pointing() const override { return true; }
};
```

**Implementation requirements:**

1. **Decode on a separate thread**, filling a ring buffer. A 2000×2000 H.264 frame costs 5–10 ms;
   inline decode would consume a third of the budget. This is the *only* extra thread and it is safe
   because decode produces identical frames in identical order regardless of timing.
2. **Greyscale conversion at decode**, not per frame in perception.
3. **Probe the real fps** from the container; re-derive `camera_divisor`. Reject or adapt if it does
   not divide `truth_hz` cleanly. Never assume 30.
4. **Bicubic interpolation for the crop**, not bilinear — bilinear smooths peaks and biases centroids
   toward pixel centres, directly harming the graded metric.
5. **INV-8:** assert `noise` and `atmosphere` generators are disabled; warn if the config enables
   them.
6. **Report centroids in screen coordinates**, converting back through the crop offset.
7. **EOF is a clean termination** — finalise the report, exit 0.
8. **Never crash on a corrupt frame** — skip, log, continue.
9. Use **container timestamps**, not a counted index, for the metric time axis.

## 8.4 Crop error characterisation (CP 8.6)

Render a synthetic beacon at known sub-pixel positions, crop at many fractional offsets, measure how
much centroid error the crop alone contributes. Report the number. You are graded on centroid
accuracy; you must know how much of the error is your own resampling.

---

# 9. SIMULATION AND PERCEPTION

## 9.1 World — never materialise the 2000×2000 canvas

In `synthetic` mode the world is a **scene description**, not a raster. The camera splats visible
emitters directly into sensor coordinates at continuous sub-pixel positions.

- 13× less pixel work (307k instead of 4M)
- **No resampling** → exact ground truth, which is what makes centroid accuracy measurable
- Scales to many clutter sources

```cpp
// world/emitters.hpp
enum class ShapeKind   : uint8_t { Square, Circle, Gaussian, Mask };
enum class EmitterKind : uint8_t { Target, Decoy, Clutter, HotSpot };

struct EmitterSoA {
    std::vector<double>   x, y, vx, vy;
    std::vector<float>    intensity;
    std::vector<uint16_t> size_px;
    std::vector<uint8_t>  shape, kind;
    std::vector<int32_t>  motion_id, mask_id;
    size_t n = 0;
};
```

`query_visible` is a linear AABB scan — ~1 µs at n < 1000, faster than any spatial index. Add a grid
only above ~10,000 emitters, and measure first.

**Clutter is mandatory for credibility:** 50–500 static bright sources, ≥1 decoy beacon (a second
near-identical target), bright edges and gradients.

## 9.2 Exact-coverage splatting

```cpp
// camera/coverage.hpp
inline double overlap_1d(double a0, double a1, double b0, double b1) {
    return std::max(0.0, std::min(a1, b1) - std::max(a0, b0));
}
// square coverage(i,j) = overlap_1d(i,i+1,x0,x1) * overlap_1d(j,j+1,y0,y1)

inline double gauss_1d(double lo, double hi, double mu, double sigma) {
    const double s = 1.0 / (sigma * kSqrt2);
    return 0.5 * (std::erf((hi - mu) * s) - std::erf((lo - mu) * s));
}
// gaussian coverage(i,j) = gauss_1d(i,i+1,cx,s) * gauss_1d(j,j+1,cy,s)
```

- **Circle:** 4×4 supersampling on boundary pixels only (radius test finds interior/exterior).
- **Mask:** bilinear-sample the precompiled coverage mask.

**Motion blur:** split the exposure into `blur_substeps` (default 8), interpolating BOTH target
motion and boresight motion, weight each `1/N`.

## 9.3 Degradation chain (order matters)

```
1. Atmosphere      I ← α·I + β                    row 24
2. Shot noise      I ← Poisson(I·k)/k             row 21
3. Read noise      I ← I + Normal(0, σ)           rows 21-22
4. Fixed pattern   I ← I·prnu[p] + fpn[p]         seeded once at load
5. Salt & pepper   10% of pixels → 0 or 255       row 21
6. Hot/dead pixels fixed seeded positions
7. Clip and quantise to 8-bit
```

| Mode | α | β |
|---|---|---|
| Clear | 1.00 | 0 |
| Haze | 0.75 | +20 |
| Rain | 0.60 | +15 |
| Fog | 0.35 | +60 |
| Low light | 0.40 | −40 |

**Poisson** — branch on λ (note: the branch changes the number of RNG draws, which is fine because λ
is deterministic, but do not change the threshold between builds you intend to compare):

```cpp
inline double poisson(double lambda, Pcg32& rng) {
    if (lambda < 30.0) {
        const double L = std::exp(-lambda);
        double p = 1.0; int k = 0;
        do { ++k; p *= rng.next_double(); } while (p > L);
        return double(k - 1);
    }
    return lambda + std::sqrt(lambda) * rng.next_normal();
}
```

**Salt & pepper — geometric skip sampling** (10× fewer RNG calls, identical distribution):

```cpp
void salt_pepper(std::span<uint8_t> img, double p, Pcg32& rng) {
    if (p <= 0) return;
    const double log1mp = std::log(1.0 - p);
    size_t i = 0;
    while (true) {
        i += size_t(std::log(1.0 - rng.next_double()) / log1mp);
        if (i >= img.size()) break;
        img[i] = (rng.next_u32() & 1u) ? 255 : 0;
        ++i;
    }
}
```

**Jitter and platform motion perturb the TRUE BORESIGHT, never the pixels.** This is physically
correct (blur follows automatically) and it prevents the tracker from implicitly knowing its own
pointing error.

```
rate_urad_s = px_per_frame × ifov_urad × camera_hz
20 px/frame → 20 × 109.08 × 30 = 65,448 µrad/s = 3.75 °/s   ← LOG THIS AT STARTUP
```

## 9.4 Perception pipeline

```cpp
// perception/pipeline.hpp
struct Detection {
    Pixel2   centroid_image;
    Pixel2   centroid_screen;      // ← logged
    float    peak, integrated, snr;
    uint16_t area_px, bbox_w, bbox_h;
    uint16_t size_est_px;          // from the winning matched-filter scale
    float    fill_ratio, aspect;
    float    ml_score;
    float    centroid_sigma_est;   // our own confidence, in pixels
};

struct IPerception {
    virtual ~IPerception() = default;
    virtual void process(const SourceFrame&, const PerceptionParams&, Arena& frame,
                         std::vector<Detection>& out) = 0;
    virtual const char* name() const = 0;
};
// ClassicalPerception · MLAssistedPerception · HybridPerception  — runtime switchable
```

### 9.4.1 Median 3×3

19-operation sorting network. **Verify exhaustively before trusting.**

```cpp
#define MN(a,b) { const auto t = std::min(a,b); b = std::max(a,b); a = t; }
inline uint8_t median9(uint8_t p0,uint8_t p1,uint8_t p2,uint8_t p3,uint8_t p4,
                       uint8_t p5,uint8_t p6,uint8_t p7,uint8_t p8) {
    MN(p1,p2) MN(p4,p5) MN(p7,p8)
    MN(p0,p1) MN(p3,p4) MN(p6,p7)
    MN(p1,p2) MN(p4,p5) MN(p7,p8)
    MN(p0,p3) MN(p5,p8) MN(p4,p7)
    MN(p3,p6) MN(p1,p4) MN(p2,p5)
    MN(p4,p7) MN(p4,p2) MN(p6,p4)
    MN(p4,p2)
    return p4;
}
```

Branch-free min/max → vectorises with `_mm256_min_epu8` / `_mm256_max_epu8`, 32 px per instruction.
0.35 ms → 0.04 ms. Keep a scalar fallback, test bit-identical.

**Rationale:** salt-and-pepper noise is always *isolated single pixels* (30,720 of them vs a 100-px
beacon — 307:1). A median removes nearly all of it while barely touching a solid blob.

### 9.4.2 Background removal — van Herk / Gil-Werman

`top_hat = image − opening(image)`, where `opening = dilate(erode(image))`.

van Herk gives **O(1) per pixel regardless of structuring-element size** — 3 comparisons per pixel
per dimension. A 25×25 SE costs the same as 5×5.

```
For a 1-D min over window k, split the row into blocks of length k:
  forward[]  = running min from each block start
  backward[] = running min from each block end
  min(i .. i+k-1) = min(backward[i], forward[i+k-1])
```

SE size = `target_size_px × 2 + 5`, clamped to [15, 51]. Store the top-hat as `int16_t`.

**Spatial, not temporal:** the camera slews, which invalidates any frame-history background model
instantly. Spatial is slew-invariant by construction.

### 9.4.3 Summed-area tables — the keystone

```cpp
// perception/sat.hpp
struct SummedArea {
    std::span<int64_t>  s;     // (W+1)*(H+1)
    std::span<uint64_t> s2;
    int W, H;
    inline int64_t box_sum(int x0,int y0,int x1,int y1) const {
        const int W1 = W + 1;
        return s[y1*W1+x1] - s[y0*W1+x1] - s[y1*W1+x0] + s[y0*W1+x0];
    }
    inline uint64_t box_sum_sq(int x0,int y0,int x1,int y1) const;
};
void build_sat(std::span<const int16_t> img, SummedArea& out);
```

**Integer accumulators, never float.** The four-corner subtraction loses precision badly in float,
and integers are bit-identical everywhere (INV-3). `int64_t` is ample: worst case ≈ 1.0e10.

| Consumer | Cost |
|---|---|
| Matched filter at any size | 4 lookups |
| 6 scales | 24 lookups |
| CFAR local mean | 8 lookups |
| CFAR local variance | 8 more |

### 9.4.4 Multi-scale matched filter

The optimal detector for a known shape in additive noise is correlation with that shape. For a square
beacon, that **is** a box sum.

Beacon size is user-defined 5–20 px and **unknown in video mode** → run 6 scales {5, 8, 11, 14, 17,
20}, take the max. Still O(1). The winning scale is a free size estimate that feeds the centroid
window.

```cpp
float matched_response(const SummedArea& sat, int x, int y, int k) {
    const int h = k / 2;
    return float(sat.box_sum(x-h, y-h, x-h+k, y-h+k)) / std::sqrt(float(k*k));
}
```

### 9.4.5 CFAR

```cpp
inline bool cfar(const SummedArea& sat, int x, int y, int T, int G, float k, float& snr) {
    const int t = T/2, g = G/2;
    const double n  = double(T*T - G*G);
    const double mu = double(sat.box_sum   (x-t,y-t,x+t+1,y+t+1)
                           - sat.box_sum   (x-g,y-g,x+g+1,y+g+1)) / n;
    const double e2 = double(sat.box_sum_sq(x-t,y-t,x+t+1,y+t+1)
                           - sat.box_sum_sq(x-g,y-g,x+g+1,y+g+1)) / n;
    const double sd = std::sqrt(std::max(e2 - mu*mu, 1e-6));
    snr = float((double(sat.box_sum(x,y,x+1,y+1)) - mu) / sd);
    return snr > k;
}
```

Defaults: `T = 61`, `G = 31`, `k = 3.9`.

**Why CFAR:** five weather modes change background level and contrast by 3×, and the evaluators'
videos have statistics you have never seen. A fixed threshold tuned for one finds nothing in another.
CFAR measures the background from the image itself. And `k` maps to a false-alarm probability
(`Pfa = Q(k)`; `k = 3.9` → `Pfa ≈ 5×10⁻⁵`), so you can state and defend it.

**Guard band is essential** — without it a bright beacon contaminates its own background estimate.

**Edges:** clamp boxes to the image and adjust `n`. Never skip a border — a beacon near the edge is
exactly when you are about to lose it.

### 9.4.6 Run-length grouping

```cpp
struct Run { int16_t y, x0, x1; int32_t label; };
struct BlobAccum {
    int64_t n;
    double  sw, swx, swy, swxx, swyy;
    int16_t x0, y0, x1, y1;
    float   peak;
};
```

Pass 1: extract runs per row (SIMD-scannable). Pass 2: union adjacent runs between rows. Pass 3:
flatten union-find, accumulate moments. ~0.05 ms — far better than full-image labelling.

**Determinism:** label numbering depends on merge order, deterministic only with a fixed scan order.
Do not parallelise the row scan.

**Bounded output:** both the run table and the blob table are sized once and
truncate rather than grow. See amendment §14.0e for why the blob table's
earlier "let it grow" policy was wrong and what replaced it.

### 9.4.7 Gating

```cpp
bool passes_gate(const BlobAccum& b, const PerceptionParams& p) {
    const int   area = int(b.n);
    const int   w = b.x1 - b.x0 + 1, h = b.y1 - b.y0 + 1;
    const float fill = float(area) / float(w * h);
    const float ar   = float(std::max(w,h)) / float(std::min(w,h));
    return area >= p.min_area && area <= p.max_area   // 12 .. 800, from the 5-20 px spec range
        && fill >= 0.35f                               // rejects diagonal noise chains
        && ar   <= 3.0f;                               // rejects streaks
}
```

---

# 10. CENTROIDING, TRACKING, SEARCH, CONTROL

## 10.1 Centroiding — 60% of the marks

### 10.1.1 The theoretical limit

```
σ_centroid ≳ w / (2 · SNR)
```

| Condition | Integrated SNR | Best achievable |
|---|---|---|
| Clear | ~50 | ~0.10 px |
| Haze | ~30 | ~0.17 px |
| Fog | ~17 | ~0.29 px |
| Fog + low light + max noise | ~7 | ~0.71 px |

Plot measured accuracy against this bound in the report.

### 10.1.2 Four estimators (all implemented, runtime switchable)

```cpp
// perception/centroid/estimators.hpp
enum class CentroidKind { CoM, WindowedCoM, SurfaceFit, MatchedPeak, Learned };

Pixel2 centroid_com        (const BlobAccum&);
Pixel2 centroid_windowed   (std::span<const int16_t> tophat, int W, Pixel2 peak, int win);
Pixel2 centroid_surface_fit(std::span<const int16_t> tophat, int W, Pixel2 peak);
Pixel2 centroid_matched_pk (const SummedArea&, Pixel2 peak, int scale);
```

| Estimator | Strength | Weakness |
|---|---|---|
| `CoM` | fast, unbiased with perfect background removal | background residual pulls it |
| `WindowedCoM` | less noise | stronger pixel-locking bias |
| `SurfaceFit` | robust to asymmetry and nearby clutter | slower |
| `MatchedPeak` | best at low SNR (noise already integrated away) | limited by response-grid resolution |

**Default:** background-subtracted `WindowedCoM` with bias correction. SAT switches to `SurfaceFit`
for large or asymmetric blobs and `Learned` at low SNR.

### 10.1.3 S-curve bias correction — the highest-leverage single item

**Problem:** centre-of-mass estimators have a *systematic* error that depends on where inside a pixel
the true centre lies. Plot error against true sub-pixel offset and you get a repeating S-shape. It is
a bias, not noise — averaging does not remove it — and at high SNR it dominates the random error.

Causes: finite summation window truncating the profile asymmetrically, background-subtraction
residual, median-filter profile reshaping.

**Procedure:**

```
1. Generate beacons at 200 known sub-pixel offsets across one pixel,
   for each of {6 sizes} × {8 SNR bins} × {4 estimators}.
2. Measure error(true_offset) → the S-curve.
3. Fit  bias(u) ≈ a·sin(2πu) + b·sin(4πu)
4. Store (a, b) per (size, snr_bin, estimator) in a compiled-in table.
5. At runtime: estimate → look up bias at the estimated fractional part →
   subtract → iterate once.
```

**Expected gain: 2–5× reduction in RMS centroid error at high SNR.** Put the before/after S-curve
plot in the report.

### 10.1.4 Uncertainty

```cpp
float centroid_sigma(float snr, int size_est_px) {
    return std::max(0.03f, float(size_est_px) / (2.0f * std::max(snr, 1.0f)));
}
```

Two uses: feeds the Kalman `R` (§10.2), and can be *validated* — over 100k frames, is the actual
error distribution consistent with the claimed sigma? That calibration plot is an advanced result.

## 10.2 Tracking

**The filter is a plain linear Kalman filter, not an EKF.** After `unproject` + commanded boresight,
the measurement directly observes position.

```
state x = [az, el, az_rate, el_rate]ᵀ            µrad, µrad/s
      F = [[1,0,T,0],[0,1,0,T],[0,0,1,0],[0,0,0,1]]
      H = [[1,0,0,0],[0,1,0,0]]
      R = diag(σ², σ²)   with σ = centroid_sigma × ifov_urad     ← adaptive
      Q = q · [[T³/3,0,T²/2,0],[0,T³/3,0,T²/2],[T²/2,0,T,0],[0,T²/2,0,T]]
```

### IMM — three models

```cpp
class ImmFilter {
    static constexpr int M = 3;          // CV, CA, CT
    std::array<KalmanFilter, M> f_;
    std::array<double, M>       mu_;     // mode probabilities
    double pi_[M][M];                    // Markov transitions, strongly diagonal (0.95 self)
public:
    void   predict(double T);
    void   update(const Angle2& z, double R);
    Angle2 position() const;             // probability-weighted
    Rate2  rate() const;
    int    dominant_mode() const;
};
```

Four steps: mix → mode-matched filter → mode probability update from innovation likelihood →
combine.

**Why it matters here:** at the figure-8 crossing, acceleration reverses sign and single-model
filters overshoot every lap. Figure-8 is a *mandatory* spec motion, so this is a before/after demo on
graded functionality. The live mode-probability plot is one of the best GUI elements.

### Association and lifecycle

- Gate: Mahalanobis `d² < χ²(2, 0.99) = 9.21`
- Single target: nearest neighbour. Multi-target: global nearest neighbour (Hungarian), ~100 lines,
  prevents identity swaps.

```
Tentative ──3 hits in 5 frames──▶ Confirmed ──miss──▶ Coasting
    │                                 ▲                   │
 5 frames, no confirm                 └────any hit────────┤
    │                                                15 misses
    ▼                                                     │
 Deleted ◀────────────────────────────────────────────────┘
```

**`lock_retention_rate = frames Confirmed ÷ frames beacon in view`** — graded in BP-2.

During Coasting the covariance grows → the gate widens automatically → short dropouts recover with no
special case.

### AMENDMENT (Stage 6) — what nearest neighbour minimises

The gate above is implemented exactly as written: `d^2 < 9.21`, chi-square with two degrees of
freedom, and the empirical acceptance rate over 20,000 draws from the filter's own predicted
distribution measures 98.8%.

The ASSOCIATION rule needed one refinement, recorded here because "nearest neighbour" above does not
say in what metric. Selecting the smallest `d^2` is wrong once §10.1.4's adaptive `R` is switched on
(CP 6.5), and wrong in the worst direction: `d^2` divides the displacement by the uncertainty a
candidate CLAIMS, so a weak, smeared detection reporting a large sigma gets a small `d^2` for free.
Measured on two candidates at identical offsets from the prediction, one crisp and one smeared:
`d^2` of 0.14 against 5.64 — a 40x preference for the least trustworthy thing in the frame.

Association therefore minimises the standard NN likelihood score

```
score = d^2 + ln|S|          ( = -2 ln L, up to a constant )
```

where the normalisation term charges a candidate for the uncertainty it claims. **Gating still uses
`d^2` alone**, because 9.21 is a probability statement about one measurement and means nothing on a
scale that includes `ln|S|`.

One further note, since the intuition is common and does not hold here: with the CWNA `Q` above and
`R = sigma^2 I`, azimuth and elevation are exactly independent, so the predicted position covariance
is ISOTROPIC at every step — `P(0,0) == P(1,1)`, `P(0,1) == 0`. The gate is a circle, not an ellipse
aligned with the velocity. Anisotropy arrives with the coordinate-turn model at CP 10.5, which is why
the filter is written in full 4x4 form rather than as two decoupled 2x2 filters.

### Priority / multi-target

```cpp
enum class TargetPolicy { SingleLock, Priority, MultiTarget };

float priority_score(const Track& t, const Weights& w) {
    return w.snr * norm(t.mean_snr) + w.stability * norm(t.hit_ratio)
         + w.centrality * (1 - norm(t.dist_from_boresight)) + w.age * norm(t.age_s);
}
// Switch ONLY if score_new > 1.25 × score_current, sustained 15 frames.
```

**The hysteresis is mandatory** — without it two similar targets make the camera oscillate, which
looks terrible live.

MultiTarget mode revisits the track with the *largest covariance* first (information-theoretic
scheduling).

## 10.3 Plant

```cpp
// plant/gimbal.hpp
struct GimbalParams {
    double max_rate_urad_s, max_accel_urad_s2;
    double tau_s = 0.020, latency_s = 0.010, encoder_lsb = 20.0, resonance_hz = 0.0;
};

class GimbalAxis {
    double pos_ = 0, rate_ = 0;
    Ring<double, 64> delay_;
    GimbalParams p_;
    uint64_t sat_frames_ = 0, total_frames_ = 0;
public:
    void step(double cmd_rate, double dt) {
        delay_.push(cmd_rate);
        const int d = int(std::lround(p_.latency_s / dt));
        const double delayed = delay_.at_back(d);
        const double want = std::clamp(delayed, -p_.max_rate_urad_s, p_.max_rate_urad_s);
        double accel = std::clamp((want - rate_) / p_.tau_s,
                                  -p_.max_accel_urad_s2, p_.max_accel_urad_s2);
        const double rate_mid = rate_ + 0.5 * accel * dt;     // midpoint integration
        rate_ = std::clamp(rate_ + accel * dt, -p_.max_rate_urad_s, p_.max_rate_urad_s);
        pos_ += rate_mid * dt;
        ++total_frames_;
        if (std::abs(rate_) >= p_.max_rate_urad_s * 0.999) ++sat_frames_;
    }
    double position()      const { return quantise(pos_, p_.encoder_lsb); }  // controller sees this
    double true_position() const { return pos_; }                            // metrics only
    double saturation_frac() const { return double(sat_frames_) / double(total_frames_); }
};
```

Saturation must be applied **inside** the step, not after. `position()` vs `true_position()` is
deliberate — the controller sees the quantised encoder, metrics see truth.

## 10.4 Control

```cpp
double AxisController::compute(double error, double target_rate_est,
                               double platform_rate_est, double dt, bool saturated) {
    const double d = -(meas_ - prev_meas_) / dt;      // derivative on measurement, not error
    prev_meas_ = meas_;
    if (!saturated) integ_ += error * dt;             // conditional integration = anti-windup
    integ_ = std::clamp(integ_, -g_.i_limit, g_.i_limit);
    return g_.kp * error + g_.ki * integ_ + g_.kd * d
         + g_.k_ff * target_rate_est                  // ← biggest single win
         - platform_rate_est;
}
```

**Velocity feedforward is the highest-value ten lines in the project.** Pure feedback lags by
`speed ÷ bandwidth`. A beacon at 200 px/s with a 3 Hz loop leaves ~11 px of lag — already over the
10 px budget before any noise.

**Anti-windup is essential** because saturation is frequent here (§1.3).

**Smith predictor** (optional, enable last): run a plant model forward by `latency_s`, control against
the prediction. Worth 30–50% bandwidth but amplifies model error.

### Mode FSM

```
Idle ─▶ Search ─▶ Detect ─▶ Acquire ─▶ Track ─▶ Handover
          ▲                              │
          └────── Reacquire ◀────────────┘
                      │
                      ▼
                    Safe
```

| Transition | Criterion |
|---|---|
| Idle → Search | run start |
| Search → Detect | ≥1 gated detection |
| Detect → Acquire | track reaches Tentative |
| Acquire → Track | track reaches Confirmed |
| Track → Reacquire | track enters Coasting |
| Reacquire → Track | track returns to Confirmed |
| Reacquire → Search | track Deleted |
| Track → Handover | RMS error < capture_range/3 for 30 consecutive frames |
| any → Safe | loss timeout exceeded |

**Handover stub** (~80 lines): a quadrant detector with ~1 mrad capture range. Report handover success
rate and time-to-handover. Completes the PAT narrative the PS only implies.

## 10.5 Search

**The geometric bound — state it openly:**

```
Screen 12.5°×12.5°, camera 4°×3° → 20 tiles → ~74.5° of travel
At 5 °/s  ≈ 15 s        At 10 °/s ≈ 7.5 s
P(beacon visible at t=0) = 7.68%
SPEC SAYS ≤ 2 s
```

**Report both metrics, clearly labelled:**

```
acquisition_cold_s   : run start → first Confirmed, beacon at random position
acquisition_in_fov_s : beacon first enters view → first Confirmed   ← intended meaning; ~0.3 s
```

Deriving the bound and engineering around it is worth more under "Understanding of the problem" than
a suspicious 1.8 s.

```cpp
class ProbabilityGrid {
    static constexpr int NX = 32, NY = 32;
    std::array<float, NX*NY> p_;
public:
    void   reset_uniform();
    void   reset_from_prior(Pixel2 mean, double sigma_px);
    void   observe(const Aabb& fov_cells, float p_detect);   // NEGATIVE INFORMATION
    void   diffuse(double dt, double target_speed_px_s);
    Pixel2 best_look(Angle2 current, double max_rate) const;
};
```

`observe` multiplies covered cells by `(1 − p_detect)` and renormalises. Looking somewhere and seeing
nothing is evidence; raster scans discard it.

`best_look` maximises `probability_mass_in_fov(cell) / (travel_time_to(cell) + dwell_s)`. The
denominator prevents thrashing across the field.

```cpp
enum class SearchStrategy { Raster, Spiral, Probabilistic, CampAndWait, Adaptive };
```

**Camp-and-wait:** for a beacon on a repeating path, past a certain speed, staying still beats
searching. Benchmark the crossover against target speed — a small, genuine, defensible result.

## 10.6 SAT supervisor

```cpp
struct Conditions {                 // 12 features, → StrategyPolicy
    float bg_sigma, target_contrast, integrated_snr;
    float detection_rate, ml_confidence_mean, centroid_sigma_mean;
    float saturation_frac, innovation_nis;
    float track_age_s, gate_utilisation;
    float imm_mode_ca, imm_mode_ct;
};

struct Strategy {
    PerceptionKind perception;
    CentroidKind   centroider;
    float          cfar_k;
    FilterKind     filter;
    PredictorKind  predictor;
    ControlGains   gains;
    SearchStrategy search;
    float          q_scale;
};

class SatSupervisor {
    Strategy   current_;
    int        dwell_ = 0;
    Conditions smoothed_;
    std::optional<OnnxModel> policy_;
public:
    const Strategy& update(const Conditions&, int frame);
};
```

**Three mandatory properties:**

1. **EMA smoothing + hysteresis.** Never react to a single frame. `kMinDwell = 30` frames (1 s)
   between switches. Without this the system chatters and degrades tracking.
2. **Bumpless switching.** Carry the integrator across a gain change, or the switch kicks the loop.
3. **Rule-table fallback** when the model is absent or `--no-ai`:

```cpp
Strategy rule_table(const Conditions& c) {
    Strategy s = kDefault;
    if (c.integrated_snr <  8.0f) { s.perception = MLAssisted; s.centroider = Learned;
                                    s.cfar_k = 3.2f; s.q_scale = 1.5f; }
    if (c.integrated_snr > 15.0f) { s.perception = Classical;  s.centroider = WindowedCoM;
                                    s.cfar_k = 4.2f; }
    if (c.saturation_frac > 0.25f){ s.gains.k_ff *= 1.3f; s.gains.ki *= 0.5f; }
    if (c.innovation_nis > kHi)   { s.q_scale *= 1.4f; }   // filter over-confident
    if (c.innovation_nis < kLo)   { s.q_scale *= 0.8f; }
    return s;
}
```

Log every switch with its trigger. GUI shows a strategy timeline; the report shows per-scenario
strategy occupancy. That turns "adaptive" from a claim into data.

---

# 11. ML INTEGRATION CONTRACT

> Training, datasets and architectures are specified in **`SAT-ML.md`**. This section is the C++
> side only.

## 11.1 The wrapper

```cpp
// ai/onnx_model.hpp
class OnnxModel {
    Ort::Env     env_;
    Ort::Session session_;
    std::vector<int64_t> in_shape_, out_shape_;
public:
    static Result<OnnxModel> load(const std::filesystem::path&);
    void run(std::span<const float> in, std::span<float> out);
    bool valid() const;
};
```

**Session options — required for INV-3:**

```cpp
Ort::SessionOptions opts;
opts.SetIntraOpNumThreads(1);
opts.SetInterOpNumThreads(1);
opts.SetGraphOptimizationLevel(ORT_ENABLE_BASIC);   // NOT ORT_ENABLE_ALL
opts.SetExecutionMode(ORT_SEQUENTIAL);
```

`ORT_ENABLE_ALL` may fuse floating-point operations differently across versions.

## 11.2 The five models and where they plug in

| Model | Plugs into | Input | Output | Runs | Budget | Fallback |
|---|---|---|---|---|---|---|
| `CentroidNet` | §10.1 step B13 | 15×15 patch + [snr, size] | (dx, dy) | 1× per frame | 0.02 ms | bias-corrected WindowedCoM |
| `CandidateNet` | §9.4 step B14 | 15×15 patch + 6 scalars | 4-class softmax | ~20× per frame | 0.15 ms | shape gate §9.4.7 |
| `RecoveryNet` | step B15 | full frame ÷2 | 80×60 heatmap | ≤10 Hz, only when CFAR empty | 2.5 ms | continue search pattern |
| `MotionNet` | §10.2 step B19 | 30×4 track history | 15-step forecast + regime | 1× per frame | 0.05 ms | IMM prediction |
| `StrategyPolicy` | §10.6 step B23 | 12-feature Conditions | strategy index | 1× per frame | 0.005 ms | `rule_table()` |

## 11.3 Loading and failure (INV-7)

```cpp
struct AiStack {
    std::optional<OnnxModel> centroid, candidate, recovery, motion, strategy;
    bool enabled = true;

    void load(const AiConfig& cfg) {
        if (!cfg.enabled) { enabled = false; return; }
        auto try_load = [&](const std::string& path, const char* name)
                        -> std::optional<OnnxModel> {
            if (path.empty()) return std::nullopt;
            auto r = OnnxModel::load(path);
            if (!r) {
                spdlog::warn("model '{}' failed to load ({}), using classical fallback",
                             name, r.error());
                if (!cfg.fallback_on_fail) throw std::runtime_error("model load failed");
                return std::nullopt;
            }
            return std::move(*r);
        };
        centroid  = try_load(cfg.centroid_net,    "CentroidNet");
        candidate = try_load(cfg.candidate_net,   "CandidateNet");
        // ...
    }
};
```

**Every call site must handle `std::nullopt` by calling the classical path.** A missing model file is
a warning, never a crash.

## 11.4 Reproducibility with ML

Network output is deterministic given fixed threads and a pinned optimisation level, but it is
floating-point, so a different ONNX Runtime *version* could shift the last bits. Handle honestly:

- **Hash the discrete decisions**, not raw model outputs — selected candidate index, chosen strategy,
  track confirmation. Integers, stable.
- **Pin the ONNX Runtime version** in `vcpkg.json`; record it in every `run.json`.
- **`--no-ai` is bit-exact** and is what the reproducibility CI job uses.

Document this precisely. Stating what is and is not bit-reproducible is more credible than an
unqualified claim.

---

# 12. GUI (explicitly graded)

| Panel | Content |
|---|---|
| Camera view | 640×480 frame; overlays: detections, gate ellipse, track history, predicted path, boresight crosshair, centroid marker with uncertainty circle |
| Screen overview | 2000×2000 downsampled, FOV rectangle, true path, probability-grid heatmap |
| **Centroiding error** | Live plot with running RMSE — the graded metric |
| Tracking error | Az/el in px and µrad, with the 10-px budget line drawn |
| Predicted vs actual | Projected path overlaid on the true path |
| IMM modes | Stacked area of mode probabilities |
| SAT timeline | Active strategy with switch markers and triggers |
| Mode FSM | Live-highlighted state graph |
| Parameters | Every TOML key, live-editable; scenario load/save |
| Metrics | Running compliance table, latency percentiles |
| Input source | Active source; in video mode: filename, frame index, decode rate |

**Live algorithm switching** — dropdowns for perception (Classical / MLAssisted / Hybrid), filter
(CV / CA / IMM), controller (P / PID / PID+FF). Being able to turn feedforward off mid-demo and watch
the error trace blow up is worth more than any table.

OpenGL use is minimal: one `GL_R8` texture upload per frame, one quad, ImGui for the rest.

---

# 13. METRICS, LOGS, REPORTS

## 13.1 Definitions — verbatim into `docs/METRICS.md` and the report

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

## 13.2 `centroid.csv` — the graded artifact

```csv
# SAT centroid log v1
# source=bench/clip_03.mp4  mode=video_screen  build=a3f21c9  utc=2026-09-14T10:22:31Z
# screen_px=2000x2000  camera_px=640x480  fps=30.000  ifov_urad=109.08
# onnxruntime=1.17.1  ai_enabled=true
# columns: frame,time_s,state,cx_screen,cy_screen,cx_cam,cy_cam,sigma_px,snr,area_px,size_est_px,bore_x,bore_y
0,0.0000,SEARCH,,,,,,,,,1000.000,1000.000
1,0.0333,DETECT,1423.812,674.209,331.812,180.209,0.142,38.4,98,10,1000.000,1000.000
2,0.0667,ACQUIRE,1428.104,675.881,334.104,181.881,0.138,39.1,101,10,1002.410,1000.612
```

**Rules:**
- Blank centroid columns when no detection (INV-9). Never stale, never interpolated.
- Include `sigma_px` — your own uncertainty.
- Include **both** screen and camera coordinates; you do not know which they want.
- The header alone must be sufficient to interpret the file.

Write the header at file open (A9), not at close — so an aborted run still leaves a valid file.

## 13.3 Compliance matrix

```
REQUIREMENT COMPLIANCE MATRIX     1,500 runs · seeds 1-50 · build a3f21c · AI enabled

Row  Requirement            Spec       Result                    Status
────────────────────────────────────────────────────────────────────────
16   Acquisition (in-view)  ≤ 2.0 s    0.31 s   (p95 0.58 s)     PASS
16   Acquisition (cold)     ≤ 2.0 s    4.7  s   (p95 8.9  s)     BOUND DERIVED — §10.5
17   Tracking error         ≤ 10 px    3.2  px  (p95 7.1  px)    PASS
17     └ fog + figure-8     ≤ 10 px    6.4  px  (p95 11.8 px)    MARGINAL
18   Target loss            < 5 %      1.8  %   (p95 4.1  %)     PASS
19   Re-acquisition         ≤ 1.0 s    0.21 s   (p95 0.44 s)     PASS
20   Processing speed       ≥ 20 FPS   412  FPS (p5  380  FPS)   PASS

Centroiding error (graded)          0.08 px RMSE, bias +0.004 px, p95 0.19 px
  └ fog + low light                 0.41 px RMSE, bias +0.021 px, p95 0.88 px
```

**An honest MARGINAL is stronger than a suspicious all-green.** Generate the same matrix with
`--no-ai` and include both — that comparison is the argument that ML did something.

## 13.4 CLI

```
sat-tracker                                                  GUI, default scenario
sat-tracker --scenario s.toml --seed 42
sat-tracker --video clip.mp4 [--truth t.csv] --out logs/     ← BP-2 entry point, must work bare
sat-tracker --headless --scenario s.toml --out logs/
sat-tracker --sweep sweep.toml --jobs 8 --out results/
sat-tracker --gen-dataset --sweep ds.toml --out data/        see SAT-ML.md
sat-tracker --verify-reproducibility --scenarios dir/ --seeds 1..20
sat-tracker --fuzz-scenarios 5000 --seed 1
sat-tracker --no-ai
sat-tracker --bench
```

---

# 14. CHECKPOINT ROADMAP

**Rules:** one checkpoint at a time, in order. Write the test first. Do not proceed until the
acceptance test passes. Four ★ GATEs stop all other work if they fail.

## 14.0j AMENDMENT — `Atmosphere` was a photometric model, not a turbulence
## one; Kolmogorov AoA jitter and log-normal scintillation are now beside it

**Status:** adopted, extending §9.3's row-24 treatment. **Applies to:** §9.3
(the damage chain), §3.2 (the scenario schema), INV-3, INV-4.

**What was wrong.** §9.3 implements spec row 24 as a five-value `Atmosphere`
enum applying an affine transform to the frame, `I <- alpha*I + beta`. Row 24
does say "user-defined reduction in contrast and brightness", so this is a
defensible *reading* — but it is a photometric model, and PS 26169 is a
free-space optical communication problem whose background is about
atmospheric **propagation**. "Fog" that only dims the image models none of the
two effects that dominate an FSOC link:

* **Angle-of-arrival jitter.** Refractive-index fluctuations tilt the arriving
  wavefront, so the beacon's apparent direction moves. This is a POINTING
  disturbance.
* **Scintillation.** The same fluctuations focus and defocus the beam, so the
  received irradiance fluctuates. This is what makes an FSOC link drop out.

The critical audit records this as P2-1 and rates the judge risk **High**: a
Dept. of Space evaluator will ask about the Fried parameter, the Greenwood
frequency and C²ₙ, and until now the honest answer was "not modelled".

**The partial defence that already existed, and why it was not enough.** Row
23's jitter is resampled once per camera frame at the boresight with a uniform
±20 px bound, which is *functionally* a crude AoA model. But it is white (no
temporal spectrum), uniform (not Gaussian), and identical in both axes. The
variance was arguable; the spectrum was absent.

**What was added.** `degrade/turbulence.hpp` and an opt-in
`[atmosphere.turbulence]` scenario block carrying r₀, the aperture, the
wavelength, the transverse wind and the scintillation index — quantities an
optical engineer can argue with, rather than tuning constants.

* **AoA** is added to the TRUE boresight beside row 23's jitter and row 25's
  platform motion, for the reason `degrade/disturbance.hpp` gives at length:
  applying it as a pixel shift would leave the commanded boresight equal to the
  true one and hand the tracker its own pointing error. Its one-axis variance
  is the standard full-aperture tilt result
  σ² = 0.182 (λ/D)² (D/r₀)^(5/3), which at r₀ = 5 cm, D = 1 m, λ = 1550 nm is
  **8.02 µrad**.
* **Scintillation** multiplies the beacon's irradiance *before* the splat, so
  it flows through the exposure integration, the damage chain, the SNR gate and
  the detector as a real irradiance fluctuation would. It is log-normal with
  unit mean, temporally correlated at r₀/V, and drawn INDEPENDENTLY per graded
  emitter — a decoy that faded in step with the beacon would stay perfectly
  separable by brightness ratio, which is the opposite of the intended
  difficulty.

**The part that took three attempts, and is the reason to believe the model.**
The variance is the easy half. The claim worth making is the **spectrum**:
Kolmogorov turbulence gives wavefront tilt a temporal PSD of f^(−11/3), and
that exponent is not a free parameter. Synthesising it is where the work was:

| Attempt | Result |
|---|---|
| Kasdin FIR directly at β = 11/3 | **f^(−1.99)** — the FIR coefficients grow as k^0.83, so a truncated filter reports the truncation's own shape |
| Split as f^(−5/3) × f^(−2) | **f^(−2.41)** — k^(−1/6) decays far too slowly to converge in 256 taps |
| Split as f^(+1/3) × (f^(−2))² | **f^(−3.59)** against an ideal −3.667 ✅ |

The third form puts the FIR at β′ = −1/3, a fractional *differentiator* whose
coefficients decay as k^(−7/6) and converge well inside the filter, followed by
two leaky one-pole integrators. Each stage is then inside the range where it is
accurate. `tests/degrade/test_turbulence.cpp` fits the slope and asserts it;
the first two attempts both fail that test, which is exactly why it exists.
(A fourth trap: the test itself initially reported f^(−2.51) for a process that
is f^(−3.64) by construction, because a rectangular window's sidelobes fall
only as f^(−2) and a periodogram measured through one stops measuring the
signal and starts measuring the window. It now uses a Nuttall window.)

**Stated simplifications**, because saying so is worth more than pretending
otherwise: below the integrators' corner the synthesised spectrum flattens
rather than continuing as f^(−2/3) (this is outer-scale saturation, a real
effect, at a defensible 5.4 s crossing time); the two axes are independent; and
aperture averaging is folded into the configured scintillation index rather
than derived from D.

**Measured, `spec_defaults.toml` vs `turbulence.toml`, seed 42, 30 s:**

| | baseline | + turbulence |
|---|---:|---:|
| centroiding RMSE (image) | 0.1975 px | **0.3144 px** |
| centroiding max | 0.8357 px | **1.5515 px** |
| lock retention | 99.78 % | 99.67 % |
| target loss (post-acquisition) | 0.00 % | 0.11 % |
| tracking error (steady state) | 16.939 px | 16.944 px |

The tracking row is unchanged and that is the correct result, not a null
result: row 23's ±20 px jitter dominates the pointing error by two orders of
magnitude, so 8 µrad of AoA cannot move it. The effect lands where it should —
on the **detector**, through scintillation-driven SNR fluctuation, which is
precisely the mechanism an FSOC evaluator would predict.

**Off by default.** `enabled` is false, so every scenario written before this
existed, and every number committed from one, is bit-identical: a disabled
model draws nothing from `Stream::Atmosphere` and the render path pays one
hoisted boolean. INV-3 is asserted both ways by the tests, and
`--verify-reproducibility` passes unchanged.
## 14.0i AMENDMENT — MotionNet SmoothL1 is on scaled residuals, and factory windows must be on the beacon

**Status:** adopted on branch `motion-predictor-ML-integration`. **Applies to:** SAT-ML.md §6.4, §6.5.

SAT-ML §6.4 writes `smooth_l1_loss(..., beta=0.5)` and says the unit is µrad. At this plate scale one pixel is ~109 µrad and a +5-frame turn residual is thousands of µrad, so β=0.5 µrad is deep in the saturated linear region on every sample. AdamW at 3e-4 then steps the O(1) GRU head by microradians per update and the residual stays ~0. Measured: 20 epochs moved val loss from 1394.39 to 1394.30 and test RMSE matched CV to three figures.

**What changed.** `MotionNet.forward` multiplies the linear head by `RESIDUAL_SCALE` (1e4 µrad) so the tensor the rest of the system sees is still a µrad residual over CV. The loss divides both `(CV + residual)` and the target by that same scale before SmoothL1. 1e4, rather than the 1e5 input normaliser, puts a few-thousand-µrad turn residual near β=0.5, so the 0.3·CE term does not drown the forecast. Inference, ONNX, and the C++ compose path are unchanged: they add a µrad residual to CV. Input normalisation stays 1e5 (`TRACK_POS_SCALE` / `kPosScale`).

**Test mix.** The hold-out file is an unseen figure-8, capped at `holdout_seed_end`, plus seeds ≡ 9 (mod 10) of every regime. A hold-out that is only a faster straight line makes §6.6's 35% +15 bar impossible, because CV is already the conditional mean on a line.

**Factory filter.** `--gen-dataset` rows where the live track is more than 12_000 µrad from FrameTruth are not windowed. Those rows are a lock on clutter; the history does not determine the truth future, so they cannot teach §6.6. Truth is used only as this gate, never as a network input (INV-9).

## 14.0h AMENDMENT — tracks-only `--gen-dataset` ships before centroid/candidate capture

**Status:** adopted on branch `motion-predictor-ML-integration`. **Applies to:** §13.4, SAT-ML.md §2, CP 11.1 / CP 11.5.

Design 13.4 still listed `--gen-dataset` as missing. This branch's assignment is MotionNet (M3), which needs `what = "tracks"` only: live tracker `[az, el, vaz, vel]` plus FrameTruth future angles, windowed in Python, split by `(scenario, seed)`.

**What shipped.** `sat-tracker --gen-dataset --scenario F --out DIR` writes `raw/{name}_{seed}.csv` from `sat_app`. Labels never enter `sat_tracking` or `sat_ai` (INV-1). `ml/datagen.py` builds the shards.

**What is not silently dropped.** The full centroid-patch and candidate-patch factories remain on the SAT-ML §2 / CP 11.1 remaining-work list. They are out of scope for this branch, not cancelled.

## 14.0g AMENDMENT — a real-time deadline model, and the load-shedding rung
## that had to be removed after it was measured

**Status:** adopted. **Applies to:** §10.4 (robustness), §15 (the frame
budget), INV-3.

The loop is a synchronous pull: `source_.next()` hands over the next frame
whenever it is asked, and the simulated clock advances one camera period per
frame however long that frame took. "Processing slower than the camera"
therefore **cannot happen by construction** — which is not a real-time
argument, it is the absence of one.

**What was added.** `engine/deadline.hpp`, a governor that is given each
frame's cost, counts the frames that overran their budget, and sheds work to
try to get back inside it. Three CLI flags: `--realtime`, `--frame-budget-ms`,
`--inject-stall MS@FRAME`. Off by default; a run without one of those flags is
byte-identical to what it was before, and the report prints nothing, because a
`deadline misses: 0` line on a run with no deadline reads as "the system kept
up" when it means "nothing was measured".

### The determinism problem, and the split that resolves it

A deadline policy driven by the wall clock is **not reproducible**. Two runs of
one scenario shed on different frames and end with different digests, and
INV-3 is the invariant the entire evidence base rests on. So the two halves are
separated:

| | Where costs come from | Reproducible? |
|---|---|---|
| **Policy** (`DeadlineGovernor`) | given to it — a plain number | n/a: a pure function |
| `--inject-stall` | a deterministic schedule; the clock is never read | **yes**, bit-exact |
| `--realtime` | the wall clock | **no**, and the report says so |

That split is what lets a test assert *"a 50 ms stall produces exactly one
miss, sheds for exactly eight frames, and re-acquires"* and have the assertion
mean something. CI uses the injected model only.

### The ladder, and the rung that was measured and removed

| Rung | Action | Saving | Costs |
|---|---|---:|---|
| 1 | skip the 3×3 median | ~635 µs | impulse rejection (row 21) |
| 2 | halve the window floor | ~400 µs | part of CFAR's training annulus |
| ~~3~~ | ~~fall back to the straw man~~ | ~~~17 ms~~ | **removed — see below** |

Shedding reacts in one frame and recovers over eight, deliberately: a miss
means the next frame is already in trouble, while un-shedding early oscillates
into the configuration that just proved too expensive.

**Rung 3 was removed after it was measured.** When shedding actually *reduces*
the frame's cost — the ordinary case — two rungs are enough. `spec_defaults`
over 20 s at a 4 ms wall-clock budget against a 3.06 ms p50:

| | no deadline | 4 ms budget (597 frames shed) |
|---|---:|---:|
| retention | 99.67 % | **99.67 %** |
| row 18 target loss | 0.00 % | **0.00 %** |
| centroiding RMSE | 0.2010 px | 0.4018 px |
| row 17 steady state | 17.063 px | **17.063 px** |

Shedding costs 0.2 px of centroiding and buys the deadline. Row 17 does not
move at all, because row 17 is jitter-limited at §1.3's 16.33 px floor and a
fifth of a pixel of detector noise is invisible underneath it.

But when shedding **cannot** reduce the cost — an external stall, CPU
contention, anything the detector's own workload does not control — the ladder
pins at its top rung for as long as the load lasts. With the straw man on that
rung, that meant running the straw man for hundreds of consecutive frames, and
the straw man locks onto the brightest thing in the frame. **Retention fell to
26.7 %.** The system gave up the target to keep the frame rate, which is the
wrong trade in every scenario this project has: a coarse-alignment loop that
has lost the beacon is not degraded, it has failed.

Both surviving rungs degrade how *well* the detector sees; neither changes
*what* it looks for, so neither can walk the track onto a different object.
The straw man is still runnable as §9.4's ablation arm — an experiment a person
chooses, never a rung a governor escalates into. §9.4 already required that it
"must never be the default"; this is the same rule applied to the default's
failure path.

### Shedding is gated on having a lock, and that was not obvious either

The first implementation charged every frame against the deadline. Frames with
no confirmed track get the whole frame (§14.0b rule 1) and legitimately cost
19.0 ms on 640×480 — that is not a fault condition, it is what searching costs.
Charging them climbed the ladder to the top within three frames of startup,
**before any track had ever confirmed**, and the run then never acquired:
retention 20.00 %, row 18 target loss 98.99 %, *identically for every budget
from 4 ms to 12 ms* — the giveaway that the collapse happened during
acquisition and had nothing to do with the budget.

The ladder now only climbs while `track.drivable()`. Misses are still counted
during search, because they are real and hiding them would flatter the report;
what changed is that they no longer provoke a response that cannot help.
Losing the lock also stands the ladder down, so that a held rung is never
applied to the first frame after re-acquisition — the worst frame to degrade.

### What this does not claim

The deadline model does not make the simulator real-time. The loop is still a
synchronous pull and the simulated clock still advances one period per frame;
what `--realtime` adds is a *measurement* of whether that would have been
sustainable, and a policy that responds when it would not. Dropping frames on
the input side — a real camera's behaviour when the consumer falls behind — is
not implemented, and `--video`'s decoder still blocks.

---

## 14.0f AMENDMENT — §14.0b's periodic full-frame sweep is now a rotating
## row-band, and the reason it was disabled no longer applies

**Status:** adopted, superseding `RoiParams::refresh_frames` as §14.0b defined
it. **Applies to:** §14.0b (the detection window), §15 (the frame budget).

§14.0b gave a confirmed track a detection window and added one hedge against
the window's single failure mode. If the track is following a decoy, the true
beacon is outside the window forever, and **nothing outside the window is ever
examined, so nothing outside the window can ever be recovered.**
`refresh_frames = N` meant "process the whole frame every N frames", which gives
the association logic a chance to see both.

It shipped disabled, and the reason was the tail. Measured on
`scenarios/spec_defaults.toml`, 20 s:

| Detector pass | perception p50 | perception p99 |
|---|---:|---:|
| Tracking window only | 1,352 µs | 1,633 µs |
| Full frame | 19,024 µs | 20,260 µs |

Against §15's 1,390 µs perception budget a full-frame pass is **13.7×** over.
Enabling the old sweep at `N = 16` therefore put a 19 ms frame into every
sixteenth frame — and since one frame in 16 is 6.25 % of the run, that spike
landed in the **p95**, not merely the p99. A feature that cannot be switched on
is not a feature, and this one could not be.

**The amendment.** `refresh_frames = N` now means "divide the frame into N
horizontal bands and sweep ONE of them each frame, rotating". The coverage
claim is identical — every row is examined once per N frames either way — and
the cost is `1/N` of the peak, paid as a flat addition to every frame instead
of a spike on one of them. Measured, same scenario and same 20 s:

| `refresh_frames` | perception p50 | perception p99 | p99 / p50 |
|---|---:|---:|---:|
| 0 (no sweep) | 1,352 µs | 1,633 µs | 1.21 |
| 16, old full-frame meaning | 1,352 µs | ~19,024 µs | ~14 |
| 16, rotating band | 2,537 µs | 2,877 µs | **1.13** |
| 8, rotating band | 3,701 µs | 3,942 µs | **1.07** |

The tail falls by **6.6×** and the distribution becomes flat. A tail you can
predict is worth more than a mean you cannot, which is the whole argument for
making the change.

**Two things this amendment deliberately does NOT do.**

*The band is swept in addition to the tracking window, never instead of it.*
Sweeping instead would cost less still, and it would break acquisition:
`tracking/track.hpp` promotes on 3 hits in 5 frames, and a scheme that shows
the target to the tracker once every N frames cannot supply them. Specification
row 16 grades acquisition; nothing grades the perception tail. Buying the
second with the first would be the wrong way round, and
`tests/perception/test_refresh_band.cpp` asserts that the window, the track
state and the lock frame are byte-for-byte unchanged when the sweep is enabled.

*The sweep does not extend to unwindowed frames.* When the track is not
confirmed the window is already the whole frame, every row is being examined,
and a band would be a second pass over pixels just processed. `refresh_band()`
returns an invalid rectangle in that case. This is also why the 19 ms figure
still appears in the p99 of a cold-start run: those are **search** frames, and
banding a search is the same bad trade as banding the window.

**The overlap is deduplicated.** Where a band crosses the window a source is
inside both passes. Handing the tracker two copies of one blob would
manufacture a rival track — a self-inflicted decoy, in the feature whose whole
purpose is surviving decoys. The window's copy is kept, because the window is
centred on the prediction and the target's CFAR annulus is complete there,
where in the band it may be hard against a horizontal edge with half its
background context clipped away.

**Still 0 by default.** The band costs real time on every frame — roughly
+1.2 ms at `N = 16` — and the decoy case it defends against does not arise in
the specification scenarios, which have one emitter. What has changed is that
it is now *affordable to turn on*, and `scenarios/hard/clutter_field.toml` is
where turning it on is worth the money.

**On the audit's diagnosis.** `docs/SIH_26169_CRITICAL_AUDIT.md` P1-7 attributes
an 18,938 µs perception **p95** to "the ROI refresh frame". Two corrections,
both from measurement. It is the **p99**, not the p95. And it cannot have been
a refresh frame: `refresh_frames` is 0 in every scenario committed to this
repository, so no refresh frame has ever run. The figure is the **unwindowed
search** path — the frames before the track confirms — which is confirmed by
running with `perception.roi=false`, where that same 19 ms becomes the p50. The
audit's prescribed remedy is still the right shape and is adopted above; its
stated cause is not the cause.

---

## 14.0e AMENDMENT — §9.4.6's blob table is bounded, and the earlier decision
## to let it grow was wrong

**Status:** adopted. **Applies to:** §9.4.6 (run-length grouping), INV-4.

§9.4.6 specifies the grouping pass but says nothing about how large the blob
table may become. The implementation reserved 4,096 components and let the
vector grow past that if a frame produced more, and the code carried a comment
defending that choice: truncating, it said, "would change what the detector
found in order to satisfy an invariant about allocation".

**That reasoning was wrong, and CP 14.1's fuzzer is what showed it.** A legal
scenario — 1920×534, heavy damage, a CFAR threshold the draw put at the low end
of its legal range — produced **14,420 components**, and the growth allocated
**524,288 bytes inside the frame window**. The Debug allocation trap aborted,
correctly. An invariant that legal configurations can violate is not an
invariant, and INV-4 is one of the nine.

Three facts settle it in favour of a bound:

1. **It is the policy the same function already applies one pass earlier.**
   Pass 1's run table has been sized at startup and truncated on overflow since
   it was written, with the rationale stated in the code: a mask far denser
   than CFAR should ever produce is a sign the threshold is wrong, not a reason
   to grow the heap mid-frame. Pass 3 was the inconsistent one.
2. **The truncation is deterministic.** Components are created in order of
   first appearance in raster order, so which ones survive a bound is a
   function of the mask alone. INV-3 is untouched, and a test asserts the
   survivors are a byte-identical prefix of the unbounded run.
3. **The overflow is reported.** `ClassicalPerception::last_blob_overflow()`
   counts the components that found no room, so a frame whose detection list is
   incomplete says so rather than passing for a clean frame with fewer sources.

The bound is **16,384**, the measured high-water mark rounded up to the next
power of two — 1.1 MB, reserved once. §9.4's own measurement for a *working*
detector is 84 blobs on CP 5.9's worst case, so anything within three orders of
magnitude of the cap is a frame on which the detector has already failed; the
cap's job is only to make that failure bounded. The theoretical maximum remains
a checkerboard mask at w·h/2 — 1.03 million components, 74 MB on a 1920×1080
frame — which is the case the 3×3 median of §9.4.1 exists to make impossible
and which no reserve should be sized against.

**A second INV-4 violation found in the same fuzz run.** §7.4's `spawn_decoy`
event calls `EmitterSoA::add` mid-run, but `build_world` reserved capacity for
only the three *build-time* emitter populations (targets, decoy beacons, static
clutter). A spawn past that capacity reallocated eleven parallel vectors inside
the frame window — 144 bytes, caught by the same trap. `build_world` now counts
the `spawn_decoy` events on the timeline into its reserve, and
`SyntheticSource`'s visibility scratch is sized against `EmitterSoA::capacity()`
rather than `n` for the same reason.

**What this says about the process.** Both defects were in code that had passed
every test for weeks. Neither was found by reasoning about the code; both were
found by a fuzzer drawing legal-but-unusual configurations while an invariant
was mechanically enforced. That is the argument for CP 14.1 and CP 14.3
existing at all, and for `just ci` running the Debug tree rather than only the
Release one.

## 14.0d AMENDMENT — §15's 0.85 ms frame budget is not reachable, and here is
## the arithmetic

**Status:** adopted. **Applies to:** §15's performance table.

The frame went from **42.55 ms to 2.62 ms** over Stage 14's work — 16×, and 382
FPS against spec row 20's 20 and §15's own 350–500 target. It did not reach
0.85 ms, and it cannot. This records why, because a budget that cannot be met is
worse than no budget: it makes every report of the real number look like a
failure rather than a result.

### The unit that makes it legible

0.85 ms at 3.5 GHz over spec row 3's 640 × 480 sensor is

> 2,975,000 cycles ÷ 307,200 pixels = **9.7 cycles per pixel, for the entire
> frame** — world, render, damage chain, detector, tracker, controller and
> snapshot together.

### What the damage chain alone costs, and why it cannot be less

Spec rows 21–22 put shot noise and read noise on every pixel, so the sensor
model needs **one Gaussian deviate per pixel**. The cheapest route that keeps
INV-3 is the one now implemented:

* a PCG32 stream advanced eight draws at a time by applying M⁸ directly — it
  must produce the *scalar path's own values*, so eight independent streams are
  not an option, and the 64-bit multiply has to be assembled from three 32-bit
  ones because AVX2 has no `vpmullq`;
* an inverse-CDF sampler — one uniform in, one normal out, no rejection;
* then the affine atmosphere, the variance sum, a square root, the fixed-pattern
  gain and offset, and the quantisation.

Measured: **4.6 ns per pixel, 16 cycles per pixel**, and the parts were measured
individually rather than assumed — removing both table gathers saves 0.45
cycles/px and removing the PCG output function saves 0.3, so the cost is spread
across the whole body rather than sitting in one instruction.

§15 budgets that line at **0.10 ms SIMD**, which is **1.1 cycles per pixel** for
a random draw, a normal deviate, a square root and five arithmetic operations.
One vectorised 64-bit multiply is already more than that. The figure is not
attainable, and it is 12% of the 0.85 ms total on its own.

### What the reachable floor actually is

Measured leaves at 2.62 ms p50 on `scenarios/compliance.toml`:

| | measured | §15 scalar | §15 SIMD |
|---|---:|---:|---:|
| `damage_chain` | 1,395 µs | 550 | 100 |
| `perception` (all of B6–B13) | 1,275 µs | 1,390 | 370 |
| `splat` | 231 µs | 40 | — |
| `background` | 72 µs | 150 | 40 |

**Perception is inside its own §15 scalar line** and would plausibly reach
400–500 µs with AVX2 on the three kernels that still dominate it (the van Herk
opening, the matched filter and CFAR). The damage chain's own floor is about
1 ms. So the reachable synthetic frame is **1.5–2.0 ms**, not 0.85.

### The part that matters for the marks

All of the excess is the **simulator** — the render and the sensor model. Neither
exists in video mode: INV-8 disables the whole damage chain when the input is a
clip, and there is nothing to splat. That is the mode Benchmark Performance-2
grades, and it is 30% of the marks.

So the honest statement of §15 is two numbers rather than one:

* **the tracker's frame** — everything that would run on a real camera — is
  inside its budget;
* **the simulator's frame** is not, and the gap is one Gaussian deviate per
  pixel, which is physics rather than engineering.

### What is not being claimed

That the remaining 2.62 ms is optimal. `splat` is still 5.8× over its line, the
three perception kernels have no vector path yet, and `--bench-kernels` exists
precisely so those can be worked on with a usable instrument. What is being
claimed is that **0.85 ms is the wrong target** and that the arithmetic above
says so independently of how good the remaining code gets.

---

## 14.0c AMENDMENT — spec row 8's edge behaviour was never implemented; and
## §10.2's priority score cannot tell a beacon from a rock

**Status:** adopted. **Applies from:** Stage 3 (row 8) and Stage 6 (§10.2).

Two defects found together, because the first one was masking the second.

### Row 8: `world.edge_behaviour` was parsed, validated, echoed, and read by nothing

The same shape as §7.4's events before they were wired up. `bounce`, `wrap` and
`exit` were all accepted by the schema and reported in `run.json`; `World::advance`
never looked at them.

The consequence is not cosmetic. On `scenarios/baseline.toml` — the
specification's own defaults, with its own default seed — row 11's random initial
position puts the beacon at (1936, 1831) on a 2000 × 2000 screen, and row 12's
linear motion carries it off the canvas within three seconds:

| frame | true beacon position |
|---|---|
| 0 | (1936, 1831) |
| 100 | (2009, 1794) — already outside |
| 800 | (2523, 1537) |

Every metric that run produced was a measurement of a beacon that was not in the
world. It is the most misleading class of defect available: everything
downstream works perfectly and reports a catastrophe.

**Implemented** as a fold of the analytically evaluated coordinate, not as a
reflected velocity. §7.2 is explicit that positions come from evaluating the
motion stack at absolute time `t` rather than from integrating, because that is
what makes the position exact for an accelerating target and what lets the
analytic velocity be reported as truth. A bounce that flipped a stored velocity
would have to become stateful and would break both properties. The triangle-wave
fold in `world/world_builder.cpp` is a pure function of the coordinate and keeps
them; `wrap` is the corresponding sawtooth; `exit` is unchanged, because a target
that leaves has left and that is a legitimate thing for a scenario to ask for.

### §10.2's priority score chooses the clutter

The design gives the score as

```
w.snr * norm(mean_snr) + w.stability * norm(hit_ratio)
+ w.centrality * (1 - norm(dist_from_boresight)) + w.age * norm(age_s)
```

and **every one of those four terms is something a bright static source scores
well on.** It is bright, it never misses, the controller has just centred it, and
it gets older every frame. §9.1 makes 120 clutter sources mandatory and draws
their intensities over 0.35× to 1.6× the beacon's, so roughly half of them are
brighter than it. The design's own weights, applied to the design's own scenario,
choose the rock — and `Tracker::step` made it worse by seeding the track from
"the strongest candidate" in the first frame that had one.

Measured on `baseline.toml` over 30 s before this amendment: the beacon was never
in view, 1,798 false-track frames per minute, 1,272 px of tracking error.

**Added: a fifth term, motion — but measured RELATIVELY.** Clutter is specified
as static and the target is specified as moving, so angular velocity is the
discriminator the other four terms lack. The first implementation scored *raw*
apparent speed and made things worse, for a reason worth recording:

The tracker converts pixels to world angles through the **commanded** boresight,
because it cannot know the true one. Rows 23 and 25 move the true boresight and
not the commanded one, so an object at a fixed world angle θ is reported at
θ − disturbance(t) and therefore appears to move at minus the platform rate. On
`baseline.toml`:

| | apparent speed |
|---|---|
| clutter (platform 15, −8 px/s) | 1,854 µrad/s |
| beacon (22, −11 px/s) | 831 µrad/s |

The clutter appears to be moving more than twice as fast as the beacon. Raw speed
does not weaken the discriminator, it **inverts** it.

The fix is ego-motion compensation and it is exact rather than approximate: every
static source shares the same apparent step every frame, so that step is
recoverable as the component-wise **median** over the live hypotheses — most of
which, in a field of 120 clutter sources, are static. Subtracting it frame by
frame, rather than at the end, is what makes it work: row 23's ±20 px of jitter
is common mode and cancels exactly in the median, and a velocity fitted against
it would otherwise need 45 frames — 1.5 s of row 16's 2 s budget — to reach the
precision that cancellation gives in twelve.

**Also added, and both are necessary:**

* a **commit threshold**. The four non-motion weights are deliberately scaled to
  sum to 0.45, which is the ceiling on what anything can score without moving;
  the threshold sits at 0.60 above it. A candidate cannot take the mount on
  brightness and stability alone, however bright and however stable.
* a **drop rule**. A promotion threshold alone is not enough: once anything is
  committed the FSM stops searching, the controller centres it, every other
  candidate leaves the field of view, and no challenger is left to out-score it.
  A committed track that scores below the threshold for 45 consecutive frames is
  dropped and the search resumes.

**What this does not fix, stated plainly.** A moving decoy is not separated by
this — it moves, it is bright, it is stable. Nor is a genuinely stationary
target distinguished from clutter; motion is one weighted term and not a gate,
so such a target competes on the same footing it did before, which is to say
badly. Both remain §11's `CandidateNet` territory and both are recorded in
`issues_till_now.md`.

Everything is data: `tracking.priority` turns the policy off entirely and the
weights, threshold, evidence requirement and hysteresis are all scenario keys, so
the arm-versus-arm comparison in `docs/RESULTS.md` runs from one binary.

---

## 14.0b AMENDMENT — the detector is given a window when the track is confirmed

**Status:** adopted. **Applies from:** Stage 5.

§9.4's pipeline runs every stage over the whole frame: median, top-hat, two
summed-area tables, a matched filter and two CFAR passes, over 307,200 pixels
(row 3), thirty times a second. Measured, that is 11.5 ms of a 16 ms frame
against §15's 1.39 ms for the same list of stages, and no amount of arithmetic
improvement closes a gap that size.

**The observation:** on a frame where the track is Confirmed, the tracker already
knows where the beacon is — to within its filter's own position sigma, which in
clear air is a couple of pixels. Searching the other 99% of the frame is not
robustness; it is arithmetic performed on the answer to a question nobody asked.

**Adopted:** when the track is Confirmed the detector is handed a window centred
on the predicted image position, with a half-size that is the larger of a fixed
floor and the filter's position sigma times a margin. When it is not — searching,
acquiring, or coasting past its own confidence — it is handed the whole frame,
because then the question really is "where is it".

CP 6.7 already names this idea as "predicted-region reacquisition"; this applies
it to the steady state as well.

**Why the floor exists, and why it is not about the target.** §9.4.5's CFAR
training annulus is 61 px across. A window narrower than that would estimate the
background from almost nothing, so the floor is 96 px of half-width regardless of
how confident the filter is.

**Why Coasting deliberately does NOT get a window.** `drivable()` is true for
Coasting as well as Confirmed, but the coasting case is exactly the one where the
prediction is least trustworthy: the detector has already failed to find the
target somewhere, and narrowing its search is the wrong response.

**Three properties that make it safe:**

* **INV-1 holds.** The window is a rectangle of pixel coordinates derived from
  the tracker's own estimate. Perception still never sees truth.
* **INV-3 holds.** The rectangle is a deterministic function of filter state.
* **It cannot hide a target the tracker had.** The window grows with the
  covariance, so a track that starts to drift widens its own window; one that
  fails M-of-N drops out of Confirmed and gets the whole frame back next frame.

**What it changes on purpose:** clutter outside the window is no longer detected
and therefore can no longer be associated.

**Implementation note.** The window is realised as a *copy* into a compact
buffer, not as a stride threaded through every kernel. Six kernels each have an
edge-clamping argument that would otherwise acquire a second meaning, and the
copy leaves each kernel reading a contiguous buffer that fits in L2 — which is
most of why the window is fast. `perception.roi` turns it off, so both arms are
runnable from one binary.

**Measured:** perception 17.3 ms → 2.9 ms p50 on `scenarios/compliance.toml`;
frame total 22.7 ms → 8.4 ms. The p99 still shows the full-frame acquisition
frames, which is honest and is what the stage table reports.

---

## 14.0a AMENDMENT — the dashboard was pulled forward to Stage 4

**Status:** adopted, superseding part of §14.0 below. **Applies from:** Stage 4.

§14.0 deferred the GUI to CP 15.0 on the argument that headless-first is faster
and the snapshot seam makes attaching it cheap later. The second half of that
argument held; the first turned out to be incomplete.

The reason for pulling it forward is that **a headless project cannot be
inspected by the person it is being built for.** Four stages of engine, damage
chain and metrics had accumulated with no way for anyone to look at a frame,
watch the loop settle, or dial the noise up and see what happens. Numbers in a
test log are evidence for someone who already knows what to look for; a window
is evidence for everyone else.

Building it also paid for itself immediately, because a window shows things a
test does not ask about. Within minutes of the first run it surfaced a defect
the headless suite had no reason to catch: **40 hot pixels stuck at 255 defeat
the brightest-pixel detector on their own**, with no noise present at all. That
is an independent argument for §9.4.1's median filter — hot pixels are isolated
single pixels, exactly what a median removes — and it had gone unnoticed because
every headless test either disabled the damage chain or was measuring something
salt-and-pepper already dominated.

**What was built** (a basic CP 15.0, not the complete §12):
camera view with truth/detection/boresight overlays, screen overview with the
true and reported paths and the FOV rectangle, the two graded error traces
plotted separately per INV-6, a live compliance table, per-stage latency
percentiles, the full specification parameter table, and live controls for the
damage chain, the clutter count and the closed loop.

**What §14.0's substitutes bought, and what stands:** every numeric acceptance
test written in place of a visual one stays. They test more than a person
watching would, they run on every commit, and CP 1.8's open-loop comparison in
particular is a stronger claim than "the camera visibly follows". The GUI is an
inspection surface, not an acceptance surface.

**Still outstanding for CP 15.1:** the IMM mode-probability panel, the SAT
strategy timeline, the mode-FSM graph and live algorithm switching, none of
which have anything to attach to until Stages 6, 10 and 12 exist.

---

## 14.0 AMENDMENT — the GUI is deferred to Stage 15

**Status:** adopted. **Applies from:** Stage 0. **Supersedes:** CP 0.5, and the GUI half of
CP 1.4, 1.8, 2.3, 2.4, 4.10, 6.6.

The roadmap as originally written grows the dashboard alongside the engine, using it as the
acceptance surface for several early checkpoints ("a white square is visible and moves", "the error
visibly settles"). That ordering was chosen so progress is watchable. The project is instead being
built **headless-first**, for three reasons:

1. **The acceptance criteria get weaker, not stronger, when they are visual.** "A white square is
   visible" is checked by a human once; `centre_of_mass(rendered) == 100.37 ± 0.001` is checked by
   CI on every commit forever. Every visual criterion below has a numeric substitute that tests
   strictly more.
2. **80% of the marks are the software running** (§3.1). The GUI is 20%, it is graded on being
   *watchable* rather than on being *early*, and it cannot be built well until there is something
   worth showing.
3. **The seam already exists.** `TripleBuffer` and `SimSnapshot` (§6.2 B30/B31) are built at their
   original checkpoint. The dashboard attaches to that seam later without touching the simulation,
   so deferring it costs no rework — which is exactly the argument §17 decision 1 makes for
   `IFrameSource`.

**This does not reduce GUI scope.** Every panel in §12 is still delivered, at Stage 15, and CP 15.1
through 15.3 are unchanged. What changes is only *when*, and what stands in as the acceptance test
meanwhile.

| Original CP | Original acceptance | Headless substitute (now authoritative) |
|---|---|---|
| 0.5 GLFW/ImGui/ImPlot shell | A window opens showing a live sine wave | **Moved to CP 15.0.** Stage 0 instead ends at the CI and OpenCV gates. |
| 1.4 Upload `GL_R8`, draw in a panel | A white square is visible and moves | Render to a `uint8` buffer; dump PGM via `--dump-frames`; assert the rendered intensity centroid equals the commanded sub-pixel position to 1e-3 |
| 1.8 ★ GATE "camera visibly pulls the square toward centre" | Watched live | Assert boresight-to-target error decreases monotonically over 60 frames and settles below 2 px; the comment-out check becomes a test with feedforward disabled |
| 2.3 Display on its own thread | Slowing display to 5 FPS does not slow the sim | Already covered by the `TripleBuffer` concurrency test (producer flat out, consumer dawdling, no tearing, drops asserted) |
| 2.4 ImPlot tracking-error trace | The error visibly settles after acquisition | The same series written to `centroid.csv`; assert settling time and steady-state RMS numerically |
| 4.10 Step response on the plot | Delay, ramp and ceiling visible | Assert the step response's delay, slew rate and ceiling against `GimbalParams` directly |
| 6.6 Mode FSM drawn live | Highlighted state matches the screen | Assert the FSM transition sequence against the expected trace for a scripted scenario |

**New CP 15.0** (inserted before 15.1): GLFW + glad + Dear ImGui + ImPlot shell, one `GL_R8` texture
upload per frame reading from the existing triple buffer. Accept when a window opens on Windows and
Linux and renders the live camera view at ≥ 60 FPS with the simulation unthrottled.

**Risk accepted:** the GUI is on the critical path for a graded deliverable and is now later in the
schedule. Mitigation: §14.2's cut list already protects Stages 0–9 and CP 15.3, and the snapshot
seam means Stage 15 is assembly rather than integration. If the schedule compresses, CP 15.0–15.2
are built against the seam in parallel with Stage 13/14 work.

---

## STAGE 0 — Foundations · 3 days

| CP | Build | Accept when |
|---|---|---|
| 0.1 | Repo, CMake, vcpkg manifest, folder structure, `.gitignore` | `cmake -B build && cmake --build build` produces a runnable binary printing a version string |
| 0.2 | `units.hpp`, `frames.hpp` — `Urad`, `Angle2`, `Pixel2`, `CameraGeometry`, `project`/`unproject` | Test: `project(unproject(p)) == p` to 1e-9 for 1000 points across the FOV; `ifov_urad()` returns 109.08 at defaults |
| 0.3 | `time.hpp` (`Clock`), `rng.hpp` (`Pcg32`, `Stream`, `RngSet`) | `Clock` rejects `truth_hz % camera_hz != 0`; two identically-seeded `RngSet`s produce identical sequences on every stream |
| 0.4 | `arena.hpp`, `ring.hpp` | Arena survives 10k alloc/release cycles without growing; `Ring` passes wraparound tests |
| ~~0.5~~ | GLFW + ImGui + ImPlot shell | **Built at Stage 4 instead — see §14.0a.** Deferred by §14.0, then pulled forward. |
| 0.6 | GitHub Actions: Windows (MSVC) + Linux (GCC), running `ctest` | Both jobs green; the Windows artifact downloads and runs |
| **0.7** | **★ GATE — 20 throwaway lines: `cv::VideoCapture` opens a committed test MP4, prints resolution/fps/frames. vcpkg `opencv4[videoio,ffmpeg]`. Run in CI.** | **Windows CI prints `1920x1080 @ 30.00 fps, 900 frames`. If this fails, STOP and solve it — 30% of marks depend on it.** |

## STAGE 1 — The skeleton loop · 1 week ★ most important stage

Goal: a camera that chases a square because your own code told it to. Ugly is fine.

| CP | Build | Accept when |
|---|---|---|
| 1.1 | `engine/frame_source.hpp` — `SourceFrame`, `FrameTruth`, `IFrameSource`. `SyntheticSource` as an empty stub | Header compiles; the stub exists. *(Defining this now costs an hour; at Stage 8 it costs a rewrite.)* |
| 1.2 | `EmitterSoA` with one hardcoded entry | Position can be queried |
| 1.3 | `overlap_1d`, square splat into a float buffer at continuous sub-pixel position | Test: place a 10×10 square at x=100.37; intensity-weighted centre of rendered pixels = 100.37 ± 0.001 |
| 1.4 | Quantise to `uint8`; dump PGM frames via `--dump-frames` | *(amended, §14.0)* The rendered intensity centroid equals the commanded sub-pixel position to 1e-3; a PGM dump is eyeball-checkable but not the acceptance test |
| 1.5 | Brightest-pixel detector | Returns the square's approximate location, drawn as an overlay |
| 1.6 | `GimbalAxis` with position, rate, **hard rate clamp only** | Commanding a huge rate produces movement at exactly `max_rate_urad_s` |
| 1.7 | P controller: `cmd = kp * (detection − boresight)` | Produces a nonzero command when off-centre |
| **1.8** | **★ GATE — wire it together. Render at current boresight → detect → control → move → render at NEW boresight. Add straight-line motion.** | **(a)** The camera visibly pulls the square toward centre. **(b)** Commenting out the line applying the command stops the following; uncommenting restores it. **(c)** `grep -r "truth\|emitter\|world" src/perception src/control` returns nothing. |

## STAGE 2 — Time and reproducibility · 4 days

| CP | Build | Accept when |
|---|---|---|
| 2.1 | 300 Hz truth loop with 10 sub-ticks per 30 Hz frame | World advances 10× per frame; `seconds()` derived from integer tick, never accumulated |
| 2.2 | `FrameTruth` emitted alongside every frame | Truth is produced, consumed by nothing yet, and lives where perception cannot include it |
| 2.3 | `SimSnapshot`, `TripleBuffer`, display on its own thread | Artificially slowing the display to 5 FPS does not slow the simulation |
| 2.4 | Tracking-error series into `centroid.csv` | *(amended, §14.0)* Settling time and steady-state RMS asserted numerically |
| 2.5 | `snapshot_hash()` — FNV-1a over frame, boresight, detection, mode | Same scenario + seed twice in one session gives identical hash sequences |
| **2.6** | **★ GATE — `--verify-reproducibility`; CI job running 5 scenarios × 3 seeds at `-O0` and `-O2`, comparing hashes** | **CI green. Then deliberately inject a `std::chrono` call into the sim path and confirm the job goes red. Fix any failure before proceeding.** |

## STAGE 3 — Configuration · 4 days

| CP | Build | Accept when |
|---|---|---|
| 3.1 | toml++ loader → `Scenario` struct | Changing `camera.fov_deg` in a file changes behaviour with no recompile |
| 3.2 | `FieldSpec` schema table, spec-row-citing errors | 15 files in `tests/bad_configs/` each produce the correct `file:line: message (specification row N)` |
| 3.3 | `IMotionComponent`, `CompositeMotion`, `linear` | A one-component TOML stack drives the emitter |
| 3.4 | All 9 components incl. `waypoints` (Catmull-Rom) | All four mandatory motions run from TOML; `linear + sinusoid` stacked produces sinusoidal motion |
| 3.5 | Closed-form velocity for every component | Test: each component's analytic velocity matches a central finite difference to 1e-6 at 100 sample times |
| 3.6 | All 25 spec rows exposed, annotated with row numbers | You can point at any table row and name the TOML key |

## STAGE 4 — Simulation fidelity · 1.5 weeks

| CP | Build | Accept when |
|---|---|---|
| 4.1 | Procedural background, viewport only | Renders in under 0.2 ms |
| 4.2 | Circle (boundary supersampling), Gaussian (erf), mask (stb_image) | All four shapes render; the mask path loads a PNG and resamples correctly |
| 4.3 | Motion blur — 8 substeps, interpolating target AND boresight | A fast slew visibly smears; `blur_substeps = 1` removes it |
| 4.4 | Five atmosphere modes as affine transforms | Fog visibly washes out; measured contrast matches the table |
| 4.5 | Poisson (λ<30 branch) + Gaussian read noise | Measured variance over 10k pixels matches theory within 2% |
| 4.6 | Salt & pepper via geometric skip sampling | Exactly ~10% corrupted; measurably faster than naive in `--bench` |
| 4.7 | PRNU/FPN maps, hot/dead pixels, seeded once | Same seed → same defect pattern across runs |
| 4.8 | Jitter on the **true boresight**, not pixels | Startup logs `20 px/frame → 65,448 µrad/s → 3.75 °/s (75% of a 5 °/s motor)` |
| 4.9 | Platform motion reusing the CP 3.4 components | All five row-25 modes work, driven by the same code as target motion |
| 4.10 | Full gimbal: accel limit, lag, delay line, encoder quantisation, `position()` vs `true_position()` | A step command shows delay, ramp and rate ceiling on the plot; the controller reads only the quantised value |
| 4.11 | N clutter sources + decoy beacons | With 120 clutter sources, the brightest-pixel detector demonstrably locks onto the wrong thing |
| 4.12 | ★ Spec-complete checkpoint | Every parameter-table row implemented, runtime-adjustable from the GUI, validated on load |

## STAGE 5 — Perception · 1.5 weeks

Scalar first, with a test, before the next kernel.

| CP | Build | Accept when |
|---|---|---|
| 5.1 | Median 3×3, 19-op network, scalar | Exhaustive test over a small alphabet proves the network; matches `cv::medianBlur` bit-for-bit on 1000 random images |
| 5.2 | van Herk top-hat | Matches `cv::morphologyEx` bit-for-bit; timing flat as SE varies 5→51 |
| 5.3 | Integer summed-area tables (sum + sum-of-squares) | 10,000 random rectangle sums match brute force **exactly**; no float anywhere |
| 5.4 | Multi-scale matched filter, 6 scales, normalised | A 10-px square scores highest at scale 11; a 5-px square at scale 5 |
| 5.5 | CFAR with guard band | With `k=3.9` on pure noise, false alarm count matches `Pfa = Q(3.9)` within 20%; the same `k` works unchanged in clear and fog |
| 5.6 | Run-length + union-find grouping with moments | Label counts match `cv::connectedComponents` on 1000 random masks |
| 5.7 | Area / fill / aspect gate | With 10% S&P + 120 clutter, candidates drop from thousands to under 25 |
| 5.8 | Centre of mass over the top-hat | On a clean frame, recovers a known sub-pixel position to within 0.05 px |
| 5.9 | ★ Worst-case checkpoint | Fog + max noise + 120 clutter + decoy: the real beacon is among the top candidates in >95% of frames |

## STAGE 6 — Tracking · 1 week

| CP | Build | Accept when |
|---|---|---|
| 6.1 | Pixels → angles via `unproject` + commanded boresight | With the camera slewing, a stationary emitter's angular position stays constant |
| 6.2 | Linear Kalman filter, CWNA `Q` | On noiseless data the estimate converges; estimated speed matches CP 3.5 analytic velocity within 2% |
| 6.3 | Mahalanobis gate (9.21) + nearest neighbour | With a decoy 60 px away, the tracker stays on the real beacon |
| 6.4 | Lifecycle FSM with M-of-N and coasting | Blanking detection for 8 frames does NOT delete the track; the camera keeps moving sensibly on prediction |
| 6.5 | Adaptive `R` from detection SNR | In fog the filter visibly relies more on prediction; error lower than with fixed `R` |
| 6.6 | Mode FSM + transition log | *(amended, §14.0)* The transition sequence matches the expected trace for a scripted scenario |
| 6.7 | Spiral search + predicted-region reacquisition | Hiding the beacon 2 s then revealing it → reacquisition in under 15 frames |

## STAGE 7 — Metrics, logs, batch · 1 week ★ THE PIVOT

After this stage, every change is measurable. Before it, you are guessing.

| CP | Build | Accept when |
|---|---|---|
| 7.1 | `MetricCollector` with every §13.1 definition, **centroiding and tracking error strictly separate** | Both appear as separate live plots; definitions written verbatim into `docs/METRICS.md` |
| 7.2 | `centroid.csv` in its **final submission format** (§13.2) | A run produces a valid CSV whose header alone suffices to interpret it; blank rows (not stale values) on no-detection frames |
| 7.3 | `--headless` | A 120 s scenario completes in under 2 s wall time |
| 7.4 | `run.json` | Contains every metric plus the full config and build hash |
| 7.5 | `--sweep` forking N worker processes | 500 runs complete in under 3 minutes on 8 cores |
| 7.6 | JSON → Jinja → self-contained `report.html` with inlined plots | Finishing a run produces a showable report with zero manual steps |
| **7.7** | **★ GATE — compliance matrix generated from a sweep, broken out per condition** | **`--sweep` over 500 runs prints the matrix with a status against spec rows 16–20 plus centroiding error. DO NOT PROCEED UNTIL THIS WORKS.** |

**You now have a submittable system.**

## STAGE 8 — Video ingest · 1 week ★ 30% of marks

| CP | Build | Accept when |
|---|---|---|
| 8.1 | Decode thread + ring buffer, greyscale at decode | Main loop never waits; a 2000×2000 clip decodes at >60 fps |
| 8.2 | Container probing, clock-divisor re-derivation | 25 fps and 60 fps files both run; an incompatible rate produces a clear message, not silent drift |
| 8.3 | `VideoScreenSource` — bicubic crop at continuous boresight; INV-8 assertion | The camera pans across a video, the crop follows smoothly at sub-pixel offsets, noise generation is disabled |
| 8.4 | `VideoDirectSource` | Both modes run from the same TOML with one key changed |
| 8.5 | Auto-detection from resolution, logged, config-overridable | 2000×2000 → screen mode; 640×480 → direct mode |
| 8.6 | Crop error characterisation harness | You have a number, in pixels, for how much error the resampling alone contributes |
| 8.7 | `--truth` CSV ingest and self-scoring | Feeding a self-generated video with known truth reports the expected error |
| 8.8 | ~10 nasty clips via FFmpeg: odd resolution, VFR, truncated, corrupt frame, beacon absent at start, beacon exits, colour, low bitrate | All ten run or fail cleanly. **None crash.** Runs in CI. |
| 8.9 | `sat-tracker --video clip.mp4 --out logs/` with full defaults | Works with no other arguments, produces a valid centroid log |

## STAGE 9 — Centroid accuracy · 1 week ★ 60% of marks

| CP | Build | Accept when |
|---|---|---|
| 9.1 | Four estimators, runtime switchable | All four run and are comparable on the same frame |
| 9.2 | Accuracy harness: 200 offsets × 6 sizes × 8 SNR bins | Produces the error-vs-offset curve per estimator; the S-shape is visible |
| 9.3 | S-curve fit, bias table compiled in, runtime correction with one iteration | Post-correction bias under 0.02 px at every SNR bin; before/after plot in hand |
| 9.4 | `centroid_sigma()`, fed into Kalman `R` | Calibration check over 100k frames shows actual error consistent with claimed sigma |
| 9.5 | Theoretical bound computed and plotted against measured | You can state "within 1.3× the bound at SNR 30" with a chart behind it |
| 9.6 | CI test asserting within 1.5× the bound at every SNR bin | Green; deliberately regressing the centroider turns it red |

## STAGE 10 — Control refinement · 1 week

| CP | Build | Accept when |
|---|---|---|
| 10.1 | Velocity feedforward | On a 200 px/s target, tracking error drops from ~11 px to under 4 px; on/off comparison plot captured |
| 10.2 | Anti-windup (conditional integration) | A full-field slew settles cleanly with no ringing |
| 10.3 | Platform drift estimation and cancellation | With linear platform motion, residual error drops measurably |
| 10.4 | Smith predictor | Bandwidth improves without losing stability margin. *If it destabilises, leave it off — optional.* |
| 10.5 | IMM (CV/CA/CT) + mode probability panel | On the figure-8, crossing overshoot visibly reduces; the mode plot shows the shift |
| 10.6 | Saturation fraction metric + error-vs-disturbance plot | You can state the disturbance level at which the loop breaks down, with a chart |
| 10.7 | Handover stub (quadrant detector, capture range) | FSM reaches `Handover` on a good run; success rate reported over a sweep |

## STAGE 11 — Machine learning · 2 weeks

> Full detail in **`SAT-ML.md`**. Checkpoints here are the integration milestones.

| CP | Build | Accept when |
|---|---|---|
| 11.1 | `--gen-dataset`, manifests, **run-level splitting** | 400k labelled samples with a manifest; test set uses scenarios the training set never saw |
| 11.2 | `CentroidNet` trained and exported | Test RMSE beats bias-corrected classical below SNR 10, matches it above |
| 11.3 | `OnnxModel` wrapper, `--no-ai`, graceful fallback | C++ matches PyTorch to 1e-5; deleting a model file logs a warning, not a crash; `--no-ai` passes the full compliance matrix |
| 11.4 | `CandidateNet` | False track rate drops measurably; ROC curve per weather mode |
| 11.5 | `MotionNet` (forecast + regime head feeding IMM priors) | Reacquisition time and lock retention both improve in the ablation |
| 11.6 | ★ The ablation table | Table showing centroid RMSE, tracking error, lock retention, reacquisition for each cumulative configuration, plus a fog-only breakdown |
| 11.7 | Model cards with "fails when" and "vs bound" lines | Each card written and honest about failure modes |

## STAGE 12 — SAT supervisor · 1 week

| CP | Build | Accept when |
|---|---|---|
| 12.1 | `Conditions` extraction + EMA smoothing | Features update live, visible in a GUI panel |
| 12.2 | Rule-table baseline with hysteresis and bumpless switching | Switches visible on a timeline, at most once per second, with no visible kick in the error trace |
| 12.3 | Per-strategy Monte Carlo sweep | You have `(conditions → best strategy)` pairs from measurement |
| 12.4 | `StrategyPolicy` trained and deployed | You can report whether the learned policy beats the rule table — honestly, either way |

## STAGE 13 — Acquisition strategy · 4 days

| CP | Build | Accept when |
|---|---|---|
| 13.1 | Probability grid with negative-information updates and diffusion | Looking somewhere and seeing nothing visibly reduces that region on the heatmap |
| 13.2 | All five strategies benchmarked | Mean acquisition time per strategy; camp-and-wait crossover plotted against target speed |
| 13.3 | Bound derivation written up | Report section written; compliance matrix shows both acquisition metrics, clearly labelled |

## STAGE 14 — Robustness and performance · 4 days

| CP | Build | Accept when |
|---|---|---|
| 14.1 | `--fuzz-scenarios N` sampling every parameter across its legal range, including corners | 5,000 random scenarios: no crash, no hang, no NaN in any logged value |
| 14.2 | AVX2 kernels + scalar fallback | Bit-identical to scalar on random inputs; total frame time under 1 ms |
| 14.3 | Debug `operator new` trap | A full run completes with zero steady-state allocations |
| 14.4 | Tracy + always-on lightweight per-stage timers | p50/p95/p99 per stage reportable from the **shipped** binary |

## STAGE 15 — GUI, demo, deliverables · 1.5 weeks

| CP | Build | Accept when |
|---|---|---|
| 15.0 | *(new, §14.0)* GLFW + glad + ImGui + ImPlot shell, one `GL_R8` upload per frame from the existing triple buffer | A window opens on Windows and Linux and renders the live camera view at ≥ 60 FPS with the simulation unthrottled |
| 15.1 | Every §12 panel complete | Someone who has never seen the project can watch 30 seconds and describe what it is doing |
| 15.2 | Live algorithm switching dropdowns | You can turn feedforward off live and watch the error trace blow up |
| 15.3 | ★ Minute-by-minute demo script + known-good fallback scenario preloaded | Run end-to-end **three times**, including once where someone hands you an unseen scenario file and an unseen MP4. Nothing broke. |
| 15.4 | Technical report (10–15 pp), user manual cross-referenced to spec rows, model cards, README, QUICKSTART | Someone can install and run using only the manual |
| 15.5 | Windows + Linux release builds, zipped with models/scenarios/shapes | Download onto a clean machine, unzip, double-click, it runs |

## 14.1 Demo script (Functional Verification, 20%)

| Time | Content |
|---|---|
| 0:00–1:00 | The problem in one sentence; the loop diagram; why the disturbance can beat the motor |
| 1:00–3:00 | Baseline scenario running — camera view, centroid marker, error traces |
| 3:00–5:00 | Dial damage up live: fog → max noise → platform gusts. It holds. |
| 5:00–6:30 | Turn velocity feedforward **off** live. Error trace blows up. Turn it back on. |
| 6:30–8:00 | Switch to figure-8. IMM mode probabilities shift at the crossing. |
| 8:00–9:30 | SAT strategy timeline switching as conditions change, and why |
| 9:30–11:00 | **Load an MP4 and track it live.** The differentiating moment. |
| 11:00–12:00 | The auto-generated report and the 1,500-run compliance matrix |
| 12:00–13:00 | Buffer and questions |

**Two rules:** keep a known-good scenario preloaded as a fallback, and never let the demo depend on
something you have not run twenty times.

## 14.2 If the schedule compresses

Cut in this order: Stage 13 → Stage 12 → CP 14.2 → CP 10.4–10.7.

**Never cut:** Stages 0–9, plus CP 11.1–11.3 and 11.6, plus CP 15.3. Those cover ~80% of the marks.
Stage 7 (metrics) and Stage 8 (video) are each worth more than everything after Stage 11 combined.

---

# 15. PERFORMANCE TARGETS

| Stage | Scalar | SIMD |
|---|---|---|
| World, 10 sub-ticks | 0.02 ms | — |
| Background render | 0.15 ms | 0.04 ms |
| Emitter splat, 8 substeps | 0.04 ms | — |
| Damage chain | 0.55 ms | 0.10 ms |
| Median 3×3 | 0.35 ms | 0.04 ms |
| Top-hat | 0.30 ms | 0.09 ms |
| SAT ×2 | 0.35 ms | 0.12 ms |
| Matched filter, 6 scales | 0.12 ms | 0.05 ms |
| CFAR | 0.20 ms | 0.07 ms |
| Grouping | 0.05 ms | — |
| Centroid + bias correction | 0.02 ms | — |
| CentroidNet | — | 0.02 ms |
| CandidateNet (20 patches) | — | 0.15 ms |
| MotionNet | — | 0.05 ms |
| StrategyPolicy | — | 0.005 ms |
| IMM | 0.01 ms | — |
| Control + plant | 0.01 ms | — |
| Snapshot publish | 0.03 ms | — |
| **Total (synthetic)** | | **≈ 0.85 ms** |
| Video decode (separate thread) | 5–10 ms | overlapped |
| Video crop, bicubic | 0.45 ms | 0.15 ms |
| Worst case (+ RecoveryNet) | | ≈ 3.5 ms |

| Metric | Spec | Target |
|---|---|---|
| Closed-loop FPS, GUI on | ≥ 20 | 350–500 |
| Video mode FPS | ≥ 20 | 90–140 (decode-bound) |
| Processing p99 | — | < 1.5 ms |
| Headless, 1 core | — | ~400× real time |
| Headless, 8 processes | — | ~3000× real time |
| Windows binary + DLLs | — | ~60 MB |
| Startup to first frame | — | < 800 ms |

**The ten rules that produce these:**

1. Never materialise the 2000×2000 canvas in synthetic mode
2. Integer summed-area tables → CFAR and multi-scale matching are O(1)
3. van Herk morphology → SE size is free
4. Geometric skip sampling for salt & pepper
5. Run-length grouping instead of full-image labelling
6. ML on CFAR proposals, not whole frames (8 ms → 0.15 ms)
7. Video decode on its own thread, overlapped
8. Zero allocation in steady state (INV-4), enforced by a debug trap
9. Structure of arrays everywhere hot; SIMD with a tested scalar fallback
10. Single-threaded simulation, parallelism across runs

---

# 16. TESTING REQUIREMENTS

| Layer | Requirement |
|---|---|
| Kernels | Every kernel vs brute-force reference; median network verified exhaustively |
| Kernels vs OpenCV | `cv::medianBlur`, `cv::morphologyEx`, `cv::connectedComponents` as oracles — bit-identical on 1000+ random inputs |
| SIMD vs scalar | Bit-identical on random inputs |
| Centroid accuracy | 200 offsets × 6 sizes × 8 SNR bins; assert within 1.5× the theoretical bound |
| S-curve bias | Assert post-correction bias < 0.02 px at every SNR bin |
| Filters | Kalman converges on noiseless data; NIS is χ²-consistent over 1000 runs |
| Frames | `project(unproject(p)) == p` to 1e-9 across the FOV |
| Motion algebra | Analytic velocity matches finite difference to 1e-6 for every component |
| TOML schema | Every file in `tests/bad_configs/` produces the correct spec-row-citing error |
| Scenario fuzzing | 5000 random scenarios, no crash / hang / NaN |
| Video robustness | 10 awkward clips, none crash |
| Reproducibility | 20 scenarios × 3 seeds × 2 optimisation levels, `--no-ai`, hashes compared |
| No allocation | `operator new` trap during frames, debug builds |
| ML | ONNX matches PyTorch to 1e-5; every model has a fallback test with the file deleted |
| Compliance | Nightly full sweep; matrix diffed against the previous run |
| INV-1 | CI grep: no `#include "world/` in perception, ai, tracking, search, control, plant |
| Windows | Builds, tests pass, both a synthetic and a video scenario run |

---

# 17. DECISIONS AT A GLANCE

| # | Decision | Rationale |
|---|---|---|
| 1 | `IFrameSource` from day one | BP-2 (30%) requires external MP4 ingest. Retrofitting the abstraction means editing every module. |
| 2 | Both readings of "bypass its PTZ camera" | ~200 lines removes the risk of a misreading costing 30%. One TOML key selects. |
| 3 | Centroiding accuracy is a first-class subsystem | Named in both 30% benchmark stages. Distinct from spec row 17's tracking error. |
| 4 | S-curve bias correction | 2–5× RMSE reduction at high SNR, on the most-graded metric. Almost nobody does it. |
| 5 | TOML only, motion as a sum of components | The three "user-defined" rows are covered by the component algebra + waypoint splines + image masks. Validatable, reproducible, analytic velocity. |
| 6 | Build natively on Windows in CI | Cross-compiling and *running* are different claims. A Windows runner proves the second. |
| 7 | OpenCV kept but bounded | The video requirement needs a decoder. Use it for decode, I/O and as a test oracle; write the hot-loop kernels. |
| 8 | Drop ReactPhysics3D | Nothing collides. You need a saturated second-order plant with a delay line — 120 lines you must own. |
| 9 | Defer EnTT | Under 1000 homogeneous entities. SoA is faster and simpler. |
| 10 | Never materialise the 2000×2000 canvas | 13× less work, and no resampling error corrupting the graded metric. |
| 11 | One sim thread; decode on its own | The frame costs 0.85 ms. Only decode justifies a second thread, and it cannot affect results. |
| 12 | Integer summed-area tables | Exact four-corner subtraction, identical everywhere, makes CFAR O(1). |
| 13 | Linear KF, not EKF | The measurement is linear after exact `unproject`. No Jacobians, no linearisation error. |
| 14 | ML on CFAR proposals, not whole frames | 8 ms → 0.15 ms. Cheap classical proposal, expensive learned decision. |
| 15 | Every model has a classical fallback; `--no-ai` must pass | ML is an enhancement, not a dependency. Insurance for the live demo. |
| 16 | Reject object detectors explicitly, in writing | A uniform square has no learnable appearance. Explaining why is evidence of judgement. |
| 17 | Report percentiles, never means | A 0.3 ms mean hiding a 25 ms p99 is a broken control loop. |
| 18 | Build the auto-report at Stage 7, not Stage 15 | "Automatically generated performance logs" is explicitly graded, and it makes every later change measurable. |
| 19 | Fuzz the scenario space | The evaluators supply the scenarios. Robustness to untried combinations is worth 30%. |
| 20 | Rehearse the demo three times with unseen inputs | Functional Verification is 20% and it is a live performance. |
