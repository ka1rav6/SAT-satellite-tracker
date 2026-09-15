# SAT — Architecture

What is built, how the pieces fit, and why each significant decision went the
way it did. For the specification itself see
[`SAT-DESIGN.md`](SAT-DESIGN.md); for measured results see
[`RESULTS.md`](RESULTS.md); for how to run it see [`MANUAL.md`](MANUAL.md).

---

## 1. The problem in one paragraph

A free-space optical terminal has to point a narrow beam at a moving partner.
Coarse alignment is done by a pan–tilt camera that must find a beacon spot on a
2000 × 2000 screen, decide where it is to sub-pixel accuracy, and drive the
mount to keep it centred — through atmospheric degradation, sensor noise,
platform motion and camera jitter, at 30 frames per second.

The central difficulty is that **the disturbance can exceed the actuator's
authority**. Spec row 23 allows ±20 px of jitter per frame; the mount's
authority at 5 °/s is about 26 px per frame. The loop is not comfortably
over-powered, and every design decision downstream follows from that.

---

## 2. The shape of the system

```
                      ┌──────────────── SIMULATION SIDE ────────────────┐
  scenario.toml  ──▶  │  world ──▶ camera ──▶ degrade                   │
                      │  (emitters,  (exact    (atmosphere, shot, read, │
                      │   motion)    coverage)  S&P, defects, quantise) │
                      └───────────────────────┬─────────────────────────┘
                                              │  SourceFrame (pixels only)
  clip.mp4 ──▶ decode thread ──▶ bicubic crop ┤
                                              ▼
  ╔═══════════════════ INV-1 BOUNDARY: nothing below sees truth ═══════════╗
  ║  perception ──▶ tracking ──▶ search ──▶ control ──▶ plant              ║
  ║  median, top-hat,  Kalman,    spiral,    PID+FF,    gimbal model       ║
  ║  SAT, matched,     IMM,       predicted  anti-windup                   ║
  ║  CFAR, grouping,   gate,      region     mode FSM,                     ║
  ║  centroid + bias   lifecycle             handover                      ║
  ╚═══════════════════════════════════════╤════════════════════════════════╝
                                          │  commanded rate
                      ┌───────────────────┴─────────────────────────────┐
                      │  metrics  ──▶  centroid.csv, run.json,          │
                      │  (may read truth)  report.html, compliance.txt  │
                      └─────────────────────────────────────────────────┘
```

Frame acquisition is **the only place the three input modes differ**
(design §6.2 B4). Everything from perception onwards is identical code for
synthetic, `video_screen` and `video_direct` — which is what makes a result in
one mode evidence about the other.

### Module sizes

| Module | Lines | Role |
|---|---:|---|
| `core` | 2,079 | units, frames, clock, RNG, arena, ring, hashing, timers |
| `world` | 901 | emitters, the motion algebra |
| `scenario` | 1,779 | TOML loading, schema validation, sweep specs, overlays |
| `camera` | 444 | exact-coverage splatting |
| `degrade` | 629 | atmosphere, noise, jitter, platform motion |
| `perception` | 2,865 | the §9.4 detection pipeline and the centroid estimators |
| `tracking` | 1,716 | measurement, Kalman, the IMM, gating, lifecycle |
| `search` | 421 | spiral / raster acquisition patterns |
| `plant` | 247 | the gimbal model |
| `control` | 1,148 | PID + feedforward, anti-windup, Smith predictor, mode FSM, handover |
| `engine` | 3,330 | the per-frame orchestrator, frame sources, video |
| `metrics` | 2,719 | §13.1 definitions, logs, traces, reports, compliance, calibration |
| `gui` | 1,341 | the live dashboard |
| `app` | 1,830 | the commands behind `main` |
| **total** | **21,449** | plus 12,478 lines of tests |

---

## 3. The invariants, and how each is enforced

The project's claims depend on nine invariants. Convention is not enforcement,
so each is tied to a mechanism that fails loudly.

### INV-1 — the tracker cannot see ground truth

If the tracker can read the simulator's answer, every number the project
reports is worthless. Three independent mechanisms:

1. **The linker.** `perception`, `ai`, `tracking`, `search`, `control` and
   `plant` do not link `world` or `camera`. Calling a world symbol is a link
   error.
2. **Configure time.** `cmake/modules.cmake` walks each tracker-side target's
   transitive link closure and fails `cmake` if `sat_world` is reachable. This
   catches the moment the *edge is added*, not the moment it is used — adding
   the dependency without calling anything would otherwise pass silently.
   `tools/verify_inv1_guard.sh` injects a violation and confirms the guard
   fires.
3. **Source scanning.** `tools/check_source_invariants.py` greps the
   tracker-side directories for `#include "world/`, which would reach neither
   of the above if the header were header-only. This has caught two real
   violations, most recently the centroid harness including
   `camera/coverage.hpp`.

At the call site, `ClassicalPerception::process` takes `std::span<const
uint8_t>` and never a `SourceFrame` — so the truth sitting in the same struct
is unreachable even by accident.

### INV-2 — the loop is closed

One line in `engine/pipeline.cpp` feeds the controller's output into the
gimbal before the next frame is rendered. CP 1.8 verifies it by construction:
with the controller disabled the camera must *fail* to follow. A simulation
that centres the beacon regardless of the controller looks identical on screen
and proves nothing.

### INV-3 — bit-exact reproducibility

No wall clock, no `rand()`, no unseeded generators anywhere in the simulation
path. Per-`Stream` PCG32 generators seeded from one master seed through a
SplitMix64 finaliser, so seeds 1 and 2 do not produce correlated streams.

Enforced by `tools/check_source_invariants.py` (which strips comments and
string literals first, and self-tests that stripper before trusting any
result — the project's own diagnostics talk *about* the forbidden constructs).
`tools/verify_repro_guard.sh` injects a clock read into the render path and
confirms the reproducibility gate goes red.

The one thread in the program — video decode — does not break this: the only
thing crossing the boundary is a *sequence* of frames that is a pure function
of the file. The decoder is forced single-threaded, a full ring blocks rather
than dropping, and the consumer's timing affects how long it waits but never
what it receives.

### INV-4 — zero heap allocation in steady state

Every per-frame buffer is carved from an `Arena` reserved once at startup. The
snapshot triple-buffer is sized from a prototype. Candidate and measurement
vectors are reserved to their maximum. Tests assert that a series' capacity is
unchanged after 3,600 pushes.

### INV-5 — work in microradians

Pixels are a property of one camera; angles are not. The tracker's state lives
in the world angular frame, which is what makes a constant-velocity model
meaningful while the camera slews under the target at up to 10 °/s.

### INV-6 — centroiding error and tracking error are distinct

Separate fields, from different inputs, over different frame sets. Centroiding
is reported in **both** the image frame (the detector alone) and the screen
frame (which additionally carries the unmeasured pointing error), because
reporting only one would either overstate what the system delivers or let a
disturbance dominate the 60 %-weighted metric.

### INV-7 — the system runs fully without AI

There is no ML in the build. `CentroidKind::Learned` falls back to
`WindowedCoM`; `ai_enabled` is reported in every artifact. The classical path
is not a fallback bolted on — it *is* the system, and it is the baseline any
future model has to beat.

### INV-8 — no damage added in video modes

The supplied clip already contains whatever noise it contains. The degradation
chain is never called in video mode, and `check_inv8()` warns if a scenario
asks for it.

### INV-9 — never emit a stale or interpolated centroid

`centroid.csv` writes empty columns on a no-detection frame. The writer holds
no previous value, so a stale one cannot be written — the property is
structural, not disciplinary.

---

## 4. A frame, end to end

Design §6.2's B1–B31, as implemented.

**B1–B3, ten times per camera frame** (the world runs at 300 Hz, frames arrive
at 30). Advance emitters analytically; advance disturbances; step the gimbal
with the rate the controller produced *last* frame. Sub-ticking matters: a
gimbal integrated once per 33 ms would misrepresent its own acceleration limit.

**B4 — acquire.** The only mode-dependent step.

**B6–B13 — perception** (`src/perception/pipeline.cpp`):

| Step | Stage | Why this and not the obvious thing |
|---|---|---|
| B6 | median 3×3 | 19-op sorting network. Spec row 21's 10 % salt-and-pepper is 30,720 impulses against a 100-pixel beacon; a brightest-pixel detector loses outright. |
| B7 | top-hat | van Herk/Gil-Werman, O(1) per pixel regardless of structuring-element size. Removes the background so the centroid estimators' assumption holds. |
| B8 | summed-area tables | **Integer**, not float. A float32 SAT over a 640×480 image injects ~37 grey levels of error per pixel by the bottom-right corner. |
| B9 | matched filter | One scale over the image; the six-scale size estimate is evaluated *at the candidates*. |
| B10 | CFAR | On the matched-filter response **unioned with** CFAR on the top-hat. The two fail in opposite directions — measured at 19 % and 9 % of frames respectively. |
| B11 | run-length + union-find | Deterministic labels by first appearance. |
| B12 | shape gate | area, fill, aspect. Thousands of mask pixels → under 25 candidates. |
| B13 | centroid | §10.1.2's estimator seeded from the blob's centre of mass, then §10.1.3's S-curve bias correction. |

**B16–B21 — tracking** (`src/tracking/`): unproject to angles and add the
*commanded* boresight; gate each candidate on Mahalanobis distance **and** a
physical reachability bound; associate by likelihood score; Kalman predict and
update; run the lifecycle FSM.

**B24–B27 — mode and control**: the mode FSM decides whether the tracker or
the search pattern owns the aim point; the controller produces a rate.

**B28–B31 — metrics and snapshot**: the only place truth is read, and the
triple-buffered snapshot the GUI and the reproducibility fingerprint both
come from.

---

## 5. Decisions worth defending

These are the places where the obvious implementation is wrong, and the code
carries the reasoning at each site.

**The Kalman filter is linear, not an EKF.** After the angular conversion the
measurement observes the state directly, so `H` is a constant selector.
Linearising something already linear costs a Jacobian that can be wrong
silently.

**The full 4×4 form, not two decoupled 2×2 filters.** With an isotropic CWNA
`Q` the axes *are* independent, so the decoupled form would work — and would
have to be thrown away at CP 10.5, when the coordinate-turn model couples them,
which is precisely when the filter is hardest to debug.

**Joseph form for the covariance update.** The short `(I−KH)P` form is only
correct for the exactly-optimal `K` and loses symmetry in floating point. The
lifecycle permits 15 consecutive misses, so 15 predicts with no update is a
state the system reaches, and an asymmetric `P` makes the gate produce negative
distances.

**Association minimises `d² + ln|S|`, not `d²`.** `d²` divides displacement by
the uncertainty a candidate *claims*, so a garbage low-SNR detection gets a
small `d²` for free. Measured: a 40× preference for the worst candidate in the
frame.

**Two gates, not one.** A chi-square gate is a statement about the filter's own
uncertainty, and right after initialisation that uncertainty is deliberately
dishonest — a wide prior is how you admit you know nothing yet. So a track is
most vulnerable exactly when the statistical gate is weakest. The second gate
is physical: the target cannot have moved further than its own maximum speed
allows.

**Every bound is computed from the scenario, not typed in.** §7.2 specifies the
motion algebra in closed form, so a target's maximum speed and acceleration are
derivable rather than guessed. Three separate bugs came from using the *mount's*
authority where the *target's* motion was the right quantity.

**Bicubic, not bilinear, for the video crop.** Bilinear smooths peaks and biases
centroids toward pixel centres — directly into the 60 %-weighted metric.
Measured at a half-pixel offset: bicubic reports a peak of 226.6 against the
analytic 230, bilinear reports 210.

**Catmull-Rom specifically**, because it is the *interpolating* cubic: an
integer crop offset is an exact identity, so the crop contributes nothing at
all where it need not.

**The setpoint and the measurement move together, or neither does.** The aim
point used to be the filter's prediction one frame ahead, which is a velocity
feedforward implemented in the setpoint. Adding an explicit `k_ff` fed the
velocity twice and put the mount 6.7 px *ahead* of the beacon; the k_ff sweep's
optimum sat at 1 − kp·dt, which is what identified it as structural rather than
a tuning fact. CP 10.4's Smith predictor is the same statement from the other
side: advancing only the measurement biases the loop by exactly the horizon,
with the sign flipped.

**Integral action requires a live track.** Conditional integration assumes the
only way to bank an unspendable demand is to hit the rate limit. In Search the
setpoint is a scanning pattern that steps and reverses — large, persistent,
sign-flipping error with the plant never saturating — and the integrator wound
up hard enough to smear a streak the detector correctly called a candidate. An
integral term that manufactures targets is doing something it is not for.

**The controller's plant model is a copy, never a handle on the plant.** A
Smith predictor's entire risk is that model and plant disagree, and a version
that cannot disagree demonstrates nothing.

**The handover criterion is fed an observable quantity.** The obvious input is
the tracking error the metrics already compute, and it is truth-derived — a
mode FSM deciding on information the real system does not have is the
conflation INV-6 exists to prevent. What is used is the detection's distance
from the image centre, which is exactly what a co-boresighted quadrant cell
sees.

**Reports are generated in C++, not Jinja.** §14 specifies Jinja, which is
Python. The checkpoint's criterion is "zero manual steps", and a Python
post-process is a manual step unless the binary invokes it — at which point the
shipping binary depends on a Python installation that §4's stack excludes.

---

## 6. What the build enforces

| Gate | Mechanism | Self-tested |
|---|---|---|
| INV-1 link boundary | configure-time closure walk | `verify_inv1_guard.sh` injects a violation |
| INV-1 header leak | source scan | stripper self-tests before trusting a result |
| INV-3 no wall clock | source scan | `verify_repro_guard.sh` injects a clock read |
| INV-3 no `rand()` | source scan | — |
| Third-party include boundary | source scan against `modules.cmake`'s link table | re-injecting the original bug goes red |
| Bit-exact reproducibility | every scenario run twice, fingerprints compared | as above |
| Frame budget | Release-only shape and wall-clock assertions | — |

`just ci` runs: source gates, the Release suite, the Debug suite, a no-GUI
build, and a build that resolves every header-only dependency by fetching it
rather than using the system copy.

That last one exists because a system-installed header can mask a missing link
edge: `src/app/sweep.cpp` included `<nlohmann/json.hpp>` while `sat_app` linked
no JSON library, four local build configurations were green, and every job on
every CI platform failed.

---

## 7. Status

Stages 0–10 complete, plus CP 14.2 and CP 14.4. All five ★ gates passed.

| Stage | | |
|---|---|---|
| 0 | Foundations | ✅ |
| 1 | The skeleton loop ★ | ✅ |
| 2 | Time and reproducibility ★ | ✅ |
| 3 | Configuration | ✅ |
| 4 | Simulation fidelity | ✅ |
| 5 | Perception ★ | ✅ |
| 6 | Tracking | ✅ |
| 7 | Metrics, logs, batch ★ | ✅ |
| 8 | Video ingest ★ | ✅ |
| 9 | Centroid accuracy | ✅ |
| 10 | Control refinement | ✅ — 10.4 built, measured, and left **off** on purpose |
| 11 | Machine learning | **out of scope** |
| 12 | SAT supervisor | not started |
| 13 | Acquisition strategy | not started |
| 14 | Robustness and performance | CP 14.2, 14.4 done |
| 15 | GUI, demo, deliverables | dashboard exists; packaging not started |

Three Stage 10 checkpoints did not land the way the design expected them to,
and each is recorded as a measured result rather than quietly implemented as
written: CP 10.3 (the platform drift is already cancelled), CP 10.4 (the loop is
not delay-limited, so the Smith predictor costs rather than buys) and CP 10.5
(the figure-8's worst error is at the lobes, not at the crossing).

Known gaps are recorded with measurements rather than described: see
[`RESULTS.md` §9](RESULTS.md).
