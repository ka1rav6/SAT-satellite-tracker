# SAT — User manual

How to build it, how to run it, and what each artifact contains. Cross-
referenced to the specification rows it implements, per CP 15.4.

For what is built and why, see [`ARCHITECTURE.md`](ARCHITECTURE.md); for
measured results, [`RESULTS.md`](RESULTS.md).

---

## 1. Build

### Dependencies

| Library | Required | Purpose | If missing |
|---|---|---|---|
| CMake ≥ 3.20, a C++20 compiler | yes | — | — |
| OpenCV (core, videoio, imgproc) | for video | MP4 decode **only** (§4.2) | video commands report it cleanly; everything else works |
| Eigen 3.4 | yes | fixed-size linear algebra | fetched automatically |
| toml++ 3.4 | yes | the only configuration language | fetched automatically |
| nlohmann/json 3.11 | yes | `run.json` | fetched automatically |
| doctest 2.4 | tests | — | fetched automatically |
| GLFW + OpenGL | for the GUI | the dashboard | `--gui` reports it; the rest builds and passes |

System packages are preferred; anything missing and small is fetched at the
pinned version. `just info` prints what was resolved.

```bash
just setup-system-deps   # Debian/Ubuntu, optional
just build               # configure + build (Release)
just test                # the full suite
```

Without `just`:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Useful options: `-DSAT_WITH_GUI=OFF` for a headless build.

---

## 2. Commands

### The dashboard

```bash
just gui                                    # the baseline scenario
just gui scenarios/fog_figure8.toml
```

### One run, headless

```bash
just headless                               # baseline, writes into logs/
just headless "--scenario scenarios/compliance.toml --duration 20"
```

| Flag | Effect |
|---|---|
| `--scenario F` | the TOML to run |
| `--out DIR` | where artifacts go (default `logs`) |
| `--seed N` | override the scenario's seed |
| `--duration S` | override its length |
| `--stages` | per-stage p50/p95/p99 against §15's budget (CP 14.4) |
| `--no-ai` | INV-7's switch; recorded in every artifact |
| `--no-csv` | write `run.json` but not `centroid.csv` |
| `--no-report` | skip `report.html` |
| `--bench` | no artifacts, no snapshots — a pure speed run |
| `--quiet` | no stdout summary |
| `--set k=v` | override one scenario key — see below |
| `--trace` | write `trace.csv`, the per-frame **control** trace (CP 10.x) |

`--set` takes a dotted key and a TOML value, and goes through the same schema
and the same validation as the file, so `--set control.kp=400` produces §7.5's
error rather than a run. It can be repeated.

```bash
just headless "--scenario scenarios/control/fast_linear.toml --set control.k_ff=0 --trace"
```

`--trace` writes a **diagnostic**, not a deliverable: it contains truth, it is
never graded, and its own header line says so. `centroid.csv` deliberately does
not, because a graded artifact containing the answer key is worthless as
evidence. `tools/plot_control.py` renders a trace as a standalone SVG.

### Video — Benchmark Performance-2

```bash
sat-tracker --video clip.mp4 --out logs/          # works bare
just video tests/video/clips/screen_2000x2000_30fps.mp4
```

| Flag | Effect |
|---|---|
| `--truth CSV` | ground truth for self-scoring (CP 8.7) |
| `--video-mode screen\|direct` | override auto-detection |
| `--scenario F` | camera geometry and requirements |

The mode is auto-detected from resolution: a file at least 1.5× the camera in
either axis is the **screen** (the PTZ crops and pans); anything near the
camera's own size is the **camera feed** (no cropping, pointing disabled).
§8.3 implements both readings of the requirement because its wording is
ambiguous and it is worth 30 % of the marks.

### Sweeps and the compliance matrix

```bash
just sweep                     # 200 runs, ~5 minutes on 8 cores
just matrix                    # re-read the last matrix without re-running
sat-tracker --sweep scenarios/sweeps/weather.toml --out results/ --jobs 8
```

Each run is a separate **process**, for isolation: a crash or a hang takes down
one worker and is reported as a failed run, rather than taking the sweep with
it. Failures are grouped by reason with a command that reproduces each.

### Calibration

```bash
just calibrate                 # ~1 minute; writes the bias table
just build                     # compile it in
just calibrate                 # re-run to see the gain
```

Measures §10.1.3's S-curve over 200 sub-pixel offsets × 6 sizes × 8 SNR bins ×
3 estimators and regenerates
`src/perception/centroid/bias_table_generated.inc`.

### Control checkpoints (Stage 10)

```bash
just test-control          # every Stage 10 number, as an assertion
just cp101                 # velocity feedforward on/off, with the plot
just cp102                 # a saturating slew, with and without anti-windup
just cp105                 # the IMM on the figure-8, with the mode panel
just cp106                 # error and saturation against disturbance level
just cp107                 # handover success rate, 20 seeds per arm
```

Each writes into `logs/control/`. The scenarios under `scenarios/control/` are
**instruments, not compliance claims** — each removes whatever is larger than
the effect it measures, and says so in its own header. No row of the compliance
matrix is measured on them.

### Supervisor checkpoints (Stage 12)

```bash
just test-supervisor       # every Stage 12 number, as an assertion
just cp122                 # the strategy timeline through a weather change
just cp123                 # per-condition Monte Carlo: where adapting pays
```

The supervisor is **off by default**. It changes the configuration a run uses,
so a run with it on and a run with it off are different claims — the same
argument INV-7 makes for `--no-ai` — and every artifact records which.

It is worth nothing where the fixed configuration already copes and up to
**+15 points of lock retention** where it does not; see
[`RESULTS.md` §7](RESULTS.md). That is the correct behaviour, not a
disappointing result.

### Gates

```bash
just gates                     # every static invariant check
just gate-video                # ★ CP 0.7 — can this build decode MP4 at all
just gate-repro                # ★ CP 2.6 — INV-3, every scenario twice
just ci                        # everything CI runs, before you push
```

---

## 3. Artifacts

A `--headless` or `--video` run writes three files into `--out`.

### `centroid.csv` — the graded artifact (§13.2)

```
# SAT centroid log v1
# source=scenarios/baseline.toml  mode=synthetic  build=a3f21c9  utc=...
# screen_px=2000x2000  camera_px=640x480  fps=30.000  ifov_urad=109.08
# onnxruntime=none  ai_enabled=false
# columns: frame,time_s,state,cx_screen,cy_screen,cx_cam,cy_cam,sigma_px,snr,area_px,size_est_px,bore_x,bore_y
0,0.0000,SEARCH,,,,,,,,,999.500,999.500
1,0.0333,DETECT,1423.812,674.209,331.812,180.209,0.1420,38.4,98,10,999.500,999.500
```

Four properties the format guarantees:

- **The header alone suffices to interpret the file** — geometry, rates, build
  and AI state are all in it.
- **A no-detection frame leaves the measurement columns empty** (INV-9). Never
  a stale value, never an interpolation, never a zero standing in for absence.
- **Both coordinate frames** are present, because the requirement does not say
  which is wanted.
- **The header is written and flushed at open**, so an aborted run still leaves
  a valid file.

A `centroid.csv` from a previous run is valid input to `--truth`, which is what
makes the self-scoring loop one command.

### `run.json`

Every metric, plus the **full scenario echoed** — not a path to it, because a
path is not provenance — plus the build hash, the seed, the AI state and the
INV-3 frame fingerprint.

### `report.html`

Self-contained: no network, no JavaScript, no external assets. Inline SVG
plots. ~12 KB for a 240-frame run. Opens on a machine with no internet, which
is the situation a demo is in.

---

## 4. Scenarios

TOML, validated against a schema that cites the specification row it is
enforcing:

```
scenarios/bad.toml:41: gimbal.max_pan_dps = 14.0 is outside the permitted
  range [5.0, 10.0] (specification row 13).
```

| File | Purpose |
|---|---|
| `baseline.toml` | every specification default (row-by-row annotated) |
| `compliance.toml` | baseline with the beacon in view at t=0 — the sweep's base |
| `fog_figure8.toml` | fog + a figure-of-eight path |
| `maxnoise_random.toml` | spec-maximum noise, random motion |
| `video_screen.toml` / `video_direct.toml` | the two video readings |
| `adversarial/` | **reserved and empty** — the cases designed to break it are not written yet |
| `sweeps/weather.toml` | the compliance sweep's axes |

### Key sections

```toml
[sim]     truth_hz, camera_hz, control_hz, duration_s, seed
[world]   canvas_px (row 1), edge_behaviour
[camera]  resolution (row 3), fov_deg (row 4), exposure_ms, initial_pos_px (row 6)
[target]  intensity, size_px (row 10), shape.type (row 9), initial_px (row 11)
[[target.motion]]   kind = linear|circular|lissajous|spiral|sinusoid|ou_noise|... (row 12)
[gimbal]  max_pan_dps, max_tilt_dps (rows 13-14), latency_s, encoder_lsb_urad
[noise]   gaussian_sigma (row 22), salt_pepper (row 21), poisson, hot_pixels
[atmosphere]  mode = clear|haze|fog|rain|lowlight (row 24)
[disturbance] jitter_px_per_frame (row 23); [[disturbance.platform]] (row 25)
[clutter] static_sources, decoy_beacons
[control] kp, ki, kd, k_ff, i_limit, anti_windup, smith    (design §10.4)
[tracking] imm            (CP 10.5's CV/CA/CT filter; off by default)
[supervisor] enabled, min_dwell_frames, ema_tau_frames   (design §10.6)
[[event]] t_s, action = set_atmosphere|occlude_target|spawn_decoy|platform_gust
                          (design §7.4; mode, ramp_s, duration_s, offset_px,
                           magnitude_px as the action requires)
[requirements] acquisition_s, tracking_error_px, target_loss_frac,
               reacquisition_s, min_fps (rows 16-20)
```

`[control]` defaults are `kp = 8` (the loop bandwidth in rad/s — kp *is* the
bandwidth), `ki = 2`, `kd = 0.15`, `k_ff = 1`. `smith` defaults **off**: it was
built and measured and does not help on this plant, because the loop is not
delay-limited. See [`RESULTS.md` §6](RESULTS.md) for where each number comes
from.

Motion components **stack**: several `[[target.motion]]` blocks add, so a
figure-of-eight drifting on a linear trend is two entries.

`[[event]]` blocks change the world mid-run — this is how a scenario makes the
conditions move, which is the only way to exercise the supervisor:

```toml
[[event]]
t_s = 10.0 ; action = "set_atmosphere" ; mode = "fog" ; ramp_s = 1.0
```

`ramp_s` is accepted and applied as a step at its own midpoint: `Atmosphere` is
an enum, not a severity, so there is nothing continuous to interpolate.
Pretending otherwise would be a ramp in name only.

---

## 5. Reading the numbers

Three things in the output are easy to misread, and each is labelled in place.

**Two acquisition figures.** `cold` is from run start with the beacon at a
random position; `in-view` is from the moment it enters the field of view.
§10.5 derives that the first cannot meet spec row 16 by geometry, so it is
marked *bound derived* rather than FAIL.

**Two centroiding figures.** The *image* frame is the detector alone. The
*screen* frame additionally carries the pointing error the detector cannot
influence — jitter and platform drift move the true boresight and the encoder
does not see them. Reporting only the screen number would let a disturbance
dominate the 60 %-weighted metric (INV-6).

**Retention counts frames the beacon was in view for.** `retention` is
Confirmed *and* in-view frames over in-view frames. When the tracker holds a
confirmed track while the beacon is elsewhere, those frames appear on their own
line and in `false tracks`, not in the retention ratio — an earlier version
divided by the wrong numerator and reported 100 % on a run that had lost the
target entirely.

**Handover is a claim about a downstream system.** `not reached` is the correct
answer under spec row 23's jitter, which is five times the criterion; the line
prints the best sustained offset so the gap is visible rather than implied.

**`n/a` is not zero.** A metric with no samples behind it — no truth supplied,
the beacon never in view, the track never confirmed — prints `n/a` and carries
no verdict. An absent measurement is not a passing one.

---

## 6. Troubleshooting

| Symptom | Cause |
|---|---|
| `this build has no dashboard` | GLFW/OpenGL missing at configure time. Install `libglfw3-dev libgl1-mesa-dev` and reconfigure. |
| `this build has no video support` | OpenCV missing. `just gate-video` confirms. |
| `axis '...' does not change the scenario` | a mistyped sweep key. The check is behavioural — the key was applied and nothing moved. |
| `INV-8: ...` | a video run with a scenario that declares itself synthetic and enables noise. |
| `the clip runs at N fps, which does not divide truth_hz` | raise `sim.truth_hz` to a multiple of the clip's rate. |
| Debug tests slow | expected — about 8× Release. Timeouts scale accordingly. |
