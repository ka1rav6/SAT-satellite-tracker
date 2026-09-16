# `scenarios/adversarial/` — the cases written to break it

Every scenario under `scenarios/` other than these was written by someone who
knew what the system does, which is exactly why they do not find this class of
bug. These are the opposite: each one is a configuration chosen because there
is a specific reason to expect the system to handle it badly, and each file
says which reason.

They are **not** compliance claims and no row of the compliance matrix is
measured on them. Several are expected to produce poor numbers — that is the
point, and the file says what "poor" means before you run it.

| File | The attack | Measured (25 s) |
|---|---|---|
| `decoy_swarm.toml` | 16 near-identical decoys inside one field of view | **tracking 355.30 px, centroiding 203.03 px** — it locks onto a decoy and holds it |
| `target_faster_than_mount.toml` | 1,500 px/s target against an 800 px/s mount | **tracking 16,652 px**, 30 re-acquisition episodes — it cannot keep up, and degrades instead of falling over |
| `strobing_target.toml` | occluded 1 s, visible 1 s, fifteen times | **retention 48.93 %**, 19 re-acquisitions — a 1 s gap is 30 misses against a 15-frame coast budget, so every gap deletes the track |
| `all_weather_churn.toml` | the atmosphere changes every 2 s for 40 s | **retention 91.47 %**, 42 re-acquisitions — the supervisor keeps up without chattering |
| `beacon_larger_than_fov.toml` | a 20 px beacon on a 32 × 24 px field of view | **retention 99.73 %, centroiding 0.46 px** — handled, contrary to the prediction below |
| `edge_camper.toml` | a target bouncing off the canvas edge ~once per frame | **retention 99.73 %, centroiding 0.18 px** — handled, contrary to the prediction below |

### Two predictions that were wrong

Each file states, before its results, why the system was expected to struggle.
Two of those arguments did not survive contact with the measurement, and the
files keep both the argument and the refutation rather than being quietly
rewritten.

**`beacon_larger_than_fov`** was expected to break the background estimate: the
top-hat assumes a structuring element larger than the target, and CFAR assumes
a training window that excludes it. Both assumptions are violated here and the
detector works anyway — 0.46 px of centroiding error. The reason is that the
guard band and training window are sized in PIXELS on the sensor, not as a
fraction of the field of view, so shrinking the field of view does not shrink
them relative to the 20 px beacon.

**`edge_camper`** was expected to defeat the constant-velocity filter, which
is wrong twice per frame when the target bounces every frame. It does defeat
the velocity estimate — and the track survives regardless, because the
reachability gate holds it: the target never moves *far*, only *fast*, so every
measurement stays within reach. That gate was added at Stage 6 for an unrelated
reason and this is the case that shows what else it buys.

Run one with:

```bash
just headless "--scenario scenarios/adversarial/decoy_swarm.toml --duration 20"
```

`just fuzz` covers the complementary case — 5,000 *random* legal
configurations, where the value is breadth rather than intent.
