# The demo script (design §14.1 — Functional Verification, 20 %)

Ten minutes, minute by minute. Every step is a command or a click that exists;
nothing here is aspirational. Run it end to end three times before presenting,
including once where someone hands you a scenario file and an MP4 you have not
seen (CP 15.3).

**Before you start**

```bash
just build && just gates
just gate-video          # ★ proves this machine can decode MP4 at all
```

If `gate-video` fails, the video half of the demo will not work and you need to
know that now rather than at 4:00. The rest still runs.

---

## 0:00–1:00 — the problem, and why it is hard

One sentence: *a moving optical terminal has to point a narrow beam at another
moving terminal, and the camera is the only sensor that can find it.*

The one number that frames everything (`docs/RESULTS.md` §8):

> The loop stops meeting the 10 px requirement at a **600 px/s** rate demand —
> three quarters of the mount's authority, with the plant never once at its
> stops. Saturation is not what breaks it; **transport lag** is.

That is the whole difficulty in one line: the disturbance can outrun the motor,
so a reactive loop cannot win and the system has to predict.

---

## 1:00–3:00 — the baseline, running

```bash
just gui
```

Point at, in this order:

1. **The camera view.** The beacon, the centroid marker, the search pattern
   when it has not acquired yet.
2. **The mode strip.** Idle → Search → Detect → Acquire → Track. Say that the
   FSM is what decides whether the tracker or the search pattern owns the aim
   point, and nothing else in the system is allowed to make that call.
3. **The error traces.** Two of them, and they are different quantities —
   INV-6. Centroiding error is the detector; tracking error is the loop. A
   system that conflated them would be reporting one number that means neither.

---

## 3:00–5:00 — dial the damage up, live

In **Run control → Damage**:

| Click | What to say |
|---|---|
| `Clean` | the loop with nothing in its way |
| `Sensor noise` | specification rows 21–22: shot noise, read noise, hot pixels |
| atmosphere → `fog` | row 24 |
| salt & pepper → 0.10 | row 21's 10 % impulse noise — **30,720 impulses against a 100-pixel beacon** |
| jitter px/frame → 20 | row 23 — **watch the tracking trace jump from ~0.5 px to ~17 px** |
| `Full spec` | everything at once |

That jitter slider is the one to spend a sentence on. It moves the **true**
boresight, not the pixels (§9.3), so the camera view barely changes while the
tracking trace multiplies by thirty. Row 23 is redrawn every frame where the
controller cannot see it, and on its own it leaves `sqrt(2·20²/3) = 16.33 px`
under row 17's 10 px budget. Say that the loop tracks to **0.45 px** with it
off and **17.02 px** with it on, that both numbers are asserted in the test
suite, and that this is why row 17 is reported as *bound derived* rather than
FAIL.

It holds. The number behind that: the median filter takes 1,593 impulses down
to **5** survivors, and the matched filter integrates the beacon over its whole
area for a factor of ~√100 in SNR before any threshold is applied.

Then click **closed loop (INV-2)** off. The camera stops following
*immediately*. That is CP 1.8's acceptance criterion and it is the only thing
that proves the camera follows because the controller told it to, rather than
because the simulator conveniently centres it.

---

## 5:00–6:30 — turn feedforward off, live

**Run control → Algorithms → velocity feedforward.**

Load `scenarios/control/fast_linear.toml` first — at specification row 12's
24.6 px/s the lag is 3 px and nobody will see it. At 200 px/s it is 25 px.

Watch the **tracking** trace, not the camera view. Measured:

| | tracking error |
|---|---|
| feedback only | **21.60 px** |
| + velocity feedforward | **2.53 px** |

Then say why it is not just a gain that happened to help: pure feedback lags by
`speed ÷ bandwidth`, so the error is *structural*, and feeding the estimated
target velocity forward removes it rather than fighting it. The controller
stops waiting to observe an error before moving.

If asked why not just raise the gain: the loop goes unstable, and the
integrator — which *does* eventually remove the lag — takes `kp/ki = 4 s` to do
it, which is far too slow to save the acquisition window rows 16 and 17 are
both scored in.

---

## 6:30–8:00 — the figure-8 and the IMM

```
File → scenarios/control/figure8.toml
```

**Run control → Algorithms → IMM** (it takes effect on the next track).

| | tracking RMS | peak |
|---|---|---|
| single CV filter | 21.75 px | 30.30 px |
| IMM (CV/CA/CT) | **17.15 px** | **22.52 px** |

Watch the mode-probability panel move around the lap.

**Say the correction out loud** — it is a better story than the original claim.
The design predicted the overshoot would be at the figure-8's *crossing*,
because that is where the acceleration reverses. It is not: at the
self-intersection both second derivatives are zero, the path is locally
straight, and a constant-velocity model is *right* there. The worst error is at
the **lobe ends**, where the curvature is tightest — 13.5 px at the crossing
against 26.5 px at the lobes.

---

## 8:00–9:00 — video, the graded path (30 % of marks)

```bash
just video tests/video/clips/screen_2000x2000_30fps.mp4
```

Two numbers:

- The crop's own centroid error is **0.0033 px RMS** over 400 sub-pixel phases —
  the virtual camera adds essentially nothing to what the detector reports.
- Self-scoring against a supplied truth CSV: **0.0036 px RMSE**.

Mention INV-8: in a video mode the damage chain is *disabled by the code path*,
not by a flag. A clip's noise is whatever was in front of the camera, and
adding more would be scoring the detector on damage we invented.

---

## 9:00–10:00 — what does not work, and the numbers for it

Do this part. It is worth more than another working demo.

| Gap | Measured |
|---|---|
| clutter and decoys | tracking 17.64 px clean → **205.20 px** with 120 clutter + a decoy |
| low light | target loss reaches **86 %** — the beacon is below the detector's floor, a different failure from mis-association |
| handover under row 23 | 100 % clean, **0 %** at ±20 px/frame — row 23's jitter is 5× the criterion, and no control law changes that |
| §15's 0.85 ms frame budget | 46.6 ms; a different processing strategy, not faster arithmetic |

And the two places the **specification** is internally inconsistent, both
reported as derived bounds rather than failures:

- Row 16 asks for 2 s cold acquisition. The geometry gives 18.7 s and
  P(visible at t=0) = 7.68 %.
- Row 17 asks for 10 px while row 23 independently allows ±20 px/frame of
  jitter on the true boresight, a **16.33 px** RMS floor no controller can go
  below.

---

## If something breaks

```bash
just gui scenarios/spec_defaults.toml   # the known-good fallback (CP 15.3)
```

`scenarios/spec_defaults.toml` is the specification's rows 1-25 with the
beacon placed inside the field of view at t=0, and it is also what `just gui`
loads with no argument. Measured, 30 s, seed 42: 0.20 px centroiding,
0.067 s acquisition, 100 % FOV containment, rows 16/18/19/20 all PASS. If an
unfamiliar scenario misbehaves, fall back to it and say what you were trying
to show.

> **This used to name the old `baseline.toml`, and that was wrong.** That file
> scores 913 px tracking RMS, zero centroiding frames and 1778 false tracks a
> minute — it was the worst run in the repository, and it was simultaneously
> the documented fallback, the default for `--gui` and `--headless`, the first
> entry in the scenario picker and the source of the README's hero screenshot.
> It is now [`scenarios/hard/cold_start_in_clutter.toml`](../scenarios/hard/cold_start_in_clutter.toml),
> its measured failure is written into its own header, and it appears in the
> picker under a "Hard cases" separator. Step 6 of this demo runs it
> deliberately.
