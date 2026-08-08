# qr-dive-guidance

[![tests](https://github.com/g0rk/qr-dive-guidance/actions/workflows/tests.yml/badge.svg)](https://github.com/g0rk/qr-dive-guidance/actions/workflows/tests.yml)
[![licence: MIT](https://img.shields.io/badge/licence-MIT-blue.svg)](LICENSE)

Autonomous terminal-dive guidance for a fixed-wing UAV onto a 2 m × 2 m
ground QR target — plus the PX4 + Gazebo environment used to measure it.

**Every number in this README came from a measurement, and the measurement is
in the repository.** Where a number turned out to be wrong, the correction is
written down next to it rather than quietly edited out.

*Türkçe dokümantasyon: **[README.tr.md](README.tr.md)***

![Dive onto the target](docs/dive.gif)

*Real simulated flight, nothing staged. The target enters frame, drifts
toward centre as the aircraft closes, and is decoded — 8 readable frames, 7 of
them fully inside the target area. The animation freezes for a moment on the
last of those seven, because at 32 m/s the whole readable window lasts about
half a second.*

---

## The problem

A fixed-wing aircraft must dive onto a QR code lying flat on the ground,
read it, and pull out safely. Three constraints make this harder than it
sounds:

- **The camera is bolted to the nose, aligned with the body axis.** It does
  not look down. It looks wherever the nose looks.
- **The target is surrounded by 45° angled plates, 3 m tall**, specifically
  so it cannot be read in level flight. Seeing the whole code requires a
  look-down angle of at least 45°.
- **The entire QR must fall inside a defined target area** in the frame —
  not merely be decoded. A code hanging over the edge is not a hit.

Everything below follows from those three facts.

---

## Why the dive trigger distance is 108 m

The obvious derivation is wrong, and it fails silently: the aircraft flies
a textbook dive and photographs nothing but grass.

The first version computed `entry_altitude / tan(pitch)` = `120 / tan(55°)`
= **84 m**. Two separate mistakes:

**1. Commanded pitch is not the achieved flight path angle.** The wing keeps
producing lift, so the aircraft travels along a shallower line than the nose
points. Recovered from a flight log: descending 80 m while covering 80.1 m
of ground gives **45°**, ten degrees shallower than the commanded 55°.

The instantaneous angle does reach ~50° mid-dive. But the *average* is what
sets the trigger distance, and early in the dive the nose has not come down
yet — a lot of ground covered, little altitude lost.

**2. It aimed at arrival, not at detection.** The target must be *in front
of* the camera when the aircraft reaches decode altitude, not underneath it.

The correct derivation has two parts:

```
d_bore = decode_altitude / tan(pitch)              = 40 / tan(55°) = 28.0 m
d_dive = (entry − decode) / tan(path_angle)        = 80 / tan(45°) = 80.1 m
trigger = 108.0 m
```

With the wrong trigger the target sat **3.9 m ahead** at 40 m altitude,
while the camera saw the ground from **17.2 m to 54.0 m** ahead. The target
was below the bottom edge of the frame at exactly the moment it needed to be
readable: **4879 frames, zero detections.**

With the corrected trigger the target stays in frame continuously:

| Altitude | Ground distance remaining | Line of sight | In frame |
|---|---|---|---|
| 120 m | 108.0 m | 48.0° | ✅ |
| 80 m | 68.0 m | 49.6° | ✅ |
| **40 m** | **28.0 m** | **55.0°** | ✅ boresight |
| 30 m | 18.0 m | 59.0° | ✅ |

Predicted vs measured in flight: 28.0 m predicted, **28.9 m measured**.

---

## The four corners

<img src="docs/hud_rotated_45deg.png" width="560" alt="Overlay on a rotated target">

The detector returns the QR's four actual corners. Drawing the axis-aligned
bounding box instead throws that away — and the box is a poor stand-in:

| View | Box area ÷ quad area |
|---|---|
| Head-on | 1.11× |
| 30° rotation | 1.86× |
| **45° rotation** | **2.00×** |

What inflates the box is **rotation**, not perspective. In the image above
the thin grey rectangle is the old drawing; the green quadrilateral is the
real target boundary.

The overlay draws both diagonals (on a tilted target the two halves are
visibly unequal — precisely the information the box destroys), labels the
corners `0-1-2-3`, and marks the centre at the **diagonal intersection**
rather than the box midpoint. Under perspective those differ by 0.0 px
head-on, 2.8 px in a realistic dive, 9.2 px at aggressive angles.

Corner *ordering* is not cosmetic: pose estimation needs the corners in a
known sequence, and a silently rotated ordering produces a plausible-looking
but wrong pose.

> **One claim was measured and turned out to be false.** It was argued that
> because the box is up to 2× larger, a valid hit could be rejected as
> "outside the target area". Wrong: the target area is an axis-aligned
> rectangle, and a quadrilateral lies inside one exactly when its four
> corners do — which is the bounding box test. The two are identical. 41
> positions and angles were tried: **zero disagreement.** The corners earn
> their place through the centre estimate, pose estimation and the overlay,
> not through the containment test.

---

## Measurements

**Decode threshold** (Gazebo render, real perception code, V1 texture,
55° look-down, 51.28° HFOV, 1920×1080). Reproduce it with one command:

```bash
bash sim/tools/measure_alt.sh
```

| Altitude | Slant range | Measured px | Decode |
|---|---|---|---|
| 60 m | 73.2 m | — | 0/4 |
| 50 m | 61.0 m | — | 0/4 |
| **40 m** | 48.8 m | **70** | **4/4** ← threshold |
| 30 m | 36.6 m | 93 | 4/4 |
| 25 m | 30.5 m | 112 | 4/4 |
| 20 m | 24.4 m | 140 | 4/4 |

The rig is six static cameras at those altitudes, each boresighted on the pad
down the dive's own look-down angle, in a world of their own
(`sim/worlds/qr_measure.sdf`). No aircraft is involved: the cameras do not
move, so this measures the render and the perception code and nothing else.

**Pull-up altitude loss**, 5 runs: 14.93 / 15.55 / 15.84 / 16.17 / 16.46 m.
The pull-up floor was therefore raised from 20 m to **30 m** — at 20 m the
aircraft bottomed out at 2.87–4.00 m, which does not crash in simulation but
is zero margin in reality.

**End to end**, camera chain live, 5 runs (2026-08-06): QR read at
**38.1–44.1 m**, **5/5 success**, every one fully inside the target area.

The spread between runs is real and larger than one set of five can show.
An earlier set of five gave 41.7–46.6 m; these gave 38.1–44.1 m. The two
overlap, the samples are small, and the texture changed in between, so the
difference is scatter rather than a trend — but 38.1 m is the worst
observation on record and `config.QR_FIRST_DETECT_ALTITUDE_M` is set from it.

---

## Continuing after the first read

Pulling up on the first valid detection collected exactly **one** decodable
frame per flight, at 70 px — right on the decode threshold. One frame
satisfies the requirement and is also zero margin: lose it to compression or
a timing hiccup and the run scores nothing.

Validation happens in a **±1 second window around the reported dive-end
time**. Descent is ~32 m/s, so one second is **32 metres of altitude** —
every decodable frame already falls inside that window. Continuing costs
nothing.

`KAMIKAZE_QR_CONTINUE_ALTITUDE_M = 35.0` is expressed as an **altitude**,
not a duration: the binding constraint is a minimum flight altitude, and the
altitude cost of "keep going for 0.3 s" varies with descent rate.

| | Before | After |
|---|---|---|
| Usable frames | 1 | **5 – 10** |
| Lowest altitude | ~29.7 m | 19.05 / 19.08 m |

"12" appeared here once; it was a single good flight. Five runs give
5 / 7 / 10 / 10 / 10. The worst is still five times what is required, and
that is the claim worth making.

---

## Portable modules

Two files are dependency-free (standard library only) and can be copied into
any project:

**`dive_geometry.py`** — the derivation above, as a callable API. Camera
geometry from lens and sensor or from FOV and aspect ratio; trigger
distance; line of sight; ground footprint; visible altitude band; detection
window. Includes `effective_path_angle_deg()` to recover the achieved path
angle from a flight log, which is the honest way to obtain that number.

**`server_time.py`** — clock-offset tracking and timestamp formatting for a
mission server. Its design rule: if the offset has never been established it
**refuses** to produce a timestamp rather than falling back to local time.
A missing offset that silently degrades looks completely normal, passes
every type check, and invalidates the entire recording.

```bash
python3 dive_geometry.py     # runnable demo, prints the table above
python3 server_time.py
```

---

## Running the simulation

```bash
./sim/install.sh ~/PX4-Autopilot
```

`gz_bridge` in this PX4 version does **not** start Gazebo itself — start gz
separately and run px4 with `PX4_GZ_STANDALONE=1`. To watch it:

```bash
bash sim/tools/watch.sh
python3 sim/tools/command.py takeoff     # then: command.py align
```

### Tools

| | |
|---|---|
| `sim/camera.yaml` | **The camera.** Hand written, and everything else is derived from it |
| `sim/tools/cam_params.py` | Reads that YAML; `--write-model` regenerates the gz model from it |
| `sim/tools/flight_video.py` | Records the flight and decodes **every frame at full resolution** |
| `sim/tools/altitude_limit.py` | Continuous seconds spent below a given altitude |
| `sim/tools/measure_alt.sh` | **One command** for the decode-threshold table above |
| `sim/tools/measure_alt.py` | Decode threshold against altitude (reads the rig out of the world) |
| `sim/tools/hud_preview.py` | Renders synthetic tilted targets to check the overlay |
| `sim/tools/make_gif.py` | Cuts the dive out of a recording as a GIF |
| `sim/tools/build_world.py` | World generator (XML tree, **not** regex); `--measure` builds the camera rig |

### Camera

DFM 37UR0234-ML · onsemi AR0234CS · 1/2.6" · 1920×1200 · 3.0 µm · 6 mm lens

```
HFOV = 2·arctan(5.76 / (2·6)) = 51.28°      VFOV = 30.22°
```

**To use a different camera, edit one file.** `sim/camera.yaml` holds the
sensor width, focal length and resolution — or just an `hfov_deg` if that is
all you know — and the Gazebo model is generated from it:

```bash
python3 sim/tools/cam_params.py --write-model
```

The parameters used to live in the Gazebo model itself, which is the wrong
shape for anyone who is not running the simulation: they have a lens and a
datasheet, not an SDF. If the YAML is missing the tools fall back to the
generated model and *say so*; if both are gone they refuse to run rather than
invent a default, because a wrong HFOV would flow into the trigger distance,
the decode table and the measurement rig at once without anything noticing.

The sensor is 16:10, but the delivery format allows only 4:3, 5:4 or 16:9 —
so the readout is cropped to 1920×1080, and the vertical FOV that matters is
30.22°, not the 33.40° the full sensor would give.

---

## Tests

```bash
python3 -m pytest tests/ -q      # 43 functions / 147 checks
```

Most tests were written **after a real bug**, and each one opens by
explaining how that bug happened:

- `test_target_coordinate.py` — the target coordinate is not hard-coded, it is
  re-derived from the world file. It once pointed 49.9 m southwest of the
  pad because two different "home" origins had been conflated.
- `test_state_construction.py` — instantiates every state machine state. One
  state was missing an import; the aircraft could not take off under any
  circumstances, and the failure surfaced only as a warning line.
- `test_gps_guidance.py` — locks the *sign* of the lateral dive correction. A
  correction that banks the wrong way does not close the offset, it widens
  it.
- `test_dive_geometry.py` — loads the portable module in a separate process
  and asserts it pulled in **no** project or heavy dependencies.

> The `check()` helper did not `assert` for a while. Sub-checks printed
> `FAIL` on screen while pytest reported green. Fixed.

---

## Known gaps

**The safety checker never runs.** `safety.py` is written, instantiated, and
then skipped: `processes/mission_controller.py` wraps the call in `if False:`,
so the three checks it implements — telemetry staleness (>1 s), minimum
relative altitude (5 m) and maximum groundspeed (80 m/s) — evaluate on no
tick of any flight. This came from the base version and was never re-enabled.

It is worth being precise about why, because "somebody forgot" is the wrong
diagnosis and would lead to the wrong fix. Measured against the real checker:

| relative altitude | result |
|---|---|
| 0.0 m — on the ground | **ABORT** — "Altitude too low: 0.0m < min 5.0m" |
| 2.0 m — seconds into the climb | **ABORT** |
| 5.1 m and above | pass |

The mission controller only exempts `IDLE` and `ABORT` from the safety
transition, so switching the check on as written aborts the mission during
every single takeoff. The `if False:` is load-bearing. Making it real means
teaching the altitude check what "on the ground" and "climbing out" are —
not deleting one line.

The reason this belongs at the top of the list rather than buried: `telemetry.py`
documents its own defaults as *"safe/invalid values so the safety checker will
reject a TelemetryData that has never been updated"*. That sentence describes
a defence that is switched off three files away. Nothing in the repository
depends on it today, and nothing tests that it is off.

- Everything has been validated in simulation. There is no real flight data,
  and the camera intrinsics come from a datasheet rather than a calibration.
- The target-area margins (25 % horizontal, 10 % vertical) were read off a
  figure, not from text.
- No air-to-air tracking, no no-fly-zone avoidance, no ground-station client.
- The end-to-end numbers come from five runs each. Five is enough to show that
  the spread exists and roughly how wide it is; it is not enough to put a
  confidence interval on any of them.

## Licence

See [LICENSE](LICENSE). The base flight-control skeleton came from a shared
first version and is published here with its authors' permission; the
simulation environment, measurement tooling, dive geometry and guidance work
in this repository were added afterwards. The very first commit is that
skeleton on its own, untouched, so `git diff` against it shows exactly which
part is which. The commit history carries the reasoning for each step, and
the same reasoning is repeated in the code comments.
