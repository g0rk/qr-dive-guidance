# qr-dive-guidance

Autonomous terminal-dive guidance for a fixed-wing UAV onto a 2 m × 2 m
ground QR target — plus the PX4 + Gazebo environment used to measure it.

*Türkçe dokümantasyon: **[README.tr.md](README.tr.md)***

![Dive onto the target](docs/dive.gif)

*Real simulated flight. The target enters frame, drifts to centre as the
aircraft closes, and is decoded — 9 readable frames, 8 of them fully inside
the target area.*

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
55° dive, 51.28° HFOV, 1920×1080):

| Altitude | Slant range | Measured px | Decode |
|---|---|---|---|
| 60 m | 73.2 m | — | 0/4 |
| 50 m | 61.0 m | — | 0/4 |
| **40 m** | 48.8 m | **70** | **4/4** ← threshold |
| 30 m | 36.6 m | 93 | 4/4 |
| 20 m | 24.4 m | 139 | 4/4 |

**Pull-up altitude loss**, 5 runs: 14.93 / 15.55 / 15.84 / 16.17 / 16.46 m.
The pull-up floor was therefore raised from 20 m to **30 m** — at 20 m the
aircraft bottomed out at 2.87–4.00 m, which does not crash in simulation but
is zero margin in reality.

**End to end**, camera chain live, 5 runs: QR read at **41.7–46.6 m**,
**5/5 success**, every one fully inside the target area.

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
| Usable frames | 1 | **12** |
| Lowest altitude | ~29.7 m | 19.05 / 19.08 m |

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
bash sim/tools/izle.sh
python3 sim/tools/komut.py takeoff     # then: komut.py align
```

### Tools

| | |
|---|---|
| `sim/tools/cam_params.py` | Single source of camera parameters — reads the model |
| `sim/tools/ucus_videosu.py` | Records the flight and decodes **every frame at full resolution** |
| `sim/tools/irtifa_limiti.py` | Continuous seconds spent below a given altitude |
| `sim/tools/measure_alt.py` | Decode threshold against altitude |
| `sim/tools/hud_preview.py` | Renders synthetic tilted targets to check the overlay |
| `sim/tools/make_gif.py` | Cuts the dive out of a recording as a GIF |
| `sim/tools/build_world.py` | World generator (XML tree, **not** regex) |

### Camera

DFM 37UR0234-ML · onsemi AR0234CS · 1/2.6" · 1920×1200 · 3.0 µm · 6 mm lens

```
HFOV = 2·arctan(5.76 / (2·6)) = 51.28°      VFOV = 30.22°
```

The sensor is 16:10, but the delivery format allows only 4:3, 5:4 or 16:9 —
so the readout is cropped to 1920×1080, and the vertical FOV that matters is
30.22°, not the 33.40° the full sensor would give.

---

## Tests

```bash
python3 -m pytest tests/ -q      # 39 functions / 132 checks
```

Most tests were written **after a real bug**, and each one opens by
explaining how that bug happened:

- `test_hedef_koordinati.py` — the target coordinate is not hard-coded, it is
  re-derived from the world file. It once pointed 49.9 m southwest of the
  pad because two different "home" origins had been conflated.
- `test_durum_kurulumu.py` — instantiates every state machine state. One
  state was missing an import; the aircraft could not take off under any
  circumstances, and the failure surfaced only as a warning line.
- `test_gps_gudum.py` — locks the *sign* of the lateral dive correction. A
  correction that banks the wrong way does not close the offset, it widens
  it.
- `test_dive_geometry.py` — loads the portable module in a separate process
  and asserts it pulled in **no** project or heavy dependencies.

> The `check()` helper did not `assert` for a while. Sub-checks printed
> `KALDI` on screen while pytest reported green. Fixed.

---

## Known gaps

- Everything has been validated in simulation. There is no real flight data,
  and the camera intrinsics come from a datasheet rather than a calibration.
- The target-area margins (25 % horizontal, 10 % vertical) were read off a
  figure, not from text.
- No air-to-air tracking, no no-fly-zone avoidance, no ground-station client.
- Comments are being migrated to English; the older modules are still in
  Turkish.

## Licence

See [LICENSE](LICENSE). The base flight-control skeleton came from a shared
first version; the simulation environment, measurement tooling, dive
geometry and guidance work in this repository were added afterwards. The
commit history carries the reasoning for each step — those messages are in
Turkish, but the same reasoning is repeated in the code comments.
