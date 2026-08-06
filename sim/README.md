# Simulation assets

The PX4 gz (Garden) simulation environment for this repository.

## Install

```bash
./install.sh ~/PX4-Autopilot
cd ~/PX4-Autopilot && make px4_sitl gz_rc_cessna_cam
```

## Contents

| | What it is |
|---|---|
| `models/qr_pad` | **Base model.** 2×2 m QR with 45°, 3 m plates on all four sides (rulebook p.17). Kept for reference — the world does *not* use it |
| `models/qr_pad_v1` | **The model the world uses.** Same geometry, corrected texture |
| `models/nose_cam` | Nose camera. Derived from PX4's `mono_cam`, **with real hardware parameters** |
| `models/rc_cessna_cam` | `rc_cessna` + `nose_cam`, patterned on PX4's `x500_mono_cam` |
| `models/grass_field` | Textured grass around the pad, so detection is tested against a realistic background |
| `worlds/qr_target.sdf` | `default.sdf` + `qr_pad` @ (500 m east, 0) |
| `airframes/4009_gz_rc_cessna_cam` | Selects the world and the model |

## Camera parameters — no longer an assumption

**The real hardware was confirmed on 2026-08-05:**

| | Value |
|---|---|
| Model | **DFM 37UR0234-ML** (The Imaging Source) |
| Sensor | onsemi **AR0234CS** CMOS |
| Format | **1/2.6"** |
| Resolution | 1920 × 1200 |
| Pixel size | 3.0 µm |
| Effective area (vendor) | 5.62 mm × 3.20 mm |
| Lens | 6 mm |

An earlier session had *assumed* "6 mm + 2 MP → 1/2.6" (AR0234)". **The
assumption turned out to be right**, so none of the earlier range or geometry
measurements were invalidated.

`nose_cam` is **deliberately different** from PX4's `mono_cam`:

| | PX4 mono_cam | nose_cam |
|---|---|---|
| HFOV | 1.74 rad = 99.7° | **0.8950 rad = 51.28°** |
| Resolution | 640×480 | **1920×1080** |

### Where the HFOV comes from

```
HFOV = 2 · arctan( sensor_width / (2 · focal_length) )
     = 2 · arctan( 5.76 / (2 · 6) ) = 51.28° = 0.8950 rad
```

### ⚠️ The vendor's page contradicts itself

| | Pixel count × 3.0 µm | Vendor says |
|---|---|---|
| Horizontal | 1920 × 3.0 µm = **5.76 mm** | 5.62 mm |
| Vertical | 1200 × 3.0 µm = **3.60 mm** | 3.20 mm |
| Vertical (1080 rows) | 1080 × 3.0 µm = **3.24 mm** | ← "3.20" is far closer to this |

So the "effective area" probably does not describe the full 1920×1200 array.
The renderer's pinhole model has to be consistent with the pixel geometry: gz
samples 1920 columns, each 3.0 µm, so **5.76 mm** is what is used.

**The size and direction of the uncertainty:**

| Sensor width | HFOV |
|---|---|
| 5.76 mm | 51.28° |
| 5.62 mm | 50.19° |

The difference is only **1.09° (2.1 %)**. A wider HFOV means more world fits
into the same 1920 pixels, so the target looks **smaller**. Using 51.28° when
the truth is 50.19° therefore makes the QR appear 2.1 % smaller and
**underestimates** the decode range. The error is on the safe side, and the
measured 40 m decode threshold is unaffected because it was found in 10 m
steps.

> **The definitive answer comes from measurement:** once the camera is in
> hand, a chessboard and OpenCV `calibrateCamera`. This is not a datasheet
> argument to be won.

### Why 1080 rows and not 1200

The sensor is 1920×1200 = **16:10**. The rulebook (p.12) allows only **4:3,
5:4 and 16:9** — **16:10 is not on the list.** The judge video has to be 16:9,
so the readout is cropped to 1080 rows (60 dropped from the top and 60 from
the bottom).

The vertical field of view that results:
```
VFOV = 2 · arctan( tan(HFOV/2) · 1080/1920 ) = 30.22°
```
The full sensor would give 33.40° — which cannot be used.

## Target coordinate

`qr_pad` sits at (X = 500 m east, Y = 0) in the world.

⚠️ **There are two different "home" origins, and confusing them costs 50 m.**
This section previously documented the wrong one, which is exactly the bug it
should have prevented:

| | Latitude / Longitude | When it applies |
|---|---|---|
| PX4's documented default | 47.397742 / 8.545594 | simulators **other than** gz |
| The gz world's own `<spherical_coordinates>` | **47.397971 / 8.546164** | **under gz — this is the one** |

They are 49.9 m apart. Deriving the target from the first one put it 49.9 m
southwest of the pad; since the pad is 2 m × 2 m, that is 25 times its edge
length, and the aircraft dived at empty grass.

The correct values, which are what `config.py` holds:

```python
TARGET_LATITUDE_DEG  = 47.3979711
TARGET_LONGITUDE_DEG = 8.5527992
```

Verified: 500.0 m at a bearing of 90.0° from the world origin. This is not
left to trust — `tests/test_hedef_koordinati.py` reads the world SDF and
re-derives both numbers, so the world and the config cannot drift apart in
silence.

## Known gaps

- The models have no `<collision>`, only `<visual>` — the aircraft flies
  straight through them. That is fine for this purpose, but it does mean the
  simulation cannot tell you anything about an impact.
- The camera intrinsics come from a datasheet, not a calibration (see above).
- `sim/tools/measure_alt.py` subscribes to `/cam60` … `/cam20`, but nothing in
  this repository generates a world with those cameras. The decode-threshold
  measurement it reports was taken with a multi-camera world that was never
  committed. Reproducing it currently means building that world by hand.

### Gaps that used to be listed here and are now fixed

- ~~QR Version 2 while the rulebook asks for Version 1~~ — fixed by
  `qr_pad_v1` (21×21 modules), which is what the world loads.
- ~~Quiet zone of about 0.2 modules~~ — `qr_pad_v1` uses 2 modules
  (80 px / 840 px). The base `qr_pad` still has the small quiet zone on
  purpose, as a record of what the problem looked like.
