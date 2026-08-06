"""Terminal dive geometry for a nose-mounted, body-fixed camera.

Standalone and dependency-free: standard library only. No autopilot, no
simulator, no vision library. Copy this file into any project.

WHAT PROBLEM THIS SOLVES
------------------------
An aircraft dives at a ground target and must SEE that target with a camera
that is bolted to the nose, pointing along the body axis. The camera does
not look down -- it looks where the nose looks. Getting the dive to start at
the right distance is therefore a geometry problem, and getting it wrong is
silent: the aircraft flies a perfect dive and the target is simply not in
frame.

TWO MISTAKES THIS MODULE EXISTS TO PREVENT
------------------------------------------
1. Using COMMANDED PITCH as if it were the FLIGHT PATH ANGLE.
   On a fixed-wing aircraft the wing keeps producing lift during the dive,
   so the aircraft travels along a shallower line than the nose points.
   Measured on a real run: commanded pitch -55 deg, instantaneous flight
   path angle about -50 deg, but the AVERAGE over the dive only 45 deg --
   because early in the dive the nose has not come down yet, so the aircraft
   covers a lot of ground while losing little altitude. The average is what
   sets the trigger distance.

2. Aiming at the moment of ARRIVAL instead of the moment of DETECTION.
   The target must be in front of the camera when the aircraft reaches
   decode altitude -- not underneath it. A derivation that only asks "when
   do I arrive over the target" puts the target below the bottom edge of
   the frame exactly when it needs to be readable.

Both mistakes were made, measured, and corrected in the project this module
came from. With the wrong trigger distance the target sat 3.9 m ahead at
40 m altitude while the camera saw the ground from 17.2 m to 54.0 m ahead:
4879 frames, zero detections.

CONVENTIONS
-----------
All angles in degrees, all distances in metres. Angles below the horizon are
positive (a 55 deg dive is passed as 55.0, not -55.0). Horizontal distance
means ground distance to the target, not slant range.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = [
    "CameraGeometry",
    "DiveProfile",
    "trigger_distance_m",
    "line_of_sight_deg",
    "visible_altitude_band",
    "detection_window",
    "effective_path_angle_deg",
    "ground_footprint_m",
]


# ---------------------------------------------------------------- camera ---
@dataclass(frozen=True)
class CameraGeometry:
    """Field of view of a body-fixed camera.

    Only the vertical field of view matters for dive geometry; the
    horizontal FOV decides lateral tolerance, not whether the target is in
    frame during a wings-level dive.

    Build it from the lens and sensor with `from_sensor`, or pass the
    angles directly if you calibrated the camera (recommended -- a
    calibration beats a datasheet).
    """

    hfov_deg: float
    vfov_deg: float

    @classmethod
    def from_sensor(cls, sensor_width_mm: float, sensor_height_mm: float,
                    focal_length_mm: float) -> "CameraGeometry":
        """Pinhole model: fov = 2*atan(sensor_extent / (2*focal_length)).

        NOTE: use the sensor extent you ACTUALLY READ OUT. Cropping a
        16:10 sensor to 16:9 (common, because many video specs disallow
        16:10) reduces the vertical extent and therefore the vertical FOV.
        Feeding the full sensor height while reading out a crop overstates
        VFOV and makes the target look more visible than it is.
        """
        f = float(focal_length_mm)
        if f <= 0:
            raise ValueError("focal_length_mm must be positive")
        return cls(
            hfov_deg=math.degrees(2 * math.atan(sensor_width_mm / (2 * f))),
            vfov_deg=math.degrees(2 * math.atan(sensor_height_mm / (2 * f))),
        )

    @classmethod
    def from_hfov_and_aspect(cls, hfov_deg: float, width_px: int,
                             height_px: int) -> "CameraGeometry":
        """Derive VFOV from HFOV and the rendered/readout aspect ratio."""
        h = math.radians(hfov_deg)
        v = 2 * math.atan(math.tan(h / 2) * height_px / width_px)
        return cls(hfov_deg=hfov_deg, vfov_deg=math.degrees(v))


# ------------------------------------------------------------------ dive ---
@dataclass(frozen=True)
class DiveProfile:
    """Everything that defines the dive, in one place.

    entry_altitude_m
        Altitude above ground at which the dive begins.
    decode_altitude_m
        Altitude at which the target first becomes reliably detectable.
        MEASURE THIS -- do not compute it from pixel maths alone. Real
        decoding depends on compression, motion blur, texture and lighting.
    pitch_deg
        Commanded nose-down attitude during the dive (positive number).
        The camera boresight points this far below the horizon.
    path_angle_deg
        AVERAGE achieved flight path angle over the dive (positive number).
        This is NOT the commanded pitch. See `effective_path_angle_deg` to
        recover it from a flight log.
    """

    entry_altitude_m: float
    decode_altitude_m: float
    pitch_deg: float
    path_angle_deg: float

    def __post_init__(self) -> None:
        if not 0.0 < self.pitch_deg < 90.0:
            raise ValueError("pitch_deg must be in (0, 90), nose-down positive")
        if not 0.0 < self.path_angle_deg < 90.0:
            raise ValueError("path_angle_deg must be in (0, 90)")
        if self.decode_altitude_m >= self.entry_altitude_m:
            raise ValueError("decode_altitude_m must be below entry_altitude_m")
        if self.decode_altitude_m <= 0.0:
            raise ValueError("decode_altitude_m must be positive")


# ------------------------------------------------------------- functions ---
def effective_path_angle_deg(altitude_lost_m: float,
                             ground_distance_covered_m: float) -> float:
    """Recover the achieved flight path angle from a flight log.

    This is the honest way to obtain `DiveProfile.path_angle_deg`: fly the
    dive once, take two points, and solve. Do not guess it and do not reuse
    the commanded pitch.

    Worked example from a real run: the aircraft lost 80.0 m of altitude
    while covering 80.1 m of ground, which gives 45.0 deg -- ten degrees
    shallower than the commanded 55 deg pitch.
    """
    if ground_distance_covered_m <= 0.0:
        raise ValueError("ground_distance_covered_m must be positive")
    return math.degrees(math.atan2(altitude_lost_m, ground_distance_covered_m))


def trigger_distance_m(profile: DiveProfile) -> float:
    """Ground distance from the target at which to start the dive.

    Two parts:

        d_bore = decode_altitude / tan(pitch)
            How far ahead the target must still be when the aircraft
            reaches decode altitude, so that it sits on the camera
            boresight rather than underneath the aircraft.

        d_dive = (entry_altitude - decode_altitude) / tan(path_angle)
            Ground distance the aircraft actually covers while descending
            from entry altitude to decode altitude.

        trigger = d_dive + d_bore

    Dropping `d_bore` is the classic error: it puts the target directly
    below the aircraft at the exact moment it needs to be readable.
    """
    d_bore = profile.decode_altitude_m / math.tan(math.radians(profile.pitch_deg))
    d_dive = ((profile.entry_altitude_m - profile.decode_altitude_m)
              / math.tan(math.radians(profile.path_angle_deg)))
    return d_dive + d_bore


def line_of_sight_deg(altitude_m: float, ground_distance_m: float) -> float:
    """Angle below the horizon from the aircraft to the target."""
    if ground_distance_m <= 0.0:
        return 90.0
    return math.degrees(math.atan2(altitude_m, ground_distance_m))


def ground_footprint_m(camera: CameraGeometry, pitch_deg: float,
                       altitude_m: float) -> tuple[float, float]:
    """(nearest, farthest) ground distance visible ahead of the aircraft.

    The camera sees a band from `pitch - vfov/2` to `pitch + vfov/2` below
    the horizon. Anything closer than the near edge is under the aircraft
    and out of frame -- which is exactly how a well-flown dive can still
    photograph nothing but empty ground.
    """
    top = pitch_deg - camera.vfov_deg / 2.0
    bottom = pitch_deg + camera.vfov_deg / 2.0
    if top <= 0.0:
        raise ValueError("camera sees above the horizon at this pitch")
    far = altitude_m / math.tan(math.radians(top))
    near = altitude_m / math.tan(math.radians(min(bottom, 89.999)))
    return near, far


def visible_altitude_band(camera: CameraGeometry, profile: DiveProfile,
                          trigger_m: float | None = None,
                          floor_altitude_m: float = 0.0,
                          steps: int = 200) -> tuple[float, float]:
    """Altitude range over which the target stays inside the vertical FOV.

    Returns (highest, lowest) altitude at which the target is in frame.
    A correct trigger distance keeps the target visible continuously from
    dive entry down to the pull-up floor; the target enters near the top of
    the frame and drifts to the centre as the aircraft closes.
    """
    trigger = trigger_distance_m(profile) if trigger_m is None else trigger_m
    top = profile.pitch_deg - camera.vfov_deg / 2.0
    bottom = profile.pitch_deg + camera.vfov_deg / 2.0
    tan_path = math.tan(math.radians(profile.path_angle_deg))

    highest = lowest = None
    lo = max(floor_altitude_m, 0.0)
    for i in range(steps + 1):
        alt = profile.entry_altitude_m - (profile.entry_altitude_m - lo) * i / steps
        remaining = trigger - (profile.entry_altitude_m - alt) / tan_path
        if remaining <= 0.0:
            break
        los = line_of_sight_deg(alt, remaining)
        if top <= los <= bottom:
            if highest is None:
                highest = alt
            lowest = alt
    if highest is None:
        return (0.0, 0.0)
    return (highest, lowest)


def detection_window(profile: DiveProfile, exit_altitude_m: float,
                     descent_rate_m_s: float, fps: float) -> tuple[float, int]:
    """(seconds, frames) available between decode altitude and dive exit.

    Use this to decide whether to pull up on the first detection or keep
    descending for a few more frames. One usable frame may satisfy a rule
    book, but one frame is also zero margin: lose it to compression or a
    timing hiccup and the mission scores nothing.

    Pass the MEASURED first-detection altitude as `decode_altitude_m` and
    use the WORST observation, not the best.

    Measure `fps` as well, do not assume it. In the project this came from,
    the number written down was 30 while the camera chain actually delivered
    22.3 -- every frame count derived from it was a third too high, and
    nothing in the code could have caught that.
    """
    if exit_altitude_m >= profile.decode_altitude_m:
        return (0.0, 0)
    if descent_rate_m_s <= 0.0:
        raise ValueError("descent_rate_m_s must be positive")
    seconds = (profile.decode_altitude_m - exit_altitude_m) / descent_rate_m_s
    return (seconds, int(seconds * fps))


# ------------------------------------------------------------------ demo ---
if __name__ == "__main__":
    # Values measured on the project this module came from.
    cam = CameraGeometry.from_hfov_and_aspect(hfov_deg=51.28,
                                              width_px=1920, height_px=1080)
    dive = DiveProfile(entry_altitude_m=120.0, decode_altitude_m=40.0,
                       pitch_deg=55.0, path_angle_deg=45.0)

    trigger = trigger_distance_m(dive)
    print("camera   : HFOV %.2f deg, VFOV %.2f deg" % (cam.hfov_deg, cam.vfov_deg))
    print("recovered path angle from log: %.1f deg"
          % effective_path_angle_deg(80.0, 80.1))
    print("trigger distance: %.1f m" % trigger)
    print()
    print("  alt      remaining   LOS     in frame")
    for alt in (120, 100, 80, 60, 50, 40, 35, 30):
        remaining = trigger - (dive.entry_altitude_m - alt) / math.tan(
            math.radians(dive.path_angle_deg))
        los = line_of_sight_deg(alt, remaining)
        top = dive.pitch_deg - cam.vfov_deg / 2
        bottom = dive.pitch_deg + cam.vfov_deg / 2
        print("  %5.1f    %7.1f   %5.1f   %s"
              % (alt, remaining, los, "yes" if top <= los <= bottom else "NO"))
    print()
    hi, lo = visible_altitude_band(cam, dive, floor_altitude_m=30.0)
    print("target visible from %.1f m down to %.1f m" % (hi, lo))
    # 22.3 fps: counted off the frame reports of five recorded flights, not
    # taken from a datasheet.
    secs, frames = detection_window(dive, exit_altitude_m=35.0,
                                    descent_rate_m_s=32.0, fps=22.3)
    print("detection window to 35 m exit: %.2f s -> %d frames" % (secs, frames))
