# tests/test_dive_geometry.py - the standalone dive geometry module
#
#     python3 tests/test_dive_geometry.py
#
# dive_geometry.py exists to be PORTABLE: other projects should be able to
# copy it. So there are two things to test here:
#   1. Is the arithmetic right (checked against measured values)
#   2. Is the module GENUINELY standalone (does it run without touching any
#      project module)
#
# The second one breaks easily: someone later decides to "just read that
# constant from config", the module quietly couples itself to the project and
# loses its portability. This test catches that.

from __future__ import annotations

import math
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import dive_geometry as dg

PASS = []


def check(label, cond, detail=""):
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "PASS" if cond else "FAIL", detail))
    assert cond, "%s  %s" % (label, detail)


def test_standalone():
    print("\n1) IS THE MODULE GENUINELY STANDALONE")
    # ⚠️ Import it in a SEPARATE Python process with only the module's own
    #    directory on sys.path. Even though config.py, states/ and processes/
    #    are reachable from the project root, the module must NOT touch them.
    # ⚠️ ADDING IT TO sys.modules IS REQUIRED. With `from __future__ import
    #    annotations`, dataclass resolves string annotations by looking at
    #    sys.modules[cls.__module__].__dict__. Without the entry it fails with
    #    "NoneType object has no attribute __dict__".
    code = (
        "import sys, importlib.util, pathlib;"
        "p = pathlib.Path(%r) / 'dive_geometry.py';"
        "spec = importlib.util.spec_from_file_location('dg', p);"
        "m = importlib.util.module_from_spec(spec);"
        "sys.modules['dg'] = m;"
        "spec.loader.exec_module(m);"
        "forbidden = [n for n in sys.modules "
        "         if n.split('.')[0] in ('config','states','processes',"
        "                                'mavsdk','cv2','numpy','rclpy',"
        "                                'pyzbar','vehicle','telemetry')];"
        "print('FORBIDDEN:', forbidden);"
        "print('TRIGGER:', m.trigger_distance_m("
        "  m.DiveProfile(120.0, 40.0, 55.0, 45.0)))"
    ) % ROOT
    r = subprocess.run([sys.executable, "-c", code], capture_output=True,
                       text=True, cwd=os.path.dirname(ROOT))
    output = (r.stdout or "") + (r.stderr or "")
    check("imported cleanly in a separate process", r.returncode == 0,
          output.strip().splitlines()[-1] if output.strip() else "")
    check("pulled in NO project or heavy dependency", "FORBIDDEN: []" in output,
          [l for l in output.splitlines() if l.startswith("FORBIDDEN")][:1])


def test_trigger_distance():
    print("\n2) TRIGGER DISTANCE - against measured values")
    p = dg.DiveProfile(entry_altitude_m=120.0, decode_altitude_m=40.0,
                       pitch_deg=55.0, path_angle_deg=45.0)
    t = dg.trigger_distance_m(p)
    # d_bore = 40/tan(55) = 28.00 ; d_dive = 80/tan(45) = 80.00
    check("trigger = d_dive + d_bore", abs(t - 108.0) < 0.05, "%.3f m" % t)

    # ⚠️ THE OLD, WRONG DERIVATION: entry/tan(pitch) = 120/tan(55) = 84 m.
    #    Measured: with that trigger the aircraft arrived at 40 m altitude
    #    with 3.9 m to go and the target sat BELOW the frame. This check locks
    #    the two values apart so neither can drift into the other.
    old = 120.0 / math.tan(math.radians(55.0))
    check("clearly different from the old (wrong) derivation", abs(t - old) > 20.0,
          "new %.1f vs old %.1f m" % (t, old))


def test_path_angle_recovered_from_log():
    print("\n3) PATH ANGLE RECOVERED FROM A FLIGHT LOG")
    # From a real run: 80.0 m of descent over 80.1 m of ground.
    g = dg.effective_path_angle_deg(80.0, 80.1)
    check("the measured log gives 45 degrees", abs(g - 45.0) < 0.2,
          "%.2f degrees" % g)
    # ⚠️ The commanded pitch was 55. This locks in that the two are DIFFERENT:
    #    not seeing that difference was the root cause of the 42.5 m miss.
    check("clearly different from the commanded pitch (55)", abs(g - 55.0) > 5.0,
          "difference %.1f degrees" % abs(g - 55.0))


def test_target_stays_in_frame():
    print("\n4) TARGET STAYS IN FRAME THROUGHOUT THE DIVE")
    cam = dg.CameraGeometry.from_hfov_and_aspect(51.28, 1920, 1080)
    check("VFOV derived from the aspect ratio", abs(cam.vfov_deg - 30.22) < 0.05,
          "%.2f degrees" % cam.vfov_deg)

    p = dg.DiveProfile(120.0, 40.0, 55.0, 45.0)
    hi, lo = dg.visible_altitude_band(cam, p, floor_altitude_m=30.0)
    check("visible without a gap from entry (120) to the floor (30)",
          hi >= 119.0 and lo <= 30.5, "%.1f m -> %.1f m" % (hi, lo))

    # At decode altitude it must sit exactly on the boresight.
    remaining = dg.trigger_distance_m(p) - (120.0 - 40.0) / math.tan(math.radians(45.0))
    los = dg.line_of_sight_deg(40.0, remaining)
    check("at decode altitude, LOS = pitch (boresight)",
          abs(los - 55.0) < 0.5, "LOS %.2f vs pitch 55.0" % los)


def test_ground_footprint():
    print("\n5) GROUND FOOTPRINT - the 'target ended up underneath us' bug")
    cam = dg.CameraGeometry.from_hfov_and_aspect(51.28, 1920, 1080)
    near, far = dg.ground_footprint_m(cam, pitch_deg=51.7, altitude_m=40.0)
    check("at 40 m the camera sees from 17 to 54 m",
          16.0 < near < 19.0 and 52.0 < far < 56.0,
          "%.1f - %.1f m" % (near, far))
    # With the old trigger the target was 3.9 m ahead -> outside the near edge
    check("a target 3.9 m ahead is OUTSIDE the frame (the measured case)",
          3.9 < near, "target at 3.9 m, near edge %.1f m" % near)


def test_validation():
    print("\n6) ARE INVALID INPUTS REJECTED (false-positive guard)")
    for name, kw in (
        ("pitch 0",        dict(entry_altitude_m=120, decode_altitude_m=40,
                                pitch_deg=0.0, path_angle_deg=45)),
        ("decode > entry", dict(entry_altitude_m=40, decode_altitude_m=120,
                                pitch_deg=55, path_angle_deg=45)),
        ("path 90",        dict(entry_altitude_m=120, decode_altitude_m=40,
                                pitch_deg=55, path_angle_deg=90.0)),
    ):
        try:
            dg.DiveProfile(**kw)
            check("%s rejected" % name, False, "it was accepted!")
        except ValueError:
            check("%s rejected" % name, True)


if __name__ == "__main__":
    test_standalone()
    test_trigger_distance()
    test_path_angle_recovered_from_log()
    test_target_stays_in_frame()
    test_ground_footprint()
    test_validation()
    print("\n%d/%d checks passed" % (sum(PASS), len(PASS)))
    sys.exit(0 if all(PASS) else 1)
