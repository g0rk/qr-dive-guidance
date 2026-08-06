# tests/test_target_coordinate.py - does the target in config REALLY point at
# the qr_pad in the gz world?
#
#     python3 tests/test_target_coordinate.py
#
# ⚠️ THIS TEST WAS WRITTEN AFTER A REAL BUG (2026-08-05).
#
#    config.TARGET_LATITUDE_DEG / TARGET_LONGITUDE_DEG were two hand-written
#    numbers, and they pointed 49.9 m SOUTHWEST of the pad. The QR pad is
#    2 m x 2 m, so the error was 25 TIMES its edge length: the aircraft dived
#    at empty grass.
#
#    THE CAUSE - TWO DIFFERENT "HOME" ORIGINS:
#      1) PX4's documented default          47.397742 / 8.545594
#      2) the gz world's own <spherical_coordinates> tag
#         47.397971 / 8.546164     <- this is the one that applies under gz
#    They are 49.9 m apart. The target had been derived by adding 500 m to (1).
#
#    This class of bug is SILENT: the code runs, the aircraft flies, it dives
#    - just at the wrong place. No unit test catches it, because both numbers
#    are perfectly valid coordinates. So this test RE-DERIVES the numbers FROM
#    THE WORLD FILE and compares them against config. If the world moves, the
#    test fails.

from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import logging
logging.disable(logging.CRITICAL)

import config
from utils.geo_utils import offset_lat_lon, distance_m, bearing_deg

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(ROOT, "sim", "worlds", "qr_target.sdf")

# How much deviation is allowed? The QR pad is 2 m x 2 m. For the aircraft to
# put the QR inside the target area, the aim point has to be on the pad; 1 m
# is half the pad - very generous, but it catches the "50 m wrong" class of
# error with certainty.
TOLERANCE_M = 1.0

PASS = []


def check(label, cond, detail=""):
    # ⚠️ The `assert` was added later: this helper used to NOT fail pytest, so
    #    the suite stayed green while individual checks printed FAIL.
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "PASS" if cond else "FAIL", detail))
    assert cond, "%s  %s" % (label, detail)


def world_origin():
    """The origin from the gz world's <spherical_coordinates> tag."""
    sc = ET.parse(WORLD).getroot().find("world/spherical_coordinates")
    assert sc is not None, "no <spherical_coordinates> in the world"
    return (float(sc.find("latitude_deg").text),
            float(sc.find("longitude_deg").text))


def qr_pad_position():
    """From the qr_pad include's <pose>: (x=east, y=north) in metres."""
    for inc in ET.parse(WORLD).getroot().iter("include"):
        name = inc.find("name")
        if name is not None and name.text == "qr_pad":
            p = inc.find("pose").text.split()
            return float(p[0]), float(p[1])
    raise AssertionError("no qr_pad include in the world")


def test_world_file_is_readable():
    print("\n1) THE WORLD FILE CAN BE READ")
    lat, lon = world_origin()
    check("world origin found", True, "%.9f / %.9f" % (lat, lon))
    east, north = qr_pad_position()
    check("qr_pad pose found", True, "east=%.1f m north=%.1f m" % (east, north))

    # ⚠️ This is the exact source of the bug. The world's origin is NOT the
    #    same as PX4's documented default; that is why this test exists.
    px4_lat, px4_lon = 47.397742, 8.545594
    gap = distance_m(lat, lon, px4_lat, px4_lon)
    check("world origin != PX4 default (the source of the bug)",
          gap > 10.0, "%.1f m apart" % gap)


def test_config_target_is_on_the_pad():
    print("\n2) IS THE CONFIG TARGET ACTUALLY ON THE PAD")
    o_lat, o_lon = world_origin()
    east, north = qr_pad_position()

    exp_lat, exp_lon = offset_lat_lon(o_lat, o_lon, north_m=north, east_m=east)
    deviation = distance_m(exp_lat, exp_lon,
                           config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG)

    check("target derived from the world", True, "%.7f / %.7f" % (exp_lat, exp_lon))
    check("target held in config", True, "%.7f / %.7f" % (
        config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG))
    check("deviation <= %.1f m" % TOLERANCE_M, deviation <= TOLERANCE_M,
          "deviation = %.2f m" % deviation)
    assert deviation <= TOLERANCE_M, (
        "the config target is %.1f m off the pad. World origin %.9f/%.9f, "
        "qr_pad at (%.1f east, %.1f north) -> the target should be %.7f/%.7f."
        % (deviation, o_lat, o_lon, east, north, exp_lat, exp_lon))


def test_distance_and_bearing():
    print("\n3) DISTANCE AND BEARING FROM THE ORIGIN TO THE TARGET")
    o_lat, o_lon = world_origin()
    east, north = qr_pad_position()

    d = distance_m(o_lat, o_lon,
                   config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG)
    b = bearing_deg(o_lat, o_lon,
                    config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG)

    exp_d = (east ** 2 + north ** 2) ** 0.5
    check("distance matches the pose", abs(d - exp_d) <= TOLERANCE_M,
          "%.1f m (expected %.1f)" % (d, exp_d))
    # qr_pad at (500, 0) -> due east -> 90 degrees
    if north == 0.0 and east > 0:
        check("bearing is due east (90 degrees)", abs(b - 90.0) <= 0.5,
              "%.2f degrees" % b)


def test_approach_chain_is_consistent():
    print("\n4) IS THE TARGET CONSISTENT WITH THE APPROACH GEOMETRY")
    # The ghost waypoint is placed APPROACH_GHOST_DISTANCE_M behind the
    # target. If the target is too close to the launch point, the aircraft
    # overshoots before it can finish the turn. That was the FIRST symptom by
    # which the old bug was diagnosed.
    o_lat, o_lon = world_origin()
    d = distance_m(o_lat, o_lon,
                   config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG)
    check("target is far enough out for the ghost waypoint",
          d > config.APPROACH_GHOST_DISTANCE_M * 0.5,
          "target at %.0f m, ghost distance %.0f m" % (
              d, config.APPROACH_GHOST_DISTANCE_M))


if __name__ == "__main__":
    test_world_file_is_readable()
    test_config_target_is_on_the_pad()
    test_distance_and_bearing()
    test_approach_chain_is_consistent()
    print("\n%d/%d checks passed" % (sum(PASS), len(PASS)))
    sys.exit(0 if all(PASS) else 1)
