# tests/test_gps_guidance.py - GPS-based lateral guidance during the dive
#
#     python3 tests/test_gps_guidance.py
#
# ⚠️ THIS TEST WAS WRITTEN AFTER A REAL BUG (2026-08-05).
#    The dive held a FIXED attitude (roll=0) and MISSED the target by 42.5 m.
#    The QR pad is 2x2 m, so the miss was 21 times its edge length -> the pad
#    never entered the frame -> 0 QR detections in 4879 frames. The visual
#    centering could not engage either, because it has to SEE the QR first
#    (chicken and egg).
#
#    A SIGN ERROR IS FATAL IN THIS JOB: a correction that banks the wrong way
#    WIDENS the offset instead of closing it, and you only find out by flying.
#    So the direction is locked down here, before flight.

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import logging
logging.disable(logging.CRITICAL)

import config
from states.dive_state import DiveState
from utils.geo_utils import offset_lat_lon, distance_m

PASS = []


def check(label, cond, detail=""):
    # ⚠️ The `assert` was added later: this helper used to NOT fail pytest, so
    #    the suite stayed green while individual checks printed FAIL.
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "PASS" if cond else "FAIL", detail))
    assert cond, "%s  %s" % (label, detail)


class FakeTelemetry:
    """The smallest object that can stand in for telemetry."""
    def __init__(self, lat, lon, heading):
        self.latitude_deg = lat
        self.longitude_deg = lon
        self.heading_deg = heading
        self.rel_alt_m = 100.0


def position(north_m, east_m):
    """A position offset from the target by the given amount."""
    return offset_lat_lon(config.TARGET_LATITUDE_DEG,
                          config.TARGET_LONGITUDE_DEG,
                          north_m, east_m)


def test_sign_convention():
    print("\n1) SIGN CONVENTION - does the correction bank the RIGHT way")
    d = DiveState()

    # Aircraft 200 m WEST of the target, nose pointing due EAST (90 degrees).
    # The target is straight ahead -> bearing error ~0 -> roll should be ~0.
    lat, lon = position(0, -200)
    roll, error, distance = d._gps_roll(FakeTelemetry(lat, lon, 90.0))
    check("target dead ahead -> roll ~ 0", abs(roll) < 1.0,
          "roll=%+.2f error=%+.2f distance=%.0f" % (roll, error, distance))

    # Same place, but nose pointing NORTH (0 degrees). The target is to the
    # EAST, i.e. to the RIGHT -> bank right -> POSITIVE roll.
    roll, error, distance = d._gps_roll(FakeTelemetry(lat, lon, 0.0))
    check("target to the RIGHT -> POSITIVE roll", roll > 0,
          "roll=%+.2f error=%+.2f" % (roll, error))

    # Nose pointing SOUTH (180 degrees). The target is to the LEFT -> negative.
    roll, error, distance = d._gps_roll(FakeTelemetry(lat, lon, 180.0))
    check("target to the LEFT -> negative roll", roll < 0,
          "roll=%+.2f error=%+.2f" % (roll, error))


def test_clamping():
    print("\n2) CLAMPING - the dive is the riskiest phase, do not forget it")
    d = DiveState()
    lat, lon = position(0, -200)
    # Nose pointing exactly AWAY from the target (270 = west) -> the error is
    # close to 180 degrees.
    roll, error, distance = d._gps_roll(FakeTelemetry(lat, lon, 270.0))
    check("roll is clamped on an extreme error",
          abs(roll) <= config.KAMIKAZE_GPS_MAX_ROLL_DEG + 1e-6,
          "roll=%+.1f limit=%.1f (raw error %+.1f)" % (
              roll, config.KAMIKAZE_GPS_MAX_ROLL_DEG, error))


def test_correction_freezes_when_close():
    print("\n3) THE CORRECTION FREEZES CLOSE TO THE TARGET")
    # ⚠️ Within a few metres, a tiny position error can swing the bearing by
    #    180 degrees; correcting on it makes the aircraft bank hard at the
    #    last moment.
    d = DiveState()
    lat, lon = position(0, -5)     # 5 m from the target
    roll, error, distance = d._gps_roll(FakeTelemetry(lat, lon, 90.0))
    check("no roll produced when very close", roll is None,
          "distance=%.1f m, threshold=%.1f m" % (
              distance, config.KAMIKAZE_GPS_MIN_DISTANCE_M))

    # Just beyond the threshold it must work again (false-positive guard).
    lat, lon = position(0, -(config.KAMIKAZE_GPS_MIN_DISTANCE_M + 20))
    roll, error, distance = d._gps_roll(FakeTelemetry(lat, lon, 0.0))
    check("works again beyond the threshold", roll is not None,
          "distance=%.1f m roll=%+.2f" % (distance, roll if roll else 0))


def test_can_be_disabled():
    print("\n4) THE FEATURE CAN BE TURNED OFF (falls back to a blind dive)")
    d = DiveState()
    lat, lon = position(0, -200)
    previous = config.KAMIKAZE_GPS_GUIDANCE
    try:
        config.KAMIKAZE_GPS_GUIDANCE = False
        roll, _, _ = d._gps_roll(FakeTelemetry(lat, lon, 0.0))
        check("no roll produced while disabled", roll is None, "roll=%s" % roll)
    finally:
        config.KAMIKAZE_GPS_GUIDANCE = previous


def test_missing_telemetry():
    print("\n5) 0/0 IS NOT SILENTLY TREATED AS A POSITION")
    # ⚠️ lat=lon=0 does not mean "Gulf of Guinea", it means "no data".
    #    Computing a bearing from it would aim the aircraft at Africa.
    d = DiveState()
    roll, _, _ = d._gps_roll(FakeTelemetry(0.0, 0.0, 0.0))
    check("no roll produced on empty telemetry", roll is None, "roll=%s" % roll)


def test_continue_threshold_is_consistent():
    print("\n7) THE POST-QR CONTINUE THRESHOLD")
    # ⚠️ The continue threshold must be ABOVE the floor. If it were below, the
    #    floor would fire first and the continue setting would DO NOTHING - and
    #    silently: the code runs, raises nothing, and only ever collects one
    #    frame.
    continue_alt = config.KAMIKAZE_QR_CONTINUE_ALTITUDE_M
    floor = config.DIVE_PULL_UP_ALTITUDE_M
    check("continue threshold > pull-up floor", continue_alt > floor,
          "continue=%.1f > floor=%.1f" % (continue_alt, floor))

    # The continue threshold must sit BELOW the first-detection altitude
    # measured in flight, otherwise there is nothing to continue for.
    # ⚠️ This uses QR_FIRST_DETECT_ALTITUDE_M (41.7, the WORST in-flight
    #    observation), NOT QR_DECODE_ALTITUDE_M (40, the fixed-camera test).
    #    The first version of this check used 40 and the test failed - which
    #    was the correct outcome: treating two different measurements as the
    #    same thing is precisely this project's recurring mistake.
    check("continue threshold < in-flight first-detection altitude",
          continue_alt < config.QR_FIRST_DETECT_ALTITUDE_M,
          "continue=%.1f < first_detect=%.1f" % (
              continue_alt, config.QR_FIRST_DETECT_ALTITUDE_M))

    # Frames gained: (first_detect - continue_threshold) / descent_rate
    DESCENT_M_S = 32.0      # measured
    FPS = 30.0
    window_s = (config.QR_FIRST_DETECT_ALTITUDE_M - continue_alt) / DESCENT_M_S
    frames = window_s * FPS
    # ~6 frames on the WORST observation; ~8 on the average one (43.3 m).
    # The rulebook needs ONE valid frame, so 6 is six times the margin.
    check("at least 5 valid frames in the worst case", frames >= 5.0,
          "%.2f s -> ~%.0f frames (with the worst observation, %.1f m)" % (
              window_s, frames, config.QR_FIRST_DETECT_ALTITUDE_M))

    # ⚠️ The rulebook (p.20) validates within +-1 second of the dive end. The
    #    gap between the first detection and the dive end has to FIT in it.
    check("first detection is inside the +-1 s window around the dive end",
          window_s <= 1.0, "%.2f s apart (limit 1.00)" % window_s)

    # Estimate the lowest point from the measured altitude loss.
    WORST_LOSS = 16.46
    lowest = continue_alt - WORST_LOSS
    check("estimated lowest point > 15 m", lowest > 15.0,
          "%.1f - %.1f = %.1f m" % (continue_alt, WORST_LOSS, lowest))


def test_pull_up_floor_was_raised():
    print("\n6) THE PULL-UP FLOOR WAS RAISED BY MEASUREMENT")
    # The WORST measured altitude loss was 16.46 m (over 5 runs).
    WORST_LOSS = 16.46
    margin = config.DIVE_PULL_UP_ALTITUDE_M - WORST_LOSS
    check("the floor exceeds the worst measured loss",
          config.DIVE_PULL_UP_ALTITUDE_M > WORST_LOSS,
          "floor=%.1f m, loss=%.1f m -> margin=%.1f m" % (
              config.DIVE_PULL_UP_ALTITUDE_M, WORST_LOSS, margin))
    check("margin of at least 10 m", margin >= 10.0, "margin=%.1f m" % margin)
    # The floor has to stay BELOW the decode threshold, or the QR is never seen.
    check("floor is below the 40 m decode threshold",
          config.DIVE_PULL_UP_ALTITUDE_M < 40.0,
          "floor=%.1f m < 40 m" % config.DIVE_PULL_UP_ALTITUDE_M)


if __name__ == "__main__":
    test_sign_convention()
    test_clamping()
    test_correction_freezes_when_close()
    test_can_be_disabled()
    test_missing_telemetry()
    test_continue_threshold_is_consistent()
    test_pull_up_floor_was_raised()
    print("\n%d/%d checks passed" % (sum(PASS), len(PASS)))
    sys.exit(0 if all(PASS) else 1)
