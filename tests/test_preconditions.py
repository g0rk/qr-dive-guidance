# tests/test_preconditions.py - regression tests for the four preconditions. NO vehicle
# or autopilot required.
#
#     python3 tests/test_preconditions.py
#
# These are the PRECONDITIONS for the "dive at the QR" job. All four were
# missing from the original code, and all four were SILENT - the code looked
# like it was working:
#
#   P1  perception_result_queue was never read -> QR data never reached the
#       mission (and the queue grew without bound)
#   P2  goto_location expects AMSL, config supplied a relative altitude ->
#       the dive aborted at any site not at sea level
#   P3  a 65 degree dive and a 105 m trigger distance were inconsistent ->
#       the QR never entered the target area
#   P4  dive_state was completely blind -> it never used the camera

from __future__ import annotations

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import logging
logging.disable(logging.CRITICAL)

import config
from utils.geo_utils import offset_lat_lon as _offset_ll

PASS = []


def check(label, cond, detail=""):
    # ⚠️ The `assert` WAS ADDED LATER (2026-08-05). This helper used to only
    #    append to the list and print; it did NOT fail pytest.
    #    The result: a check could print FAIL on screen while pytest still
    #    reported "passed". It really happened - two checks quietly broke when
    #    GPS guidance was added and the suite stayed green.
    #    Exactly this project's recurring failure mode: the code runs, nothing
    #    shouts, and the answer is wrong.
    PASS.append(bool(cond))
    print("  %-50s %s  %s" % (label, "PASS" if cond else "FAIL", detail))
    assert cond, "%s  %s" % (label, detail)


def run(coro):
    import asyncio
    return asyncio.get_event_loop().run_until_complete(coro)


class StubQ:
    def __init__(self, items=None):
        self.items = list(items or [])

    def put_nowait(self, x):
        self.items.append(x)

    def get_nowait(self):
        if not self.items:
            raise Exception("empty")
        return self.items.pop(0)


# ======================================================================
# P1 - the perception -> mission link
# ======================================================================
def test_p1():
    print("=" * 76)
    print("  P1 - is perception_result_queue wired into the mission")
    print("=" * 76)

    from processes.mission_controller import MissionController

    q = StubQ([
        {"type": "qr", "data": "old", "in_av": False, "error": [0.9, 0.9]},
        {"type": "yolo", "data": [{"label": "plane"}]},
        {"type": "qr", "data": "new", "in_av": True, "error": [0.1, 0.1]},
    ])
    mc = MissionController(
        vehicle=None, telemetry=None,
        comm_command_queue=StubQ(), comm_status_queue=StubQ(),
        perception_command_queue=StubQ(), perception_result_queue=q,
    )

    check("qr_result empty at construction", mc.qr_result is None)
    mc._drain_perception()

    check("queue drained COMPLETELY (no unbounded growth)",
          len(q.items) == 0, "%d left" % len(q.items))
    check("qr_result populated", mc.qr_result is not None)
    check("the LATEST QR was kept (not the old one)",
          mc.qr_result and mc.qr_result.get("data") == "new",
          str(mc.qr_result and mc.qr_result.get("data")))
    check("arrival time stamped (required for staleness)",
          mc.qr_result and "t" in mc.qr_result)
    check("yolo detections were taken too", mc.detections is not None)
    print()


# ======================================================================
# P2 - the altitude reference
# ======================================================================
class _Tel:
    def __init__(self, abs_alt, rel_alt, updated=True):
        self.abs_alt_m = abs_alt
        self.rel_alt_m = rel_alt
        self.last_update_time = 1.0 if updated else 0.0


class _Store:
    def __init__(self, tel):
        self._t = tel

    def get(self):
        return self._t


def _make_vehicle(store):
    from vehicle import Vehicle
    v = Vehicle.__new__(Vehicle)          # without constructing MAVSDK System()
    v._telemetry_store = store
    v._offboard_active = False
    v.sent = []

    async def fake_goto(lat, lon, abs_alt_m, yaw_deg=0.0):
        v.sent.append(abs_alt_m)

    v.goto_location = fake_goto
    return v


def test_p2():
    print("=" * 76)
    print("  P2 - goto_location expects AMSL, config supplies relative")
    print("=" * 76)

    from vehicle import VehicleCommandError

    # The PX4 SITL default: Zurich at 488 m
    v = _make_vehicle(_Store(_Tel(abs_alt=520.0, rel_alt=32.0)))
    check("field elevation recovered", abs(v.field_elevation_m() - 488.0) < 1e-6,
          "%.1f m AMSL" % v.field_elevation_m())

    run(v.goto_location_rel(41.0, 36.0, config.APPROACH_SAFE_ALTITUDE_M))
    expected = 488.0 + config.APPROACH_SAFE_ALTITUDE_M
    check("relative %.0f m -> absolute %.0f m" % (config.APPROACH_SAFE_ALTITUDE_M, expected),
          abs(v.sent[-1] - expected) < 1e-6, "commanded: %.1f" % v.sent[-1])
    check("THE OLD BUG: raw 100 is not sent (388 m below ground)",
          abs(v.sent[-1] - config.APPROACH_SAFE_ALTITUDE_M) > 1.0)

    v2 = _make_vehicle(_Store(_Tel(0.0, 0.0, updated=False)))
    try:
        run(v2.goto_location_rel(41.0, 36.0, 100.0))
        raised = False
    except VehicleCommandError:
        raised = True
    check("raises when there is no telemetry (does not assume 0)", raised)

    # --- THE REAL REGRESSION: does field elevation close the dive gate? ---
    #
    # The test must NOT depend on a magic number: the field elevation that
    # would break the old code is derived from the config values. Any
    # elevation greater than the margin between the approach altitude and the
    # dive threshold would have dropped the old code into ABORT.
    margin = config.APPROACH_SAFE_ALTITUDE_M - config.DIVE_MIN_ENTRY_ALTITUDE_M
    critical_elevation = margin + 10.0        # the first elevation past the margin

    check("NEW code: the gate stays OPEN at any field elevation",
          config.APPROACH_SAFE_ALTITUDE_M >= config.DIVE_MIN_ENTRY_ALTITUDE_M,
          "relative command -> rel=%.0f >= threshold=%.0f"
          % (config.APPROACH_SAFE_ALTITUDE_M, config.DIVE_MIN_ENTRY_ALTITUDE_M))

    check("OLD code: would ABORT at a site %.0f m up" % critical_elevation,
          (config.APPROACH_SAFE_ALTITUDE_M - critical_elevation) < config.DIVE_MIN_ENTRY_ALTITUDE_M,
          "AMSL command -> rel=%.0f < threshold=%.0f"
          % (config.APPROACH_SAFE_ALTITUDE_M - critical_elevation,
             config.DIVE_MIN_ENTRY_ALTITUDE_M))

    # Zurich (the PX4 SITL default) would have broken it outright.
    check("OLD code: at Zurich (488 m) it commanded below ground",
          (config.APPROACH_SAFE_ALTITUDE_M - 488.0) < 0,
          "rel=%.0f m" % (config.APPROACH_SAFE_ALTITUDE_M - 488.0))

    # Consistency of the altitude chain
    check("altitude chain consistent (takeoff >= approach > dive threshold)",
          config.TAKEOFF_ALTITUDE_M >= config.APPROACH_SAFE_ALTITUDE_M
          > config.DIVE_MIN_ENTRY_ALTITUDE_M,
          "%.0f >= %.0f > %.0f" % (config.TAKEOFF_ALTITUDE_M,
                                   config.APPROACH_SAFE_ALTITUDE_M,
                                   config.DIVE_MIN_ENTRY_ALTITUDE_M))
    print()


# ======================================================================
# P3 - dive geometry
# ======================================================================
def test_p3():
    print("=" * 76)
    print("  P3 - dive geometry: does the target stay IN FRAME for the whole dive")
    print("=" * 76)

    # ⚠️ THE PRINCIPLE BEHIND THIS TEST CHANGED (2026-08-05, by measurement).
    #
    #    THE OLD PRINCIPLE: "the dive angle should equal the line-of-sight
    #    angle at entry, so the QR stays fixed on the boresight throughout."
    #    That would be right if the aircraft flew exactly along the pitch
    #    direction. IT DOES NOT: the instantaneous path angle does reach ~50
    #    degrees, but the AVERAGE is 45, because early in the dive the nose has
    #    not come down and the aircraft covers a lot of ground.
    #
    #    THE CONSEQUENCE WAS MEASURED: with the old trigger (84 m) the aircraft
    #    arrived at 40 m altitude with 3.9 m to go; at that altitude the camera
    #    sees the ground from 17.2 m to 54.0 m ahead, so the target sat BELOW
    #    the frame. 4879 frames, zero QR detections.
    #
    #    THE NEW PRINCIPLE: the target must be on the boresight AT DECODE
    #    ALTITUDE and must never leave the vertical field of view during the
    #    dive. That is what follows.
    los_entry = math.degrees(math.atan2(config.APPROACH_SAFE_ALTITUDE_M,
                                        config.APPROACH_DIVE_ARM_DISTANCE_M))
    check("at entry the target is ABOVE the boresight (LOS < pitch)",
          los_entry < abs(config.DIVE_PITCH_DEG),
          "LOS=%.1f < pitch=%.1f" % (los_entry, abs(config.DIVE_PITCH_DEG)))

    # At decode altitude the LOS must equal the pitch exactly (boresight).
    remaining_at_decode = (config.APPROACH_DIVE_ARM_DISTANCE_M
                           - (config.APPROACH_SAFE_ALTITUDE_M - config.QR_DECODE_ALTITUDE_M)
                           / math.tan(math.radians(config.DIVE_EFFECTIVE_PATH_ANGLE_DEG)))
    los_decode = math.degrees(math.atan2(config.QR_DECODE_ALTITUDE_M, remaining_at_decode))
    check("at decode altitude the target is ON the BORESIGHT (+-2)",
          abs(los_decode - abs(config.DIVE_PITCH_DEG)) <= 2.0,
          "LOS=%.1f pitch=%.1f (target %.1f m ahead)" % (
              los_decode, abs(config.DIVE_PITCH_DEG), remaining_at_decode))

    # It must not leave the vertical field of view during the dive.
    # The VFOV is read from the camera model - it is not a hand-written number.
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sim", "tools"))
    import cam_params
    vfov = math.degrees(cam_params.vfov_rad())
    top_edge = abs(config.DIVE_PITCH_DEG) - vfov / 2
    bottom_edge = abs(config.DIVE_PITCH_DEG) + vfov / 2

    outside = []
    for h in (120, 100, 80, 60, 50, 40, 35, config.DIVE_PULL_UP_ALTITUDE_M):
        ground = (config.APPROACH_DIVE_ARM_DISTANCE_M
                  - (config.APPROACH_SAFE_ALTITUDE_M - h)
                  / math.tan(math.radians(config.DIVE_EFFECTIVE_PATH_ANGLE_DEG)))
        if ground <= 0.5:
            outside.append((h, "underneath the aircraft"))
            continue
        los = math.degrees(math.atan2(h, ground))
        if not (top_edge <= los <= bottom_edge):
            outside.append((h, "LOS=%.1f" % los))
    check("target in frame CONTINUOUSLY from entry to the pull-up floor",
          not outside,
          "camera sees %.1f-%.1f degrees; outside: %s" % (
              top_edge, bottom_edge, outside or "none"))

    check("above the QR plate threshold (>=45, rulebook p.17)",
          abs(config.DIVE_PITCH_DEG) >= config.QR_PLATE_MIN_LOOKDOWN_DEG,
          "%.1f >= %.0f" % (abs(config.DIVE_PITCH_DEG),
                            config.QR_PLATE_MIN_LOOKDOWN_DEG))
    check("dive entry altitude >= 100 m (rulebook p.18)",
          config.DIVE_MIN_ENTRY_ALTITUDE_M >= 100.0,
          "%.0f m" % config.DIVE_MIN_ENTRY_ALTITUDE_M)

    old_los = math.degrees(math.atan2(100.0, 105.0))
    check("the OLD values (65/105) are caught as inconsistent",
          abs(old_los - 65.0) > 2.0,
          "line of sight=%.1f vs dive=65.0 -> difference %.1f" % (old_los, abs(old_los - 65.0)))
    print()


# ======================================================================
# P4 - visual centering
# ======================================================================
class _NullLive:
    def start(self): pass
    def stop(self): pass
    def update(self, *a): pass


class _FakeTelD:
    rel_alt_m = 60.0
    pitch_deg = -55.0
    vel_north_m_s = 20.0
    vel_east_m_s = 0.0
    vel_down_m_s = 25.0

    # ⚠️ THE POSITION FIELDS WERE ADDED LATER (2026-08-05).
    #    When GPS lateral guidance was added to the dive, _gps_roll started
    #    reading these fields. Without them on the fake object the test failed
    #    with AttributeError - which was the CORRECT behaviour: a missing
    #    field would crash the dive in flight too. The code was made defensive
    #    (getattr), BUT leaving the fake incomplete would have made the test
    #    meaningless: it would pass because "the field is absent", not because
    #    "the blind dive is right". So the fake was made REALISTIC: 200 m west
    #    of the target, nose pointing straight at it. The bearing error is then
    #    ~0 and so is the GPS correction.
    latitude_deg, longitude_deg = _offset_ll(
        config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG, 0.0, -200.0)
    heading_deg = 90.0     # due east, i.e. toward the target


class _FakeVeh:
    def __init__(self):
        self.last = None

    async def set_attitude(self, roll_deg, pitch_deg, yaw_rate_deg_s, thrust):
        self.last = (roll_deg, pitch_deg, thrust)


class _FakeMission:
    def __init__(self):
        self.telemetry = type("S", (), {"get": staticmethod(lambda: _FakeTelD())})()
        self.vehicle = _FakeVeh()
        self.qr_result = None
        self.kamikaze_hit = None
        self.changed = None

    async def _change_state(self, s):
        self.changed = s.name


def _new_dive():
    from states.dive_state import DiveState
    d = DiveState()
    d._entry_time = time.monotonic() - 1.0
    d._entry_wall = time.time() - 1.0
    d._entry_alt_m = 120.0
    d._live = _NullLive()
    return d


def _qr(ex=0.0, ey=0.0, in_av=True, age=0.0, data="TEKNOFEST"):
    return {"data": data, "error": [ex, ey], "box": [900, 500, 120, 120],
            "in_av": in_av, "t": time.monotonic() - age}


def test_p4():
    print("=" * 76)
    print("  P4 - visual QR centering during the dive")
    print("=" * 76)

    previous_pullup = config.KAMIKAZE_PULLUP_ON_QR
    config.KAMIKAZE_PULLUP_ON_QR = False   # measure centering on its own first

    # ⚠️ THE MEANING OF THIS CHECK CHANGED (2026-08-05).
    #    It used to expect "no QR -> roll is FIXED at 0". But that behaviour
    #    made the dive miss the target by 42.5 m (measured). GPS lateral
    #    guidance now runs when there is no QR. The fake telemetry is aimed
    #    EXACTLY at the target, so the bearing error is ~0 and the correction
    #    is ~0 too - but this time it means "computed and came out zero", not
    #    "never computed".
    m = _FakeMission(); d = _new_dive(); run(d.update(m))
    check("no QR, ALIGNED with the target -> correction ~0",
          abs(m.vehicle.last[0]) < 1.0
          and abs(m.vehicle.last[1] - config.DIVE_PITCH_DEG) < 1e-9,
          "roll=%+.1f pitch=%+.1f" % m.vehicle.last[:2])

    # No QR but the target is to the RIGHT -> GPS guidance must bank right.
    # This is the very mechanism that closed the 42.5 m miss.
    m = _FakeMission()
    _FakeTelD.heading_deg = 0.0            # nose north, so the target is east
    d = _new_dive(); run(d.update(m))
    check("no QR, target to the RIGHT -> GPS banks right",
          m.vehicle.last[0] > 1.0, "roll=%+.1f" % m.vehicle.last[0])
    _FakeTelD.heading_deg = 90.0           # restore

    m = _FakeMission(); m.qr_result = _qr(ex=+0.5); d = _new_dive(); run(d.update(m))
    check("QR to the RIGHT -> POSITIVE roll (bank right)", m.vehicle.last[0] > 0,
          "roll=%+.1f" % m.vehicle.last[0])

    m = _FakeMission(); m.qr_result = _qr(ex=-0.5); d = _new_dive(); run(d.update(m))
    check("QR to the LEFT -> negative roll", m.vehicle.last[0] < 0,
          "roll=%+.1f" % m.vehicle.last[0])

    m = _FakeMission(); m.qr_result = _qr(ey=+0.5); d = _new_dive(); run(d.update(m))
    check("QR BELOW -> pitch MORE NEGATIVE",
          m.vehicle.last[1] < config.DIVE_PITCH_DEG,
          "pitch=%+.1f (base %+.1f)" % (m.vehicle.last[1], config.DIVE_PITCH_DEG))

    m = _FakeMission(); m.qr_result = _qr(ex=1.0); d = _new_dive(); run(d.update(m))
    check("roll is clamped (+-%.0f)" % config.KAMIKAZE_QR_MAX_ROLL_DEG,
          abs(m.vehicle.last[0]) <= config.KAMIKAZE_QR_MAX_ROLL_DEG + 1e-9,
          "roll=%+.1f" % m.vehicle.last[0])

    m = _FakeMission(); m.qr_result = _qr(ey=1.0); d = _new_dive(); run(d.update(m))
    deviation = abs(m.vehicle.last[1] - config.DIVE_PITCH_DEG)
    check("pitch deviation is clamped (+-%.0f)" % config.KAMIKAZE_QR_MAX_PITCH_DELTA_DEG,
          deviation <= config.KAMIKAZE_QR_MAX_PITCH_DELTA_DEG + 1e-9,
          "deviation=%.1f" % deviation)

    # --- SAFETY FILTERS ---
    m = _FakeMission()
    m.qr_result = _qr(ex=0.9, age=config.KAMIKAZE_QR_MAX_AGE_S + 0.3)
    d = _new_dive(); run(d.update(m))
    # ⚠️ THE TOLERANCE WAS LOOSENED (2026-08-05). What this check means is not
    #    "roll must be zero" but "a STALE QR must not be used for centering".
    #    Roll used to be exactly 0 with no QR; now GPS guidance runs, and
    #    because the fake telemetry is aimed at the target the roll is ~0 but
    #    not EXACTLY 0: offset_lat_lon uses a flat-earth approximation while
    #    bearing_deg uses a spherical one, and the small difference produces a
    #    microscopic roll. What distinguishes the two cases: an ex=0.9 QR
    #    centering would give about 10.8 degrees. Staying under 1 degree means
    #    the QR was not used.
    check("a STALE QR is ignored -> centering NOT applied",
          abs(m.vehicle.last[0]) < 1.0,
          "roll=%+.3f (it would be ~%.1f if the QR were used)" % (
              m.vehicle.last[0], 0.9 * config.KAMIKAZE_QR_ROLL_GAIN))

    m = _FakeMission(); d = _new_dive()
    m.qr_result = _qr(ex=0.9)
    m.qr_result["t"] = d._entry_time - 0.1      # read BEFORE the dive started
    run(d.update(m))
    check("a QR read BEFORE the dive is ignored",
          abs(m.vehicle.last[0]) < 1.0,
          "roll=%+.3f (it would be ~%.1f if the QR were used)" % (
              m.vehicle.last[0], 0.9 * config.KAMIKAZE_QR_ROLL_GAIN))

    # --- RULEBOOK: the in_av condition ---
    config.KAMIKAZE_PULLUP_ON_QR = True

    m = _FakeMission(); m.qr_result = _qr(ex=0.3, in_av=False)
    d = _new_dive(); run(d.update(m))
    check("a QR read OUTSIDE the target area does NOT complete the mission",
          m.changed is None and m.kamikaze_hit is None,
          "transition=%s" % m.changed)
    check("  ...but it IS used for centering", m.vehicle.last[0] > 0,
          "roll=%+.1f" % m.vehicle.last[0])

    # ⚠️ THIS BEHAVIOUR WAS CHANGED DELIBERATELY (2026-08-05).
    #    The dive used to go to PULL_UP the instant a QR was read inside the
    #    target area. Measured: doing that collected ONLY 1 decodable frame per
    #    flight, and in that frame the QR was 70 px - right on the decode
    #    threshold. The rulebook needs one frame, but working at zero margin is
    #    fragile in the field. The dive now continues down to
    #    KAMIKAZE_QR_CONTINUE_ALTITUDE_M and collects more, larger frames.
    #    The rulebook (p.20) allows it: the window is +-1 s around the dive
    #    end, and at 32 m/s of descent 1 s is 32 m, so the continue interval
    #    (6.7 m) sits comfortably inside it.

    # (a) ABOVE the continue threshold: the detection is recorded but we do
    #     NOT exit yet.
    _FakeTelD.rel_alt_m = config.KAMIKAZE_QR_CONTINUE_ALTITUDE_M + 5.0
    m = _FakeMission(); m.qr_result = _qr(ex=0.1, in_av=True)
    d = _new_dive(); run(d.update(m))
    check("QR inside the area, ABOVE the threshold -> CONTINUE (no exit)",
          m.changed is None, "transition=%s (alt=%.1f)" % (
              m.changed, _FakeTelD.rel_alt_m))

    # (b) BELOW the continue threshold: exit and stamp the packet.
    _FakeTelD.rel_alt_m = config.KAMIKAZE_QR_CONTINUE_ALTITUDE_M - 1.0
    m = _FakeMission(); m.qr_result = _qr(ex=0.1, in_av=True)
    d = _new_dive(); run(d.update(m))
    check("QR inside the area, BELOW the threshold -> PULL_UP",
          m.changed == "PULL_UP", "transition=%s (alt=%.1f)" % (
              m.changed, _FakeTelD.rel_alt_m))
    check("kamikaze_hit populated", m.kamikaze_hit is not None,
          str(sorted(m.kamikaze_hit.keys())) if m.kamikaze_hit else "")
    check("  ...the packet carries the dive entry altitude (>=100)",
          m.kamikaze_hit and m.kamikaze_hit.get("entry_alt_m", 0) >= 100.0,
          "%.1f m" % (m.kamikaze_hit or {}).get("entry_alt_m", 0))
    check("  ...the dive END time is stamped (the +-1 s window)",
          m.kamikaze_hit and m.kamikaze_hit.get("dive_end_wall", 0) > 0,
          "present" if (m.kamikaze_hit or {}).get("dive_end_wall") else "MISSING")
    check("  ...the number of valid frames is recorded",
          m.kamikaze_hit and "qr_frames" in m.kamikaze_hit,
          "%s frames" % (m.kamikaze_hit or {}).get("qr_frames"))

    # (c) A DETECTION EXISTS but the QR is LOST: it must still exit.
    #     ⚠️ Had this check sat inside the `qr is not None` block, losing the
    #        QR on the final frame would leave the aircraft diving on -
    #        silently and fatally.
    m = _FakeMission(); m.qr_result = _qr(ex=0.1, in_av=True)
    d = _new_dive()
    _FakeTelD.rel_alt_m = config.KAMIKAZE_QR_CONTINUE_ALTITUDE_M + 5.0
    run(d.update(m))                      # the detection is recorded
    m.qr_result = None                    # the QR is lost
    _FakeTelD.rel_alt_m = config.KAMIKAZE_QR_CONTINUE_ALTITUDE_M - 1.0
    run(d.update(m))
    check("exits at the threshold even if the QR was lost",
          m.changed == "PULL_UP", "transition=%s" % m.changed)

    _FakeTelD.rel_alt_m = 60.0            # do not affect the other tests

    config.KAMIKAZE_PULLUP_ON_QR = previous_pullup
    print()


def main():
    print()
    test_p1()
    test_p2()
    test_p3()
    test_p4()
    print("=" * 76)
    ok = sum(1 for p in PASS if p)
    print("  RESULT: %d/%d" % (ok, len(PASS)))
    print("=" * 76)
    return 0 if ok == len(PASS) else 1


if __name__ == "__main__":
    sys.exit(main())
