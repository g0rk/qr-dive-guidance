# tests/test_approach_state.py - the state that decides WHEN to dive.
#
#     python3 tests/test_approach_state.py
#
# ⚠️ THIS ONE WAS NOT WRITTEN AFTER A BUG. It was written after a COVERAGE
#    MEASUREMENT, and saying so matters, because every other test file here
#    opens with a real failure and inventing one would be the exact dishonesty
#    this repository argues against.
#
#    Measured (coverage 7.15.3, `coverage run --source=. -m pytest tests/`):
#
#        states/approach_state.py     51 statements     51 missed     0%
#
#    Not one line ran. The module was never even imported by the suite. That
#    is the state holding APPROACH_DIVE_ARM_DISTANCE_M - the number this whole
#    repository is about - and the altitude gate that stands between a dive
#    and an abort.
#
#    WHY IT SLIPPED THROUGH test_state_construction.py: that file iterates
#    TRANSITION_TABLE, which lists only the states reachable by an EXTERNAL
#    command. The mission takes exactly two (`takeoff`, `align`), so APPROACH,
#    DIVE and PULL_UP - the entire dive chain - sat outside it. The bug that
#    caused that file to exist (TakeoffState used config without importing it,
#    so constructing it raised NameError and the aircraft could not take off)
#    could happen in this state today and nothing would notice.
#
# WHAT IS LOCKED DOWN HERE, and why each one can fail silently:
#
#   1. The dive arms at the DERIVED distance. A flipped comparison (`>=` for
#      `<=`) passes every other test in this suite and simply never dives.
#   2. Too low -> ABORT, not DIVE. A safety branch that had never executed.
#   3. Nothing happens until the approach waypoint was accepted.
#   4. The ghost waypoint goes BEYOND the target. A sign error puts it between
#      the aircraft and the target, so the aircraft levels off short of it.
#   5. The altitude is commanded RELATIVE. Handing goto_location an AMSL-less
#      number was a real bug (see P2 in test_preconditions.py); nothing checked
#      that this state calls the converting method.
#   6. A refused waypoint PROPAGATES, so the FSM can fall back to ABORT
#      instead of sitting in APPROACH forever.

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import logging
logging.disable(logging.CRITICAL)

import config
from states.approach_state import ApproachState
from utils.geo_utils import distance_m, offset_lat_lon
from vehicle import VehicleCommandError

PASS = []


def check(label, cond, detail=""):
    # ⚠️ The `assert` matters: this helper used to only print, so a check could
    #    say FAIL on screen while pytest stayed green.
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "PASS" if cond else "FAIL", detail))
    assert cond, "%s  %s" % (label, detail)


def run(coro):
    import asyncio
    return asyncio.get_event_loop().run_until_complete(coro)


# ----------------------------------------------------------------------
# The smallest fakes that can stand in for the real thing
# ----------------------------------------------------------------------
class _NullLive:
    """rich.Live writes to the terminal. The dive decision does not depend on
    it, and a live display inside a test run fights with pytest's capture."""
    def start(self): pass
    def stop(self): pass
    def update(self, *a): pass


class _Tel:
    def __init__(self, lat, lon, rel_alt_m):
        self.latitude_deg = lat
        self.longitude_deg = lon
        self.rel_alt_m = rel_alt_m


class _StubQ:
    def __init__(self):
        self.items = []

    def put_nowait(self, x):
        self.items.append(x)


class _FakeVehicle:
    """Records which method was called with what. `refuse=True` makes the
    autopilot reject the waypoint, which is check 6."""
    def __init__(self, refuse=False):
        self.rel_calls = []
        self.abs_calls = []
        self.refuse = refuse

    async def goto_location_rel(self, lat, lon, rel_alt_m, yaw_deg=0.0):
        if self.refuse:
            raise VehicleCommandError("simulated autopilot refusal")
        self.rel_calls.append((lat, lon, rel_alt_m))

    async def goto_location(self, lat, lon, abs_alt_m, yaw_deg=0.0):
        self.abs_calls.append((lat, lon, abs_alt_m))


class _FakeMission:
    def __init__(self, tel, vehicle=None):
        self.telemetry = type("S", (), {"get": staticmethod(lambda: tel)})()
        self.vehicle = vehicle if vehicle is not None else _FakeVehicle()
        self.perception_command_queue = _StubQ()
        self.changed = None

    async def _change_state(self, state):
        self.changed = state.name


def _west_of_target(metres):
    """A position `metres` west of the target, i.e. flying due east at it."""
    return offset_lat_lon(config.TARGET_LATITUDE_DEG,
                          config.TARGET_LONGITUDE_DEG, 0.0, -metres)


def _armed_approach():
    """An APPROACH that has already had its waypoint accepted."""
    state = ApproachState()
    state._live = _NullLive()
    state._command_sent = True
    return state


def _decision_at(offset_m, rel_alt_m=None):
    """Run one APPROACH tick from `offset_m` west of the target.
    Returns the state name it transitioned to, or None."""
    if rel_alt_m is None:
        rel_alt_m = config.APPROACH_SAFE_ALTITUDE_M
    lat, lon = _west_of_target(offset_m)
    mission = _FakeMission(_Tel(lat, lon, rel_alt_m))
    run(_armed_approach().update(mission))
    return mission.changed


# ======================================================================
# 1 - the trigger distance
# ======================================================================
def test_dive_arms_at_the_derived_distance():
    print("\n1) THE DIVE ARMS AT THE DERIVED DISTANCE")
    arm = config.APPROACH_DIVE_ARM_DISTANCE_M

    check("well outside the arm distance -> no transition",
          _decision_at(arm + 20.0) is None,
          "%.1f m > %.1f m" % (arm + 20.0, arm))
    check("well inside the arm distance -> DIVE",
          _decision_at(arm - 20.0) == "DIVE",
          "%.1f m <= %.1f m" % (arm - 20.0, arm))

    # ⚠️ The two checks above only prove the behaviour is different far from
    #    the threshold. They would still pass if the constant were 60 m or
    #    150 m. So MEASURE where the handover actually happens instead of
    #    asserting the number the code was handed: bisect the distance at
    #    which the decision flips, and compare that to the config value.
    #    This catches a wrong constant, a flipped comparison and a unit error
    #    with one check.
    low, high = 50.0, 200.0                   # low dives, high does not
    assert _decision_at(low) == "DIVE" and _decision_at(high) is None, \
        "the bracket for the bisection is wrong"
    for _ in range(40):
        mid = 0.5 * (low + high)
        if _decision_at(mid) == "DIVE":
            low = mid
        else:
            high = mid

    # The bisection ran on a flat-earth offset; the state measures a spherical
    # distance. Convert before comparing, so the two use the same yardstick.
    lat, lon = _west_of_target(0.5 * (low + high))
    measured = distance_m(lat, lon, config.TARGET_LATITUDE_DEG,
                          config.TARGET_LONGITUDE_DEG)
    check("the measured handover distance IS the configured one",
          abs(measured - arm) < 1.0,
          "measured %.3f m vs config %.3f m (diff %.3f)" % (
              measured, arm, measured - arm))

    # And the configured one has to stay DERIVED, not hand-typed.
    import dive_geometry
    derived = dive_geometry.trigger_distance_m(config._DIVE_PROFILE)
    check("the constant still comes from dive_geometry, not by hand",
          abs(arm - derived) < 1e-9, "%.6f m" % derived)


# ======================================================================
# 2 - the altitude gate
# ======================================================================
def test_too_low_aborts_instead_of_diving():
    print("\n2) TOO LOW INSIDE THE ARM DISTANCE -> ABORT, NOT DIVE")
    # ⚠️ Never executed before this test existed. Diving from below the entry
    #    altitude is the one case where the right answer is to give up: the
    #    pull-up floor is 30 m and the measured altitude loss after the
    #    pull-up command is up to 16.46 m, so there is no room to spend.
    inside = config.APPROACH_DIVE_ARM_DISTANCE_M - 10.0
    floor = config.DIVE_MIN_ENTRY_ALTITUDE_M

    check("just below the entry altitude -> ABORT",
          _decision_at(inside, rel_alt_m=floor - 0.1) == "ABORT",
          "alt=%.1f m < %.1f m" % (floor - 0.1, floor))
    # False-positive guard: a state that ABORTed unconditionally would also
    # pass the check above.
    check("just above the entry altitude -> DIVE",
          _decision_at(inside, rel_alt_m=floor + 0.1) == "DIVE",
          "alt=%.1f m >= %.1f m" % (floor + 0.1, floor))
    check("outside the arm distance, too low -> still nothing",
          _decision_at(config.APPROACH_DIVE_ARM_DISTANCE_M + 20.0,
                       rel_alt_m=floor - 50.0) is None,
          "the altitude gate must not fire on its own")


# ======================================================================
# 3 - nothing happens before the waypoint was accepted
# ======================================================================
def test_no_transition_before_the_waypoint_was_accepted():
    print("\n3) NOTHING HAPPENS UNTIL THE WAYPOINT IS ACCEPTED")
    # ⚠️ If goto_location_rel failed, _command_sent stays False and the
    #    aircraft is not flying an approach at all. Arming the dive on
    #    distance alone would then commit to a dive from an unknown attitude.
    lat, lon = _west_of_target(1.0)          # 1 m away: as inside as it gets
    mission = _FakeMission(_Tel(lat, lon, config.APPROACH_SAFE_ALTITUDE_M))
    state = ApproachState()
    state._live = _NullLive()                # _command_sent left False
    run(state.update(mission))
    check("1 m from the target but no waypoint sent -> no transition",
          mission.changed is None, "transition=%s" % mission.changed)


# ======================================================================
# 4, 5 - what on_enter actually commands
# ======================================================================
def _enter_from(offset_m, vehicle=None):
    lat, lon = _west_of_target(offset_m)
    mission = _FakeMission(_Tel(lat, lon, config.APPROACH_SAFE_ALTITUDE_M),
                           vehicle=vehicle)
    state = ApproachState()
    state._live = _NullLive()
    run(state.on_enter(mission))
    return state, mission


def test_ghost_waypoint_goes_beyond_the_target():
    print("\n4) THE GHOST WAYPOINT IS BEYOND THE TARGET")
    # ⚠️ The point of a ghost waypoint is to fly THROUGH the target on a
    #    settled straight line instead of arriving and levelling off on it.
    #    Reverse the bearing by mistake and the waypoint lands between the
    #    aircraft and the target - the aircraft then levels off SHORT, and
    #    nothing raises: it just never gets to dive.
    #
    #    ⚠️ The comment in approach_state.py says "along the reverse bearing
    #       (from target back toward drone)". The code does the opposite of
    #       that parenthesis, and the code is right. Measured here rather than
    #       argued from the comment.
    away = 1000.0
    state, mission = _enter_from(away)
    ghost_lat, ghost_lon, _ = mission.vehicle.rel_calls[0]
    lat, lon = _west_of_target(away)

    to_target = distance_m(lat, lon, config.TARGET_LATITUDE_DEG,
                           config.TARGET_LONGITUDE_DEG)
    to_ghost = distance_m(lat, lon, ghost_lat, ghost_lon)
    past = distance_m(config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG,
                      ghost_lat, ghost_lon)

    check("the ghost is FARTHER from the aircraft than the target is",
          to_ghost > to_target,
          "aircraft->ghost %.1f m > aircraft->target %.1f m" % (to_ghost, to_target))
    check("it sits the configured distance past the target",
          abs(past - config.APPROACH_GHOST_DISTANCE_M) < 1.0,
          "target->ghost %.1f m (config %.1f)" % (
              past, config.APPROACH_GHOST_DISTANCE_M))
    check("aircraft, target and ghost are on one line",
          abs(to_ghost - (to_target + past)) < 1.0,
          "%.1f vs %.1f + %.1f" % (to_ghost, to_target, past))


def test_the_approach_altitude_is_commanded_relative():
    print("\n5) THE APPROACH ALTITUDE IS COMMANDED RELATIVE")
    # ⚠️ P2 in test_preconditions.py proves goto_location_rel converts
    #    correctly. Nothing proved that THIS state calls it. Swapping it back
    #    to goto_location leaves that test green and puts the waypoint 388 m
    #    below ground at the PX4 SITL default site (Zurich, 488 m).
    state, mission = _enter_from(1000.0)
    check("goto_location_rel was called exactly once",
          len(mission.vehicle.rel_calls) == 1,
          "%d call(s)" % len(mission.vehicle.rel_calls))
    check("the AMSL method was NOT called",
          not mission.vehicle.abs_calls,
          "%d call(s)" % len(mission.vehicle.abs_calls))
    check("the commanded altitude is APPROACH_SAFE_ALTITUDE_M",
          abs(mission.vehicle.rel_calls[0][2]
              - config.APPROACH_SAFE_ALTITUDE_M) < 1e-9,
          "%.1f m" % mission.vehicle.rel_calls[0][2])
    check("the waypoint is marked as sent", state._command_sent is True)

    # Perception has to be switched to QR before the dive, or the camera runs
    # the whole descent looking for the wrong thing.
    cmds = mission.perception_command_queue.items
    check("perception was put into QR mode and started",
          {"cmd": "set_mode", "mode": "qr"} in cmds and {"cmd": "start"} in cmds,
          str(cmds))


# ======================================================================
# 6 - a refused waypoint has to propagate
# ======================================================================
def test_a_refused_waypoint_propagates():
    print("\n6) A REFUSED WAYPOINT PROPAGATES")
    # ⚠️ approach_state catches VehicleCommandError only to log it, then
    #    re-raises so _change_state can fall back to ABORT. Turning that
    #    `raise` into a swallow leaves the aircraft in APPROACH forever with
    #    no waypoint and no error - the silent failure this project keeps
    #    running into.
    lat, lon = _west_of_target(1000.0)
    mission = _FakeMission(_Tel(lat, lon, config.APPROACH_SAFE_ALTITUDE_M),
                           vehicle=_FakeVehicle(refuse=True))
    state = ApproachState()
    state._live = _NullLive()

    raised = False
    try:
        run(state.on_enter(mission))
    except VehicleCommandError:
        raised = True
    check("a refusal is re-raised, not swallowed", raised)
    check("the waypoint is NOT marked as sent", state._command_sent is False,
          "_command_sent=%s" % state._command_sent)

    # ...and with it not sent, the tick stays inert even on top of the target.
    lat, lon = _west_of_target(1.0)
    mission2 = _FakeMission(_Tel(lat, lon, config.APPROACH_SAFE_ALTITUDE_M))
    run(state.update(mission2))
    check("a failed approach cannot fall into a dive",
          mission2.changed is None, "transition=%s" % mission2.changed)


if __name__ == "__main__":
    test_dive_arms_at_the_derived_distance()
    test_too_low_aborts_instead_of_diving()
    test_no_transition_before_the_waypoint_was_accepted()
    test_ghost_waypoint_goes_beyond_the_target()
    test_the_approach_altitude_is_commanded_relative()
    test_a_refused_waypoint_propagates()
    print("\n%d/%d checks passed" % (sum(PASS), len(PASS)))
    sys.exit(0 if all(PASS) else 1)
