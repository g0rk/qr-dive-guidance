# tests/test_pull_up_state.py - the state that gets the aircraft back off the deck.
#
#     python3 tests/test_pull_up_state.py
#
# ⚠️ WRITTEN AFTER A COVERAGE MEASUREMENT, not after a bug - same as
#    test_approach_state.py, and worth saying rather than inventing a story.
#
#        states/pull_up_state.py     44 statements     29 missed     34%
#
#    The 34 % was module-level imports and the class body. The whole of
#    update() - lines 49-93, every decision this state makes - had never run
#    in a test. That is the recovery: DIVE hands over at 30 m, and the
#    measured altitude loss after the command is up to 16.46 m, so this code
#    is what stands between the aircraft and the ground with about 13 m in
#    hand.
#
# THE TWO FAILURES THAT WOULD NOT ANNOUNCE THEMSELVES:
#
#   1. A SIGN ERROR ON THE PITCH. DIVE_PITCH_DEG is -55 (nose down) and
#      PULL_UP_PITCH_DEG is +25 (nose up); the two live eight lines apart in
#      config.py. Get the sign wrong and the state is still called PULL_UP,
#      still logs "climbing to 50.0m", still exits cleanly - into the ground.
#   2. SKIPPING A TICK. This state's own docstring says MAVSDK offboard
#      reverts if attitude commands stop arriving. Any "only send it when
#      something changed" optimisation drops the aircraft out of offboard
#      mid-recovery, and nothing raises.

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import logging
logging.disable(logging.CRITICAL)

import config
from states.pull_up_state import PullUpState
from vehicle import VehicleCommandError

PASS = []


def check(label, cond, detail=""):
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "PASS" if cond else "FAIL", detail))
    assert cond, "%s  %s" % (label, detail)


def run(coro):
    import asyncio
    return asyncio.get_event_loop().run_until_complete(coro)


# ----------------------------------------------------------------------
class _NullLive:
    def start(self): pass

    def stop(self): pass

    def update(self, *a): pass


class _Tel:
    def __init__(self, rel_alt_m):
        self.rel_alt_m = rel_alt_m


class _StubQ:
    def __init__(self):
        self.items = []

    def put_nowait(self, x):
        self.items.append(x)


class _FakeVehicle:
    """Records every command. `fail_hold` / `fail_attitude` make the autopilot
    refuse, which is how the two abort paths get exercised."""

    def __init__(self, fail_hold=False, fail_attitude=False):
        self.attitudes = []
        self.holds = 0
        self.fail_hold = fail_hold
        self.fail_attitude = fail_attitude

    async def set_attitude(self, roll_deg, pitch_deg, yaw_rate_deg_s, thrust):
        if self.fail_attitude:
            raise VehicleCommandError("simulated attitude refusal")
        self.attitudes.append((roll_deg, pitch_deg, yaw_rate_deg_s, thrust))

    async def hold(self, *args, **kwargs):
        if self.fail_hold:
            raise VehicleCommandError("simulated hold refusal")
        self.holds += 1


class _FakeMission:
    def __init__(self, rel_alt_m, vehicle=None):
        tel = _Tel(rel_alt_m)
        self.telemetry = type("S", (), {"get": staticmethod(lambda: tel)})()
        self.vehicle = vehicle if vehicle is not None else _FakeVehicle()
        self.perception_command_queue = _StubQ()
        self.changed = None

    async def _change_state(self, state):
        self.changed = state.name


def _state():
    pull_up = PullUpState()
    pull_up._live = _NullLive()
    return pull_up


# ======================================================================
# 1 - which way is up
# ======================================================================
def test_the_nose_goes_up():
    print("\n1) WHICH WAY THE NOSE GOES")
    climbing = config.PULL_UP_SAFE_ALTITUDE_M - 20.0
    mission = _FakeMission(climbing)
    run(_state().update(mission))

    check("an attitude command was sent", len(mission.vehicle.attitudes) == 1,
          "%d command(s)" % len(mission.vehicle.attitudes))
    roll, pitch, yaw_rate, thrust = mission.vehicle.attitudes[0]

    # ⚠️ THE WHOLE POINT. Positive is nose-up in this convention, which is why
    #    DIVE_PITCH_DEG is negative. Asserting "pitch == config value" would
    #    pass even if the config itself were signed wrong, so assert the SIGN
    #    against the dive as well.
    check("the commanded pitch is nose-UP (positive)", pitch > 0,
          "pitch=%+.1f deg" % pitch)
    check("...and the opposite sign to the dive",
          pitch * config.DIVE_PITCH_DEG < 0,
          "pull-up %+.1f vs dive %+.1f" % (pitch, config.DIVE_PITCH_DEG))
    check("it is the configured pull-up pitch",
          abs(pitch - config.PULL_UP_PITCH_DEG) < 1e-9,
          "%+.1f deg" % config.PULL_UP_PITCH_DEG)

    # ⚠️ The dive is flown at zero throttle. Climbing out of it at zero
    #    throttle, from 30 m with up to 16.46 m already spent, is not a climb.
    check("the throttle is opened for the climb", thrust > 0.0,
          "thrust=%.2f (the dive uses %.2f)" % (thrust, config.DIVE_THROTTLE))
    check("wings level during the recovery", abs(roll) < 1e-9,
          "roll=%+.1f" % roll)


# ======================================================================
# 2 - every tick, without exception
# ======================================================================
def test_a_command_goes_out_every_tick():
    print("\n2) A COMMAND GOES OUT ON EVERY TICK")
    # ⚠️ MAVSDK offboard reverts if the stream stops. Ten identical ticks must
    #    produce ten commands, not one - "nothing changed so skip it" is a
    #    plausible optimisation and a fatal one.
    mission = _FakeMission(config.PULL_UP_SAFE_ALTITUDE_M - 20.0)
    pull_up = _state()
    for _ in range(10):
        run(pull_up.update(mission))
    check("10 identical ticks produced 10 commands",
          len(mission.vehicle.attitudes) == 10,
          "%d sent" % len(mission.vehicle.attitudes))
    check("...all of them the same command",
          len(set(mission.vehicle.attitudes)) == 1,
          "%d distinct" % len(set(mission.vehicle.attitudes)))
    check("and it never exited early", mission.changed is None,
          "transition=%s" % mission.changed)


# ======================================================================
# 3 - when the recovery is over
# ======================================================================
def test_it_exits_at_the_safe_altitude():
    print("\n3) EXIT AT THE SAFE ALTITUDE, NOT BEFORE")
    safe = config.PULL_UP_SAFE_ALTITUDE_M

    below = _FakeMission(safe - 0.1)
    run(_state().update(below))
    check("just below the safe altitude -> keep climbing",
          below.changed is None and len(below.vehicle.attitudes) == 1,
          "transition=%s" % below.changed)

    at = _FakeMission(safe)
    run(_state().update(at))
    check("at the safe altitude -> HOLD", at.changed == "HOLD",
          "transition=%s" % at.changed)
    check("...the autopilot was actually commanded to hold",
          at.vehicle.holds == 1, "%d hold command(s)" % at.vehicle.holds)
    check("...and no attitude command was sent on that tick",
          not at.vehicle.attitudes,
          "%d attitude command(s)" % len(at.vehicle.attitudes))

    # ⚠️ If the exit altitude were at or below the altitude DIVE hands over
    #    at, this state would finish on its first tick and the aircraft would
    #    be left where the dive dropped it.
    check("the exit altitude is above where the dive hands over",
          safe > config.DIVE_PULL_UP_ALTITUDE_M,
          "%.1f m > %.1f m" % (safe, config.DIVE_PULL_UP_ALTITUDE_M))


# ======================================================================
# 4 - the two refusals
# ======================================================================
def test_a_refused_command_aborts():
    print("\n4) A REFUSED COMMAND ABORTS, IT DOES NOT CARRY ON")
    # ⚠️ Both paths matter and they fail at different moments. A refused
    #    attitude command means the aircraft is not being flown; a refused
    #    hold means the recovery finished but nothing took over. Carrying on
    #    to HOLD in either case reports a success that did not happen.
    attitude_refused = _FakeMission(config.PULL_UP_SAFE_ALTITUDE_M - 20.0,
                                    vehicle=_FakeVehicle(fail_attitude=True))
    run(_state().update(attitude_refused))
    check("a refused attitude command -> ABORT",
          attitude_refused.changed == "ABORT",
          "transition=%s" % attitude_refused.changed)

    hold_refused = _FakeMission(config.PULL_UP_SAFE_ALTITUDE_M,
                                vehicle=_FakeVehicle(fail_hold=True))
    run(_state().update(hold_refused))
    check("a refused hold command -> ABORT, not HOLD",
          hold_refused.changed == "ABORT",
          "transition=%s" % hold_refused.changed)


# ======================================================================
# 5 - leaving the state tidily
# ======================================================================
def test_perception_is_stopped_on_the_way_out():
    print("\n5) PERCEPTION IS STOPPED ON THE WAY OUT")
    # APPROACH starts perception on entry; something has to stop it, or the
    # camera pipeline keeps running for the rest of the flight. This is the
    # other half of that pair.
    mission = _FakeMission(config.PULL_UP_SAFE_ALTITUDE_M)
    pull_up = _state()
    run(pull_up.on_enter(mission))
    run(pull_up.on_exit(mission))
    check("a stop command was queued for perception",
          {"cmd": "stop"} in mission.perception_command_queue.items,
          str(mission.perception_command_queue.items))


if __name__ == "__main__":
    test_the_nose_goes_up()
    test_a_command_goes_out_every_tick()
    test_it_exits_at_the_safe_altitude()
    test_a_refused_command_aborts()
    test_perception_is_stopped_on_the_way_out()
    print("\n%d/%d checks passed" % (sum(PASS), len(PASS)))
    sys.exit(0 if all(PASS) else 1)
