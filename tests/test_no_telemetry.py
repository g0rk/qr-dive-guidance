# tests/test_no_telemetry.py - what the mission does when telemetry never arrives,
# and the safety checker that is switched off.
#
#     python3 tests/test_no_telemetry.py
#
# ⚠️ THIS FILE LOCKS DOWN A GAP THAT IS DELIBERATELY LEFT OPEN, which is
#    unusual enough to explain.
#
#    telemetry.py documents its own defaults as "safe/invalid values so the
#    safety checker will reject a TelemetryData that has never been updated".
#    That sentence leans on a defence that mission_controller.py disables:
#    the call to SafetyChecker.check is commented out and the branch that
#    would act on it is wrapped in `if False:`.
#
#    The README says so under "Known gaps", and then admits the real problem:
#    "nothing tests that it is off". So a claim about the running system was
#    resting on prose alone. This file measures it instead.
#
#    THE `if False:` IS LOAD-BEARING, not an oversight. Measured: the altitude
#    check has no notion of being on the ground, and only IDLE and ABORT are
#    exempt from the safety transition, so enabling the checker as written
#    aborts the mission during every single takeoff. Section 3 below is a
#    TRIPWIRE: if it fails, nothing is broken - somebody enabled the checker,
#    and they should read this comment and the README before going further.

from __future__ import annotations

import ast
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import logging
logging.disable(logging.CRITICAL)

import config
from safety import SafetyChecker
from telemetry import TelemetryData, TelemetryStore
from utils.geo_utils import distance_m

PASS = []


def check(label, cond, detail=""):
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "PASS" if cond else "FAIL", detail))
    assert cond, "%s  %s" % (label, detail)


def run(coro):
    import asyncio
    return asyncio.get_event_loop().run_until_complete(coro)


# ======================================================================
# 1 - telemetry that never arrived has to be recognisable as such
# ======================================================================
def test_never_updated_telemetry_is_recognisable():
    print("\n1) TELEMETRY THAT NEVER ARRIVED")
    fresh_from_the_box = TelemetryData()

    # ⚠️ THE ONLY FIELD THAT CARRIES THE "no data" SIGNAL IS THE AGE.
    #    Position defaults to 0.0/0.0 and altitude to 0.0 - perfectly ordinary
    #    numbers that no arithmetic can tell apart from a real reading. If
    #    last_update_time ever defaulted to time.monotonic() "so the age is
    #    sensible", telemetry that never arrived would look permanently FRESH
    #    and every staleness check in the project would become a no-op.
    check("age of never-updated telemetry is infinite",
          fresh_from_the_box.age_s == float("inf"),
          "age=%s" % fresh_from_the_box.age_s)
    check("...which is over the staleness limit",
          fresh_from_the_box.age_s > config.MAX_TELEMETRY_AGE_S,
          "limit=%.1f s" % config.MAX_TELEMETRY_AGE_S)
    check("groundspeed of never-updated telemetry is 0",
          fresh_from_the_box.groundspeed_m_s == 0.0)

    # False-positive guard: the age has to become finite once data arrives,
    # or the check above would pass on a store that is simply broken.
    store = TelemetryStore()
    store.update_position(47.0, 8.0, 500.0, 120.0)
    updated = store.get()
    check("age becomes finite after one update",
          updated.age_s < 1.0, "age=%.4f s" % updated.age_s)

    # get() must hand out a COPY, or the FSM reads fields that change under it
    # halfway through a tick.
    snapshot = store.get()
    store.update_position(0.0, 0.0, 0.0, 0.0)
    check("get() returns a snapshot, not a live reference",
          snapshot.rel_alt_m == 120.0, "%.1f m" % snapshot.rel_alt_m)


# ======================================================================
# 2 - the checker, if it ran, would reject it
# ======================================================================
def test_the_checker_would_reject_it():
    print("\n2) WHAT THE SAFETY CHECKER WOULD SAY")
    checker = SafetyChecker()

    result = checker.check(TelemetryData())
    check("never-updated telemetry is rejected", not result.ok, result.reason)
    # The ORDER matters: staleness is checked first, so that is the message an
    # operator would see. Reporting "altitude too low" for a link that is down
    # would send them looking in the wrong place.
    check("...and the reason given is staleness, not altitude",
          "stale" in result.reason.lower(), result.reason)

    # Fresh, airborne, sane speed -> passes. Without this the section above
    # would also pass on a checker that rejected everything.
    import time
    ok_tel = TelemetryData(rel_alt_m=120.0, vel_north_m_s=30.0,
                           last_update_time=time.monotonic())
    check("fresh and airborne passes", checker.check(ok_tel).ok,
          checker.check(ok_tel).reason)

    # ⚠️ THIS IS THE MEASUREMENT BEHIND THE `if False:`. The altitude check
    #    has no notion of "on the ground", so a perfectly healthy aircraft
    #    sitting on the runway fails it.
    on_the_ground = TelemetryData(rel_alt_m=0.0,
                                  last_update_time=time.monotonic())
    ground_result = checker.check(on_the_ground)
    check("fresh telemetry ON THE GROUND is rejected too",
          not ground_result.ok, ground_result.reason)
    check("...and that is the altitude check, not staleness",
          "altitude" in ground_result.reason.lower(), ground_result.reason)

    climbing_out = TelemetryData(rel_alt_m=2.0,
                                 last_update_time=time.monotonic())
    check("seconds into the climb it is still rejected",
          not checker.check(climbing_out).ok,
          checker.check(climbing_out).reason)
    just_above = TelemetryData(rel_alt_m=config.MIN_REL_ALT_M + 0.1,
                               last_update_time=time.monotonic())
    check("just above the floor it passes", checker.check(just_above).ok,
          "%.1f m >= %.1f m" % (just_above.rel_alt_m, config.MIN_REL_ALT_M))

    overspeed = TelemetryData(rel_alt_m=120.0,
                              vel_north_m_s=config.MAX_GROUNDSPEED_M_S + 10.0,
                              last_update_time=time.monotonic())
    check("overspeed is rejected", not checker.check(overspeed).ok,
          checker.check(overspeed).reason)


# ======================================================================
# 3 - THE TRIPWIRE: the checker is built, and it is not consulted
# ======================================================================
def _checker_sites():
    """Where mission_controller BUILDS the checker, and where it CALLS it.

    Parsed rather than grepped, because a comment is not code: the call in
    that file is commented out, and `grep _safety.check` cannot tell the
    difference. ast only ever sees what Python would actually run.

    ⚠️ Source inspection, and it is used on purpose. The claim being pinned is
       about the shape of the running program - "this checker is constructed
       and its verdict is never read" - and the alternative, driving a real
       tick loop, needs a connected autopilot.
    """
    source = open(os.path.join(ROOT, "processes", "mission_controller.py"),
                  encoding="utf-8").read()
    built, called = [], []
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, (ast.Assign, ast.AnnAssign)):
            targets = (node.targets if isinstance(node, ast.Assign)
                       else [node.target])
            value = node.value
            for target in targets:
                if (isinstance(target, ast.Attribute)
                        and target.attr == "_safety"
                        and isinstance(value, ast.Call)
                        and isinstance(value.func, ast.Name)
                        and value.func.id == "SafetyChecker"):
                    built.append(node.lineno)
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "check"
                and isinstance(node.func.value, ast.Attribute)
                and node.func.value.attr == "_safety"):
            called.append(node.lineno)
    return built, called


def test_the_checker_is_built_but_never_consulted():
    print("\n3) TRIPWIRE - THE CHECKER IS BUILT AND NOT CONSULTED")
    built, called = _checker_sites()

    # ⚠️ It is built inside _loop(), NOT in __init__ - constructing a
    #    MissionController does not give you a checker. This test asserted
    #    otherwise at first and failed, which is how the fact was found.
    #
    #    Distinguishing "disabled" from "deleted" matters: removing the
    #    checker outright is a different change and should read differently
    #    here rather than quietly satisfying "nothing calls it".
    check("mission_controller still constructs a SafetyChecker",
          len(built) == 1, "line %s" % (built or "NOWHERE - it was removed"))

    check("...and nothing in the tick loop consults it",
          not called,
          "call sites: %s" % (called or "none - the gap is where the README says"))


# ======================================================================
# 4 - so what protects the dive? nothing on purpose - measure it
# ======================================================================
class _Tel:
    def __init__(self, lat, lon, rel_alt_m):
        self.latitude_deg = lat
        self.longitude_deg = lon
        self.rel_alt_m = rel_alt_m


class _NullLive:
    def start(self): pass

    def stop(self): pass

    def update(self, *a): pass


class _FakeMission:
    def __init__(self, tel):
        self.telemetry = type("S", (), {"get": staticmethod(lambda: tel)})()
        self.changed = None

    async def _change_state(self, state):
        self.changed = state.name


def test_approach_does_not_dive_on_absent_telemetry():
    print("\n4) WITH THE CHECKER OFF, WHAT STOPS A DIVE ON NO DATA")
    from states.approach_state import ApproachState

    empty = TelemetryData()          # 0.0 / 0.0, the defaults
    gulf_of_guinea = distance_m(empty.latitude_deg, empty.longitude_deg,
                                config.TARGET_LATITUDE_DEG,
                                config.TARGET_LONGITUDE_DEG)

    state = ApproachState()
    state._live = _NullLive()
    state._command_sent = True       # the harshest case: already armed
    mission = _FakeMission(_Tel(empty.latitude_deg, empty.longitude_deg,
                                empty.rel_alt_m))
    run(state.update(mission))

    # ⚠️ SAY WHAT THIS ACTUALLY IS. The dive does not trigger, but not because
    #    anything checked whether telemetry had arrived. (0, 0) is a real point
    #    in the Gulf of Guinea, and it happens to be thousands of kilometres
    #    from the target, so the distance test fails. That is luck with a
    #    comfortable margin, not a guard - and writing it down as luck is the
    #    point of this check.
    check("no dive on never-updated telemetry", mission.changed is None,
          "transition=%s" % mission.changed)
    check("...because (0,0) is %.0f km away, not because anything checked"
          % (gulf_of_guinea / 1000.0),
          gulf_of_guinea > 100.0 * config.APPROACH_DIVE_ARM_DISTANCE_M,
          "%.0f m vs an arm distance of %.1f m" % (
              gulf_of_guinea, config.APPROACH_DIVE_ARM_DISTANCE_M))

    # And the altitude gate would not have saved it either: 0.0 m is below the
    # dive entry threshold, so IF the distance test ever passed with empty
    # telemetry the state would ABORT rather than dive. Worth knowing which of
    # the two is actually load-bearing.
    check("the altitude gate would refuse it as well",
          empty.rel_alt_m < config.DIVE_MIN_ENTRY_ALTITUDE_M,
          "%.1f m < %.1f m" % (empty.rel_alt_m,
                               config.DIVE_MIN_ENTRY_ALTITUDE_M))


if __name__ == "__main__":
    test_never_updated_telemetry_is_recognisable()
    test_the_checker_would_reject_it()
    test_the_checker_is_built_but_never_consulted()
    test_approach_does_not_dive_on_absent_telemetry()
    print("\n%d/%d checks passed" % (sum(PASS), len(PASS)))
    sys.exit(0 if all(PASS) else 1)
