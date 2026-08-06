# tests/test_state_construction.py - can every FSM state actually be CONSTRUCTED?
#
#     python3 tests/test_state_construction.py
#
# ⚠️ THIS TEST WAS WRITTEN AFTER A REAL BUG (2026-08-05).
#
#    states/takeoff_state.py used `config.TAKEOFF_ALTITUDE_M` but had no
#    `import config`. Constructing TakeoffState() therefore raised NameError,
#    and THE AIRCRAFT COULD NOT TAKE OFF UNDER ANY CIRCUMSTANCES.
#
#    WHY IT WAS SILENT: CommandRouter wraps state construction in try/except
#    (command_router.py:106) and turns the crash into a WARNING:
#        "Command rejected: Failed to instantiate TakeoffState:
#         name 'config' is not defined"
#    The program does not crash; the command is just quietly rejected. Anyone
#    not reading the log spends hours on "why won't it take off".
#
#    WHY NO TEST CAUGHT IT: none of the existing tests CONSTRUCTED the states
#    - they all exercised perception and geometry functions. Importing a
#    module does NOT prove that a class inside it can be instantiated:
#    NameError happens at call time, not at import time.
#
#    The regression came in when _target_alt_m moved from a literal into
#    config and the import was not added with it.

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import logging
logging.disable(logging.CRITICAL)

from utils.command_router import TRANSITION_TABLE, CommandRouter

PASS = []


def check(label, cond, detail=""):
    # ⚠️ The `assert` was added later: this helper used to NOT fail pytest, so
    #    the suite stayed green while individual checks printed FAIL.
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "PASS" if cond else "FAIL", detail))
    assert cond, "%s  %s" % (label, detail)


def test_every_state_can_be_constructed():
    print("\n1) CAN EVERY STATE CLASS BE CONSTRUCTED")
    failures = []
    for command, entry in sorted(TRANSITION_TABLE.items()):
        cls = entry["target"]
        try:
            instance = cls()
            check("%-10s -> %s()" % (command, cls.__name__), True,
                  "name=%s" % getattr(instance, "name", "?"))
        except Exception as exc:
            failures.append((cls.__name__, exc))
            check("%-10s -> %s()" % (command, cls.__name__), False,
                  "%s: %s" % (type(exc).__name__, exc))
    assert not failures, "states that could not be constructed: %s" % failures


def test_router_accepts_takeoff():
    print("\n2) DOES THE ROUTER ACCEPT 'takeoff' FROM IDLE")
    # This is exactly where the original bug showed up: the router returned
    # ok=False and the reason was a NameError. Locking that scenario down is
    # the whole reason this test exists.
    result = CommandRouter().resolve('{"command": "takeoff"}', "IDLE")
    check("command accepted", result.ok,
          "reason: %s" % (result.reason or "-"))
    check("target state produced", result.new_state is not None,
          "state: %s" % (result.new_state.name if result.new_state else "NONE"))
    assert result.ok, "takeoff was rejected from IDLE: %s" % result.reason


def test_router_accepts_align():
    print("\n3) DOES THE ROUTER ACCEPT 'align' FROM HOLD")
    result = CommandRouter().resolve('{"command": "align"}', "HOLD")
    check("command accepted", result.ok, "reason: %s" % (result.reason or "-"))
    assert result.ok, "align was rejected from HOLD: %s" % result.reason


def test_invalid_commands_are_rejected():
    print("\n4) ARE INVALID COMMANDS REJECTED (false-positive guard)")
    # To be sure the test measures anything at all: a router that accepts
    # every command would be useless.
    r1 = CommandRouter().resolve('{"command": "no_such_command"}', "IDLE")
    check("unknown command rejected", not r1.ok, r1.reason or "")
    r2 = CommandRouter().resolve('{"command": "takeoff"}', "DIVE")
    check("takeoff rejected from DIVE (allowed_from)", not r2.ok,
          r2.reason or "")


if __name__ == "__main__":
    test_every_state_can_be_constructed()
    test_router_accepts_takeoff()
    test_router_accepts_align()
    test_invalid_commands_are_rejected()
    print("\n%d/%d checks passed" % (sum(PASS), len(PASS)))
    sys.exit(0 if all(PASS) else 1)
