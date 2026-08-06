# tests/test_server_time.py - the competition server's clock
#
#     python3 tests/test_server_time.py
#
# ⚠️ THE MOST CRITICAL BEHAVIOUR: produce NO timestamp when unsynced.
#
#    Falling back to local time silently produces numbers that pass every
#    type check, look completely normal, and INVALIDATE THE ENTIRE RECORDING.
#    The rulebook (p.13) says footage that carries no server time, or a
#    different time, will not be evaluated. In other words: you fly a perfect
#    sortie and score nothing.
#
#    Failing LOUDLY on the first attempt is recoverable; a quiet wrong number
#    is not. Locking that refusal in is this test's real job.

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from server_time import ServerClock, ServerClockError, ServerTime

PASS = []


def check(label, cond, detail=""):
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "PASS" if cond else "FAIL", detail))
    assert cond, "%s  %s" % (label, detail)


def test_refuses_when_unsynced():
    print("\n1) REFUSES WHEN UNSYNCED (the actual safety feature)")
    c = ServerClock()
    check("not synced at construction", not c.synced)
    check("age_s is None", c.age_s is None)
    for name, call in (("now()", lambda: c.now()),
                       ("at()", lambda: c.at(time.monotonic()))):
        try:
            call()
            check("%s refused" % name, False, "it silently produced a value!")
        except ServerClockError as e:
            check("%s refused" % name, True, str(e)[:46])


def test_conversion_is_correct():
    print("\n2) CONVERSION IS CORRECT")
    c = ServerClock()
    t0 = time.monotonic()
    c.sync(gun=14, saat=11, dakika=29, saniye=4, milisaniye=653,
           received_monotonic=t0)
    check("ready after sync", c.synced)

    s = c.at(t0)
    check("the sync instant itself round-trips exactly",
          (s.gun, s.saat, s.dakika, s.saniye, s.milisaniye)
          == (14, 11, 29, 4, 653), str(s))

    # 2.5 seconds later
    s2 = c.at(t0 + 2.5)
    check("2.5 s later is correct", (s2.saniye, s2.milisaniye) == (7, 153), str(s2))

    # Minute and hour rollover
    s3 = c.at(t0 + 31.0 * 60.0)
    check("31 minutes later rolls the hour over",
          (s3.saat, s3.dakika) == (12, 0), str(s3))


def test_converting_a_past_instant():
    print("\n3) CONVERTING A PAST INSTANT (what the mission packet needs)")
    # ⚠️ The dive-end instant is captured with time.monotonic() while the
    #    aircraft is still diving, but the packet is assembled seconds later.
    #    Converting at send time would report the moment the PACKET was BUILT,
    #    and the rulebook's +-1 s window would then land nowhere near the
    #    frames that actually matter.
    c = ServerClock()
    t0 = time.monotonic()
    c.sync(gun=14, saat=11, dakika=29, saniye=4, milisaniye=653,
           received_monotonic=t0)
    t_dive_end = t0 + 10.0
    t_packet = t0 + 10.8            # the packet is built 800 ms later

    correct = c.at(t_dive_end)
    wrong = c.at(t_packet)
    delta_ms = ((wrong.saniye - correct.saniye) * 1000
                + (wrong.milisaniye - correct.milisaniye))
    check("the past instant and the packet instant DIFFER", abs(delta_ms - 800) < 5,
          "%d ms apart" % delta_ms)
    check("the correct one is the dive end (14.653)",
          (correct.saniye, correct.milisaniye) == (14, 653), str(correct))


def test_packet_format():
    print("\n4) PACKET FORMAT (per the communication document)")
    c = ServerClock()
    c.sync(gun=14, saat=11, dakika=41, saniye=3, milisaniye=141)
    d = c.now().as_dict()
    check("fields are complete and correctly named",
          sorted(d) == ["dakika", "gun", "milisaniye", "saat", "saniye"],
          str(sorted(d)))
    check("all values are int", all(isinstance(v, int) for v in d.values()))
    # ⚠️ There is NO month and NO year - that is the protocol. Inventing them
    #    is an invitation to error.
    check("no month/year field was INVENTED",
          "ay" not in d and "yil" not in d and "year" not in d)
    check("overlay text has millisecond precision",
          len(ServerTime(14, 11, 41, 3, 141).overlay_text()) == 12,
          ServerTime(14, 11, 41, 3, 141).overlay_text())


def test_invalid_input():
    print("\n5) INVALID INPUT IS REJECTED (false-positive guard)")
    c = ServerClock()
    for name, kw in (("hour 24", dict(gun=1, saat=24, dakika=0, saniye=0, milisaniye=0)),
                     ("minute 60", dict(gun=1, saat=0, dakika=60, saniye=0, milisaniye=0)),
                     ("millisecond 1000", dict(gun=1, saat=0, dakika=0, saniye=0, milisaniye=1000)),
                     ("day 0", dict(gun=0, saat=0, dakika=0, saniye=0, milisaniye=0))):
        try:
            c.sync(**kw)
            check("%s rejected" % name, False, "it was accepted!")
        except ValueError:
            check("%s rejected" % name, True)
    check("invalid syncs did NOT corrupt the clock", not c.synced)


def test_resync():
    print("\n6) RE-SYNCING UPDATES THE OFFSET")
    c = ServerClock()
    t0 = time.monotonic()
    c.sync(gun=1, saat=10, dakika=0, saniye=0, milisaniye=0, received_monotonic=t0)
    first = c.status()["offset_s"]
    # The server turned out to be 2 seconds ahead (a latency correction).
    c.sync(gun=1, saat=10, dakika=0, saniye=2, milisaniye=0, received_monotonic=t0)
    second = c.status()["offset_s"]
    check("offset updated", abs((second - first) - 2.0) < 1e-6,
          "%.3f -> %.3f" % (first, second))
    check("sync counter incremented", c.status()["sync_count"] == 2)


if __name__ == "__main__":
    test_refuses_when_unsynced()
    test_conversion_is_correct()
    test_converting_a_past_instant()
    test_packet_format()
    test_invalid_input()
    test_resync()
    print("\n%d/%d checks passed" % (sum(PASS), len(PASS)))
    sys.exit(0 if all(PASS) else 1)
