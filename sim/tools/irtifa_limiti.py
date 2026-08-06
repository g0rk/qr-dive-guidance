#!/usr/bin/env python3
"""From a flight log, report how many seconds were spent below a given altitude.

    python3 sim/tools/irtifa_limiti.py [flight.csv]

WHY THIS EXISTS
---------------
The rulebook (p.29) says the minimum and maximum flight altitudes will be
announced to the teams only after the competition site has been decided. So
the minimum altitude is NOT KNOWN YET.

The rulebook (p.21, kamikaze) says that dropping below the flight altitude
limit during a kamikaze run counts as leaving the permitted area.

The penalty structure:
    p.29  1-3 times, each SHORTER than 10 s   -> no penalty
          a 4th time (still under 10 s)       -> -200
    p.33  a single excursion LONGER than 10 s -> -200 AND disqualification

So what matters is not "did we go below the limit" but "HOW LONG did we stay
below it". This tool extracts that duration for a range of thresholds, so that
when the limit is finally announced, compliance can be checked without flying
anything again.

⚠️ The kamikaze can only score once per competition (p.19), so in a normal
   run there is a SINGLE short excursion per sortie - the 4-times rule has
   plenty of headroom.
"""
import csv
import sys

CSV = sys.argv[1] if len(sys.argv) > 1 else "/tmp/flight.csv"
DISQUALIFY_S = 10.0          # rulebook p.33
THRESHOLDS = (60, 50, 40, 35, 30, 25, 20, 15)


def intervals(rows, threshold):
    """Durations of the UNBROKEN stretches spent below `threshold`."""
    out, start, was_below = [], None, False
    for r in rows:
        below = float(r["rel_alt_m"]) < threshold
        t = float(r["t"])
        if below and not was_below:
            start = t
        elif not below and was_below:
            out.append(t - start)
        was_below = below
    if was_below and start is not None:
        out.append(float(rows[-1]["t"]) - start)
    return out


def main():
    try:
        everything = sorted(csv.DictReader(open(CSV)), key=lambda r: float(r["t"]))
    except OSError as e:
        print("could not read the log: %s" % e)
        return 1
    if not everything:
        print("not enough data")
        return 1

    # ⚠️ EXCLUDE THE TAKE-OFF CLIMB.
    #    The first version scanned the whole log and reported "118 seconds in
    #    violation" at the 60 m threshold -- because the aircraft HAS TO pass
    #    through 60 m on the way up. That is not an altitude violation, it is
    #    a normal climb. The rulebook defines the limit for the cruise phase.
    #    The analysis therefore starts at the moment the aircraft FIRST
    #    reaches mission altitude.
    MISSION_ALTITUDE = 100.0
    start_i = None
    for i, r in enumerate(everything):
        if float(r["rel_alt_m"]) >= MISSION_ALTITUDE:
            start_i = i
            break
    if start_i is None:
        print("the aircraft never reached %d m - there is no mission phase"
              % MISSION_ALTITUDE)
        return 1
    rows = everything[start_i:]
    print("  (analysis starts when %.1f m is first reached: t=%.1f s;"
          " the take-off climb is excluded)" % (MISSION_ALTITUDE, float(rows[0]["t"])))

    print("=" * 68)
    print("  ALTITUDE LIMIT COMPLIANCE  (%s)" % CSV)
    print("=" * 68)
    print("  lowest altitude: %.2f m" % min(float(r["rel_alt_m"]) for r in rows))
    print()
    print("  %-8s %-8s %-10s %s" % ("thresh", "count", "longest", "verdict"))
    print("  " + "-" * 52)

    for threshold in THRESHOLDS:
        a = intervals(rows, threshold)
        if not a:
            print("  %-8s %-8s %-10s %s" % ("%d m" % threshold, "0", "-",
                                            "never went below"))
            continue
        longest = max(a)
        if longest >= DISQUALIFY_S:
            verdict = "DISQUALIFICATION RISK (>= %.0f s)" % DISQUALIFY_S
        elif len(a) >= 4:
            verdict = "-200 (the 4th-time rule)"
        else:
            verdict = "safe"
        print("  %-8s %-8d %-10s %s" % (
            "%d m" % threshold, len(a), "%.2f s" % longest, verdict))

    print()
    print("  Rule: %.0f s in one go -> -200 AND disqualification;"
          "  a 4th short excursion -> -200" % DISQUALIFY_S)
    print("  Once the minimum flight altitude is announced, read off that row.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
