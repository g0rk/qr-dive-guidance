"""Competition server clock: offset tracking and timestamp formatting.

Standalone: standard library only. No autopilot, no network client. The
transport (HTTP, serial, whatever) lives elsewhere and simply feeds
readings in through `sync()`.

WHY THIS EXISTS
---------------
Every packet sent to the competition server carries a timestamp in the
SERVER's clock, not ours. Two independent failure modes follow:

1. A judge video that shows the wrong clock is not evaluated at all.
   The rulebook is blunt about it: recordings without the server clock, or
   showing a different clock, are discarded. A mission can be flown
   perfectly and score nothing.

2. A kamikaze hit is validated in a +-1 second window around the reported
   dive-end time. An offset error larger than a second moves that window
   off the frames that actually show the target.

THE DESIGN RULE THAT MATTERS
----------------------------
If the offset has never been established, this module REFUSES to produce a
timestamp. It does not fall back to local time.

That refusal is the whole point. A missing offset that silently degrades
to local time produces timestamps that look completely normal, pass every
type check, and invalidate the entire recording. A loud failure at the
first attempt is recoverable; a quiet wrong number is not.

CLOCK CHOICE
------------
The offset is held against `time.monotonic()`, not `time.time()`. Wall
clock can jump: NTP correction, manual change, daylight saving. A jump
mid-flight would shift every subsequent timestamp with no indication that
anything happened. Monotonic time only moves forward at a steady rate.

FORMAT
------
The server uses day-of-month plus time, with no month and no year:

    {"gun": 14, "saat": 11, "dakika": 29, "saniye": 4, "milisaniye": 653}

Field names are part of the wire protocol and stay in Turkish.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

__all__ = ["ServerClockError", "ServerTime", "ServerClock"]


class ServerClockError(RuntimeError):
    """Raised when a timestamp is requested before the clock is synced."""


@dataclass(frozen=True)
class ServerTime:
    """One instant in the server's clock."""

    gun: int
    saat: int
    dakika: int
    saniye: int
    milisaniye: int

    def as_dict(self) -> dict:
        """Exactly the shape the server expects in a packet."""
        return {
            "gun": self.gun,
            "saat": self.saat,
            "dakika": self.dakika,
            "saniye": self.saniye,
            "milisaniye": self.milisaniye,
        }

    def __str__(self) -> str:
        return "%02d %02d:%02d:%02d.%03d" % (
            self.gun, self.saat, self.dakika, self.saniye, self.milisaniye)

    def overlay_text(self) -> str:
        """Millisecond-precision text for the top-right of the judge video."""
        return "%02d:%02d:%02d.%03d" % (
            self.saat, self.dakika, self.saniye, self.milisaniye)


# Seconds in a day, used to wrap around midnight.
_DAY_S = 86400.0


class ServerClock:
    """Tracks the offset between the local monotonic clock and server time.

    Typical use:

        clock = ServerClock()
        # ... ground station reads the server and calls:
        clock.sync(gun=14, saat=11, dakika=29, saniye=4, milisaniye=653)
        # ... later, anywhere:
        stamp = clock.now()

    Thread-safe: the mission loop, the recorder and the packet builder all
    read it from different threads.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        # Server seconds-since-midnight-of-`_gun` minus local monotonic.
        self._offset: float | None = None
        self._gun: int | None = None
        self._last_sync_monotonic: float | None = None
        self._sync_count = 0

    # ------------------------------------------------------------ sync ---
    def sync(self, gun: int, saat: int, dakika: int, saniye: int,
             milisaniye: int, received_monotonic: float | None = None) -> None:
        """Record a server reading and recompute the offset.

        `received_monotonic` is the local `time.monotonic()` value captured
        as close as possible to the moment the reading arrived. Pass it
        explicitly when the reading travelled through a queue or a socket;
        using "now" for a reading that is already 200 ms old bakes that
        200 ms into every timestamp afterwards.
        """
        for ad, deger, ust in (("saat", saat, 23), ("dakika", dakika, 59),
                               ("saniye", saniye, 59),
                               ("milisaniye", milisaniye, 999)):
            if not 0 <= deger <= ust:
                raise ValueError("%s out of range: %r" % (ad, deger))
        if not 1 <= gun <= 31:
            raise ValueError("gun out of range: %r" % gun)

        t_local = time.monotonic() if received_monotonic is None else received_monotonic
        server_s = saat * 3600.0 + dakika * 60.0 + saniye + milisaniye / 1000.0
        with self._lock:
            self._offset = server_s - t_local
            self._gun = gun
            self._last_sync_monotonic = t_local
            self._sync_count += 1

    # ----------------------------------------------------------- query ---
    @property
    def synced(self) -> bool:
        with self._lock:
            return self._offset is not None

    @property
    def age_s(self) -> float | None:
        """Seconds since the last sync, or None if never synced."""
        with self._lock:
            if self._last_sync_monotonic is None:
                return None
            return time.monotonic() - self._last_sync_monotonic

    def now(self) -> ServerTime:
        """Server time right now.

        Raises `ServerClockError` if never synced -- deliberately. See the
        module docstring.
        """
        return self.at(time.monotonic())

    def at(self, monotonic_s: float) -> ServerTime:
        """Server time corresponding to a past local monotonic reading.

        This is the one that matters for the kamikaze packet: the dive-end
        instant is captured with `time.monotonic()` while the aircraft is
        still diving, and converted afterwards. Converting at send time
        instead would report the moment the packet was built, which can be
        hundreds of milliseconds later.
        """
        with self._lock:
            offset, gun = self._offset, self._gun
        if offset is None or gun is None:
            raise ServerClockError(
                "server clock never synced -- refusing to invent a timestamp. "
                "Call sync() with a server reading first."
            )

        total = monotonic_s + offset
        gun_kaymasi, gun_ici = divmod(total, _DAY_S)
        # Midnight rollover: the server sends no month, so day is advanced
        # locally. Wrapping past the 31st is not modelled -- a competition
        # round does not span a month boundary, and inventing calendar
        # arithmetic from a month-less protocol would be guesswork.
        yeni_gun = gun + int(gun_kaymasi)

        saat, kalan = divmod(gun_ici, 3600.0)
        dakika, saniye_f = divmod(kalan, 60.0)
        saniye = int(saniye_f)
        milisaniye = int(round((saniye_f - saniye) * 1000.0))
        if milisaniye == 1000:            # rounding can push it over
            milisaniye = 0
            saniye += 1
        return ServerTime(gun=yeni_gun, saat=int(saat), dakika=int(dakika),
                          saniye=saniye, milisaniye=milisaniye)

    # ------------------------------------------------------------ info ---
    def status(self) -> dict:
        """Diagnostics -- surface this in the ground station UI."""
        with self._lock:
            return {
                "synced": self._offset is not None,
                "sync_count": self._sync_count,
                "offset_s": self._offset,
                "age_s": (None if self._last_sync_monotonic is None
                          else time.monotonic() - self._last_sync_monotonic),
            }


# ------------------------------------------------------------------ demo ---
if __name__ == "__main__":
    clock = ServerClock()

    try:
        clock.now()
    except ServerClockError as e:
        print("before sync ->", e)

    clock.sync(gun=14, saat=11, dakika=29, saniye=4, milisaniye=653)
    print("after sync  ->", clock.now())
    print("packet      ->", clock.now().as_dict())
    print("overlay     ->", clock.now().overlay_text())

    # Converting a past instant, the way the kamikaze packet needs to.
    t_dive_end = time.monotonic()
    time.sleep(0.25)
    print("dive end captured 0.25 s ago ->", clock.at(t_dive_end))
    print("status      ->", clock.status())
