# telemetry.py — Telemetry data model and async store fed by MAVSDK streams.

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import logging
import copy
import math
import time

logger = logging.getLogger("TELEMETRY")

@dataclass
class TelemetryData:
    """
    Frozen snapshot of drone state at a single point in time.

    All fields default to safe/invalid values so the safety checker will
    reject a TelemetryData that has never been updated.
    """

    # Position
    latitude_deg: float = 0.0
    longitude_deg: float = 0.0
    abs_alt_m: float = 0.0
    rel_alt_m: float = 0.0

    # NED velocity
    vel_north_m_s: float = 0.0
    vel_east_m_s: float = 0.0
    vel_down_m_s: float = 0.0

    # Attitude / heading
    heading_deg: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0

    # Status
    is_armed: bool = False
    flight_mode: str = "UNKNOWN"

    # Timestamp of last update (monotonic clock)
    last_update_time: float = 0.0

    @property
    def age_s(self) -> float:
        """Seconds since the last telemetry update."""
        if self.last_update_time == 0.0:
            return float("inf")
        return time.monotonic() - self.last_update_time

    @property
    def groundspeed_m_s(self) -> float:
        """Horizontal speed computed from NED velocity (excludes vertical)."""
        return math.hypot(self.vel_north_m_s, self.vel_east_m_s)

class TelemetryStore:
    """
    Thread-safe(ish) container updated by async MAVSDK subscription tasks.

    Callers retrieve a shallow copy via get() so that no field mutates
    mid-tick while the FSM is processing.
    """

    def __init__(self) -> None:
        self._data = TelemetryData()

    # Update helpers 
    # Each ``update_*`` method is called from a dedicated asyncio task that
    # iterates an MAVSDK async generator.

    def update_position(self, latitude_deg: float, longitude_deg: float, abs_alt_m: float, rel_alt_m: float) -> None:
        self._data.latitude_deg = latitude_deg
        self._data.longitude_deg = longitude_deg
        self._data.abs_alt_m = abs_alt_m
        self._data.rel_alt_m = rel_alt_m
        self._data.last_update_time = time.monotonic()

    def update_velocity(self, vel_north: float, vel_east: float, vel_down: float) -> None:
        self._data.vel_north_m_s = vel_north
        self._data.vel_east_m_s = vel_east
        self._data.vel_down_m_s = vel_down
        self._data.last_update_time = time.monotonic()

    def update_heading(self, heading_deg: float) -> None:
        self._data.heading_deg = heading_deg
        self._data.last_update_time = time.monotonic()

    def update_armed(self, is_armed: bool) -> None:
        self._data.is_armed = is_armed
        self._data.last_update_time = time.monotonic()

    def update_flight_mode(self, mode: str) -> None:
        self._data.flight_mode = mode
        self._data.last_update_time = time.monotonic()

    def update_attitude(self, roll_deg: float, pitch_deg: float, heading_deg: float) -> None:
        self._data.roll_deg = roll_deg
        self._data.pitch_deg = pitch_deg
        self._data.heading_deg = heading_deg
        self._data.last_update_time = time.monotonic()

    # Read access
    def get(self) -> TelemetryData:
        """
        Return a shallow copy of the current telemetry snapshot.
        Prevents mid-tick mutation by the async updater tasks.
        """
        return copy.copy(self._data)
