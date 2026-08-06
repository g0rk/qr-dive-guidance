# states/pursuit_state.py — Close on a moving target using offboard attitude.
#
# Every tick:
#   1. Read the target position from mission.target
#   2. Compute the bearing from our own position to the target
#   3. Turn the bearing error into a roll set-point (bank-to-turn for yaw)
#   4. Turn the altitude error into a pitch set-point, holding TARGET_ALT_M
#   5. Push forward at a fixed throttle (config.PURSUIT_THROTTLE)
#
# There is no exit condition — the state waits for an external command
# (hold, abort, and so on).

from __future__ import annotations

import math
import time
import logging
from typing import TYPE_CHECKING

from rich.live import Live

from states.base_state import BaseState
from vehicle import VehicleCommandError

if TYPE_CHECKING:
    from processes.mission_controller import MissionController

import config

logger = logging.getLogger("PURSUIT")

# Fixed altitude target (metres). Move this into config.py if it needs tuning.
TARGET_ALT_M = 100.0


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Bearing from one GPS point to another, in degrees (0-360)."""
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    d_lon  = math.radians(lon2 - lon1)

    x = math.sin(d_lon) * math.cos(lat2_r)
    y = (math.cos(lat1_r) * math.sin(lat2_r)
         - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(d_lon))

    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360.0) % 360.0


def _angle_diff(a: float, b: float) -> float:
    """Shortest signed angle from a to b, in (-180, +180]."""
    diff = (b - a + 180.0) % 360.0 - 180.0
    return diff


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle surface distance between two points, in metres."""
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lam = math.radians(lon2 - lon1)
    a = math.sin(d_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _alt_hold_pitch(own) -> tuple[float, float]:
    """Pitch set-point that holds our own altitude at TARGET_ALT_M.

    NOTE: the field name `own.rel_alt_m` depends on the Telemetry class in
    use (it might be relative_altitude_m, alt_m and so on) — adjust it if
    that class changes.
    """
    current_alt = own.rel_alt_m
    alt_error = TARGET_ALT_M - current_alt  # positive: we are low and must climb

    pitch_cmd = _clamp(
        config.PURSUIT_BASE_PITCH_DEG + alt_error * config.PURSUIT_PITCH_GAIN,
        config.PURSUIT_MIN_PITCH_DEG,
        config.PURSUIT_MAX_PITCH_DEG,
    )
    return pitch_cmd, alt_error


class PursuitState(BaseState):
    """
    Offboard attitude state that steers a fixed-wing aircraft at a target.

    Control strategy:
      - Yaw: bearing error -> proportional roll set-point (bank-to-turn)
      - Altitude: proportional pitch set-point holding TARGET_ALT_M
      - Speed: fixed throttle (config.PURSUIT_THROTTLE) — raise that constant
        in config.py if the intercept needs to be faster.

    Format of mission.target (matches target_stream.py):
        {"type": "target", "lat": float, "lon": float, "alt": float, ...}
    """

    name = "PURSUIT"

    def __init__(self) -> None:
        self._entry_time: float = 0.0
        self._live = Live("", refresh_per_second=10, transient=True)

    async def on_enter(self, mission: MissionController) -> None:
        self._live.start()
        self._entry_time = time.monotonic()

        mission.perception_command_queue.put_nowait({"cmd": "set_mode", "mode": "yolo"})
        mission.perception_command_queue.put_nowait({"cmd": "start"})

        if mission.target is None:
            logger.warning("PURSUIT entered but mission.target is None — will wait for target")

        logger.info("PURSUIT started, target altitude=%.1fm", TARGET_ALT_M)

    async def on_exit(self, mission: MissionController) -> None:
        self._live.stop()
        elapsed = time.monotonic() - self._entry_time
        logger.info("Exiting PURSUIT after %.1fs", elapsed)

    async def update(self, mission: MissionController) -> None:
        tel = mission.target            # the target's telemetry
        own = mission.telemetry.get()   # our own telemetry
        elapsed = time.monotonic() - self._entry_time

        # No target yet: hold altitude, fly straight, wait for one to arrive.
        if tel is None:
            # await self._send_level(mission, own)  # level-flight helper
            self._live.update("[PURSUIT] Waiting for a target, flying level...")
            return

        # Bearing and distance to the target
        bearing = _bearing_deg(own.latitude_deg, own.longitude_deg, tel["lat"], tel["lon"])
        distance_m = _haversine_m(own.latitude_deg, own.longitude_deg, tel["lat"], tel["lon"])

        # Bearing error against our current heading -> roll set-point
        yaw_error = _angle_diff(own.heading_deg, bearing)

        # 1. ROLL — turn toward the target (bank-to-turn).
        #    The error is scaled by config.PURSUIT_ROLL_GAIN.
        roll_cmd = _clamp(
            yaw_error * config.PURSUIT_ROLL_GAIN,
            config.PURSUIT_MIN_ROLL_DEG,
            config.PURSUIT_MAX_ROLL_DEG
        )

        # 2. PITCH — hold altitude.
        pitch_cmd, alt_error = _alt_hold_pitch(own)

        # Status line for the live view
        status_msg = (
            f"[PURSUIT] target: {distance_m:.1f}m | "
            f"AltErr: {alt_error:.1f}m -> Pitch: {pitch_cmd:.1f}° | "
            f"YawErr: {yaw_error:.1f}° -> Roll: {roll_cmd:.1f}°"
        )
        self._live.update(status_msg)

        # 3. Send the offboard command.
        try:
            # On a fixed wing the turn comes from roll, so the yaw rate is
            # left at zero.
            await mission.vehicle.set_attitude(
                roll_deg=roll_cmd,
                pitch_deg=pitch_cmd,
                yaw_rate_deg_s=0.0,  # bank-to-turn, so no yaw rate
                thrust=config.PURSUIT_THROTTLE
            )

        except VehicleCommandError as e:
            logger.error("Offboard command failed: %s", e)
        except Exception as e:
            logger.error("Unexpected error in PURSUIT: %s", e)

def _clamp(val: float, min_val: float, max_val: float) -> float:
    """Clamp a value between min and max."""
    return max(min_val, min(val, max_val))
