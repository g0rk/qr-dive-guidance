# states/dive_state.py — Safety-critical controlled nose-down dive using attitude control.

from __future__ import annotations

from typing import TYPE_CHECKING
from rich.live import Live
import logging
import time

from utils.geo_utils import ground_speed_m_s

from states.base_state import BaseState
from vehicle import VehicleCommandError

if TYPE_CHECKING:
    from processes.mission_controller import MissionController

import config

logger = logging.getLogger("DIVE")

class DiveState(BaseState):
    """
    Execute a controlled nose-down dive using attitude set-points.

    ⚠️  This is the highest-risk state in the mission.

    The attitude command must be sent **every single FSM tick** — MAVSDK
    offboard control reverts to the previous mode if commands stop arriving.

    Exit conditions (checked every tick, in priority order):
      1. Hard time limit  → AbortState
      2. Altitude floor   → PullUpState
    """

    name = "DIVE"

    def __init__(self) -> None:
        self._entry_time: float = 0.0
        self._entry_alt_m: float = 0.0
        self._live = Live("", refresh_per_second=10, transient=True)

    async def on_enter(self, mission: MissionController) -> None:
        self._live.start()
        tel = mission.telemetry.get()

        # Entry altitude validation — reject if too low
        if tel.rel_alt_m < config.DIVE_MIN_ENTRY_ALTITUDE_M:
            logger.error(
                "Dive entry rejected: alt %.1fm < min %.1fm",
                tel.rel_alt_m, config.DIVE_MIN_ENTRY_ALTITUDE_M,
            )
            raise ValueError("Insufficient altitude for dive")

        self._entry_time = time.monotonic()
        self._entry_alt_m = tel.rel_alt_m

        logger.info(
            "DIVE INITIATED — entry_alt=%.1fm, pitch=%.1f°, throttle=%.2f, "
            "pull_up_alt=%.1fm, max_duration=%.1fs",
            self._entry_alt_m,
            config.DIVE_PITCH_DEG,
            config.DIVE_THROTTLE,
            config.DIVE_PULL_UP_ALTITUDE_M,
            config.DIVE_MAX_DURATION_S,
        )

    async def on_exit(self, mission: MissionController) -> None:
        self._live.stop()
        elapsed = time.monotonic() - self._entry_time
        logger.info("Exiting DIVE after %.1fs", elapsed)

    async def update(self, mission: MissionController) -> None:
        tel = mission.telemetry.get()
        elapsed = time.monotonic() - self._entry_time

        # 1. Hard time limit — abort if dive exceeds max duration
        if elapsed > config.DIVE_MAX_DURATION_S:
            logger.error("DIVE timeout after %.1fs — aborting", elapsed)
            from states.abort_state import AbortState

            await mission._change_state(AbortState())
            return

        # 2. Altitude floor guard — trigger pull-up
        if tel.rel_alt_m <= config.DIVE_PULL_UP_ALTITUDE_M:
            logger.info(
                "Pull-up altitude reached (%.1fm). -> PULL_UP", tel.rel_alt_m
            )
            from states.pull_up_state import PullUpState

            await mission._change_state(PullUpState())
            return

        # 3. Send attitude command EVERY tick — do not skip
        try:
            await mission.vehicle.set_attitude(
                roll_deg=config.DIVE_ROLL_DEG,
                pitch_deg=config.DIVE_PITCH_DEG,
                yaw_rate_deg_s=0.0,
                thrust=config.DIVE_THROTTLE,
            )
        except VehicleCommandError as e:
            logger.error("Attitude command failed in DIVE: %s — aborting", e)
            from states.abort_state import AbortState

            await mission._change_state(AbortState())
            return

        dive_text = (
            f"[DIVE] alt={tel.rel_alt_m:.1f}m "
            f"elapsed={elapsed:.1f}s "
            f"pitch={tel.pitch_deg} "
            f"v_ground={ground_speed_m_s(tel.vel_north_m_s, tel.vel_east_m_s)} m/s "
            f"v_down={tel.vel_down_m_s} m/s"
        )

        self._live.update(dive_text)
        logger.info(dive_text, extra={"tick": True})
