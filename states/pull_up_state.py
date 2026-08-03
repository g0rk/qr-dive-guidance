# states/pull_up_state.py — Climb back to safe altitude after dive.

from __future__ import annotations

from typing import TYPE_CHECKING
from rich.live import Live
import logging

from states.base_state import BaseState
from vehicle import VehicleCommandError

if TYPE_CHECKING:
    from processes.mission_controller import MissionController

import config

logger = logging.getLogger("PULL_UP")

class PullUpState(BaseState):
    """
    Pull the nose up and climb back to safe altitude.

    Like DiveState, attitude commands must be sent **every single tick**
    — MAVSDK offboard reverts if commands stop.

    Exit condition: ``rel_alt_m >= PULL_UP_SAFE_ALTITUDE_M``
    → command HOLD and transition to WaitForLoiterState.
    """

    name = "PULL_UP"

    def __init__(self) -> None:
        self._live = Live("", refresh_per_second=10, transient=True)

    async def on_enter(self, mission: MissionController) -> None:
        self._live.start()
        tel = mission.telemetry.get()
        logger.info(
            "PULL_UP initiated at %.1fm — climbing to %.1fm",
            tel.rel_alt_m, config.PULL_UP_SAFE_ALTITUDE_M,
        )

    async def on_exit(self, mission: MissionController) -> None:
        mission.perception_command_queue.put_nowait({"cmd": "stop"})
        self._live.stop()
        logger.info("Exiting PULL_UP")

    async def update(self, mission: MissionController) -> None:
        tel = mission.telemetry.get()

        # Check if safe altitude reached
        if tel.rel_alt_m >= config.PULL_UP_SAFE_ALTITUDE_M:
            logger.info(
                "Safe altitude reached (%.1fm >= %.1fm) — mission complete",
                tel.rel_alt_m, config.PULL_UP_SAFE_ALTITUDE_M,
            )

            try:
                await mission.vehicle.hold()
            except VehicleCommandError as e:
                logger.error("HOLD command failed after pull-up: %s — aborting", e)
                from states.abort_state import AbortState

                await mission._change_state(AbortState())
                return

            from states.hold_state import HoldState

            await mission._change_state(HoldState())
            return

        # Send attitude command every tick — do not skip
        try:
            await mission.vehicle.set_attitude(
                roll_deg=0.0,
                pitch_deg=config.PULL_UP_PITCH_DEG,
                yaw_rate_deg_s=0.0,
                thrust=config.PULL_UP_THROTTLE,
            )
        except VehicleCommandError as e:
            logger.error("Attitude command failed in PULL_UP: %s — aborting", e)
            from states.abort_state import AbortState

            await mission._change_state(AbortState())
            return

        pull_up_text = (
            f"[PULL_UP] alt={tel.rel_alt_m:.1f}m "
            f"target={config.PULL_UP_SAFE_ALTITUDE_M:.0f}m"
        )

        self._live.update(pull_up_text)
        logger.info(pull_up_text, extra={"tick": True})