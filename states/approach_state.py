# states/approach_state.py — Fly toward the target via a ghost waypoint for straight-in approach.

from __future__ import annotations

from typing import TYPE_CHECKING
from rich.live import Live
import logging

from utils.geo_utils import bearing_deg, distance_m, point_from_bearing

from states.base_state import BaseState
from vehicle import VehicleCommandError

if TYPE_CHECKING:
    from processes.mission_controller import MissionController

import config

logger = logging.getLogger("APPROACH")

class ApproachState(BaseState):
    """
    Fly toward the target using a ghost waypoint placed behind the target
    to establish a stable straight-in approach path.

    On entry, computes the ghost waypoint and commands goto_location.
    Each tick, monitors distance to target. When within arming distance,
    validates altitude and transitions to DiveState.
    """

    name = "APPROACH"

    def __init__(self) -> None:
        super().__init__()
        self._command_sent: bool = False
        self._live = Live("", refresh_per_second=10, transient=True)

    async def on_enter(self, mission: MissionController) -> None:
        self._live.start()
        logger.info("Entered APPROACH — computing ghost waypoint")

        mission.perception_command_queue.put_nowait({"cmd": "set_mode", "mode": "qr"})
        mission.perception_command_queue.put_nowait({"cmd": "start"})

        tel = mission.telemetry.get()

        # Bearing from drone to target
        target_bearing = bearing_deg(
            tel.latitude_deg, tel.longitude_deg,
            config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG,
        )

        # Ghost point: APPROACH_GHOST_DISTANCE_M behind the target along the
        # reverse bearing (from target back toward drone).
        ghost_lat, ghost_lon = point_from_bearing(
            config.TARGET_LATITUDE_DEG,
            config.TARGET_LONGITUDE_DEG,
            target_bearing,
            config.APPROACH_GHOST_DISTANCE_M,
        )

        logger.info(
            "Ghost waypoint: (%.6f, %.6f) — %.0fm behind target on bearing %.1f°",
            ghost_lat, ghost_lon, config.APPROACH_GHOST_DISTANCE_M, target_bearing,
        )

        try:
            await mission.vehicle.goto_location(
                ghost_lat, ghost_lon, config.APPROACH_SAFE_ALTITUDE_M,
            )
            self._command_sent = True
        except VehicleCommandError as e:
            logger.error("Failed to command approach waypoint: %s — aborting", e)
            from states.abort_state import AbortState
            raise  # Let change_state catch it and fall back to AbortState

    async def on_exit(self, mission: MissionController) -> None:
        self._live.stop()
        logger.info("Exiting APPROACH")

    async def update(self, mission: MissionController) -> None:
        if not self._command_sent:
            return

        tel = mission.telemetry.get()

        dist = distance_m(
            tel.latitude_deg, tel.longitude_deg,
            config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG,
        )

        approach_text = (
            f"[APPROACH] dist={dist:7.1f}m "
            f"alt={tel.rel_alt_m:6.1f}m "
            f"limit={config.APPROACH_DIVE_ARM_DISTANCE_M:.0f}m"
        )

        self._live.update(approach_text)
        logger.info(approach_text, extra={"tick": True})

        # Check if within dive arming distance
        if dist <= config.APPROACH_DIVE_ARM_DISTANCE_M:
            # Validate altitude before committing to dive
            if tel.rel_alt_m < config.DIVE_MIN_ENTRY_ALTITUDE_M:
                logger.error(
                    "Refusing dive: altitude too low (%.1f m < %.1f m)",
                    tel.rel_alt_m, config.DIVE_MIN_ENTRY_ALTITUDE_M,
                )
                from states.abort_state import AbortState

                await mission._change_state(AbortState())
                return

            from states.dive_state import DiveState

            logger.info(
                "Within arming distance (%.1fm) — transitioning to DIVE",
                dist,
            )
            await mission._change_state(DiveState())