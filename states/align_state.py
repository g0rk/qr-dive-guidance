from __future__ import annotations

from typing import TYPE_CHECKING
from rich.live import Live
import logging

from utils.geo_utils import angle_error_deg, bearing_deg, ground_speed_m_s, ground_track_deg

from states.base_state import BaseState

if TYPE_CHECKING:
    from processes.mission_controller import MissionController

import config

logger = logging.getLogger("LOITER_ALIGN")

class AlignState(BaseState):
    """
    While loitering, accumulate consecutive ticks where the aircraft
    ground track is aligned toward the target within threshold.

    Transition to ApproachState once ``LOITER_EXIT_REQUIRED_COUNT``
    consecutive aligned ticks are observed.
    """

    name = "LOITER_ALIGN"

    def __init__(self) -> None:
        super().__init__()
        self._ready_count: int = 0
        self._live = Live("", refresh_per_second=10, transient=True)

    async def on_enter(self, mission: MissionController) -> None:
        self._ready_count = 0
        self._live.start()
        logger.info("Entered LOITER_ALIGN — aligning toward target")

    async def on_exit(self, mission: MissionController) -> None:
        self._live.stop()
        logger.info("Exiting LOITER_ALIGN (ready_count=%d)", self._ready_count)

    async def update(self, mission: MissionController) -> None:
        tel = mission.telemetry.get()

        target_bearing = bearing_deg(
            tel.latitude_deg, tel.longitude_deg,
            config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG,
        )

        track = ground_track_deg(tel.vel_north_m_s, tel.vel_east_m_s)
        speed = ground_speed_m_s(tel.vel_north_m_s, tel.vel_east_m_s)
        error = angle_error_deg(target_bearing, track)

        if speed < config.MIN_GROUND_SPEED_M_S:
            self._ready_count = 0
            logger.warning(
                "Ground speed too low: %.1f m/s < %.1f m/s — resetting count",
                speed, config.MIN_GROUND_SPEED_M_S,
            )
            return

        if abs(error) <= config.LOITER_EXIT_ANGLE_THRESHOLD_DEG:
            self._ready_count += 1
        else:
            self._ready_count = 0

        align_text = (
            f"[cyan][ALIGN][/cyan] track={track:6.1f}°  target={target_bearing:6.1f}°  "
            f"error={error:+6.1f}°  v_ground={speed:5.1f}m/s  "
            f"count=[bold]{self._ready_count}/{config.LOITER_EXIT_REQUIRED_COUNT}[/bold]"
        )

        self._live.update(align_text)
        logger.info(align_text, extra={"tick": True})

        if self._ready_count >= config.LOITER_EXIT_REQUIRED_COUNT:
            from states.approach_state import ApproachState
            logger.info(
                "Alignment achieved (%d consecutive ticks) — transitioning to APPROACH",
                self._ready_count,
            )
            await mission._change_state(ApproachState())