# states/hold_state.py — Command HOLD and wait for autopilot to confirm the mode.

from __future__ import annotations

from typing import TYPE_CHECKING
import logging
import time

from states.base_state import BaseState

if TYPE_CHECKING:
    from processes.mission_controller import MissionController

logger = logging.getLogger("HOLD")

_LOG_INTERVAL_S = 2.0
_HOLD_TIMEOUT_S = 10.0

class HoldState(BaseState):
    """
    Command the autopilot into HOLD/LOITER mode and wait for confirmation.
    Once the mode is confirmed, the state idles — a higher-level trigger
    (operator command, mission event, etc.) is expected to drive the next transition.
    """

    name = "HOLD"

    def __init__(self) -> None:
        super().__init__()
        self._hold_requested: bool = False
        self._hold_request_time: float = 0.0
        self._last_log_time: float = 0.0

    async def on_enter(self, mission: MissionController) -> None:
        logger.info("Entering HOLD")
        self._hold_requested = False
        self._hold_request_time = 0.0
        self._last_log_time = 0.0

    async def on_exit(self, mission: MissionController) -> None:
        logger.info("Exiting HOLD")

    async def update(self, mission: MissionController) -> None:
        tel = mission.telemetry.get()
        now = time.monotonic()

        # Step 1 — Send hold command once
        if not self._hold_requested:
            logger.info("Sending HOLD command …")
            await mission.vehicle.hold()
            self._hold_requested = True
            self._hold_request_time = now
            return

        # Step 2 — Wait for autopilot to confirm the mode
        if tel.flight_mode not in ("HOLD", "LOITER"):
            if now - self._hold_request_time >= _HOLD_TIMEOUT_S:
                logger.error("Autopilot did not enter HOLD within %.1fs — commanding RTL", _HOLD_TIMEOUT_S)
                await mission.vehicle.rtl()
                return

            if now - self._last_log_time >= _LOG_INTERVAL_S:
                logger.info("Waiting for HOLD confirmation — current mode: %s (%.1fs elapsed)",
                            tel.flight_mode, now - self._hold_request_time)
                self._last_log_time = now
            return

        # Step 3 — Holding confirmed, idle
        if now - self._last_log_time >= _LOG_INTERVAL_S:
            logger.debug("Holding at %.1f m — awaiting next command", tel.rel_alt_m)
            self._last_log_time = now