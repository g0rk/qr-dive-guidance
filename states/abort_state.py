# states/abort_state.py — Emergency fallback state. Commands RTL and stops the FSM.

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from states.base_state import BaseState

if TYPE_CHECKING:
    from processes.mission_controller import MissionController

logger = logging.getLogger("ABORT")

class AbortState(BaseState):
    """
    Emergency fallback — command RTL and halt the mission loop.

    If RTL fails, the vehicle may be uncontrolled. This is logged at
    CRITICAL level so it is impossible to miss.
    """

    name = "ABORT"

    async def on_enter(self, mission: MissionController) -> None:
        logger.critical("ABORT STATE — commanding RTL")
        try:
            await mission.vehicle.rtl()
        except Exception as e:
            logger.critical("RTL command failed: %s — vehicle may be uncontrolled", e)
        mission.running = False

    async def on_exit(self, mission: MissionController) -> None:
        logger.info("Exiting ABORT state")

    async def update(self, mission: MissionController) -> None:
        # The mission loop will exit because running is False.
        pass
