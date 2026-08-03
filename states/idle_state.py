# states/idle_state.py — Benign no-op state for manual/ground testing.

from __future__ import annotations

from typing import TYPE_CHECKING
import logging

from states.base_state import BaseState

if TYPE_CHECKING:
    from processes.mission_controller import MissionController

logger = logging.getLogger("IDLE")

class IdleState(BaseState):
    """
    No-op state. The vehicle remains in its current flight mode.

    Used for manual/ground testing or as a terminal state when no
    further autonomous action is required.
    """

    name = "IDLE"

    async def on_enter(self, mission: MissionController) -> None:
        logger.info("Entered IDLE state")

    async def on_exit(self, mission: MissionController) -> None:
        logger.info("Exiting IDLE state")

    async def update(self, mission: MissionController) -> None:
        pass  # Intentional no-op
