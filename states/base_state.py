# states/base_state.py — Abstract base class for all FSM states.

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from processes.mission_controller import MissionController

class BaseState(ABC):
    """
    Abstract base for every mission state.

    Subclasses must override ``update``. ``on_enter`` and ``on_exit``
    are optional lifecycle hooks with default no-op implementations.
    """

    name: str = "BASE"

    async def on_enter(self, mission: MissionController) -> None:
        """Called once when the FSM transitions into this state."""

    async def on_exit(self, mission: MissionController) -> None:
        """Called once when the FSM transitions out of this state."""

    @abstractmethod
    async def update(self, mission: MissionController) -> None:
        """Called every FSM tick while this state is active."""
        ...
