# command_router.py — Incoming JSON command validator and state factory.
#
# Usage inside MissionController
#   router = CommandRouter()
#   result = router.resolve(raw_json, current_state_name)
#   if result.ok:
#       await self._change_state(result.new_state)
#   else:
#       logger.warning("Command rejected: %s", result.reason)

from __future__ import annotations

from dataclasses import dataclass

from typing import Type
import logging
import json

from states.base_state import BaseState

# Import every state that can be triggered via command.
# Add new states here as the mission grows.
from states.takeoff_state  import TakeoffState
from states.pursuit_state  import PursuitState
from states.align_state    import AlignState
from states.abort_state    import AbortState
from states.idle_state     import IdleState
from states.hold_state     import HoldState

logger = logging.getLogger("MISSION")

# WILDCARD can be used to allow transition from ANY state
_WILDCARD = "*"

# This table is for debug do not use in production
TRANSITION_TABLE: dict[str, dict] = {
    "takeoff": {
        "allowed_from": {"IDLE"},
        "target":        TakeoffState,
    },
    "hold": {
        "allowed_from": _WILDCARD,      # accepted from any active state
        "target":        HoldState,
    },
    "align": {
        "allowed_from": _WILDCARD,      # accepted from any active state
        "target":        AlignState,
    },
    "idle": {
        "allowed_from": _WILDCARD,      # accepted from any active state
        "target":        IdleState,
    },
    "abort": {
        "allowed_from": _WILDCARD,      # accepted from any active state
        "target":        AbortState,
    },
    "pursuit": {
        "allowed_from": _WILDCARD,
        "target": PursuitState,
    },
}

@dataclass
class RouteResult:
    ok:        bool
    reason:    str | None = None
    new_state: BaseState | None = None

class CommandRouter:
    def resolve(self, raw: str, current_state_name: str) -> RouteResult:
        # 1. Parse JSON
        try:
            data = json.loads(raw)
        except json.JSONDecodeError as exc:
            return RouteResult(ok=False, reason=f"Invalid JSON: {exc}")

        if not isinstance(data, dict):
            return RouteResult(ok=False, reason="Command must be a JSON object")

        # 2. Extract command name
        command = data.get("command")
        if not command:
            return RouteResult(ok=False, reason="Missing 'command' field")

        command = str(command).lower().strip()

        # 3. Look up transition table
        entry = TRANSITION_TABLE.get(command)
        if entry is None:
            return RouteResult(ok=False, reason=f"Unknown command: '{command}'")

        # 4. Check allowed_from
        allowed_from = entry["allowed_from"]
        if allowed_from != _WILDCARD:
            if current_state_name not in allowed_from:
                return RouteResult(
                    ok=False,
                    reason=(
                        f"Command '{command}' is not allowed from state '{current_state_name}'. Allowed from: {sorted(allowed_from)}"
                    ),
                )

        # 5. Instantiate target state
        target_cls: Type[BaseState] = entry["target"]
        try:
            new_state = target_cls()
        except Exception as exc:
            return RouteResult(
                ok=False,
                reason=f"Failed to instantiate {target_cls.__name__}: {exc}",
            )

        logger.info(
            "Command '%s' accepted from state '%s' → %s",
            command, current_state_name, target_cls.__name__,
        )
        return RouteResult(ok=True, new_state=new_state)