# states/takeoff_state.py — Arm the vehicle and command takeoff.

from __future__ import annotations

from typing import TYPE_CHECKING
import logging
import time

from states.base_state import BaseState

# ⚠️ THIS IMPORT WAS MISSING, AND IT MADE TAKE-OFF IMPOSSIBLE.
#    __init__ uses config.TAKEOFF_ALTITUDE_M (below); without the import,
#    constructing TakeoffState() raised NameError. Because CommandRouter
#    wraps construction in try/except, that never crashed - it degraded into
#    a single warning line:
#        "Command rejected: Failed to instantiate TakeoffState:
#         name 'config' is not defined"
#    so the aircraft simply never took off, silently. The regression came in
#    when _target_alt_m moved from a literal into config and the import was
#    not added with it. tests/test_durum_kurulumu.py now locks this down by
#    instantiating every state.
import config

if TYPE_CHECKING:
    from processes.mission_controller import MissionController

logger = logging.getLogger("TAKEOFF")

# How long to wait between arm checks before giving up
_ARM_TIMEOUT_S = 10.0
_LOG_INTERVAL_S = 2.0

class TakeoffState(BaseState):
    """
    Arm the vehicle, wait until armed, then command takeoff.
    Once airborne, transition to HoldState.
    """

    name = "TAKEOFF"

    def __init__(self) -> None:
        super().__init__()
        # ⚠️ This used to be a literal (100) in the code. It moved into config
        #    because it is TIED to the dive chain: the rulebook (p.18)
        #    requires the dive to start at or above 100 m
        #    (DIVE_MIN_ENTRY_ALTITUDE_M) and the approach is flown at that
        #    altitude too (APPROACH_SAFE_ALTITUDE_M). All three have to move
        #    together.
        self._target_alt_m = config.TAKEOFF_ALTITUDE_M
        self._arm_requested: bool = False
        self._takeoff_requested: bool = False
        self._arm_request_time: float = 0.0
        self._last_log_time: float = 0.0

    async def on_enter(self, mission: MissionController) -> None:
        logger.info("Entering TAKEOFF — target altitude: %.1f m", self._target_alt_m)
        self._arm_requested = False
        self._takeoff_requested = False
        self._arm_request_time = 0.0
        self._last_log_time = 0.0

    async def on_exit(self, mission: MissionController) -> None:
        logger.info("Exiting TAKEOFF")

    async def update(self, mission: MissionController) -> None:
        tel = mission.telemetry.get()
        now = time.monotonic()

        # Step 1 — Arm if not already armed
        if not self._arm_requested:
            logger.info("Sending arm command …")
            await mission.vehicle.arm()
            self._arm_requested = True
            self._arm_request_time = now
            return

        # Step 2 — Wait until autopilot confirms armed
        if not tel.is_armed:
            if now - self._arm_request_time >= _ARM_TIMEOUT_S:
                logger.error("Vehicle did not arm within %.1fs — aborting", _ARM_TIMEOUT_S)
                await mission.vehicle.rtl()
                return

            if now - self._last_log_time >= _LOG_INTERVAL_S:
                logger.info("Waiting for arm confirmation … (%.1fs elapsed)", now - self._arm_request_time)
                self._last_log_time = now
            return

        # Step 3 — Arm confirmed; command takeoff once
        if not self._takeoff_requested:
            logger.info("Armed — commanding takeoff to %.1f m", self._target_alt_m)
            await mission.vehicle.takeoff(self._target_alt_m)
            self._takeoff_requested = True
            return

        # Step 4 — Wait until target altitude is approximately reached
        if tel.rel_alt_m >= self._target_alt_m * 0.95:
            from states.hold_state import HoldState

            logger.info("Target altitude reached (%.1f m) — transitioning to HOLD", tel.rel_alt_m)
            await mission._change_state(HoldState())