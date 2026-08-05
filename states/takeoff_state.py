# states/takeoff_state.py — Arm the vehicle and command takeoff.

from __future__ import annotations

from typing import TYPE_CHECKING
import logging
import time

from states.base_state import BaseState

# ⚠️ BU IMPORT EKSIKTI ve UCAGIN KALKMASINI TAMAMEN ENGELLIYORDU.
#    __init__ icinde config.TAKEOFF_ALTITUDE_M kullaniliyor (asagida);
#    import olmayinca TakeoffState() kurulurken NameError atiyordu.
#    CommandRouter kurulumu try/except ile sardigi icin cokme, sadece
#    "Command rejected: Failed to instantiate TakeoffState: name 'config'
#    is not defined" seklinde bir UYARIYA donusuyordu -> sessiz kaliyordu.
#    Gerileme kaynagi: 4ec2179'da _target_alt_m koddan config'e tasindi,
#    import eklenmedi. tests/test_durum_kurulumu.py bunu kilitliyor.
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
        # ⚠️ Eskiden koda gomuluydu (100). Dalis zinciriyle BAGLI oldugu icin
        #    config'e tasindi: sartname s.18 dalisa >=100 m'den baslamayi sart
        #    kosuyor (DIVE_MIN_ENTRY_ALTITUDE_M) ve yaklasma da bu irtifada
        #    yapiliyor (APPROACH_SAFE_ALTITUDE_M). Ucu birlikte degismeli.
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