# states/dive_state.py — Safety-critical controlled nose-down dive using attitude control.

from __future__ import annotations

from typing import TYPE_CHECKING
from rich.live import Live
import logging
import time

from utils.geo_utils import ground_speed_m_s

from states.base_state import BaseState
from vehicle import VehicleCommandError

if TYPE_CHECKING:
    from processes.mission_controller import MissionController

import config

logger = logging.getLogger("DIVE")


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(val, hi))


class DiveState(BaseState):
    """
    Execute a controlled nose-down dive using attitude set-points.

    ⚠️  This is the highest-risk state in the mission.

    The attitude command must be sent **every single FSM tick** — MAVSDK
    offboard control reverts to the previous mode if commands stop arriving.

    Exit conditions (checked every tick, in priority order):
      1. Hard time limit  → AbortState
      2. Altitude floor   → PullUpState
    """

    name = "DIVE"

    def __init__(self) -> None:
        self._entry_time: float = 0.0
        self._entry_wall: float = 0.0     # duvar saati - kamikaze paketi icin
        self._entry_alt_m: float = 0.0
        self._live = Live("", refresh_per_second=10, transient=True)

    def _fresh_qr(self, mission: MissionController):
        """
        Yalnizca BU dalis sirasinda gorulmus ve BAYAT OLMAYAN QR'i dondur. [P4]

        Iki filtre de emniyet geregi:
          - t > _entry_time : yaklasma fazinda okunan eski bir QR dalisi
            daha baslamadan bitirmesin.
          - yas <= KAMIKAZE_QR_MAX_AGE_S : bayat konumla dalis duzeltmek,
            hedefin YANINA yonelmek demektir. Veri eskiyse kor dalisa donulur.
        """
        qr = getattr(mission, "qr_result", None)
        if not qr or not qr.get("data") or "t" not in qr or "error" not in qr:
            return None
        if qr["t"] <= self._entry_time:
            return None
        if (time.monotonic() - qr["t"]) > config.KAMIKAZE_QR_MAX_AGE_S:
            return None
        return qr

    async def on_enter(self, mission: MissionController) -> None:
        self._live.start()
        tel = mission.telemetry.get()

        # Entry altitude validation — reject if too low
        if tel.rel_alt_m < config.DIVE_MIN_ENTRY_ALTITUDE_M:
            logger.error(
                "Dive entry rejected: alt %.1fm < min %.1fm",
                tel.rel_alt_m, config.DIVE_MIN_ENTRY_ALTITUDE_M,
            )
            raise ValueError("Insufficient altitude for dive")

        self._entry_time = time.monotonic()
        # Duvar saati burada yakalanmali; sonradan geriye donuk hesaplamak
        # zaman kaymasina acik olur. Kamikaze paketinin baslangic zamani.
        self._entry_wall = time.time()
        self._entry_alt_m = tel.rel_alt_m

        logger.info(
            "DIVE INITIATED — entry_alt=%.1fm, pitch=%.1f°, throttle=%.2f, "
            "pull_up_alt=%.1fm, max_duration=%.1fs",
            self._entry_alt_m,
            config.DIVE_PITCH_DEG,
            config.DIVE_THROTTLE,
            config.DIVE_PULL_UP_ALTITUDE_M,
            config.DIVE_MAX_DURATION_S,
        )

    async def on_exit(self, mission: MissionController) -> None:
        self._live.stop()
        elapsed = time.monotonic() - self._entry_time
        logger.info("Exiting DIVE after %.1fs", elapsed)

    async def update(self, mission: MissionController) -> None:
        tel = mission.telemetry.get()
        elapsed = time.monotonic() - self._entry_time

        # 1. Hard time limit — abort if dive exceeds max duration
        if elapsed > config.DIVE_MAX_DURATION_S:
            logger.error("DIVE timeout after %.1fs — aborting", elapsed)
            from states.abort_state import AbortState

            await mission._change_state(AbortState())
            return

        # 2. Altitude floor guard — trigger pull-up
        if tel.rel_alt_m <= config.DIVE_PULL_UP_ALTITUDE_M:
            logger.info(
                "Pull-up altitude reached (%.1fm). -> PULL_UP", tel.rel_alt_m
            )
            from states.pull_up_state import PullUpState

            await mission._change_state(PullUpState())
            return

        qr = self._fresh_qr(mission)

        # 3. Gorev hedefi tamamlandi mi?  [P4]
        #    ⚠️ SARTNAME s.18: vurus tespiti icin QR'in OKUNMASI YETMEZ,
        #       "QR kod sinirlarinin TAMAMI Hedef Vurus Alani'nda olmalidir"
        #       ve "sinir tespit degerlendirmesi icin TOLERANS PAYI MEVCUT
        #       DEGILDIR". Bu yuzden gecis `in_av` sartina bagli. AV disinda
        #       okunan QR merkezlemede kullanilir (hedefe yonelmek icin) ama
        #       gorevi tamamlamis SAYILMAZ - erken pull-up puani kaybettirir.
        if qr is not None and config.KAMIKAZE_PULLUP_ON_QR and qr.get("in_av"):
            logger.info(
                "QR okundu ve AV ICINDE (%r) @ alt=%.1fm -> PULL_UP",
                qr.get("data"), tel.rel_alt_m,
            )
            mission.kamikaze_hit = {
                "dive_start_wall": self._entry_wall,
                "dive_end_wall":   time.time(),
                "qr_text":         qr.get("data"),
                "qr_box":          qr.get("box"),
                "entry_alt_m":     self._entry_alt_m,   # sartname: >=100 m kaniti
                "alt_m":           tel.rel_alt_m,
            }
            from states.pull_up_state import PullUpState

            await mission._change_state(PullUpState())
            return

        # 4. Attitude komutu - QR gorunuyorsa gorsel merkezleme, yoksa kor dalis
        roll_cmd = config.DIVE_ROLL_DEG
        pitch_cmd = config.DIVE_PITCH_DEG
        centering = "kor"

        if qr is not None and config.KAMIKAZE_QR_CENTERING:
            ex, ey = qr["error"]        # normalize [-1,+1]

            # ex > 0 (QR sagda) -> saga yat -> POZITIF roll
            roll_cmd = _clamp(
                ex * config.KAMIKAZE_QR_ROLL_GAIN,
                -config.KAMIKAZE_QR_MAX_ROLL_DEG,
                +config.KAMIKAZE_QR_MAX_ROLL_DEG,
            )
            # ey > 0 (QR asagida) -> burnu daha asagi -> pitch DAHA NEGATIF.
            # DIVE_PITCH_DEG etrafinda dar bir bantta sinirlanir.
            pitch_cmd = _clamp(
                config.DIVE_PITCH_DEG - ey * config.KAMIKAZE_QR_PITCH_GAIN,
                config.DIVE_PITCH_DEG - config.KAMIKAZE_QR_MAX_PITCH_DELTA_DEG,
                config.DIVE_PITCH_DEG + config.KAMIKAZE_QR_MAX_PITCH_DELTA_DEG,
            )
            centering = "QR ex=%+.2f ey=%+.2f in_av=%s" % (ex, ey, qr.get("in_av"))

        # Attitude komutu HER TICK gonderilmeli - MAVSDK offboard, komut akisi
        # kesilirse onceki moda doner.
        try:
            await mission.vehicle.set_attitude(
                roll_deg=roll_cmd,
                pitch_deg=pitch_cmd,
                yaw_rate_deg_s=0.0,
                thrust=config.DIVE_THROTTLE,
            )
        except VehicleCommandError as e:
            logger.error("Attitude command failed in DIVE: %s — aborting", e)
            from states.abort_state import AbortState

            await mission._change_state(AbortState())
            return

        dive_text = (
            f"[DIVE] alt={tel.rel_alt_m:.1f}m "
            f"elapsed={elapsed:.1f}s "
            f"pitch={tel.pitch_deg} "
            f"v_ground={ground_speed_m_s(tel.vel_north_m_s, tel.vel_east_m_s)} m/s "
            f"v_down={tel.vel_down_m_s} m/s"
        )

        self._live.update(dive_text)
        logger.info(dive_text, extra={"tick": True})
