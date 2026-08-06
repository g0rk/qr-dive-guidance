# states/dive_state.py — Safety-critical controlled nose-down dive using attitude control.

from __future__ import annotations

from typing import TYPE_CHECKING
from rich.live import Live
import logging
import time

from utils.geo_utils import (
    ground_speed_m_s,
    bearing_deg,
    angle_error_deg,
    distance_m,
)

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
        self._entry_wall: float = 0.0     # wall clock - needed for the mission packet
        self._entry_alt_m: float = 0.0
        self._live = Live("", refresh_per_second=10, transient=True)
        # State for "keep going after the QR is read":
        self._qr_hit: dict | None = None   # the LATEST valid (in target area) detection
        self._qr_hit_count: int = 0        # how many frames produced a valid detection
        self._qr_first_alt_m: float = 0.0  # altitude of the first valid detection

    def _fresh_qr(self, mission: MissionController):
        """
        Return a QR seen DURING THIS DIVE that is not STALE.

        Both filters exist for safety:
          - t > _entry_time : an old QR read during the approach must not end
            a dive that has barely started.
          - age <= KAMIKAZE_QR_MAX_AGE_S : correcting a dive from a stale
            position means steering at where the target USED to be. When the
            data is old we fall back to a blind dive.
        """
        qr = getattr(mission, "qr_result", None)
        if not qr or not qr.get("data") or "t" not in qr or "error" not in qr:
            return None
        if qr["t"] <= self._entry_time:
            return None
        if (time.monotonic() - qr["t"]) > config.KAMIKAZE_QR_MAX_AGE_S:
            return None
        return qr

    def _gps_roll(self, tel):
        """Lateral guidance when no QR is visible: bearing error -> roll.

        WHY IT EXISTS: the dive used to hold a fixed attitude (roll=0) and
        missed the target by 42.5 m (measured 2026-08-05). The QR pad is only
        2x2 m, so it never entered the frame at all - which meant the visual
        centering could never engage either. Chicken and egg.

        Returns: (roll_cmd, bearing_error_deg, distance_m), or
        (None, None, None) when it cannot be computed.

        ⚠️ LIMITS: in a steep dive, roll does not change heading as directly
           as it does in level flight - with the nose 55 degrees down the lift
           vector turns closer to sideways. So the gain and the limit have to
           be tuned BY MEASUREMENT, not from theory. The initial values are a
           starting point, nothing more.
        """
        if not config.KAMIKAZE_GPS_GUIDANCE:
            return None, None, None

        # ⚠️ DEFENSIVE READS. Raising an AttributeError over a missing field
        #    IN THE MIDDLE OF THE DIVE is unacceptable: update() would crash
        #    and the aircraft would be left without commands. If a field is
        #    absent we FALL BACK to a blind dive rather than crash.
        lat = getattr(tel, "latitude_deg", None)
        lon = getattr(tel, "longitude_deg", None)
        hdg = getattr(tel, "heading_deg", None)
        if lat is None or lon is None or hdg is None:
            return None, None, None

        # Is the telemetry valid? A 0/0 position does not mean "Gulf of
        # Guinea", it means "no data" - computing a bearing from it would aim
        # the aircraft at Africa.
        if not lat and not lon:
            return None, None, None

        distance = distance_m(
            lat, lon,
            config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG,
        )
        # ⚠️ Very close in, the bearing stops meaning anything: within a few
        #    metres a tiny position error swings it by 180 degrees and the
        #    aircraft banks hard at the last moment. No correction is applied
        #    inside that radius.
        if distance < config.KAMIKAZE_GPS_MIN_DISTANCE_M:
            return None, None, distance

        target_bearing = bearing_deg(
            lat, lon,
            config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG,
        )
        error = angle_error_deg(target_bearing, hdg)
        roll = _clamp(
            error * config.KAMIKAZE_GPS_ROLL_GAIN,
            -config.KAMIKAZE_GPS_MAX_ROLL_DEG,
            +config.KAMIKAZE_GPS_MAX_ROLL_DEG,
        )
        return roll, error, distance

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
        # The wall clock has to be captured here; reconstructing it afterwards
        # is open to drift. This is the mission packet's start time.
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
            # ⚠️ IF WE REACH THE FLOOR HOLDING A VALID DETECTION, DO NOT LOSE
            #    THE PACKET. Normally the 35 m continue threshold (3b) fires
            #    first. But in a fast descent a single tick can jump from 36 m
            #    to 29 m, and then THIS branch runs first. Without stamping
            #    the packet here too, a QR that was actually read would be
            #    thrown away in silence.
            if self._qr_hit is not None:
                self._qr_hit["dive_end_wall"] = time.time()
                self._qr_hit["qr_frames"] = self._qr_hit_count
                self._qr_hit["qr_first_alt_m"] = self._qr_first_alt_m
                mission.kamikaze_hit = self._qr_hit
                logger.info(
                    "Altitude floor (%.1fm) reached BEFORE the continue "
                    "threshold -- packet recorded anyway (%d valid frames, QR=%r)",
                    tel.rel_alt_m, self._qr_hit_count,
                    self._qr_hit.get("qr_text"),
                )
            else:
                logger.info(
                    "Pull-up altitude reached (%.1fm), QR NOT READ. -> PULL_UP",
                    tel.rel_alt_m,
                )
            from states.pull_up_state import PullUpState

            await mission._change_state(PullUpState())
            return

        qr = self._fresh_qr(mission)

        # 3. Has the mission objective been met?
        #    ⚠️ The rulebook (p.18) is explicit: DECODING the QR is not
        #       enough. "The QR code's boundaries must lie ENTIRELY within the
        #       target area", and "no tolerance is allowed for the boundary
        #       assessment". So the transition is gated on `in_av`. A QR read
        #       outside the target area is still used for centering (to steer
        #       toward it) but does NOT count as mission complete - pulling up
        #       early on one would throw the score away.
        # 3a. RECORD THE VALID DETECTION, but do not exit yet.
        #     Each new detection overwrites the previous one: the QR grows as
        #     the aircraft descends, so the LAST detection is the most
        #     reliable.
        if qr is not None and qr.get("in_av"):
            if self._qr_hit is None:
                self._qr_first_alt_m = tel.rel_alt_m
                logger.info(
                    "QR read and INSIDE the target area (%r) @ alt=%.1fm -- "
                    "continuing down to %.1f m (to collect more valid frames)",
                    qr.get("data"), tel.rel_alt_m,
                    config.KAMIKAZE_QR_CONTINUE_ALTITUDE_M,
                )
            self._qr_hit_count += 1
            self._qr_hit = {
                "dive_start_wall": self._entry_wall,
                "qr_text":         qr.get("data"),
                "qr_box":          qr.get("box"),
                "entry_alt_m":     self._entry_alt_m,   # evidence of the >=100 m rule
                "alt_m":           tel.rel_alt_m,
            }

        # 3b. WE HAVE A DETECTION and reached the continue threshold -> exit.
        #
        # ⚠️ This check must sit OUTSIDE the `qr is not None` block: even if
        #    the QR is lost after the detection (blur, framing), the exit must
        #    still happen. Nested inside, a QR missing on the final frame
        #    would leave the aircraft diving on - silently and fatally.
        #
        # dive_end_wall is stamped HERE, not at the first detection: the
        # rulebook's +-1 s window is defined around the DIVE END time, and the
        # dive really does end here. The first detection is about 0.34 s
        # earlier, comfortably inside the window.
        if (self._qr_hit is not None and config.KAMIKAZE_PULLUP_ON_QR
                and tel.rel_alt_m <= config.KAMIKAZE_QR_CONTINUE_ALTITUDE_M):
            self._qr_hit["dive_end_wall"] = time.time()
            self._qr_hit["qr_frames"] = self._qr_hit_count
            self._qr_hit["qr_first_alt_m"] = self._qr_first_alt_m
            mission.kamikaze_hit = self._qr_hit
            logger.info(
                "CONTINUE complete @ alt=%.1fm -> PULL_UP  "
                "(first detection at %.1f m, %d valid frames total, QR=%r)",
                tel.rel_alt_m, self._qr_first_alt_m, self._qr_hit_count,
                self._qr_hit.get("qr_text"),
            )
            from states.pull_up_state import PullUpState

            await mission._change_state(PullUpState())
            return

        # 4. Attitude command - THREE LAYERS, IN PRIORITY ORDER
        #      1) QR visible     -> visual centering (most precise)
        #      2) no QR          -> GPS lateral guidance
        #      3) no telemetry   -> blind dive (the original behaviour)
        roll_cmd = config.DIVE_ROLL_DEG
        pitch_cmd = config.DIVE_PITCH_DEG
        centering = "blind"

        if qr is None:
            gps_roll, bearing_error, distance = self._gps_roll(tel)
            if gps_roll is not None:
                roll_cmd = gps_roll
                centering = "GPS err=%+.1f deg dist=%.0f m" % (
                    bearing_error, distance)
            elif distance is not None:
                # Too close to the target: the bearing is meaningless, so the
                # correction is frozen.
                centering = "GPS frozen (dist=%.0f m)" % distance

        if qr is not None and config.KAMIKAZE_QR_CENTERING:
            ex, ey = qr["error"]        # normalised to [-1,+1]

            # ex > 0 (QR to the right) -> bank right -> POSITIVE roll
            roll_cmd = _clamp(
                ex * config.KAMIKAZE_QR_ROLL_GAIN,
                -config.KAMIKAZE_QR_MAX_ROLL_DEG,
                +config.KAMIKAZE_QR_MAX_ROLL_DEG,
            )
            # ey > 0 (QR below) -> nose further down -> pitch MORE NEGATIVE.
            # Clamped to a narrow band around DIVE_PITCH_DEG.
            pitch_cmd = _clamp(
                config.DIVE_PITCH_DEG - ey * config.KAMIKAZE_QR_PITCH_GAIN,
                config.DIVE_PITCH_DEG - config.KAMIKAZE_QR_MAX_PITCH_DELTA_DEG,
                config.DIVE_PITCH_DEG + config.KAMIKAZE_QR_MAX_PITCH_DELTA_DEG,
            )
            centering = "QR ex=%+.2f ey=%+.2f in_av=%s" % (ex, ey, qr.get("in_av"))

        # The attitude command must be sent EVERY TICK - MAVSDK offboard
        # reverts to the previous mode if the command stream stops.
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
            f"v_down={tel.vel_down_m_s} m/s "
            # ⚠️ WHICH LAYER IS FLYING MUST BE LOGGED: without it, "why did the
            #    dive miss" cannot be answered from the recording.
            f"| guidance={centering} roll={roll_cmd:+.1f} pitch_cmd={pitch_cmd:+.1f}"
        )

        self._live.update(dive_text)
        logger.info(dive_text, extra={"tick": True})
