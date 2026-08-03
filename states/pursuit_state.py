# states/pursuit_state.py — Offboard attitude ile hedefe yaklaşma.
#
# Her tick'te:
#   1. mission.target'tan hedef konum okunur
#   2. Kendi konumumuzdan hedefe bearing hesaplanır
#   3. Bearing farkına göre roll set-point üretilir (yaw için bank-to-turn)
#   4. Kendi irtifamız TARGET_ALT_M'de tutulacak şekilde pitch set-point üretilir
#   5. Max throttle ile ileri itilir (hız config.PURSUIT_THROTTLE'a bağlı)
#
# Çıkış yok — dışarıdan komut (hold, abort vb.) bekler.

from __future__ import annotations

import math
import time
import logging
from typing import TYPE_CHECKING

from rich.live import Live

from states.base_state import BaseState
from vehicle import VehicleCommandError

if TYPE_CHECKING:
    from processes.mission_controller import MissionController

import config

logger = logging.getLogger("PURSUIT")

# Sabit irtifa hedefi (metre). İstersen config.py'ye taşı.
TARGET_ALT_M = 100.0


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """İki GPS noktası arasındaki yönü (0-360°) döner."""
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    d_lon  = math.radians(lon2 - lon1)

    x = math.sin(d_lon) * math.cos(lat2_r)
    y = (math.cos(lat1_r) * math.sin(lat2_r)
         - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(d_lon))

    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360.0) % 360.0


def _angle_diff(a: float, b: float) -> float:
    """a'dan b'ye en kısa açı farkı (-180, +180]."""
    diff = (b - a + 180.0) % 360.0 - 180.0
    return diff


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """İki nokta arasındaki yüzey mesafesi (metre)."""
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lam = math.radians(lon2 - lon1)
    a = math.sin(d_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _alt_hold_pitch(own) -> tuple[float, float]:
    """Kendi irtifamızı TARGET_ALT_M'de tutacak pitch set-point'i üretir.

    NOT: own.altitude_m alan adı senin Telemetry sınıfına göre farklı
    olabilir (örn. relative_altitude_m, alt_m vb.) — kendi sınıfına göre
    düzelt.
    """
    current_alt = own.rel_alt_m
    alt_error = TARGET_ALT_M - current_alt  # pozitif: aşağıdayız, tırmanmalıyız

    pitch_cmd = _clamp(
        config.PURSUIT_BASE_PITCH_DEG + alt_error * config.PURSUIT_PITCH_GAIN,
        config.PURSUIT_MIN_PITCH_DEG,
        config.PURSUIT_MAX_PITCH_DEG,
    )
    return pitch_cmd, alt_error


class PursuitState(BaseState):
    """
    Sabit kanatlı aracı hedefe yönlendiren offboard attitude state'i.

    Kontrol stratejisi:
      - Yaw: bearing farkı → orantılı roll set-point (bank-to-turn)
      - İrtifa: TARGET_ALT_M'ye göre orantılı pitch set-point (alt-hold)
      - Hız: sabit max throttle (config.PURSUIT_THROTTLE) — yakalamayı
        hızlandırmak istiyorsan bu sabiti config.py'de yükselt.

    mission.target dict formatı (target_stream.py ile uyumlu):
        {"type": "target", "lat": float, "lon": float, "alt": float, ...}
    """

    name = "PURSUIT"

    def __init__(self) -> None:
        self._entry_time: float = 0.0
        self._live = Live("", refresh_per_second=10, transient=True)

    async def on_enter(self, mission: MissionController) -> None:
        self._live.start()
        self._entry_time = time.monotonic()

        mission.perception_command_queue.put_nowait({"cmd": "set_mode", "mode": "yolo"})
        mission.perception_command_queue.put_nowait({"cmd": "start"})

        if mission.target is None:
            logger.warning("PURSUIT entered but mission.target is None — will wait for target")

        logger.info("PURSUIT started, target altitude=%.1fm", TARGET_ALT_M)

    async def on_exit(self, mission: MissionController) -> None:
        self._live.stop()
        elapsed = time.monotonic() - self._entry_time
        logger.info("Exiting PURSUIT after %.1fs", elapsed)

    async def update(self, mission: MissionController) -> None:
        tel = mission.target  # hedef telemetrisi
        own = mission.telemetry.get()  # kendi telemetrimiz
        elapsed = time.monotonic() - self._entry_time

        # Henüz target gelmediyse irtifayı koru, düz uç, komutu bekle
        if tel is None:
            # await self._send_level(mission, own) # Kendi alt yapınızdaki düz uçuş fonksiyonu
            self._live.update("[PURSUIT] Hedef bekleniyor, düz uçuşta...")
            return

        # Bearing ve mesafe hesapla
        bearing = _bearing_deg(own.latitude_deg, own.longitude_deg, tel["lat"], tel["lon"])
        distance_m = _haversine_m(own.latitude_deg, own.longitude_deg, tel["lat"], tel["lon"])

        # Mevcut yaw'dan bearing farkı -> roll set-point
        yaw_error = _angle_diff(own.heading_deg, bearing)

        # 1. ROLL (YATIŞ) KONTROLÜ - Hedefe yönelmek (Bank-to-turn) için
        # config.PURSUIT_ROLL_GAIN ile hatayı çarpıyoruz.
        roll_cmd = _clamp(
            yaw_error * config.PURSUIT_ROLL_GAIN,
            config.PURSUIT_MIN_ROLL_DEG,
            config.PURSUIT_MAX_ROLL_DEG
        )

        # 2. PITCH (YUNUSLAMA) KONTROLÜ - İrtifayı korumak için
        pitch_cmd, alt_error = _alt_hold_pitch(own)

        # UI için durum mesajı oluştur
        status_msg = (
            f"[PURSUIT] Hedef: {distance_m:.1f}m | "
            f"AltErr: {alt_error:.1f}m -> Pitch: {pitch_cmd:.1f}° | "
            f"YawErr: {yaw_error:.1f}° -> Roll: {roll_cmd:.1f}°"
        )
        self._live.update(status_msg)

        # 3. OFFBOARD KOMUTUNUN GÖNDERİLMESİ
        try:
            # MAVSDK offboard.set_attitude komutu (Kullandığınız kütüphanenin API'sine göre uyarlayınız)
            
            # Sabit kanatta dönüş roll ile sağlanır, yaw genelde mevcut heading veya 0 bırakılır.
            await mission.vehicle.set_attitude(
                roll_deg=roll_cmd,
                pitch_deg=pitch_cmd,
                yaw_rate_deg_s=0.0,  # Bank-to-turn yaptığımız için yaw rate sıfır
                thrust=config.PURSUIT_THROTTLE
            )
                        
        except VehicleCommandError as e:
            logger.error("Offboard komutu gönderilemedi: %s", e)
        except Exception as e:
            logger.error("Beklenmeyen bir hata oluştu: %s", e)

def _clamp(val: float, min_val: float, max_val: float) -> float:
    """Verilen değeri min ve max sınırları arasında tutar."""
    return max(min_val, min(val, max_val))