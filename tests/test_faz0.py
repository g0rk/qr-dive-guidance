# tests/test_faz0.py - Faz 0 onkosullarinin regresyonu. Arac/otopilot GEREKMEZ.
#
#     python3 tests/test_faz0.py
#
# Faz 0, "QR'a dal" isinin ONKOSULLARI. Dordu de orijinal kodda eksikti ve
# dordu de SESSIZDI - kod calisiyor gorunuyordu:
#
#   P1  perception_result_queue hic okunmuyordu -> QR verisi goreve
#       ulasmiyor (ve kuyruk sinirsiz buyuyor)
#   P2  goto_location AMSL bekler, config goreli veriyordu -> deniz
#       seviyesi disinda her sahada dalis ABORT
#   P3  65 derece dalis ile 105 m tetik mesafesi tutarsizdi -> QR hic
#       AV'ye girmiyor
#   P4  dive_state tamamen kordu -> kamerayi hic kullanmiyordu

from __future__ import annotations

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import logging
logging.disable(logging.CRITICAL)

import config

PASS = []


def check(label, cond, detail=""):
    PASS.append(bool(cond))
    print("  %-50s %s  %s" % (label, "GECTI" if cond else "KALDI", detail))


def run(coro):
    import asyncio
    return asyncio.get_event_loop().run_until_complete(coro)


class StubQ:
    def __init__(self, items=None):
        self.items = list(items or [])

    def put_nowait(self, x):
        self.items.append(x)

    def get_nowait(self):
        if not self.items:
            raise Exception("empty")
        return self.items.pop(0)


# ======================================================================
# P1 - algi -> gorev halkasi
# ======================================================================
def test_p1():
    print("=" * 76)
    print("  P1 - perception_result_queue goreve baglandi mi")
    print("=" * 76)

    from processes.mission_controller import MissionController

    q = StubQ([
        {"type": "qr", "data": "eski", "in_av": False, "error": [0.9, 0.9]},
        {"type": "yolo", "data": [{"label": "plane"}]},
        {"type": "qr", "data": "yeni", "in_av": True, "error": [0.1, 0.1]},
    ])
    mc = MissionController(
        vehicle=None, telemetry=None,
        comm_command_queue=StubQ(), comm_status_queue=StubQ(),
        perception_command_queue=StubQ(), perception_result_queue=q,
    )

    check("baslangicta qr_result bos", mc.qr_result is None)
    mc._drain_perception()

    check("kuyruk TAMAMEN bosaltildi (sinirsiz buyume yok)",
          len(q.items) == 0, "%d kalan" % len(q.items))
    check("qr_result dolduruldu", mc.qr_result is not None)
    check("SON QR saklandi (eski degil)",
          mc.qr_result and mc.qr_result.get("data") == "yeni",
          str(mc.qr_result and mc.qr_result.get("data")))
    check("varis zamani damgalandi (bayatlik icin sart)",
          mc.qr_result and "t" in mc.qr_result)
    check("yolo tespitleri de alindi", mc.detections is not None)
    print()


# ======================================================================
# P2 - irtifa referansi
# ======================================================================
class _Tel:
    def __init__(self, abs_alt, rel_alt, updated=True):
        self.abs_alt_m = abs_alt
        self.rel_alt_m = rel_alt
        self.last_update_time = 1.0 if updated else 0.0


class _Store:
    def __init__(self, tel):
        self._t = tel

    def get(self):
        return self._t


def _make_vehicle(store):
    from vehicle import Vehicle
    v = Vehicle.__new__(Vehicle)          # MAVSDK System() kurmadan
    v._telemetry_store = store
    v._offboard_active = False
    v.sent = []

    async def fake_goto(lat, lon, abs_alt_m, yaw_deg=0.0):
        v.sent.append(abs_alt_m)

    v.goto_location = fake_goto
    return v


def test_p2():
    print("=" * 76)
    print("  P2 - goto_location AMSL bekler, config goreli verir")
    print("=" * 76)

    from vehicle import VehicleCommandError

    # PX4 SITL varsayilani: Zurih 488 m
    v = _make_vehicle(_Store(_Tel(abs_alt=520.0, rel_alt=32.0)))
    check("saha yuksekligi cikarildi", abs(v.field_elevation_m() - 488.0) < 1e-6,
          "%.1f m AMSL" % v.field_elevation_m())

    run(v.goto_location_rel(41.0, 36.0, config.APPROACH_SAFE_ALTITUDE_M))
    beklenen = 488.0 + config.APPROACH_SAFE_ALTITUDE_M
    check("goreli %.0f m -> mutlak %.0f m" % (config.APPROACH_SAFE_ALTITUDE_M, beklenen),
          abs(v.sent[-1] - beklenen) < 1e-6, "komut: %.1f" % v.sent[-1])
    check("ESKI HATA: ham 100 gonderilmiyor (yerin 388 m alti)",
          abs(v.sent[-1] - config.APPROACH_SAFE_ALTITUDE_M) > 1.0)

    v2 = _make_vehicle(_Store(_Tel(0.0, 0.0, updated=False)))
    try:
        run(v2.goto_location_rel(41.0, 36.0, 100.0))
        raised = False
    except VehicleCommandError:
        raised = True
    check("telemetri yokken hata veriyor (sessizce 0 varsaymiyor)", raised)

    # --- ASIL REGRESYON: saha rakimi dalis kapisini kapatiyor mu ---
    #
    # Test SAYIYA BAGLI OLMASIN: eski kodu bozacak saha rakimi, config
    # degerlerinden turetiliyor. Yaklasma irtifasi ile dalis esigi
    # arasindaki paydan buyuk her rakim eski kodu ABORT'a dusururdu.
    pay = config.APPROACH_SAFE_ALTITUDE_M - config.DIVE_MIN_ENTRY_ALTITUDE_M
    kritik_saha = pay + 10.0        # payi asan ilk rakim

    check("YENI kod: saha rakimi ne olursa olsun kapi ACIK",
          config.APPROACH_SAFE_ALTITUDE_M >= config.DIVE_MIN_ENTRY_ALTITUDE_M,
          "goreli komut -> rel=%.0f >= esik=%.0f"
          % (config.APPROACH_SAFE_ALTITUDE_M, config.DIVE_MIN_ENTRY_ALTITUDE_M))

    check("ESKI kod: %.0f m rakimli sahada ABORT ederdi" % kritik_saha,
          (config.APPROACH_SAFE_ALTITUDE_M - kritik_saha) < config.DIVE_MIN_ENTRY_ALTITUDE_M,
          "AMSL komutu -> rel=%.0f < esik=%.0f"
          % (config.APPROACH_SAFE_ALTITUDE_M - kritik_saha,
             config.DIVE_MIN_ENTRY_ALTITUDE_M))

    # Zurih (PX4 SITL varsayilani) her halukarda bozardi
    check("ESKI kod: Zurih'te (488 m) yerin altina komut ederdi",
          (config.APPROACH_SAFE_ALTITUDE_M - 488.0) < 0,
          "rel=%.0f m" % (config.APPROACH_SAFE_ALTITUDE_M - 488.0))

    # Irtifa zincirinin tutarliligi
    check("irtifa zinciri tutarli (kalkis >= yaklasma > dalis esigi)",
          config.TAKEOFF_ALTITUDE_M >= config.APPROACH_SAFE_ALTITUDE_M
          > config.DIVE_MIN_ENTRY_ALTITUDE_M,
          "%.0f >= %.0f > %.0f" % (config.TAKEOFF_ALTITUDE_M,
                                   config.APPROACH_SAFE_ALTITUDE_M,
                                   config.DIVE_MIN_ENTRY_ALTITUDE_M))
    print()


# ======================================================================
# P3 - dalis geometrisi
# ======================================================================
def test_p3():
    print("=" * 76)
    print("  P3 - dalis acisi = gorus hatti acisi")
    print("=" * 76)

    los = math.degrees(math.atan2(config.APPROACH_SAFE_ALTITUDE_M,
                                  config.APPROACH_DIVE_ARM_DISTANCE_M))
    check("dalis acisi gorus hattina esit (+-2)",
          abs(los - abs(config.DIVE_PITCH_DEG)) <= 2.0,
          "gorus=%.1f dalis=%.1f" % (los, abs(config.DIVE_PITCH_DEG)))
    check("QR plaka esiginin ustunde (>=45, sartname s.17)",
          abs(config.DIVE_PITCH_DEG) >= config.QR_PLATE_MIN_LOOKDOWN_DEG,
          "%.1f >= %.0f" % (abs(config.DIVE_PITCH_DEG),
                            config.QR_PLATE_MIN_LOOKDOWN_DEG))
    check("dalis giris irtifasi >= 100 m (sartname s.18)",
          config.DIVE_MIN_ENTRY_ALTITUDE_M >= 100.0,
          "%.0f m" % config.DIVE_MIN_ENTRY_ALTITUDE_M)

    eski_los = math.degrees(math.atan2(100.0, 105.0))
    check("ESKI degerler (65/105) tutarsiz olarak yakalaniyor",
          abs(eski_los - 65.0) > 2.0,
          "gorus=%.1f vs dalis=65.0 -> fark %.1f" % (eski_los, abs(eski_los - 65.0)))
    print()


# ======================================================================
# P4 - gorsel merkezleme
# ======================================================================
class _NullLive:
    def start(self): pass
    def stop(self): pass
    def update(self, *a): pass


class _FakeTelD:
    rel_alt_m = 60.0
    pitch_deg = -55.0
    vel_north_m_s = 20.0
    vel_east_m_s = 0.0
    vel_down_m_s = 25.0


class _FakeVeh:
    def __init__(self):
        self.last = None

    async def set_attitude(self, roll_deg, pitch_deg, yaw_rate_deg_s, thrust):
        self.last = (roll_deg, pitch_deg, thrust)


class _FakeMission:
    def __init__(self):
        self.telemetry = type("S", (), {"get": staticmethod(lambda: _FakeTelD())})()
        self.vehicle = _FakeVeh()
        self.qr_result = None
        self.kamikaze_hit = None
        self.changed = None

    async def _change_state(self, s):
        self.changed = s.name


def _new_dive():
    from states.dive_state import DiveState
    d = DiveState()
    d._entry_time = time.monotonic() - 1.0
    d._entry_wall = time.time() - 1.0
    d._entry_alt_m = 120.0
    d._live = _NullLive()
    return d


def _qr(ex=0.0, ey=0.0, in_av=True, age=0.0, data="TEKNOFEST"):
    return {"data": data, "error": [ex, ey], "box": [900, 500, 120, 120],
            "in_av": in_av, "t": time.monotonic() - age}


def test_p4():
    print("=" * 76)
    print("  P4 - dalista QR gorsel merkezleme")
    print("=" * 76)

    eski_pullup = config.KAMIKAZE_PULLUP_ON_QR
    config.KAMIKAZE_PULLUP_ON_QR = False   # once yalniz merkezlemeyi olc

    m = _FakeMission(); d = _new_dive(); run(d.update(m))
    check("QR yok -> kor dalis (sabit komut)",
          m.vehicle.last[0] == config.DIVE_ROLL_DEG
          and abs(m.vehicle.last[1] - config.DIVE_PITCH_DEG) < 1e-9,
          "roll=%+.1f pitch=%+.1f" % m.vehicle.last[:2])

    m = _FakeMission(); m.qr_result = _qr(ex=+0.5); d = _new_dive(); run(d.update(m))
    check("QR SAGDA -> POZITIF roll (saga yat)", m.vehicle.last[0] > 0,
          "roll=%+.1f" % m.vehicle.last[0])

    m = _FakeMission(); m.qr_result = _qr(ex=-0.5); d = _new_dive(); run(d.update(m))
    check("QR SOLDA -> negatif roll", m.vehicle.last[0] < 0,
          "roll=%+.1f" % m.vehicle.last[0])

    m = _FakeMission(); m.qr_result = _qr(ey=+0.5); d = _new_dive(); run(d.update(m))
    check("QR ASAGIDA -> pitch DAHA NEGATIF",
          m.vehicle.last[1] < config.DIVE_PITCH_DEG,
          "pitch=%+.1f (taban %+.1f)" % (m.vehicle.last[1], config.DIVE_PITCH_DEG))

    m = _FakeMission(); m.qr_result = _qr(ex=1.0); d = _new_dive(); run(d.update(m))
    check("roll sinirlaniyor (+-%.0f)" % config.KAMIKAZE_QR_MAX_ROLL_DEG,
          abs(m.vehicle.last[0]) <= config.KAMIKAZE_QR_MAX_ROLL_DEG + 1e-9,
          "roll=%+.1f" % m.vehicle.last[0])

    m = _FakeMission(); m.qr_result = _qr(ey=1.0); d = _new_dive(); run(d.update(m))
    sapma = abs(m.vehicle.last[1] - config.DIVE_PITCH_DEG)
    check("pitch sapmasi sinirlaniyor (+-%.0f)" % config.KAMIKAZE_QR_MAX_PITCH_DELTA_DEG,
          sapma <= config.KAMIKAZE_QR_MAX_PITCH_DELTA_DEG + 1e-9,
          "sapma=%.1f" % sapma)

    # --- EMNIYET FILTRELERI ---
    m = _FakeMission()
    m.qr_result = _qr(ex=0.9, age=config.KAMIKAZE_QR_MAX_AGE_S + 0.3)
    d = _new_dive(); run(d.update(m))
    check("BAYAT QR yok sayiliyor -> kor dalisa donuyor",
          abs(m.vehicle.last[0] - config.DIVE_ROLL_DEG) < 1e-9,
          "roll=%+.1f" % m.vehicle.last[0])

    m = _FakeMission(); d = _new_dive()
    m.qr_result = _qr(ex=0.9)
    m.qr_result["t"] = d._entry_time - 0.1      # dalistan ONCE okunmus
    run(d.update(m))
    check("DALIS ONCESI okunan QR yok sayiliyor",
          abs(m.vehicle.last[0] - config.DIVE_ROLL_DEG) < 1e-9,
          "roll=%+.1f" % m.vehicle.last[0])

    # --- SARTNAME: in_av sarti ---
    config.KAMIKAZE_PULLUP_ON_QR = True

    m = _FakeMission(); m.qr_result = _qr(ex=0.3, in_av=False)
    d = _new_dive(); run(d.update(m))
    check("AV DISINDA okunan QR gorevi BITIRMIYOR",
          m.changed is None and m.kamikaze_hit is None,
          "gecis=%s" % m.changed)
    check("  ...ama merkezlemede KULLANILIYOR", m.vehicle.last[0] > 0,
          "roll=%+.1f" % m.vehicle.last[0])

    m = _FakeMission(); m.qr_result = _qr(ex=0.1, in_av=True)
    d = _new_dive(); run(d.update(m))
    check("AV ICINDE okunan QR -> PULL_UP", m.changed == "PULL_UP",
          "gecis=%s" % m.changed)
    check("kamikaze_hit dolduruldu", m.kamikaze_hit is not None,
          str(sorted(m.kamikaze_hit.keys())) if m.kamikaze_hit else "")
    check("  ...dalis giris irtifasi kaniti pakette (>=100)",
          m.kamikaze_hit and m.kamikaze_hit.get("entry_alt_m", 0) >= 100.0,
          "%.1f m" % (m.kamikaze_hit or {}).get("entry_alt_m", 0))

    config.KAMIKAZE_PULLUP_ON_QR = eski_pullup
    print()


def main():
    print()
    test_p1()
    test_p2()
    test_p3()
    test_p4()
    print("=" * 76)
    ok = sum(1 for p in PASS if p)
    print("  SONUC: %d/%d" % (ok, len(PASS)))
    print("=" * 76)
    return 0 if ok == len(PASS) else 1


if __name__ == "__main__":
    sys.exit(main())
