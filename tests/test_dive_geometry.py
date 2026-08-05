# tests/test_dive_geometry.py - bagimsiz dalis geometrisi modulu
#
#     python3 tests/test_dive_geometry.py
#
# dive_geometry.py'nin var olma sebebi TASINABILIRLIK: baska projeler
# kopyalayip kullanabilsin. Bu yuzden burada iki sey siniyoruz:
#   1. Hesaplar dogru mu (olculen degerlerle karsilastirarak)
#   2. Modul GERCEKTEN bagimsiz mi (proje modullerine dokunmadan calisiyor mu)
#
# Ikincisi kolay bozulur: birisi ileride "config'ten su sabiti alayim" der,
# modul sessizce projeye baglanir ve tasinabilirligini kaybeder. Test bunu
# yakalar.

from __future__ import annotations

import math
import os
import subprocess
import sys

KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, KOK)

import dive_geometry as dg

PASS = []


def check(label, cond, detail=""):
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "GECTI" if cond else "KALDI", detail))
    assert cond, "%s  %s" % (label, detail)


def test_bagimsizlik():
    print("\n1) MODUL GERCEKTEN BAGIMSIZ MI")
    # ⚠️ Ayri bir Python surecinde, SADECE modulun bulundugu dizin
    #    sys.path'te olacak sekilde import ediyoruz. Proje kokundeki
    #    config.py, states/, processes/ erisilebilir olsa bile modul
    #    onlara DOKUNMAMALI.
    # ⚠️ sys.modules'a EKLEMEK SART. dataclass, `from __future__ import
    #    annotations` ile birlikte string tip aciklamalarini cozmek icin
    #    sys.modules[cls.__module__].__dict__'e bakiyor. Eklemezsek
    #    "NoneType object has no attribute __dict__" ile patliyor.
    kod = (
        "import sys, importlib.util, pathlib;"
        "p = pathlib.Path(%r) / 'dive_geometry.py';"
        "spec = importlib.util.spec_from_file_location('dg', p);"
        "m = importlib.util.module_from_spec(spec);"
        "sys.modules['dg'] = m;"
        "spec.loader.exec_module(m);"
        "yasak = [n for n in sys.modules "
        "         if n.split('.')[0] in ('config','states','processes',"
        "                                'mavsdk','cv2','numpy','rclpy',"
        "                                'pyzbar','vehicle','telemetry')];"
        "print('YASAKLI:', yasak);"
        "print('TETIK:', m.trigger_distance_m("
        "  m.DiveProfile(120.0, 40.0, 55.0, 45.0)))"
    ) % KOK
    r = subprocess.run([sys.executable, "-c", kod], capture_output=True,
                       text=True, cwd=os.path.dirname(KOK))
    cikti = (r.stdout or "") + (r.stderr or "")
    check("ayri surecte hatasiz import edildi", r.returncode == 0,
          cikti.strip().splitlines()[-1] if cikti.strip() else "")
    check("proje/agir bagimlilik CEKMEDI", "YASAKLI: []" in cikti,
          [l for l in cikti.splitlines() if l.startswith("YASAKLI")][:1])


def test_tetik_mesafesi():
    print("\n2) TETIK MESAFESI - olculen degerlerle")
    p = dg.DiveProfile(entry_altitude_m=120.0, decode_altitude_m=40.0,
                       pitch_deg=55.0, path_angle_deg=45.0)
    t = dg.trigger_distance_m(p)
    # d_bore = 40/tan(55) = 28.00 ; d_dive = 80/tan(45) = 80.00
    check("trigger = d_dive + d_bore", abs(t - 108.0) < 0.05, "%.3f m" % t)

    # ⚠️ ESKI YANLIS TURETME: entry/tan(pitch) = 120/tan(55) = 84 m.
    #    Olculdu: o tetikle ucak 40 m'de hedefe 3.9 m kala variyordu ve
    #    hedef kadrajin ALTINDA kaliyordu. Test bu iki degerin AYRISTIGINI
    #    kilitler - biri digerine kaymasin.
    eski = 120.0 / math.tan(math.radians(55.0))
    check("eski (yanlis) turetmeden belirgin farkli", abs(t - eski) > 20.0,
          "yeni %.1f vs eski %.1f m" % (t, eski))


def test_yol_acisi_geri_cozumu():
    print("\n3) YOL ACISI UCUS KAYDINDAN GERI COZULUYOR")
    # Gercek kosumdan: 80.0 m dusus, 80.1 m yatay yol
    g = dg.effective_path_angle_deg(80.0, 80.1)
    check("olculen kayittan 45 derece cikiyor", abs(g - 45.0) < 0.2,
          "%.2f derece" % g)
    # ⚠️ Komut edilen pitch 55'ti. Ikisinin FARKLI oldugunu kilitliyoruz:
    #    bu farki gormemek 42.5 m'lik iskalamanin kok sebebiydi.
    check("komut edilen pitch'ten (55) belirgin farkli", abs(g - 55.0) > 5.0,
          "fark %.1f derece" % abs(g - 55.0))


def test_kadraj_ici():
    print("\n4) HEDEF DALIS BOYUNCA KADRAJDA")
    cam = dg.CameraGeometry.from_hfov_and_aspect(51.28, 1920, 1080)
    check("VFOV en-boy oranindan turetildi", abs(cam.vfov_deg - 30.22) < 0.05,
          "%.2f derece" % cam.vfov_deg)

    p = dg.DiveProfile(120.0, 40.0, 55.0, 45.0)
    hi, lo = dg.visible_altitude_band(cam, p, floor_altitude_m=30.0)
    check("giristen (120) tabana (30) kesintisiz gorunur",
          hi >= 119.0 and lo <= 30.5, "%.1f m -> %.1f m" % (hi, lo))

    # Decode irtifasinda tam bore-sight'ta olmali
    kalan = dg.trigger_distance_m(p) - (120.0 - 40.0) / math.tan(math.radians(45.0))
    los = dg.line_of_sight_deg(40.0, kalan)
    check("decode irtifasinda LOS = pitch (bore-sight)",
          abs(los - 55.0) < 0.5, "LOS %.2f vs pitch 55.0" % los)


def test_yer_izdusumu():
    print("\n5) YER IZDUSUMU - 'hedef ucagin altinda kaldi' hatasi")
    cam = dg.CameraGeometry.from_hfov_and_aspect(51.28, 1920, 1080)
    yakin, uzak = dg.ground_footprint_m(cam, pitch_deg=51.7, altitude_m=40.0)
    check("40 m'de kamera 17-54 m arasini goruyor",
          16.0 < yakin < 19.0 and 52.0 < uzak < 56.0,
          "%.1f - %.1f m" % (yakin, uzak))
    # Eski tetikle hedef 3.9 m ondeydi -> yakin kenarin ICINDE degil
    check("3.9 m onde olan hedef kadraj DISINDA (olculen durum)",
          3.9 < yakin, "hedef 3.9 m, yakin kenar %.1f m" % yakin)


def test_dogrulama():
    print("\n6) GECERSIZ GIRDI REDDEDILIYOR (yanlis pozitif kontrolu)")
    for ad, kw in (
        ("pitch 0",        dict(entry_altitude_m=120, decode_altitude_m=40,
                                pitch_deg=0.0, path_angle_deg=45)),
        ("decode > entry", dict(entry_altitude_m=40, decode_altitude_m=120,
                                pitch_deg=55, path_angle_deg=45)),
        ("path 90",        dict(entry_altitude_m=120, decode_altitude_m=40,
                                pitch_deg=55, path_angle_deg=90.0)),
    ):
        try:
            dg.DiveProfile(**kw)
            check("%s reddedildi" % ad, False, "kabul edildi!")
        except ValueError:
            check("%s reddedildi" % ad, True)


if __name__ == "__main__":
    test_bagimsizlik()
    test_tetik_mesafesi()
    test_yol_acisi_geri_cozumu()
    test_kadraj_ici()
    test_yer_izdusumu()
    test_dogrulama()
    print("\n%d/%d kontrol gecti" % (sum(PASS), len(PASS)))
    sys.exit(0 if all(PASS) else 1)
