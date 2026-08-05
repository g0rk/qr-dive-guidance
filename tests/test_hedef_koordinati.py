# tests/test_hedef_koordinati.py - config'teki hedef, gz dunyasindaki
# qr_pad ile GERCEKTEN ayni yeri mi gosteriyor?
#
#     python3 tests/test_hedef_koordinati.py
#
# ⚠️ BU TEST GERCEK BIR HATADAN SONRA YAZILDI (2026-08-05).
#
#    config.TARGET_LATITUDE_DEG / TARGET_LONGITUDE_DEG elle yazilmis iki
#    sayiydi ve pad'in 49.9 m GUNEYBATISINI gosteriyordu. QR pad 2 m x 2 m,
#    yani hata kenarin 25 KATI: ucak bos cimene dalardi.
#
#    SEBEP - IKI FARKLI "HOME":
#      1) PX4'un belgelenmis varsayilani   47.397742 / 8.545594
#      2) gz dunya dosyasinin kendi <spherical_coordinates> etiketi
#         47.397971 / 8.546164     <- gz simulatorken GECERLI OLAN BU
#    Aralarinda 49.9 m var. Hedef (1)'e 500 m eklenerek turetilmisti.
#
#    Bu tur bir hata SESSIZDIR: kod calisir, ucak uCar, dalar - sadece
#    yanlis yere. Hicbir birim test yakalamaz cunku iki sayi da "gecerli
#    koordinat"tir. Bu yuzden test, sayilari DUNYA DOSYASINDAN yeniden
#    turetip config ile karsilastiriyor. Dunya degisirse test duser.

from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import logging
logging.disable(logging.CRITICAL)

import config
from utils.geo_utils import offset_lat_lon, distance_m, bearing_deg

KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DUNYA = os.path.join(KOK, "sim", "worlds", "qr_target.sdf")

# Ne kadar sapmaya izin var? QR pad 2 m x 2 m. Ucagin AV'ye QR'i sokabilmesi
# icin hedefin pad uzerinde olmasi lazim; 1 m, pad'in yarisi - cok comert
# ama "50 m yanlis" sinifindaki hatalari kesin yakalar.
TOLERANS_M = 1.0

PASS = []


def check(label, cond, detail=""):
    # ⚠️ `assert` sonradan eklendi: eskiden bu fonksiyon pytest'i
    #    DUSURMUYORDU, "KALDI" yazan kontrol varken suite yesil kaliyordu.
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "GECTI" if cond else "KALDI", detail))
    assert cond, "%s  %s" % (label, detail)


def dunya_orijini():
    """gz dunyasinin <spherical_coordinates> etiketinden orijin."""
    sc = ET.parse(DUNYA).getroot().find("world/spherical_coordinates")
    assert sc is not None, "dunyada <spherical_coordinates> yok"
    return (float(sc.find("latitude_deg").text),
            float(sc.find("longitude_deg").text))


def qr_pad_konumu():
    """qr_pad include'unun <pose> etiketinden (x=dogu, y=kuzey) metre."""
    for inc in ET.parse(DUNYA).getroot().iter("include"):
        ad = inc.find("name")
        if ad is not None and ad.text == "qr_pad":
            p = inc.find("pose").text.split()
            return float(p[0]), float(p[1])
    raise AssertionError("dunyada qr_pad include'u yok")


def test_dunya_okunabiliyor():
    print("\n1) DUNYA DOSYASI OKUNUYOR")
    lat, lon = dunya_orijini()
    check("dunya orijini bulundu", True, "%.9f / %.9f" % (lat, lon))
    dogu, kuzey = qr_pad_konumu()
    check("qr_pad pozu bulundu", True, "dogu=%.1f m kuzey=%.1f m" % (dogu, kuzey))

    # ⚠️ Bu, hatanin tam kaynagi. Dunyanin orijini PX4'un belgelenmis
    #    varsayilaniyla AYNI DEGIL; testin var olma sebebi bu.
    px4_lat, px4_lon = 47.397742, 8.545594
    fark = distance_m(lat, lon, px4_lat, px4_lon)
    check("dunya orijini != PX4 varsayilani (hatanin kaynagi)",
          fark > 10.0, "aralarinda %.1f m var" % fark)


def test_config_hedefi_pad_uzerinde():
    print("\n2) CONFIG HEDEFI GERCEKTEN PAD UZERINDE MI")
    o_lat, o_lon = dunya_orijini()
    dogu, kuzey = qr_pad_konumu()

    bek_lat, bek_lon = offset_lat_lon(o_lat, o_lon, north_m=kuzey, east_m=dogu)
    sapma = distance_m(bek_lat, bek_lon,
                       config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG)

    check("dunyadan turetilen hedef", True, "%.7f / %.7f" % (bek_lat, bek_lon))
    check("config'teki hedef", True, "%.7f / %.7f" % (
        config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG))
    check("sapma <= %.1f m" % TOLERANS_M, sapma <= TOLERANS_M,
          "sapma = %.2f m" % sapma)
    assert sapma <= TOLERANS_M, (
        "config hedefi pad'den %.1f m uzakta. Dunya orijini %.9f/%.9f, "
        "qr_pad (%.1f dogu, %.1f kuzey) -> hedef %.7f/%.7f olmali."
        % (sapma, o_lat, o_lon, dogu, kuzey, bek_lat, bek_lon))


def test_mesafe_ve_kerteriz():
    print("\n3) ORIJINDEN HEDEFE MESAFE VE KERTERIZ")
    o_lat, o_lon = dunya_orijini()
    dogu, kuzey = qr_pad_konumu()

    d = distance_m(o_lat, o_lon,
                   config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG)
    b = bearing_deg(o_lat, o_lon,
                    config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG)

    bek_d = (dogu ** 2 + kuzey ** 2) ** 0.5
    check("mesafe pozla uyusuyor", abs(d - bek_d) <= TOLERANS_M,
          "%.1f m (beklenen %.1f)" % (d, bek_d))
    # qr_pad (500, 0) -> tam dogu -> 90 derece
    if kuzey == 0.0 and dogu > 0:
        check("kerteriz tam dogu (90 derece)", abs(b - 90.0) <= 0.5,
              "%.2f derece" % b)


def test_yaklasma_zinciri_tutarli():
    print("\n4) HEDEF, YAKLASMA GEOMETRISIYLE TUTARLI MI")
    # Hayalet nokta hedefin APPROACH_GHOST_DISTANCE_M kadar arkasina konuyor.
    # Hedef kalkis noktasina cok yakinsa ucak donusunu bitiremeden hedefi
    # gecer. Bu, eski hatanin ILK teshis edilen belirtisiydi.
    o_lat, o_lon = dunya_orijini()
    d = distance_m(o_lat, o_lon,
                   config.TARGET_LATITUDE_DEG, config.TARGET_LONGITUDE_DEG)
    check("hedef, hayalet mesafesinden uzak",
          d > config.APPROACH_GHOST_DISTANCE_M * 0.5,
          "hedef %.0f m, hayalet mesafesi %.0f m" % (
              d, config.APPROACH_GHOST_DISTANCE_M))


if __name__ == "__main__":
    test_dunya_okunabiliyor()
    test_config_hedefi_pad_uzerinde()
    test_mesafe_ve_kerteriz()
    test_yaklasma_zinciri_tutarli()
    print("\n%d/%d kontrol gecti" % (sum(PASS), len(PASS)))
    sys.exit(0 if all(PASS) else 1)
