# tests/test_gps_gudum.py - dalista GPS tabanli yanal gudum
#
#     python3 tests/test_gps_gudum.py
#
# ⚠️ BU TEST GERCEK BIR HATADAN SONRA YAZILDI (2026-08-05).
#    Dalis SABIT attitude tutuyordu (roll=0) ve hedefi 42.5 m ISKALIYORDU.
#    QR pad 2x2 m -> sapma kenarin 21 kati -> pad kadraja hic girmiyor ->
#    4879 karede 0 QR tespiti. Gorsel merkezleme de devreye giremiyordu
#    cunku once QR'i GORMESI gerekiyor (tavuk-yumurta).
#
#    ISARET HATASI BU ISTE OLUMCULDUR: yanlis yone yatan bir duzeltme
#    sapmayi kapatmak yerine BUYUTUR ve bunu ancak ucurunca fark edersin.
#    Bu yuzden yon, ucus oncesi burada kilitleniyor.

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import logging
logging.disable(logging.CRITICAL)

import config
from states.dive_state import DiveState
from utils.geo_utils import offset_lat_lon, distance_m

PASS = []


def check(label, cond, detail=""):
    # ⚠️ `assert` sonradan eklendi: eskiden bu fonksiyon pytest'i
    #    DUSURMUYORDU, "KALDI" yazan kontrol varken suite yesil kaliyordu.
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "GECTI" if cond else "KALDI", detail))
    assert cond, "%s  %s" % (label, detail)


class SahteTel:
    """Telemetri yerine gecen en kucuk nesne."""
    def __init__(self, lat, lon, heading):
        self.latitude_deg = lat
        self.longitude_deg = lon
        self.heading_deg = heading
        self.rel_alt_m = 100.0


def konum(kuzey_m, dogu_m):
    """Hedeften verilen kadar sapmis bir konum uret."""
    return offset_lat_lon(config.TARGET_LATITUDE_DEG,
                          config.TARGET_LONGITUDE_DEG,
                          kuzey_m, dogu_m)


def test_isaret_yonu():
    print("\n1) ISARET YONU - duzeltme DOGRU tarafa mi yatiyor")
    d = DiveState()

    # Ucak hedefin 200 m BATISINDA, burnu tam DOGUYA (90 derece) donuk.
    # Hedef tam onunde -> kerteriz hatasi ~0 -> roll ~0 olmali.
    lat, lon = konum(0, -200)
    roll, hata, mesafe = d._gps_roll(SahteTel(lat, lon, 90.0))
    check("hedef tam onde -> roll ~ 0", abs(roll) < 1.0,
          "roll=%+.2f hata=%+.2f mesafe=%.0f" % (roll, hata, mesafe))

    # Ayni yerde ama burun KUZEYE (0 derece) donuk. Hedef DOGUDA kaliyor,
    # yani SAGDA -> saga yatmali -> POZITIF roll.
    roll, hata, mesafe = d._gps_roll(SahteTel(lat, lon, 0.0))
    check("hedef SAGDA -> POZITIF roll", roll > 0,
          "roll=%+.2f hata=%+.2f" % (roll, hata))

    # Burun GUNEYE (180 derece) donuk. Hedef SOLDA -> negatif roll.
    roll, hata, mesafe = d._gps_roll(SahteTel(lat, lon, 180.0))
    check("hedef SOLDA -> negatif roll", roll < 0,
          "roll=%+.2f hata=%+.2f" % (roll, hata))


def test_sinirlama():
    print("\n2) SINIRLAMA - dalisin en riskli faz oldugu unutulmasin")
    d = DiveState()
    lat, lon = konum(0, -200)
    # Burun hedefin TAM TERSINE (270 = batiya) -> hata 180 dereceye yakin
    roll, hata, mesafe = d._gps_roll(SahteTel(lat, lon, 270.0))
    check("asiri hatada roll sinirlaniyor",
          abs(roll) <= config.KAMIKAZE_GPS_MAX_ROLL_DEG + 1e-6,
          "roll=%+.1f sinir=%.1f (ham hata %+.1f)" % (
              roll, config.KAMIKAZE_GPS_MAX_ROLL_DEG, hata))


def test_yakin_mesafede_donduruluyor():
    print("\n3) HEDEFE COK YAKINDA DUZELTME DONDURULUYOR")
    # ⚠️ Birkac metre kala kucucuk bir konum hatasi kerteriz'i 180 derece
    #    cevirebilir; duzeltme yapilirsa ucak son anda sertce yatar.
    d = DiveState()
    lat, lon = konum(0, -5)     # hedefe 5 m
    roll, hata, mesafe = d._gps_roll(SahteTel(lat, lon, 90.0))
    check("cok yakinda roll uretilmiyor", roll is None,
          "mesafe=%.1f m, esik=%.1f m" % (
              mesafe, config.KAMIKAZE_GPS_MIN_DISTANCE_M))

    # Esigin hemen ustunde yine uretilmeli (yanlis pozitif kontrolu)
    lat, lon = konum(0, -(config.KAMIKAZE_GPS_MIN_DISTANCE_M + 20))
    roll, hata, mesafe = d._gps_roll(SahteTel(lat, lon, 0.0))
    check("esigin ustunde yine calisiyor", roll is not None,
          "mesafe=%.1f m roll=%+.2f" % (mesafe, roll if roll else 0))


def test_kapatilabiliyor():
    print("\n4) OZELLIK KAPATILABILIYOR (kor dalisa donus)")
    d = DiveState()
    lat, lon = konum(0, -200)
    eski = config.KAMIKAZE_GPS_GUIDANCE
    try:
        config.KAMIKAZE_GPS_GUIDANCE = False
        roll, _, _ = d._gps_roll(SahteTel(lat, lon, 0.0))
        check("kapaliyken roll uretilmiyor", roll is None, "roll=%s" % roll)
    finally:
        config.KAMIKAZE_GPS_GUIDANCE = eski


def test_telemetri_yoksa():
    print("\n5) TELEMETRI YOKKEN SESSIZCE 0/0 VARSAYILMIYOR")
    # ⚠️ lat=lon=0 "Gine Korfezi" degil "veri yok" demektir. Oradan
    #    kerteriz hesaplamak ucagi Afrika'ya yoneltirdi.
    d = DiveState()
    roll, _, _ = d._gps_roll(SahteTel(0.0, 0.0, 0.0))
    check("bos telemetride roll uretilmiyor", roll is None, "roll=%s" % roll)


def test_devam_esigi_tutarli():
    print("\n7) QR SONRASI DEVAM ESIGI")
    # ⚠️ Devam esigi tabandan BUYUK olmali. Kucuk olsaydi taban once
    #    tetiklenir ve devam ayari HICBIR ISE YARAMAZDI - hem de sessizce:
    #    kod calisir, hata vermez, sadece hep 1 kare toplanir.
    devam = config.KAMIKAZE_QR_CONTINUE_ALTITUDE_M
    taban = config.DIVE_PULL_UP_ALTITUDE_M
    check("devam esigi > pull-up tabani", devam > taban,
          "devam=%.1f > taban=%.1f" % (devam, taban))

    # Devam esigi, UCUSTA olculen ilk tespit irtifasinin ALTINDA olmali,
    # yoksa hic devam edilmez.
    # ⚠️ Burada QR_DECODE_ALTITUDE_M (40, sabit kamera testi) DEGIL,
    #    QR_FIRST_DETECT_ALTITUDE_M (41.7, ucusta olculen EN KOTU) kullanilir.
    #    Ilk yazimda 40 kullanmistim ve test dustu - dogru davranis:
    #    iki farkli olcumu ayni sanmak tam da bu projenin tekrar eden hatasi.
    check("devam esigi < ucusta olculen ilk tespit irtifasi",
          devam < config.QR_FIRST_DETECT_ALTITUDE_M,
          "devam=%.1f < ilk_tespit=%.1f" % (
              devam, config.QR_FIRST_DETECT_ALTITUDE_M))

    # Kac kare kazanildigi: (ilk_tespit - devam_esigi) / alcalma_hizi
    ALCALMA_M_S = 32.0      # olculen
    FPS = 30.0
    pencere_s = (config.QR_FIRST_DETECT_ALTITUDE_M - devam) / ALCALMA_M_S
    kare = pencere_s * FPS
    # EN KOTU gozlemle ~6 kare; ortalama gozlemle (43.3 m) ~8 kare.
    # Sartname TEK gecerli kare istiyor, 6 kare 6 kat marj demek.
    check("en kotu durumda >= 5 gecerli kare", kare >= 5.0,
          "%.2f s -> ~%.0f kare (en kotu gozlem %.1f m ile)" % (
              pencere_s, kare, config.QR_FIRST_DETECT_ALTITUDE_M))

    # ⚠️ SARTNAME s.20: pencere dalis bitisinin +-1 saniyesi. Ilk tespit ile
    #    dalis bitisi arasindaki sure bu pencereye SIGMALI.
    check("ilk tespit, dalis bitisinin +-1 sn penceresinde",
          pencere_s <= 1.0, "aralarinda %.2f s var (sinir 1.00)" % pencere_s)

    # Olculen irtifa kaybiyla en dusuk noktanin kestirimi
    EN_KOTU_KAYIP = 16.46
    en_dusuk = devam - EN_KOTU_KAYIP
    check("kestirilen en dusuk nokta > 15 m", en_dusuk > 15.0,
          "%.1f - %.1f = %.1f m" % (devam, EN_KOTU_KAYIP, en_dusuk))


def test_pull_up_tabani_yukseldi():
    print("\n6) PULL-UP TABANI OLCUME GORE YUKSELTILDI")
    # Olculen EN KOTU irtifa kaybi 16.46 m (5 kosum).
    EN_KOTU_KAYIP = 16.46
    pay = config.DIVE_PULL_UP_ALTITUDE_M - EN_KOTU_KAYIP
    check("taban, olculen en kotu kayiptan buyuk",
          config.DIVE_PULL_UP_ALTITUDE_M > EN_KOTU_KAYIP,
          "taban=%.1f m, kayip=%.1f m -> pay=%.1f m" % (
              config.DIVE_PULL_UP_ALTITUDE_M, EN_KOTU_KAYIP, pay))
    check("pay en az 10 m", pay >= 10.0, "pay=%.1f m" % pay)
    # Taban decode esiginin ALTINDA kalmali, yoksa QR'i gorme sansi hic olmaz
    check("taban 40 m decode esiginin altinda",
          config.DIVE_PULL_UP_ALTITUDE_M < 40.0,
          "taban=%.1f m < 40 m" % config.DIVE_PULL_UP_ALTITUDE_M)


if __name__ == "__main__":
    test_isaret_yonu()
    test_sinirlama()
    test_yakin_mesafede_donduruluyor()
    test_kapatilabiliyor()
    test_telemetri_yoksa()
    test_devam_esigi_tutarli()
    test_pull_up_tabani_yukseldi()
    print("\n%d/%d kontrol gecti" % (sum(PASS), len(PASS)))
    sys.exit(0 if all(PASS) else 1)
