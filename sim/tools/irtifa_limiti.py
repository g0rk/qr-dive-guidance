#!/usr/bin/env python3
"""Ucus kaydindan "X metrenin altinda kac saniye kalindi" tablosunu cikarir.

    python3 sim/tools/irtifa_limiti.py [ucus.csv]

NEDEN VAR
---------
Sartname s.29:
    "Minimum ve maksimum ucus irtifasi, musabakalarin yapilacagi ucus sahasi
     belirlendikten sonra ... yarismacilara BILDIRILECEKTIR."
Yani minimum ucus irtifasi HENUZ BELLI DEGIL; saha belirlenince aciklanacak.

Sartname s.21 (kamikaze):
    "Kamikaze gorevi yapilirken ucus irtifa limitinin altina inildigi durumda
     ALAN DISINA CIKIS olarak degerlendirilecektir."

Ceza yapisi:
    s.29  1-3 kez, her biri 10 sn'den KISA        -> ceza yok
          4. kez (10 sn'den kisa)                 -> -200
    s.33  tek seferde 10 sn'den UZUN              -> -200 VE musabakadan ELEME

Yani belirleyici olan "limitin altina indik mi" degil, "NE KADAR SURE altinda
kaldik". Bu arac o sureyi cesitli esikler icin cikarir; limit aciklandiginda
hicbir seyi yeniden ucurmadan uygunluk kontrol edilebilir.

⚠️ Kamikazeden musabaka basina yalnizca 1 kez puan alinabilir (s.19), yani
   normal senaryoda tur basina TEK kisa ihlal olur - 4 kez kurali rahat.
"""
import csv
import sys

CSV = sys.argv[1] if len(sys.argv) > 1 else "/tmp/ucus.csv"
ELEME_SANIYE = 10.0          # sartname s.33
ESIKLER = (60, 50, 40, 35, 30, 25, 20, 15)


def araliklar(satirlar, esik):
    """Esigin altinda gecirilen KESINTISIZ araliklarin surelerini dondurur."""
    out, basla, onceki_alti = [], None, False
    for r in satirlar:
        alti = float(r["rel_alt_m"]) < esik
        t = float(r["t"])
        if alti and not onceki_alti:
            basla = t
        elif not alti and onceki_alti:
            out.append(t - basla)
        onceki_alti = alti
    if onceki_alti and basla is not None:
        out.append(float(satirlar[-1]["t"]) - basla)
    return out


def main():
    try:
        hepsi = sorted(csv.DictReader(open(CSV)), key=lambda r: float(r["t"]))
    except OSError as e:
        print("kayit okunamadi: %s" % e)
        return 1
    if not hepsi:
        print("yeterli veri yok")
        return 1

    # ⚠️ KALKIS TIRMANISINI HARIC TUT.
    #    Ilk yazimda tum kayit taraniyordu ve 60 m esigi "118 saniye ihlal"
    #    diyordu - cunku ucak kalkarken 60 m'nin ALTINDAN GECMEK ZORUNDA.
    #    Bu bir irtifa ihlali degil, normal tirmanis. Sartname sinir
    #    ihlalini seyir fazi icin tanimliyor.
    #    Analiz, ucagin gorev irtifasina ILK ULASTIGI andan baslar.
    GOREV_IRTIFASI = 100.0
    basla_i = None
    for i, r in enumerate(hepsi):
        if float(r["rel_alt_m"]) >= GOREV_IRTIFASI:
            basla_i = i
            break
    if basla_i is None:
        print("ucak %d m'ye hic cikmamis - gorev fazi yok" % GOREV_IRTIFASI)
        return 1
    satirlar = hepsi[basla_i:]
    print("  (analiz %.1f m'ye ilk ulasildigi andan basliyor: t=%.1f s;"
          " kalkis tirmanisi haric)" % (GOREV_IRTIFASI, float(satirlar[0]["t"])))

    print("=" * 68)
    print("  IRTIFA LIMITI UYGUNLUGU  (%s)" % CSV)
    print("=" * 68)
    print("  en dusuk irtifa: %.2f m" % min(float(r["rel_alt_m"]) for r in satirlar))
    print()
    print("  %-8s %-8s %-10s %s" % ("esik", "ihlal", "en uzun", "durum"))
    print("  " + "-" * 52)

    for esik in ESIKLER:
        a = araliklar(satirlar, esik)
        if not a:
            print("  %-8s %-8s %-10s %s" % ("%d m" % esik, "0", "-", "altina hic inilmedi"))
            continue
        en_uzun = max(a)
        if en_uzun >= ELEME_SANIYE:
            durum = "ELEME RISKI (>= %.0f sn)" % ELEME_SANIYE
        elif len(a) >= 4:
            durum = "-200 (4. kez kurali)"
        else:
            durum = "guvenli"
        print("  %-8s %-8d %-10s %s" % (
            "%d m" % esik, len(a), "%.2f sn" % en_uzun, durum))

    print()
    print("  Kural: tek seferde %.0f sn -> -200 VE eleme;  4. kisa ihlal -> -200" % ELEME_SANIYE)
    print("  Minimum ucus irtifasi aciklaninca ilgili satira bakilmasi yeterli.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
