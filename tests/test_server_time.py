# tests/test_server_time.py - yarisma sunucusu saati
#
#     python3 tests/test_server_time.py
#
# ⚠️ EN KRITIK DAVRANIS: senkron yokken zaman damgasi URETMEMEK.
#
#    Sessizce yerel saate dusmek, tip kontrollerinden gecen, tamamen normal
#    gorunen ve TUM KAYDI GECERSIZ KILAN sayilar uretir. Sartname s.13:
#    "sunucu saati yazmayan ya da farkli bir saat yazan goruntuler
#     degerlendirilmeyecektir." Yani kusursuz ucup sifir puan alirsin.
#
#    Ilk denemede GURULTULU patlamak kurtarilabilir; sessiz yanlis sayi
#    kurtarilamaz. Bu testin asil isi o reddi kilitlemek.

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from server_time import ServerClock, ServerClockError, ServerTime

PASS = []


def check(label, cond, detail=""):
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "GECTI" if cond else "KALDI", detail))
    assert cond, "%s  %s" % (label, detail)


def test_senkron_yokken_reddediyor():
    print("\n1) SENKRON YOKKEN REDDEDIYOR (asil emniyet ozelligi)")
    c = ServerClock()
    check("baslangicta senkron yok", not c.synced)
    check("age_s None", c.age_s is None)
    for ad, cagri in (("now()", lambda: c.now()),
                      ("at()", lambda: c.at(time.monotonic()))):
        try:
            cagri()
            check("%s reddetti" % ad, False, "sessizce deger uretti!")
        except ServerClockError as e:
            check("%s reddetti" % ad, True, str(e)[:46])


def test_donusum_dogru():
    print("\n2) DONUSUM DOGRU")
    c = ServerClock()
    t0 = time.monotonic()
    c.sync(gun=14, saat=11, dakika=29, saniye=4, milisaniye=653,
           received_monotonic=t0)
    check("senkron sonrasi hazir", c.synced)

    s = c.at(t0)
    check("senkron aninin kendisi birebir donuyor",
          (s.gun, s.saat, s.dakika, s.saniye, s.milisaniye)
          == (14, 11, 29, 4, 653), str(s))

    # 2.5 saniye sonrasi
    s2 = c.at(t0 + 2.5)
    check("2.5 sn sonrasi dogru", (s2.saniye, s2.milisaniye) == (7, 153), str(s2))

    # Dakika ve saat tasmasi
    s3 = c.at(t0 + 31.0 * 60.0)
    check("31 dakika sonrasi saati tasiriyor",
          (s3.saat, s3.dakika) == (12, 0), str(s3))


def test_gecmis_an_donusumu():
    print("\n3) GECMIS BIR ANI CEVIRMEK (kamikaze paketinin ihtiyaci)")
    # ⚠️ Dalis bitis ani ucak hala dalarken time.monotonic() ile yakalanir,
    #    paket ise saniyeler sonra kurulur. Gonderim aninda cevirseydik
    #    paketin KURULDUGU ani bildirirdik - sartnamenin +-1 sn penceresi
    #    o zaman gercek karelerin uzagina duserdi.
    c = ServerClock()
    t0 = time.monotonic()
    c.sync(gun=14, saat=11, dakika=29, saniye=4, milisaniye=653,
           received_monotonic=t0)
    t_dalis_bitis = t0 + 10.0
    t_paket = t0 + 10.8            # paket 800 ms sonra kuruluyor

    dogru = c.at(t_dalis_bitis)
    yanlis = c.at(t_paket)
    fark_ms = ((yanlis.saniye - dogru.saniye) * 1000
               + (yanlis.milisaniye - dogru.milisaniye))
    check("gecmis an ile paket ani FARKLI cikiyor", abs(fark_ms - 800) < 5,
          "aralarinda %d ms" % fark_ms)
    check("dogru olan dalis bitisi (14.653)",
          (dogru.saniye, dogru.milisaniye) == (14, 653), str(dogru))


def test_paket_bicimi():
    print("\n4) PAKET BICIMI (haberlesme dokumani)")
    c = ServerClock()
    c.sync(gun=14, saat=11, dakika=41, saniye=3, milisaniye=141)
    d = c.now().as_dict()
    check("alanlar tam ve dogru adlarda",
          sorted(d) == ["dakika", "gun", "milisaniye", "saat", "saniye"],
          str(sorted(d)))
    check("hepsi int", all(isinstance(v, int) for v in d.values()))
    # ⚠️ Ay ve yil YOK - protokol boyle. Uydurmak hataya davetiye.
    check("ay/yil alani UYDURULMAMIS",
          "ay" not in d and "yil" not in d and "year" not in d)
    check("overlay metni milisaniye hassasiyetinde",
          len(ServerTime(14, 11, 41, 3, 141).overlay_text()) == 12,
          ServerTime(14, 11, 41, 3, 141).overlay_text())


def test_gecersiz_girdi():
    print("\n5) GECERSIZ GIRDI REDDEDILIYOR (yanlis pozitif kontrolu)")
    c = ServerClock()
    for ad, kw in (("saat 24", dict(gun=1, saat=24, dakika=0, saniye=0, milisaniye=0)),
                   ("dakika 60", dict(gun=1, saat=0, dakika=60, saniye=0, milisaniye=0)),
                   ("milisaniye 1000", dict(gun=1, saat=0, dakika=0, saniye=0, milisaniye=1000)),
                   ("gun 0", dict(gun=0, saat=0, dakika=0, saniye=0, milisaniye=0))):
        try:
            c.sync(**kw)
            check("%s reddedildi" % ad, False, "kabul edildi!")
        except ValueError:
            check("%s reddedildi" % ad, True)
    check("gecersiz senkronlar saati BOZMADI", not c.synced)


def test_yeniden_senkron():
    print("\n6) YENIDEN SENKRON OFFSETI GUNCELLIYOR")
    c = ServerClock()
    t0 = time.monotonic()
    c.sync(gun=1, saat=10, dakika=0, saniye=0, milisaniye=0, received_monotonic=t0)
    ilk = c.status()["offset_s"]
    # Sunucu 2 saniye ileride cikti (gecikme duzeltmesi)
    c.sync(gun=1, saat=10, dakika=0, saniye=2, milisaniye=0, received_monotonic=t0)
    son = c.status()["offset_s"]
    check("offset guncellendi", abs((son - ilk) - 2.0) < 1e-6,
          "%.3f -> %.3f" % (ilk, son))
    check("sync sayaci arttı", c.status()["sync_count"] == 2)


if __name__ == "__main__":
    test_senkron_yokken_reddediyor()
    test_donusum_dogru()
    test_gecmis_an_donusumu()
    test_paket_bicimi()
    test_gecersiz_girdi()
    test_yeniden_senkron()
    print("\n%d/%d kontrol gecti" % (sum(PASS), len(PASS)))
    sys.exit(0 if all(PASS) else 1)
