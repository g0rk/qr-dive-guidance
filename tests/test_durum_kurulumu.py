# tests/test_durum_kurulumu.py - her FSM durumu KURULABILIYOR mu?
#
#     python3 tests/test_durum_kurulumu.py
#
# ⚠️ BU TEST GERCEK BIR HATADAN SONRA YAZILDI (2026-08-05).
#
#    states/takeoff_state.py `config.TAKEOFF_ALTITUDE_M` kullaniyordu ama
#    `import config` satiri YOKTU. Sonuc: TakeoffState() kurulurken
#    NameError atiyordu ve UCAK HICBIR KOSULDA KALKAMIYORDU.
#
#    NEDEN SESSIZDI: CommandRouter durum kurulumunu try/except ile sariyor
#    (command_router.py:106) ve cokmeyi bir UYARIYA ceviriyor:
#        "Command rejected: Failed to instantiate TakeoffState:
#         name 'config' is not defined"
#    Yani program cokmuyor, sadece komut sessizce reddediliyor. Loga
#    bakmayan biri "ucak neden kalkmiyor" diye saatlerce arar.
#
#    NEDEN HICBIR TEST YAKALAMADI: mevcut testlerin hicbiri durumlari
#    KURMUYORDU - hepsi algi/geometri fonksiyonlarini sinatiyordu.
#    Bir modulun import edilmesi, icindeki sinifin kurulabildigini
#    GOSTERMEZ: NameError calisma anindadir, import aninda degil.
#
#    Gerileme kaynagi: 4ec2179'da _target_alt_m koddan config'e tasindi,
#    import eklenmedi.

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import logging
logging.disable(logging.CRITICAL)

from utils.command_router import TRANSITION_TABLE, CommandRouter

PASS = []


def check(label, cond, detail=""):
    # ⚠️ `assert` sonradan eklendi: eskiden bu fonksiyon pytest'i
    #    DUSURMUYORDU, "KALDI" yazan kontrol varken suite yesil kaliyordu.
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "GECTI" if cond else "KALDI", detail))
    assert cond, "%s  %s" % (label, detail)


def test_her_durum_kurulabiliyor():
    print("\n1) HER DURUM SINIFI KURULABILIYOR MU")
    hatalar = []
    for komut, girdi in sorted(TRANSITION_TABLE.items()):
        sinif = girdi["target"]
        try:
            ornek = sinif()
            check("%-10s -> %s()" % (komut, sinif.__name__), True,
                  "name=%s" % getattr(ornek, "name", "?"))
        except Exception as exc:
            hatalar.append((sinif.__name__, exc))
            check("%-10s -> %s()" % (komut, sinif.__name__), False,
                  "%s: %s" % (type(exc).__name__, exc))
    assert not hatalar, "kurulamayan durumlar: %s" % hatalar


def test_router_takeoff_u_kabul_ediyor():
    print("\n2) ROUTER 'takeoff' KOMUTUNU IDLE'DAN KABUL EDIYOR MU")
    # Asil hata tam burada goruldu: router 'ok=False' donuyordu ve sebep
    # NameError'du. Bu testin var olma sebebi bu senaryoyu kilitlemek.
    sonuc = CommandRouter().resolve('{"command": "takeoff"}', "IDLE")
    check("komut kabul edildi", sonuc.ok,
          "sebep: %s" % (sonuc.reason or "-"))
    check("hedef durum uretildi", sonuc.new_state is not None,
          "durum: %s" % (sonuc.new_state.name if sonuc.new_state else "YOK"))
    assert sonuc.ok, "takeoff IDLE'dan reddedildi: %s" % sonuc.reason


def test_router_align_i_kabul_ediyor():
    print("\n3) ROUTER 'align' KOMUTUNU HOLD'DAN KABUL EDIYOR MU")
    sonuc = CommandRouter().resolve('{"command": "align"}', "HOLD")
    check("komut kabul edildi", sonuc.ok, "sebep: %s" % (sonuc.reason or "-"))
    assert sonuc.ok, "align HOLD'dan reddedildi: %s" % sonuc.reason


def test_gecersiz_komut_reddediliyor():
    print("\n4) GECERSIZ KOMUT REDDEDILIYOR MU (yanlis pozitif kontrolu)")
    # Testin gercekten bir sey olctugunden emin olmak icin: her komutu
    # kabul eden bir router ise yaramaz.
    s1 = CommandRouter().resolve('{"command": "boyle_bir_komut_yok"}', "IDLE")
    check("bilinmeyen komut reddedildi", not s1.ok, s1.reason or "")
    s2 = CommandRouter().resolve('{"command": "takeoff"}', "DIVE")
    check("takeoff DIVE'dan reddedildi (allowed_from)", not s2.ok,
          s2.reason or "")


if __name__ == "__main__":
    test_her_durum_kurulabiliyor()
    test_router_takeoff_u_kabul_ediyor()
    test_router_align_i_kabul_ediyor()
    test_gecersiz_komut_reddediliyor()
    print("\n%d/%d kontrol gecti" % (sum(PASS), len(PASS)))
    sys.exit(0 if all(PASS) else 1)
