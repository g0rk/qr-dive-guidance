# tests/test_faz2_center.py - Faz 2: gercek merkez + in_av kaynagi
#
#     python3 tests/test_faz2_center.py
#
# ⚠️ BU TEST BIR IDDIAYI SINAMAK ICIN YAZILDI:
#    "Sinirlayici kutu QR'dan 2 kat buyuk oldugu icin in_av testi gecerli
#     vurusu reddedebilir."
#    AV eksen-hizali bir dikdortgen. Bir dortgenin boyle bir dikdortgenin
#    icinde olmasi <=> dort kosesinin de icinde olmasi <=> koselerin
#    min/max'inin icinde olmasi. Kosellerin min/max'i ise KUTUNUN KENDISI.
#    Yani iki test AYNI SEY olabilir. Asagisi bunu olcuyor.

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import logging
logging.disable(logging.CRITICAL)

import numpy as np
import cv2
import qrcode
from pyzbar.pyzbar import decode

from processes.perception import polygon_to_corners, quad_center, quad_in_av

W, H = 1920, 1080
AV = (int(W * 0.25), int(H * 0.10), W - int(W * 0.25), H - int(H * 0.10))
PASS = []


def check(label, cond, detail=""):
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "GECTI" if cond else "KALDI", detail))


def qr_tile(px=500):
    q = qrcode.QRCode(version=1, error_correction=qrcode.constants.ERROR_CORRECT_M,
                      box_size=10, border=2)
    q.add_data("teknofest2026"); q.make(fit=False)
    a = cv2.cvtColor(np.array(q.make_image(fill_color="black",
                                           back_color="white").convert("RGB")),
                     cv2.COLOR_RGB2BGR)
    return cv2.resize(a, (px, px), interpolation=cv2.INTER_AREA)


TILE = qr_tile()


def scene(quad):
    f = np.full((H, W, 3), (60, 105, 70), np.uint8)
    dst = np.array(quad, np.float32)
    src = np.array([[0, 0], [TILE.shape[1], 0],
                    [TILE.shape[1], TILE.shape[0]], [0, TILE.shape[0]]], np.float32)
    M = cv2.getPerspectiveTransform(src, dst)
    warp = cv2.warpPerspective(TILE, M, (W, H))
    mask = np.zeros((H, W), np.uint8)
    cv2.fillConvexPoly(mask, dst.astype(np.int32), 255)
    f[mask > 0] = warp[mask > 0]
    return f


def rotated(cx, cy, s, deg):
    r = math.radians(deg); h = s / 2.0
    return [(cx + dx * math.cos(r) - dy * math.sin(r),
             cy + dx * math.sin(r) + dy * math.cos(r))
            for dx, dy in ((-h, -h), (h, -h), (h, h), (-h, h))]


def trapezoid(cx, cy, s, squash):
    h = s / 2.0; top = h * squash
    return [(cx - top, cy - h), (cx + top, cy - h), (cx + h, cy + h), (cx - h, cy + h)]


def detect(frame):
    x0, y0 = int(W * 0.17), int(H * 0.02)
    res = decode(cv2.cvtColor(frame[y0:H - y0, x0:W - x0], cv2.COLOR_BGR2GRAY))
    if not res:
        return None
    b = res[0]
    bx, by, bw, bh = b.rect
    return {
        "box": [x0 + bx, y0 + by, bw, bh],
        "corners": polygon_to_corners(b.polygon, x0, y0, 1),
    }


def box_in_av(box):
    x, y, bw, bh = box
    return (x >= AV[0] and y >= AV[1] and (x + bw) <= AV[2] and (y + bh) <= AV[3])


# ======================================================================
def test_iddia_kontrolu():
    print("=" * 80)
    print("  1) IDDIA KONTROLU: kutu-in_av ile dortgen-in_av hic ayrisiyor mu?")
    print("=" * 80)
    print("  AV = x[%d..%d]  y[%d..%d]" % (AV[0], AV[2], AV[1], AV[3]))
    print()

    ayrisan = 0
    toplam = 0
    ornekler = []
    for deg in (0, 15, 30, 45, 60):
        for cx in range(AV[0] - 60, AV[0] + 340, 40):      # sol kenarda tara
            quad = rotated(cx, H / 2, 300, deg)
            d = detect(scene(quad))
            if not d or not d["corners"]:
                continue
            toplam += 1
            a = box_in_av(d["box"])
            b = quad_in_av(d["corners"], *AV)
            if a != b:
                ayrisan += 1
                if len(ornekler) < 4:
                    ornekler.append((deg, cx, a, b))

    print("  taranan konum/aci: %d" % toplam)
    print("  iki testin AYRISTIGI durum: %d" % ayrisan)
    for deg, cx, a, b in ornekler:
        print("     aci=%d cx=%d -> kutu=%s dortgen=%s" % (deg, cx, a, b))
    print()

    check("iki test ozdes davraniyor (beklenen: AV eksen-hizali)",
          ayrisan == 0, "%d ayrisma" % ayrisan)
    print()
    print("  -> AV eksen-hizali oldugu icin 'dortgen icinde' ile 'kutu icinde'")
    print("     MATEMATIKSEL OLARAK AYNI kosul. Kutunun alaninin 2 kat olmasi")
    print("     bunu degistirmiyor. Faz 1'de yazdigim 'gecerli vurus reddedilir'")
    print("     gerekcesi bu test icin GECERSIZ.")
    print()


def test_rect_vs_polygon_bbox():
    print("=" * 80)
    print("  2) pyzbar `rect`, polygon'un bbox'i ile ayni mi?")
    print("=" * 80)
    en_buyuk = 0.0
    for deg in (0, 20, 40, 60):
        d = detect(scene(rotated(W / 2, H / 2, 320, deg)))
        if not d or not d["corners"]:
            continue
        xs = [p[0] for p in d["corners"]]; ys = [p[1] for p in d["corners"]]
        pb = [min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys)]
        fark = max(abs(a - b) for a, b in zip(d["box"], pb))
        en_buyuk = max(en_buyuk, fark)
        print("  aci=%2d  rect=%s  polygon-bbox=%s  fark=%d px"
              % (deg, d["box"], pb, fark))
    print()
    check("rect ~ polygon bbox (fark < 10 px)", en_buyuk < 10,
          "en buyuk fark %.0f px" % en_buyuk)
    print()


def test_merkez():
    print("=" * 80)
    print("  3) MERKEZ: kosegen kesisimi vs kose ortalamasi vs kutu ortasi")
    print("=" * 80)
    print("  %-26s %12s %12s" % ("sahne", "ort. sapma", "kutu sapma"))
    print("  " + "-" * 54)

    en_buyuk_ort = 0.0
    en_buyuk_kutu = 0.0
    for ad, squash in (("perspektif yok", 1.00), ("gercekci dalis", 0.95),
                       ("agresif", 0.85), ("cok agresif", 0.70)):
        quad = trapezoid(W / 2, H / 2, 320, squash)
        d = detect(scene(quad))
        if not d or not d["corners"]:
            print("  %-26s decode YOK" % ad); continue
        c = d["corners"]
        kk = quad_center(c)                                   # kosegen kesisimi
        ort = (sum(p[0] for p in c) / 4.0, sum(p[1] for p in c) / 4.0)
        kutu = (d["box"][0] + d["box"][2] / 2.0, d["box"][1] + d["box"][3] / 2.0)
        d_ort = math.hypot(kk[0] - ort[0], kk[1] - ort[1])
        d_kutu = math.hypot(kk[0] - kutu[0], kk[1] - kutu[1])
        en_buyuk_ort = max(en_buyuk_ort, d_ort)
        en_buyuk_kutu = max(en_buyuk_kutu, d_kutu)
        print("  %-26s %9.1f px %9.1f px" % (ad, d_ort, d_kutu))

    print()
    check("kosegen kesisimi ile ortalama AYRISIYOR (perspektifte)",
          en_buyuk_ort > 1.0, "en buyuk %.1f px" % en_buyuk_ort)

    # Eksen-hizali karede uc yontem de ayni cikmali
    d = detect(scene(rotated(W / 2, H / 2, 320, 0)))
    c = d["corners"]; kk = quad_center(c)
    kutu = (d["box"][0] + d["box"][2] / 2.0, d["box"][1] + d["box"][3] / 2.0)
    check("eksen-hizali karede kosegen == kutu ortasi",
          math.hypot(kk[0] - kutu[0], kk[1] - kutu[1]) < 2.0,
          "%.1f px" % math.hypot(kk[0] - kutu[0], kk[1] - kutu[1]))

    # Donmus karede de simetri geregi ayni olmali
    d = detect(scene(rotated(W / 2, H / 2, 320, 40)))
    c = d["corners"]; kk = quad_center(c)
    kutu = (d["box"][0] + d["box"][2] / 2.0, d["box"][1] + d["box"][3] / 2.0)
    check("donmus karede de kosegen == kutu ortasi (simetri)",
          math.hypot(kk[0] - kutu[0], kk[1] - kutu[1]) < 3.0,
          "%.1f px" % math.hypot(kk[0] - kutu[0], kk[1] - kutu[1]))
    print()


def test_dejenere():
    print("=" * 80)
    print("  4) DEJENERE GIRDI")
    print("=" * 80)
    check("None -> None", quad_center(None) is None)
    check("3 kose -> None", quad_center([[0, 0], [1, 0], [1, 1]]) is None)
    check("cakisik kosegen -> None",
          quad_center([[0, 0], [1, 1], [2, 2], [3, 3]]) is None)
    c = quad_center([[0, 0], [10, 0], [10, 10], [0, 10]])
    check("birim kare -> (5,5)", c and abs(c[0] - 5) < 1e-6 and abs(c[1] - 5) < 1e-6,
          str(c))
    check("quad_in_av: tamamen icerde",
          quad_in_av([[500, 200], [600, 200], [600, 300], [500, 300]], *AV))
    check("quad_in_av: bir kose disarda",
          not quad_in_av([[470, 200], [600, 200], [600, 300], [470, 300]], *AV))
    print()


def main():
    print()
    test_iddia_kontrolu()
    test_rect_vs_polygon_bbox()
    test_merkez()
    test_dejenere()
    print("=" * 80)
    ok = sum(1 for p in PASS if p)
    print("  SONUC: %d/%d" % (ok, len(PASS)))
    print("=" * 80)
    return 0 if ok == len(PASS) else 1


if __name__ == "__main__":
    sys.exit(main())
