# tests/test_faz1_corners.py - Faz 1: QR'in 4 kosesinin cikarilmasi
#
#     python3 tests/test_faz1_corners.py
#
# NEDEN: `barcode.rect` eksen-hizali SINIRLAYICI KUTUDUR. QR dalista
# ~55 derecelik aciyla ve zeminde donmus gorulur; goruntude bir KARE degil
# bir DORTGEN olusur. Kutu o dortgeni cevreler -> QR'dan buyuktur.
# Bu test hem cikarmayi dogrular hem de FARKI OLCER (Faz 2'nin gerekcesi).

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

from processes.perception import polygon_to_corners

W, H = 1920, 1080
PASS = []


def check(label, cond, detail=""):
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "GECTI" if cond else "KALDI", detail))


def qr_tile(px=500):
    q = qrcode.QRCode(version=1, error_correction=qrcode.constants.ERROR_CORRECT_M,
                      box_size=10, border=2)
    q.add_data("teknofest2026")
    q.make(fit=False)
    a = cv2.cvtColor(
        np.array(q.make_image(fill_color="black", back_color="white").convert("RGB")),
        cv2.COLOR_RGB2BGR)
    return cv2.resize(a, (px, px), interpolation=cv2.INTER_AREA)


TILE = qr_tile()


def scene_from_quad(quad):
    """Verilen dortgene QR'i perspektifle yerlestir."""
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


def axis_aligned(cx, cy, s):
    h = s / 2.0
    return [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)]


def rotated(cx, cy, s, deg):
    r = math.radians(deg)
    h = s / 2.0
    out = []
    for dx, dy in ((-h, -h), (h, -h), (h, h), (-h, h)):
        out.append((cx + dx * math.cos(r) - dy * math.sin(r),
                    cy + dx * math.sin(r) + dy * math.cos(r)))
    return out


def perspective_quad(cx, cy, s, squash=0.95):
    """
    Yamuk: uzak kenar dar, yakin kenar genis (dalis perspektifi).

    squash=0.95 GERCEKCI: 55 derece bakis acisiyla 2 m QR'a ~30 m egik
    mesafeden bakildiginda yakin/uzak kenar orani ~1.04. Daha agresif
    degerler (0.55 gibi) ne gercegi temsil eder ne de decode olur.
    """
    h = s / 2.0
    top = h * squash
    return [(cx - top, cy - h), (cx + top, cy - h), (cx + h, cy + h), (cx - h, cy + h)]


def poly_area(pts):
    a = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return abs(a) / 2.0


def detect(frame):
    """Faz 1'in yolunu taklit et: ROI kirp, decode, koseleri geri cevir."""
    x0, y0 = int(W * 0.17), int(H * 0.02)
    x3, y3 = W - x0, H - y0
    roi = frame[y0:y3, x0:x3]
    res = decode(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY))
    if not res:
        return None, None, None
    b = res[0]
    bx, by, bw, bh = b.rect
    box = [x0 + bx, y0 + by, bw, bh]
    corners = polygon_to_corners(b.polygon, x0, y0, 1)
    return b, box, corners


# ======================================================================
def test_extraction():
    print("=" * 80)
    print("  1) KOSELERIN CIKARILMASI ve TAM KARE KOORDINATI")
    print("=" * 80)

    quad = axis_aligned(W / 2, H / 2, 300)
    b, box, corners = detect(scene_from_quad(quad))
    check("eksen-hizali QR decode edildi", b is not None)
    check("4 kose donduruldu", corners is not None and len(corners) == 4,
          str(corners))

    if corners:
        # ⚠️ pyzbar QR'in GERCEK sinirini verir, resmin sinirini DEGIL.
        #    Uretilen karo 2 modulluk sessiz bolge (quiet zone) iceriyor:
        #    Versiyon 1 = 21 modul + 2+2 boslu = 25 modul.
        #    Yani 300 px'lik karoda QR'in kendisi 300*21/25 = 252 px olmali.
        beklenen = 300.0 * 21.0 / 25.0
        genislik = max(p[0] for p in corners) - min(p[0] for p in corners)
        yukseklik = max(p[1] for p in corners) - min(p[1] for p in corners)
        check("QR sinirinin acikligi dogru (sessiz bolge haric)",
              abs(genislik - beklenen) < 8 and abs(yukseklik - beklenen) < 8,
              "%dx%d px (beklenen %.0f)" % (genislik, yukseklik, beklenen))

        # ROI ofseti geri cevrildi mi: merkez kare merkezinde olmali
        mcx = sum(p[0] for p in corners) / 4.0
        mcy = sum(p[1] for p in corners) / 4.0
        sapma = math.hypot(mcx - W / 2.0, mcy - H / 2.0)
        check("ROI ofseti geri cevrildi (tam kare koordinati)", sapma < 8,
              "merkez sapmasi %.1f px" % sapma)
        check("koseler kare merkezine yakin (ROI koordinati DEGIL)",
              all(abs(p[0] - W / 2) < 250 for p in corners),
              "x araligi %d..%d" % (min(p[0] for p in corners),
                                    max(p[0] for p in corners)))
    print()


def test_determinism():
    print("=" * 80)
    print("  2) SIRA TUTARLI MI (ayni sahne -> ayni cikti)")
    print("=" * 80)

    quad = rotated(W / 2, H / 2, 320, 33)
    f = scene_from_quad(quad)
    _, _, c1 = detect(f)
    _, _, c2 = detect(f)
    check("iki kosumda ayni sira", c1 is not None and c1 == c2, str(c1))

    # Farkli donme acilarinda da 4 kose gelmeli
    ok = 0
    for deg in (0, 15, 30, 45, 60, 75):
        _, _, c = detect(scene_from_quad(rotated(W / 2, H / 2, 320, deg)))
        if c and len(c) == 4:
            ok += 1
    check("cesitli donme acilarinda 4 kose", ok >= 5, "%d/6 aci" % ok)
    print()


def test_degenerate():
    print("=" * 80)
    print("  3) DEJENERE DURUM (pyzbar 4'ten farkli nokta dondurebilir)")
    print("=" * 80)

    class P:
        def __init__(self, x, y):
            self.x, self.y = x, y

    check("None -> None", polygon_to_corners(None, 0, 0, 1) is None)
    check("3 nokta -> None",
          polygon_to_corners([P(0, 0), P(1, 0), P(1, 1)], 0, 0, 1) is None)
    check("5 nokta -> None",
          polygon_to_corners([P(0, 0), P(1, 0), P(1, 1), P(0, 1), P(0, 2)], 0, 0, 1) is None)
    c = polygon_to_corners([P(0, 0), P(10, 0), P(10, 10), P(0, 10)], 100, 50, 2)
    check("4 nokta + ofset/olcek geri cevriliyor",
          c == [[100, 50], [120, 50], [120, 70], [100, 70]] or
          sorted(c) == sorted([[100, 50], [120, 50], [120, 70], [100, 70]]),
          str(c))
    print()


def test_box_vs_polygon():
    print("=" * 80)
    print("  4) OLCUM: KUTU, QR'DAN NE KADAR BUYUK  (Faz 2'nin gerekcesi)")
    print("=" * 80)
    print("  %-34s %10s %10s %8s" % ("sahne", "kutu alan", "QR alan", "sisme"))
    print("  " + "-" * 66)

    senaryolar = [
        ("eksen-hizali", axis_aligned(W / 2, H / 2, 320)),
        ("zeminde donmus 15", rotated(W / 2, H / 2, 320, 15)),
        ("zeminde donmus 30", rotated(W / 2, H / 2, 320, 30)),
        ("zeminde donmus 45", rotated(W / 2, H / 2, 320, 45)),
        ("dalis perspektifi", perspective_quad(W / 2, H / 2, 320)),
        ("perspektif + 30 donme", None),
    ]
    # son senaryo: perspektifi dondur
    pq = perspective_quad(0, 0, 320)
    r = math.radians(30)
    senaryolar[-1] = ("perspektif + 30 donme",
                      [(W / 2 + x * math.cos(r) - y * math.sin(r),
                        H / 2 + x * math.sin(r) + y * math.cos(r)) for x, y in pq])

    en_kotu = 1.0
    for ad, quad in senaryolar:
        _, box, corners = detect(scene_from_quad(quad))
        if not corners:
            print("  %-34s %10s" % (ad, "decode YOK"))
            continue
        kutu_alan = box[2] * box[3]
        qr_alan = poly_area(corners)
        oran = kutu_alan / qr_alan if qr_alan else 0
        en_kotu = max(en_kotu, oran)
        print("  %-34s %10d %10d %7.2fx" % (ad, kutu_alan, qr_alan, oran))

    print()
    check("donmus QR'da kutu belirgin sisiyor (>1.3x)", en_kotu > 1.3,
          "en kotu %.2fx" % en_kotu)
    print()
    print("  -> Su an `in_av` testi KUTUNUN dort kosesine bakiyor. Kutu QR'dan")
    print("     %.0f%% buyuk oldugunda, QR tamamen AV'nin icindeyken bile kutu" % ((en_kotu - 1) * 100))
    print("     disari tasip in_av=False verebilir -> GECERLI VURUS REDDEDILIR.")
    print("     Faz 2 bu testi gercek dortgene baglayacak.")
    print()


def main():
    print()
    test_extraction()
    test_determinism()
    test_degenerate()
    test_box_vs_polygon()
    print("=" * 80)
    ok = sum(1 for p in PASS if p)
    print("  SONUC: %d/%d" % (ok, len(PASS)))
    print("=" * 80)
    return 0 if ok == len(PASS) else 1


if __name__ == "__main__":
    sys.exit(main())
