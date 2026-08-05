#!/usr/bin/env python3
"""HUD cizimini gozle dogrulamak icin: egik bir QR sahnesi uretip render eder."""
import os, sys, math
SIM = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, SIM)
import logging; logging.disable(logging.CRITICAL)

import numpy as np, cv2, qrcode
from pyzbar.pyzbar import decode
from processes.perception import PerceptionProcess, polygon_to_corners, quad_center, quad_in_av
import config

W, H = 1920, 1080


def qr_tile(px=420):
    # mask_pattern is pinned. The library picks a mask from the payload, and
    # the mask decides how well the pattern survives downscaling and the
    # perspective warp below. Measured over a 0-90 deg sweep: mask 1 decodes
    # at every angle, the auto-picked mask does not. It must also match the
    # world texture in sim/models/qr_pad_v1 - one source of truth, not two.
    q = qrcode.QRCode(version=1, error_correction=qrcode.constants.ERROR_CORRECT_M,
                      box_size=10, border=2, mask_pattern=1)
    q.add_data("dive_pad"); q.make(fit=False)
    a = cv2.cvtColor(np.array(q.make_image(fill_color="black",
                                           back_color="white").convert("RGB")),
                     cv2.COLOR_RGB2BGR)
    return cv2.resize(a, (px, px), interpolation=cv2.INTER_AREA)


def sahne(donme_deg, perspektif=0.22):
    """Cimen zemin + egik/perspektifli QR."""
    f = np.full((H, W, 3), (60, 105, 70), np.uint8)
    f += np.random.randint(-8, 8, f.shape, dtype=np.int16).astype(np.uint8)
    tile = qr_tile()
    s = tile.shape[0]
    src = np.float32([[0, 0], [s, 0], [s, s], [0, s]])
    cx, cy, r = W * 0.5, H * 0.52, s * 0.62
    pts = []
    for i, (dx, dy) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
        a = math.radians(donme_deg)
        rx = dx * math.cos(a) - dy * math.sin(a)
        ry = dx * math.sin(a) + dy * math.cos(a)
        k = 1.0 - perspektif * (ry + 1) / 2.0     # ust kenar daha uzak
        pts.append([cx + rx * r * k, cy + ry * r * k])
    Mx = cv2.getPerspectiveTransform(src, np.float32(pts))
    warp = cv2.warpPerspective(tile, Mx, (W, H), borderMode=cv2.BORDER_TRANSPARENT,
                               dst=f.copy())
    mask = cv2.warpPerspective(np.full((s, s), 255, np.uint8), Mx, (W, H))
    f[mask > 0] = warp[mask > 0]
    return f


class Sahte(PerceptionProcess):
    def __init__(self):
        pass


def main():
    p = Sahte()
    out = "/tmp/hud"
    os.makedirs(out, exist_ok=True)
    av_x1, av_y1 = int(W * config.AV_MARGIN_X), int(H * config.AV_MARGIN_Y)
    av_x2, av_y2 = W - av_x1, H - av_y1

    for ad, donme in (("duz", 0.0), ("egik30", 30.0), ("egik45", 45.0)):
        f = sahne(donme)
        bulunan = decode(f)
        if not bulunan:
            print("  %s: decode YOK" % ad); continue
        b = bulunan[0]
        corners = polygon_to_corners(getattr(b, "polygon", None), 0, 0, 1)
        x, y, bw, bh = b.rect
        qc = quad_center(corners)
        if corners and qc:
            in_av = quad_in_av(corners, av_x1, av_y1, av_x2, av_y2)
            cx, cy = qc; kaynak = "quad"
        else:
            in_av = True; cx, cy = x + bw / 2, y + bh / 2; kaynak = "box"

        cv2.rectangle(f, (av_x1, av_y1), (av_x2, av_y2), (255, 255, 0), 2)
        renk = (0, 255, 0) if in_av else (0, 165, 255)
        p._draw_qr_overlay(f, corners, (x, y, bw, bh), (cx, cy), renk,
                           b.data.decode(), in_av, kaynak)

        # kutu ile dortgeni karsilastir: kutuyu ince gri ciz
        cv2.rectangle(f, (x, y), (x + bw, y + bh), (150, 150, 150), 1)
        kutu_alan = bw * bh
        dort_alan = cv2.contourArea(np.array(corners, np.int32)) if corners else kutu_alan
        cv2.putText(f, "rotation %.0f deg   box/quad area = %.2fx"
                    % (donme, kutu_alan / max(dort_alan, 1)),
                    (30, H - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        kirp = f[max(0, int(cy) - 380):int(cy) + 380, max(0, int(cx) - 640):int(cx) + 640]
        cv2.imwrite("%s/%s.png" % (out, ad), kirp)
        print("  %s -> box/quad=%.2fx  merkez=(%d,%d)  kaynak=%s"
              % (ad, kutu_alan / max(dort_alan, 1), cx, cy, kaynak))


if __name__ == "__main__":
    main()
