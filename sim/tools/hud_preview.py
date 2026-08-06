#!/usr/bin/env python3
"""Render a tilted QR scene so the HUD overlay can be checked by eye."""
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
    # mask_pattern is pinned, and the value was chosen by measurement in
    # Gazebo - not by this synthetic scene.
    #
    # The library picks a mask from the payload. The mask changes the black
    # and white pattern completely, and at the decode threshold - where a
    # module is about two pixels wide - the pattern decides whether the code
    # survives at all. Mask 1 was picked first, from this synthetic scene
    # alone: it decodes at every angle of a 0-90 deg sweep. That was the
    # wrong yardstick. Measured against the real gz render, 5 frames per
    # altitude, 55 deg look-down:
    #
    #     altitude      45 m   40 m   35 m   30 m
    #     mask 1        0/5    0/5    5/5    5/5
    #     mask 6        0/5    5/5    5/5    5/5     <- 69 px at 40 m
    #
    # Mask 6 buys a whole 5 m of decode altitude, which is what the trigger
    # distance in config.py is derived from. It also passes the 0-90 deg
    # sweep, so the preview images below still render.
    #
    # This must match the world texture in sim/models/qr_pad_v1 - one source
    # of truth, not two.
    q = qrcode.QRCode(version=1, error_correction=qrcode.constants.ERROR_CORRECT_M,
                      box_size=10, border=2, mask_pattern=6)
    q.add_data("dive_pad"); q.make(fit=False)
    a = cv2.cvtColor(np.array(q.make_image(fill_color="black",
                                           back_color="white").convert("RGB")),
                     cv2.COLOR_RGB2BGR)
    return cv2.resize(a, (px, px), interpolation=cv2.INTER_AREA)


def scene(rotation_deg, perspective=0.22):
    """A grass background with a rotated, perspective-warped QR on it."""
    f = np.full((H, W, 3), (60, 105, 70), np.uint8)
    f += np.random.randint(-8, 8, f.shape, dtype=np.int16).astype(np.uint8)
    tile = qr_tile()
    s = tile.shape[0]
    src = np.float32([[0, 0], [s, 0], [s, s], [0, s]])
    cx, cy, r = W * 0.5, H * 0.52, s * 0.62
    pts = []
    for i, (dx, dy) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
        a = math.radians(rotation_deg)
        rx = dx * math.cos(a) - dy * math.sin(a)
        ry = dx * math.sin(a) + dy * math.cos(a)
        k = 1.0 - perspective * (ry + 1) / 2.0    # the far edge is smaller
        pts.append([cx + rx * r * k, cy + ry * r * k])
    Mx = cv2.getPerspectiveTransform(src, np.float32(pts))
    warp = cv2.warpPerspective(tile, Mx, (W, H), borderMode=cv2.BORDER_TRANSPARENT,
                               dst=f.copy())
    mask = cv2.warpPerspective(np.full((s, s), 255, np.uint8), Mx, (W, H))
    f[mask > 0] = warp[mask > 0]
    return f


class _Shim(PerceptionProcess):
    def __init__(self):
        pass


def main():
    p = _Shim()
    out = "/tmp/hud"
    os.makedirs(out, exist_ok=True)
    av_x1, av_y1 = int(W * config.AV_MARGIN_X), int(H * config.AV_MARGIN_Y)
    av_x2, av_y2 = W - av_x1, H - av_y1

    # Output names match docs/ so the mapping is obvious.
    for name, rotation in (("hud_head_on", 0.0), ("hud_rotated_30deg", 30.0),
                           ("hud_rotated_45deg", 45.0)):
        f = scene(rotation)
        found = decode(f)
        if not found:
            print("  %s: NO decode" % name); continue
        b = found[0]
        corners = polygon_to_corners(getattr(b, "polygon", None), 0, 0, 1)
        x, y, bw, bh = b.rect
        qc = quad_center(corners)
        if corners and qc:
            in_av = quad_in_av(corners, av_x1, av_y1, av_x2, av_y2)
            cx, cy = qc; source = "quad"
        else:
            in_av = True; cx, cy = x + bw / 2, y + bh / 2; source = "box"

        cv2.rectangle(f, (av_x1, av_y1), (av_x2, av_y2), (255, 255, 0), 2)
        color = (0, 255, 0) if in_av else (0, 165, 255)
        p._draw_qr_overlay(f, corners, (x, y, bw, bh), (cx, cy), color,
                           b.data.decode(), in_av, source)

        # Compare the box against the quadrilateral: draw the box thin and grey.
        cv2.rectangle(f, (x, y), (x + bw, y + bh), (150, 150, 150), 1)
        box_area = bw * bh
        quad_area = cv2.contourArea(np.array(corners, np.int32)) if corners else box_area
        cv2.putText(f, "rotation %.0f deg   box/quad area = %.2fx"
                    % (rotation, box_area / max(quad_area, 1)),
                    (30, H - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        crop = f[max(0, int(cy) - 380):int(cy) + 380, max(0, int(cx) - 640):int(cx) + 640]
        cv2.imwrite("%s/%s.png" % (out, name), crop)
        print("  %s -> box/quad=%.2fx  centre=(%d,%d)  source=%s"
              % (name, box_area / max(quad_area, 1), cx, cy, source))


if __name__ == "__main__":
    main()
