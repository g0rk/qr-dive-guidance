# tests/test_center.py - the true centre, and where in_av comes from
#
#     python3 tests/test_center.py
#
# ⚠️ THIS TEST WAS WRITTEN TO EXAMINE A CLAIM:
#    "because the bounding box is up to 2x larger than the QR, the in_av test
#     can reject a valid hit."
#    The target area is an axis-aligned rectangle. A quadrilateral lies inside
#    such a rectangle <=> all four of its corners do <=> the min/max of those
#    corners does. And the min/max of the corners IS THE BOUNDING BOX ITSELF.
#    So the two tests may well be the same thing. What follows measures it.

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
    # ⚠️ The `assert` was added later: this helper used to NOT fail pytest, so
    #    the suite stayed green while individual checks printed FAIL.
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "PASS" if cond else "FAIL", detail))
    assert cond, "%s  %s" % (label, detail)


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
def test_the_claim():
    print("=" * 80)
    print("  1) TESTING THE CLAIM: do box-in_av and quad-in_av ever disagree?")
    print("=" * 80)
    print("  target area = x[%d..%d]  y[%d..%d]" % (AV[0], AV[2], AV[1], AV[3]))
    print()

    disagreements = 0
    total = 0
    examples = []
    for deg in (0, 15, 30, 45, 60):
        for cx in range(AV[0] - 60, AV[0] + 340, 40):      # sweep across the left edge
            quad = rotated(cx, H / 2, 300, deg)
            d = detect(scene(quad))
            if not d or not d["corners"]:
                continue
            total += 1
            a = box_in_av(d["box"])
            b = quad_in_av(d["corners"], *AV)
            if a != b:
                disagreements += 1
                if len(examples) < 4:
                    examples.append((deg, cx, a, b))

    print("  positions/angles swept: %d" % total)
    print("  cases where the two DISAGREE: %d" % disagreements)
    for deg, cx, a, b in examples:
        print("     angle=%d cx=%d -> box=%s quad=%s" % (deg, cx, a, b))
    print()

    check("the two tests behave identically (as expected: the area is axis-aligned)",
          disagreements == 0, "%d disagreements" % disagreements)
    print()
    print("  -> Because the target area is axis-aligned, 'quad inside' and")
    print("     'box inside' are MATHEMATICALLY THE SAME condition. The box")
    print("     having twice the area does not change that. The earlier")
    print("     argument that 'a valid hit gets rejected' is INVALID, and this")
    print("     is the measurement that refutes it.")
    print()


def test_rect_vs_polygon_bbox():
    print("=" * 80)
    print("  2) is pyzbar's `rect` the same as the polygon's bbox?")
    print("=" * 80)
    largest = 0.0
    for deg in (0, 20, 40, 60):
        d = detect(scene(rotated(W / 2, H / 2, 320, deg)))
        if not d or not d["corners"]:
            continue
        xs = [p[0] for p in d["corners"]]; ys = [p[1] for p in d["corners"]]
        pb = [min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys)]
        diff = max(abs(a - b) for a, b in zip(d["box"], pb))
        largest = max(largest, diff)
        print("  angle=%2d  rect=%s  polygon-bbox=%s  diff=%d px"
              % (deg, d["box"], pb, diff))
    print()
    check("rect ~ polygon bbox (diff < 10 px)", largest < 10,
          "largest diff %.0f px" % largest)
    print()


def test_centre():
    print("=" * 80)
    print("  3) CENTRE: diagonal intersection vs corner average vs box midpoint")
    print("=" * 80)
    print("  %-26s %12s %12s" % ("scene", "vs average", "vs box"))
    print("  " + "-" * 54)

    largest_avg = 0.0
    largest_box = 0.0
    for name, squash in (("no perspective", 1.00), ("realistic dive", 0.95),
                         ("aggressive", 0.85), ("very aggressive", 0.70)):
        quad = trapezoid(W / 2, H / 2, 320, squash)
        d = detect(scene(quad))
        if not d or not d["corners"]:
            print("  %-26s NO decode" % name); continue
        c = d["corners"]
        diag = quad_center(c)                                 # diagonal intersection
        avg = (sum(p[0] for p in c) / 4.0, sum(p[1] for p in c) / 4.0)
        box = (d["box"][0] + d["box"][2] / 2.0, d["box"][1] + d["box"][3] / 2.0)
        d_avg = math.hypot(diag[0] - avg[0], diag[1] - avg[1])
        d_box = math.hypot(diag[0] - box[0], diag[1] - box[1])
        largest_avg = max(largest_avg, d_avg)
        largest_box = max(largest_box, d_box)
        print("  %-26s %9.1f px %9.1f px" % (name, d_avg, d_box))

    print()
    check("the diagonal intersection DIVERGES from the average (under perspective)",
          largest_avg > 1.0, "largest %.1f px" % largest_avg)

    # On an axis-aligned square all three methods must agree.
    d = detect(scene(rotated(W / 2, H / 2, 320, 0)))
    c = d["corners"]; diag = quad_center(c)
    box = (d["box"][0] + d["box"][2] / 2.0, d["box"][1] + d["box"][3] / 2.0)
    check("axis-aligned square: diagonal == box midpoint",
          math.hypot(diag[0] - box[0], diag[1] - box[1]) < 2.0,
          "%.1f px" % math.hypot(diag[0] - box[0], diag[1] - box[1]))

    # On a rotated square they must still agree, by symmetry.
    d = detect(scene(rotated(W / 2, H / 2, 320, 40)))
    c = d["corners"]; diag = quad_center(c)
    box = (d["box"][0] + d["box"][2] / 2.0, d["box"][1] + d["box"][3] / 2.0)
    check("rotated square too: diagonal == box midpoint (symmetry)",
          math.hypot(diag[0] - box[0], diag[1] - box[1]) < 3.0,
          "%.1f px" % math.hypot(diag[0] - box[0], diag[1] - box[1]))
    print()


def test_degenerate():
    print("=" * 80)
    print("  4) DEGENERATE INPUT")
    print("=" * 80)
    check("None -> None", quad_center(None) is None)
    check("3 corners -> None", quad_center([[0, 0], [1, 0], [1, 1]]) is None)
    check("collinear diagonals -> None",
          quad_center([[0, 0], [1, 1], [2, 2], [3, 3]]) is None)
    c = quad_center([[0, 0], [10, 0], [10, 10], [0, 10]])
    check("unit square -> (5,5)", c and abs(c[0] - 5) < 1e-6 and abs(c[1] - 5) < 1e-6,
          str(c))
    check("quad_in_av: fully inside",
          quad_in_av([[500, 200], [600, 200], [600, 300], [500, 300]], *AV))
    check("quad_in_av: one corner outside",
          not quad_in_av([[470, 200], [600, 200], [600, 300], [470, 300]], *AV))
    print()


def main():
    print()
    test_the_claim()
    test_rect_vs_polygon_bbox()
    test_centre()
    test_degenerate()
    print("=" * 80)
    ok = sum(1 for p in PASS if p)
    print("  RESULT: %d/%d" % (ok, len(PASS)))
    print("=" * 80)
    return 0 if ok == len(PASS) else 1


if __name__ == "__main__":
    sys.exit(main())
