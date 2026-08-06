# tests/test_corners.py - extracting the QR's 4 corners
#
#     python3 tests/test_corners.py
#
# WHY: `barcode.rect` is an axis-aligned BOUNDING BOX. In a dive the QR is
# seen at about 55 degrees and rotated on the ground; in the image it is not
# a SQUARE but a QUADRILATERAL. The box encloses that quadrilateral, so it is
# larger than the QR itself.
# This test verifies the extraction and MEASURES THE DIFFERENCE.

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
    # ⚠️ The `assert` was added later: this helper used to NOT fail pytest, so
    #    the suite stayed green while individual checks printed FAIL.
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "PASS" if cond else "FAIL", detail))
    assert cond, "%s  %s" % (label, detail)


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
    """Place the QR into the given quadrilateral with a perspective warp."""
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
    A trapezoid: the far edge narrow, the near edge wide (dive perspective).

    squash=0.95 is REALISTIC: looking at a 2 m QR from about 30 m of slant
    range at 55 degrees, the near/far edge ratio is roughly 1.04. More
    aggressive values (0.55, say) represent neither reality nor anything that
    decodes.
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
    """Mimic the real path: crop the ROI, decode, map the corners back."""
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
    print("  1) CORNER EXTRACTION AND FULL-FRAME COORDINATES")
    print("=" * 80)

    quad = axis_aligned(W / 2, H / 2, 300)
    b, box, corners = detect(scene_from_quad(quad))
    check("axis-aligned QR decoded", b is not None)
    check("4 corners returned", corners is not None and len(corners) == 4,
          str(corners))

    if corners:
        # ⚠️ pyzbar returns the QR's REAL boundary, not the image's boundary.
        #    The generated tile includes a 2-module quiet zone:
        #    Version 1 = 21 modules + 2 + 2 of margin = 25 modules.
        #    So in a 300 px tile the QR itself should be 300*21/25 = 252 px.
        expected = 300.0 * 21.0 / 25.0
        width = max(p[0] for p in corners) - min(p[0] for p in corners)
        height = max(p[1] for p in corners) - min(p[1] for p in corners)
        check("QR boundary spans the right size (quiet zone excluded)",
              abs(width - expected) < 8 and abs(height - expected) < 8,
              "%dx%d px (expected %.0f)" % (width, height, expected))

        # Was the ROI offset undone? The centre must be the frame centre.
        mcx = sum(p[0] for p in corners) / 4.0
        mcy = sum(p[1] for p in corners) / 4.0
        deviation = math.hypot(mcx - W / 2.0, mcy - H / 2.0)
        check("ROI offset undone (full-frame coordinates)", deviation < 8,
              "centre off by %.1f px" % deviation)
        check("corners near the frame centre (NOT ROI coordinates)",
              all(abs(p[0] - W / 2) < 250 for p in corners),
              "x range %d..%d" % (min(p[0] for p in corners),
                                  max(p[0] for p in corners)))
    print()


def test_determinism():
    print("=" * 80)
    print("  2) IS THE ORDER CONSISTENT (same scene -> same output)")
    print("=" * 80)

    quad = rotated(W / 2, H / 2, 320, 33)
    f = scene_from_quad(quad)
    _, _, c1 = detect(f)
    _, _, c2 = detect(f)
    check("same order across two runs", c1 is not None and c1 == c2, str(c1))

    # 4 corners must come back at a variety of rotation angles too.
    ok = 0
    for deg in (0, 15, 30, 45, 60, 75):
        _, _, c = detect(scene_from_quad(rotated(W / 2, H / 2, 320, deg)))
        if c and len(c) == 4:
            ok += 1
    check("4 corners across various rotations", ok >= 5, "%d/6 angles" % ok)
    print()


def test_degenerate():
    print("=" * 80)
    print("  3) DEGENERATE CASES (pyzbar can return other than 4 points)")
    print("=" * 80)

    class P:
        def __init__(self, x, y):
            self.x, self.y = x, y

    check("None -> None", polygon_to_corners(None, 0, 0, 1) is None)
    check("3 points -> None",
          polygon_to_corners([P(0, 0), P(1, 0), P(1, 1)], 0, 0, 1) is None)
    check("5 points -> None",
          polygon_to_corners([P(0, 0), P(1, 0), P(1, 1), P(0, 1), P(0, 2)], 0, 0, 1) is None)
    c = polygon_to_corners([P(0, 0), P(10, 0), P(10, 10), P(0, 10)], 100, 50, 2)
    check("4 points, offset and scale undone",
          c == [[100, 50], [120, 50], [120, 70], [100, 70]] or
          sorted(c) == sorted([[100, 50], [120, 50], [120, 70], [100, 70]]),
          str(c))
    print()


def test_box_vs_polygon():
    print("=" * 80)
    print("  4) MEASUREMENT: HOW MUCH BIGGER IS THE BOX THAN THE QR")
    print("=" * 80)
    print("  %-34s %10s %10s %8s" % ("scene", "box area", "QR area", "inflation"))
    print("  " + "-" * 66)

    scenarios = [
        ("axis-aligned", axis_aligned(W / 2, H / 2, 320)),
        ("rotated on the ground 15", rotated(W / 2, H / 2, 320, 15)),
        ("rotated on the ground 30", rotated(W / 2, H / 2, 320, 30)),
        ("rotated on the ground 45", rotated(W / 2, H / 2, 320, 45)),
        ("dive perspective", perspective_quad(W / 2, H / 2, 320)),
        ("perspective + 30 rotation", None),
    ]
    # Last scenario: rotate the perspective quad.
    pq = perspective_quad(0, 0, 320)
    r = math.radians(30)
    scenarios[-1] = ("perspective + 30 rotation",
                     [(W / 2 + x * math.cos(r) - y * math.sin(r),
                       H / 2 + x * math.sin(r) + y * math.cos(r)) for x, y in pq])

    worst = 1.0
    for name, quad in scenarios:
        _, box, corners = detect(scene_from_quad(quad))
        if not corners:
            print("  %-34s %10s" % (name, "NO decode"))
            continue
        box_area = box[2] * box[3]
        qr_area = poly_area(corners)
        ratio = box_area / qr_area if qr_area else 0
        worst = max(worst, ratio)
        print("  %-34s %10d %10d %7.2fx" % (name, box_area, qr_area, ratio))

    print()
    check("the box inflates noticeably on a rotated QR (>1.3x)", worst > 1.3,
          "worst %.2fx" % worst)
    print()
    print("  -> The box is up to %.0f%% larger than the QR it claims to outline."
          % ((worst - 1) * 100))
    print()
    print("  ⚠️ AN EARLIER VERSION OF THIS TEST DREW A CONCLUSION THAT WAS")
    print("     LATER MEASURED AND FOUND FALSE. It argued that because the box")
    print("     is so much larger, a QR fully inside the target area could")
    print("     still be reported as outside, rejecting a valid hit.")
    print()
    print("     That is wrong. The target area is an AXIS-ALIGNED rectangle,")
    print("     and a quadrilateral lies inside one exactly when its four")
    print("     corners do - which is the bounding box test. The two are")
    print("     identical. tests/test_center.py measures it across 41")
    print("     positions and angles: zero disagreement.")
    print()
    print("     The corners earn their place through the CENTRE estimate,")
    print("     pose estimation and the overlay - not through containment.")
    print()


def main():
    print()
    test_extraction()
    test_determinism()
    test_degenerate()
    test_box_vs_polygon()
    print("=" * 80)
    ok = sum(1 for p in PASS if p)
    print("  RESULT: %d/%d" % (ok, len(PASS)))
    print("=" * 80)
    return 0 if ok == len(PASS) else 1


if __name__ == "__main__":
    sys.exit(main())
