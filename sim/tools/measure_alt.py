#!/usr/bin/env python3
"""Measure QR decode against altitude - REAL gz render, REAL perception code."""
import math
import os
import sys

# ⚠️ The repository root is derived FROM THIS FILE'S OWN LOCATION - never
#    written as an absolute path. It used to be hard-coded as
#    "/mnt/c/Users/<name>/...", which both failed on anyone else's machine
#    and leaked a username into a public repository.
SIM = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, SIM)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cam_params  # the single source of camera parameters (reads model.sdf)

import logging
logging.disable(logging.WARNING)

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from processes.perception import PerceptionProcess

ALTS = [60, 50, 40, 30, 25, 20]
WANT = 4
OUT = "/tmp/alt"
os.makedirs(OUT, exist_ok=True)


class Q:
    def __init__(self):
        self.items = []

    def put_nowait(self, x):
        self.items.append(x)

    def get_nowait(self):
        raise Exception("empty")


class M(Node):
    def __init__(self):
        super().__init__("alt_measure")
        self.b = CvBridge()
        self.p = PerceptionProcess(Q(), Q())
        self.n = {a: 0 for a in ALTS}
        self.hit = {a: 0 for a in ALTS}
        self.px = {a: 0 for a in ALTS}
        self.inav = {a: 0 for a in ALTS}
        for a in ALTS:
            self.create_subscription(Image, "/cam%d" % a, self._cb(a), 1)

    def _cb(self, alt):
        def cb(msg):
            if self.n[alt] >= WANT:
                return
            f = self.b.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.n[alt] += 1
            vis = f.copy()
            self.p.result_queue = Q()
            self.p._process_qr(vis, set())
            r = self.p.result_queue.items
            if r:
                self.hit[alt] += 1
                b = r[0]["box"]
                self.px[alt] = max(self.px[alt], max(b[2], b[3]))
                if r[0].get("in_av"):
                    self.inav[alt] += 1
                if self.hit[alt] == 1:
                    self.p._draw_hud(vis, vis.shape[1], vis.shape[0], 30.0)
                    cv2.imwrite(os.path.join(OUT, "alt%02d.png" % alt), vis)
            elif self.n[alt] == WANT:
                self.p._draw_hud(vis, vis.shape[1], vis.shape[0], 30.0)
                cv2.imwrite(os.path.join(OUT, "alt%02d_yok.png" % alt), vis)
        return cb

    def done(self):
        return all(v >= WANT for v in self.n.values())


def main():
    rclpy.init()
    node = M()
    t = 0.0
    while rclpy.ok() and not node.done() and t < 45.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        t += 0.2

    # ⚠️ This used to be written out as `1920 / math.degrees(0.8954)`. If the
    #    camera parameter changed in model.sdf, this table would silently
    #    produce a wrong "theoretical px" column. It now comes from the single
    #    source (cam_params.py).
    ppd = cam_params.pixels_per_degree()
    print()
    print("=" * 74)
    print("  QR DECODE vs ALTITUDE  (gz render, real perception code)")
    print("  V1 QR, 2-module quiet zone, 2 m plates, 55 degree dive")
    print("=" * 74)
    print("  %6s %8s %8s %9s %8s %8s" %
          ("alt", "ground", "slant", "theory px", "measured", "decode"))
    print("  " + "-" * 60)
    for a in ALTS:
        d = a / math.tan(math.radians(55.0))
        slant = math.hypot(a, d)
        teo = math.degrees(2 * math.atan(1.0 / slant)) * ppd
        oran = "%d/%d" % (node.hit[a], node.n[a])
        print("  %5d m %7.1f m %7.1f m %8.0f %8s %8s%s" %
              (a, d, slant, teo, node.px[a] or "-", oran,
               "  (AV %d)" % node.inav[a] if node.hit[a] else ""))

    ok = [a for a in ALTS if node.hit[a] > 0]
    print()
    if ok:
        print("  HIGHEST ALTITUDE THAT DECODES: %d m" % max(ok))
        print("     (few frames per step; instability near the threshold is normal)")
    else:
        print("  NO DECODE AT ANY ALTITUDE")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
