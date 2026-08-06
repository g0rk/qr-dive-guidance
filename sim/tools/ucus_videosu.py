#!/usr/bin/env python3
"""Record the camera feed to MP4 for a whole flight, running the REAL
perception code on every frame.

    python3 sim/tools/ucus_videosu.py [seconds]

IT DOES TWO JOBS:
  1. DIAGNOSIS - does the QR enter the frame, does it decode when it does,
     and is it inside the target area? A row is written for every frame.
  2. REVIEW - the output is /tmp/flight.mp4, so what the aircraft saw can be
     watched back.

⚠️ THE DECODE RUNS AT FULL RESOLUTION; only the video is written scaled
   down. Decoding from a downscaled frame would artificially shorten the
   range and corrupt the measurement.
"""
import os
import sys
import time

SIM = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, SIM)

import logging
logging.disable(logging.WARNING)

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import config
from processes.perception import (PerceptionProcess, polygon_to_corners,
                                  quad_center, quad_in_av)
from pyzbar.pyzbar import decode as zbar_decode


class _OverlayShim(PerceptionProcess):
    """Borrows the real drawing code without starting a perception process.

    ⚠️ WHY: this tool used to draw its OWN overlay. That meant the recorded
       video did not show what the aircraft actually drew - there were two
       separate drawing implementations that could drift apart silently. A
       recording has to show the very thing it claims to show.
    """

    def __init__(self):
        pass

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
MP4 = "/tmp/flight.mp4"
VIDEO_SCALE = 0.5          # downscale for the video only (decode is unaffected)
REPORT = "/tmp/frame_report.txt"


class Recorder(Node):
    def __init__(self):
        super().__init__("ucus_videosu")
        self.bridge = CvBridge()
        self.writer = None
        self.n = 0
        self.decoded = 0
        self.in_av = 0
        self.t0 = time.monotonic()
        self._overlay = _OverlayShim()
        self.report = open(REPORT, "w")
        self.report.write("frame,t,decode,text,in_av,qr_px\n")
        self.create_subscription(Image, "/camera", self._frame, 1)
        print("listening on /camera  ->  %s" % MP4, flush=True)

    def _frame(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]
        self.n += 1
        t = time.monotonic() - self.t0

        # --- Target area rectangle ---
        ax1, ay1 = int(w * config.AV_MARGIN_X), int(h * config.AV_MARGIN_Y)
        ax2, ay2 = w - ax1, h - ay1

        # --- Decode at FULL resolution ---
        text, in_av, qr_px = "", False, 0
        found = zbar_decode(frame)
        if found:
            b = found[0]
            text = b.data.decode("utf-8", "replace")
            self.decoded += 1
            corners = polygon_to_corners(getattr(b, "polygon", None), 0, 0, 1)
            x, y, bw, bh = b.rect
            qr_px = max(bw, bh)
            qc = quad_center(corners)
            if corners and qc is not None:
                in_av = quad_in_av(corners, ax1, ay1, ax2, ay2)
                cx, cy = qc
                source = "quad"
            else:
                in_av = (x >= ax1 and y >= ay1
                         and (x + bw) <= ax2 and (y + bh) <= ay2)
                cx, cy = x + bw / 2.0, y + bh / 2.0
                source = "box"
            if in_av:
                self.in_av += 1
            color = (0, 255, 0) if in_av else (0, 165, 255)
            # The overlay the AIRCRAFT actually draws - not a second copy.
            self._overlay._draw_qr_overlay(frame, corners, (x, y, bw, bh),
                                           (cx, cy), color, text, in_av,
                                           source)

        cv2.rectangle(frame, (ax1, ay1), (ax2, ay2), (255, 255, 0), 2)
        cv2.putText(frame, "AV", (ax1 + 8, ay1 + 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, "frame %d   t=%.1fs" % (self.n, t),
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (230, 230, 230), 2)

        self.report.write("%d,%.3f,%d,%s,%d,%d\n" % (
            self.n, t, 1 if text else 0, text, 1 if in_av else 0, qr_px))

        small = cv2.resize(frame, None, fx=VIDEO_SCALE, fy=VIDEO_SCALE)
        if self.writer is None:
            self.writer = cv2.VideoWriter(
                MP4, cv2.VideoWriter_fourcc(*"mp4v"), 20.0,
                (small.shape[1], small.shape[0]))
        self.writer.write(small)

        if self.n % 100 == 0:
            print("  frame %d  t=%.0fs  decoded=%d  in_av=%d"
                  % (self.n, t, self.decoded, self.in_av), flush=True)

    def close(self):
        if self.writer:
            self.writer.release()
        self.report.close()


def main():
    rclpy.init()
    d = Recorder()
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < DURATION:
        rclpy.spin_once(d, timeout_sec=0.2)
    d.close()
    print()
    print("=" * 60)
    print("  total frames      : %d" % d.n)
    print("  QR decoded        : %d" % d.decoded)
    print("  inside target area: %d" % d.in_av)
    print("  video             : %s" % MP4)
    print("  frame report      : %s" % REPORT)
    print("=" * 60)


if __name__ == "__main__":
    main()
