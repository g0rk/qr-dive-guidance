#!/usr/bin/env python3
"""Ucus boyunca kamera goruntusunu MP4'e kaydeder + her karede GERCEK algi
kodunu kosturur.

    python3 sim/tools/ucus_videosu.py [saniye]

IKI ISE YARAR:
  1. TESHIS - QR kadraja giriyor mu, giriyorsa decode oluyor mu, AV'nin
     icinde mi? Her kare icin kayit tutulur.
  2. IZLEME - ciktisi /tmp/ucus.mp4; ucagin gordugunu gozle izlemek icin.

⚠️ DECODE TAM COZUNURLUKTE yapilir; video kucultulerek yazilir. Kucultulmus
   kareden decode etmek menzili yapay olarak kisaltir ve olcumu bozar.
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
from processes.perception import polygon_to_corners, quad_center, quad_in_av
from pyzbar.pyzbar import decode as zbar_decode

SURE = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
MP4 = "/tmp/ucus.mp4"
KAYIT_OLCEK = 0.5          # video icin kucultme (decode'a etki etmez)
RAPOR = "/tmp/kare_raporu.txt"


class Kaydedici(Node):
    def __init__(self):
        super().__init__("ucus_videosu")
        self.bridge = CvBridge()
        self.yazici = None
        self.n = 0
        self.decode_sayisi = 0
        self.av_ici = 0
        self.t0 = time.monotonic()
        self.rapor = open(RAPOR, "w")
        self.rapor.write("kare,t,decode,metin,av_ici,qr_px\n")
        self.create_subscription(Image, "/camera", self._kare, 1)
        print("dinleniyor: /camera  ->  %s" % MP4, flush=True)

    def _kare(self, msg):
        kare = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = kare.shape[:2]
        self.n += 1
        t = time.monotonic() - self.t0

        # --- AV dortgeni (sartname Sekil 2/4) ---
        ax1, ay1 = int(w * config.AV_MARGIN_X), int(h * config.AV_MARGIN_Y)
        ax2, ay2 = w - ax1, h - ay1

        # --- TAM COZUNURLUKTE decode ---
        metin, av_ici, qr_px = "", False, 0
        bulundu = zbar_decode(kare)
        if bulundu:
            b = bulundu[0]
            metin = b.data.decode("utf-8", "replace")
            self.decode_sayisi += 1
            kose = polygon_to_corners(getattr(b, "polygon", None), 0, 0, 1)
            qr_px = max(b.rect.width, b.rect.height)
            if kose and quad_center(kose) is not None:
                av_ici = quad_in_av(kose, ax1, ay1, ax2, ay2)
                if av_ici:
                    self.av_ici += 1
                renk = (0, 255, 0) if av_ici else (0, 165, 255)
                for i in range(4):
                    cv2.line(kare, tuple(kose[i]), tuple(kose[(i + 1) % 4]), renk, 3)

        cv2.rectangle(kare, (ax1, ay1), (ax2, ay2), (255, 255, 0), 2)
        cv2.putText(kare, "kare %d  t=%.1fs  %s%s" % (
            self.n, t, metin or "QR yok", "  [AV ICI]" if av_ici else
            ("  [AV DISI]" if metin else "")),
            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
            (0, 255, 0) if av_ici else (0, 0, 255), 2)

        self.rapor.write("%d,%.3f,%d,%s,%d,%d\n" % (
            self.n, t, 1 if metin else 0, metin, 1 if av_ici else 0, qr_px))

        kucuk = cv2.resize(kare, None, fx=KAYIT_OLCEK, fy=KAYIT_OLCEK)
        if self.yazici is None:
            self.yazici = cv2.VideoWriter(
                MP4, cv2.VideoWriter_fourcc(*"mp4v"), 20.0,
                (kucuk.shape[1], kucuk.shape[0]))
        self.yazici.write(kucuk)

        if self.n % 100 == 0:
            print("  kare %d  t=%.0fs  decode=%d  AV_ici=%d"
                  % (self.n, t, self.decode_sayisi, self.av_ici), flush=True)

    def kapat(self):
        if self.yazici:
            self.yazici.release()
        self.rapor.close()


def main():
    rclpy.init()
    d = Kaydedici()
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < SURE:
        rclpy.spin_once(d, timeout_sec=0.2)
    d.kapat()
    print()
    print("=" * 60)
    print("  toplam kare      : %d" % d.n)
    print("  QR decode edilen : %d" % d.decode_sayisi)
    print("  AV ICINDE olan   : %d" % d.av_ici)
    print("  video            : %s" % MP4)
    print("  kare raporu      : %s" % RAPOR)
    print("=" * 60)


if __name__ == "__main__":
    main()
