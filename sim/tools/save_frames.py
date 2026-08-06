#!/usr/bin/env python3
"""Save frames arriving over the ROS2 bridge as PNGs, running the real
perception code on each one."""
import os
import sys

# ⚠️ The repository root is derived FROM THIS FILE'S OWN LOCATION - never
#    written as an absolute path. It used to be hard-coded as
#    "/mnt/c/Users/<name>/...", which both failed on anyone else's machine
#    and leaked a username into a public repository.
SIM = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, SIM)

import logging
logging.disable(logging.WARNING)

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from processes.perception import PerceptionProcess

OUT = "/tmp/frames"
os.makedirs(OUT, exist_ok=True)


class Q:
    def __init__(self):
        self.items = []

    def put_nowait(self, x):
        self.items.append(x)

    def get_nowait(self):
        raise Exception("empty")


class Saver(Node):
    def __init__(self, topics, want=6):
        super().__init__("frame_saver")
        self.bridge = CvBridge()
        self.want = want
        self.n = {t: 0 for t in topics}
        self.hit = {t: 0 for t in topics}
        self.p = PerceptionProcess(Q(), Q())
        for t in topics:
            self.create_subscription(Image, t, self._make_cb(t), 1)
        print("  subscribed to: %s" % ", ".join(topics))

    def _make_cb(self, topic):
        def cb(msg):
            if self.n[topic] >= self.want:
                return
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.n[topic] += 1
            tag = topic.strip("/").replace("/", "_")

            if self.n[topic] == 1:
                p = os.path.join(OUT, "%s_ham.png" % tag)
                cv2.imwrite(p, frame)
                std = float(frame.std())
                print("  %-12s %dx%d  std=%.1f  %s"
                      % (topic, frame.shape[1], frame.shape[0], std,
                         "FLAT (no scene)" if std < 3 else "scene present"))

            vis = frame.copy()
            self.p.result_queue = Q()
            self.p._process_qr(vis, set())
            self.p._draw_hud(vis, vis.shape[1], vis.shape[0], 30.0)
            res = self.p.result_queue.items
            if res:
                self.hit[topic] += 1
                if self.hit[topic] == 1:
                    r = res[0]
                    print("  %-12s QR FOUND data=%r in_av=%s src=%s"
                          % (topic, r.get("data"), r.get("in_av"), r.get("src")))
                    print("  %-12s   box=%s" % ("", r.get("box")))
                    print("  %-12s   corners=%s" % ("", r.get("corners")))
                    cv2.imwrite(os.path.join(OUT, "%s_tespit.png" % tag), vis)
            if self.n[topic] == self.want and self.hit[topic] == 0:
                cv2.imwrite(os.path.join(OUT, "%s_hud.png" % tag), vis)
        return cb

    def done(self):
        return all(v >= self.want for v in self.n.values())


def main():
    topics = sys.argv[1:] or ["/camera"]
    rclpy.init()
    node = Saver(topics)
    t = 0.0
    while rclpy.ok() and not node.done() and t < 30.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        t += 0.2
    print()
    for k in node.n:
        print("  %-12s frames=%d  QR=%d" % (k, node.n[k], node.hit[k]))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
