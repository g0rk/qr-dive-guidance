#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import struct
import time
import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Settings are inlined here on purpose, so this bridge has no dependency on
# config.py and can be run on its own.
TOPIC_NAME = '/camera'
TCP_HOST = '127.0.0.1'
TCP_PORT = 5005
RECONNECT_INTERVAL = 2.0  # how often to retry when the server is not up

class StandaloneROSTCPSender(Node):
    def __init__(self):
        super().__init__('ros_to_tcp_bridge_standalone')
        self.bridge = CvBridge()

        self.client_socket = None
        self.is_connected = False
        self.last_connect_try = 0.0

        # Subscribe immediately; frames are dropped until the TCP side is up.
        self.subscription = self.create_subscription(
            Image,
            TOPIC_NAME,
            self._image_callback,
            1
        )

        self.get_logger().info("ROS 2 TCP sender started. Waiting for camera data...")

        # A ROS timer (every 0.1 s) checks the connection and reconnects if
        # needed, so reconnection never blocks the ROS executor.
        self.create_timer(0.1, self._check_connection)

    def _check_connection(self):
        """Try to connect to the TCP server in the background, without blocking ROS."""
        if self.is_connected:
            return

        now = time.time()
        if now - self.last_connect_try < RECONNECT_INTERVAL:
            return

        self.last_connect_try = now
        try:
            if self.client_socket:
                self.client_socket.close()

            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.settimeout(1.0)  # do not hang while connecting

            self.get_logger().info(f"Connecting to TCP server -> {TCP_HOST}:{TCP_PORT}")
            self.client_socket.connect((TCP_HOST, TCP_PORT))

            self.client_socket.settimeout(None)  # blocking sends from here on
            self.is_connected = True
            self.get_logger().info("Connected to TCP server. Streaming frames.")
        except socket.error:
            self.get_logger().warn(f"TCP server not up. Retrying in {RECONNECT_INTERVAL} s...")
            self.is_connected = False

    def _image_callback(self, msg: Image):
        # If the TCP server is not ready, drop the frame instead of encoding it.
        if not self.is_connected or self.client_socket is None:
            return

        try:
            # ROS image -> OpenCV matrix
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # No ROI here: the whole frame is JPEG-encoded and sent. Cropping
            # happens on the perception side, which is where the target area
            # is defined.
            encoded, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not encoded:
                return

            data = buffer.tobytes()
            size = len(data)

            # Wire format: [ 4-byte big-endian size ] + [ JPEG data ]
            self.client_socket.sendall(struct.pack(">L", size) + data)

        except (socket.error, BrokenPipeError) as se:
            self.get_logger().error(f"TCP connection dropped while sending: {se}. Will reconnect...")
            self.is_connected = False
        except Exception as e:
            self.get_logger().error(f"Frame processing error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = StandaloneROSTCPSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down on user request...")
    finally:
        if node.client_socket:
            node.client_socket.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
