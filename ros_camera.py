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

# Ayarları doğrudan buraya gömdüm, config bağımlılığı da kalmasın diye:
TOPIC_NAME = '/camera'
TCP_HOST = '127.0.0.1'
TCP_PORT = 5005
RECONNECT_INTERVAL = 2.0  # Sunucu yoksa kaç saniyede bir denesin?

class StandaloneROSTCPSender(Node):
    def __init__(self):
        super().__init__('ros_to_tcp_bridge_standalone')
        self.bridge = CvBridge()
        
        self.client_socket = None
        self.is_connected = False
        self.last_connect_try = 0.0

        # ROS Aboneliğini hemen başlatıyoruz
        self.subscription = self.create_subscription(
            Image, 
            TOPIC_NAME, 
            self._image_callback, 
            1
        )
        
        self.get_logger().info("ROS 2 TCP Sender Başlatıldı. Kamera verisi bekleniyor...")

        # Bağlantıyı kontrol etmek ve gerekirse yeniden bağlanmak için bir ROS Timer (0.1 saniyede bir tetiklenir)
        self.create_timer(0.1, self._check_connection)

    def _check_connection(self):
        """TCP Sunucusuna arka planda, ROS'u kilitlemeden bağlanmayı dener."""
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
            self.client_socket.settimeout(1.0) # Bağlanırken kilitlenmesin
            
            self.get_logger().info(f"TCP Sunucusuna bağlanılıyor -> {TCP_HOST}:{TCP_PORT}")
            self.client_socket.connect((TCP_HOST, TCP_PORT))
            
            self.client_socket.settimeout(None) # Gönderim için bloklamasız moda al
            self.is_connected = True
            self.get_logger().info("TCP Sunucusuna BAĞLANDI! Görüntü aktarımı aktif.")
        except socket.error:
            self.get_logger().warn(f"TCP Sunucusu aktif değil. {RECONNECT_INTERVAL} saniye sonra tekrar denenecek...")
            self.is_connected = False

    def _image_callback(self, msg: Image):
        # Eğer TCP sunucusu hazır değilse görüntüyü işlemeyip direkt geçiyoruz
        if not self.is_connected or self.client_socket is None:
            return

        try:
            # ROS imajını OpenCV matrisine çevir
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # ROI falan yok, direkt tüm kareyi JPEG olarak sıkıştır
            encoded, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not encoded:
                return
                
            data = buffer.tobytes()
            size = len(data)
            
            # Paket: [ 4 Byte Boyut ] + [ JPEG Data ]
            self.client_socket.sendall(struct.pack(">L", size) + data)
            
        except (socket.error, BrokenPipeError) as se:
            self.get_logger().error(f"TCP Bağlantısı gönderim sırasında koptu: {se}. Yeniden bağlanılacak...")
            self.is_connected = False
        except Exception as e:
            self.get_logger().error(f"Görüntü işleme hatası: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = StandaloneROSTCPSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Kullanıcı isteğiyle kapatılıyor...")
    finally:
        if node.client_socket:
            node.client_socket.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()