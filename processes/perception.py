from __future__ import annotations

from enum import Enum
import multiprocessing as mp
import logging
import socket
import struct
import time
import datetime

import numpy as np
import cv2

from pyzbar.pyzbar import decode
from logger import setup_logger
import config

logger = logging.getLogger("Perception")

class PerceptionMode(Enum):
    QR   = "qr"
    YOLO = "yolo"
    BOTH = "both"

class PerceptionProcess(mp.Process):
    """
    Tek process içinde QR ve/veya YOLO algılama. Savaşan İHA HUD Entegreli.

    Komut kuyruğu mesajları:
        {"cmd": "start"}                          → algılamayı aktifleştir
        {"cmd": "stop"}                           → algılamayı durdur
        {"cmd": "set_mode", "mode": "qr|yolo|both"} → modu değiştir
        {"cmd": "kill"}                           → process'i sonlandır

    Sonuç kuyruğu mesajları:
        {"type": "qr",   "data": "<metin>"}
        {"type": "yolo", "data": [{"label": str, "conf": float, "box": [x,y,w,h]}, ...]}
    """

    def __init__(
        self,
        command_queue: mp.Queue,
        result_queue: mp.Queue,
        host: str = "127.0.0.1",
        port: int = 5005,
        mode: PerceptionMode = PerceptionMode.QR,
        yolo_model_path: str = "yolov8n.pt",
    ) -> None:
        super().__init__(name="perception_process", daemon=True)
        self.command_queue  = command_queue
        self.result_queue   = result_queue
        self.host           = host
        self.port           = port
        self._mode          = mode
        self.yolo_model_path = yolo_model_path

        self._start_event = mp.Event()
        self._stop_event  = mp.Event()
        # Mode paylaşımı için Value + Lock
        self._mode_value  = mp.Value("i", mode.value == "yolo" and 1 or
                                         mode.value == "both" and 2 or 0)

    # ------------------------------------------------------------------
    # Process entry point
    # ------------------------------------------------------------------
    def run(self) -> None:
        setup_logger(level=logging.INFO, log_to_file=False)

        # YOLO'yu yalnızca gerektiğinde import et
        yolo_model = self._load_yolo_if_needed()

        qr_seen = set()

        # FPS hesaplamak için
        prev_time    = time.time()
        fps_smoothed = 0.0

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((self.host, self.port))
        server_socket.listen(1)
        server_socket.settimeout(1.0)

        logger.info("PerceptionProcess listening on %s:%d  mode=%s", self.host, self.port, self._mode.value)

        if getattr(config, "QR_SHOW_WINDOW", False):
            cv2.namedWindow("Perception View", cv2.WINDOW_NORMAL)

        conn         = None
        data_buffer  = b""
        payload_size = struct.calcsize(">L")

        try:
            while not self._stop_event.is_set():
                self._drain_commands()

                if conn is None:
                    try:
                        conn, addr = server_socket.accept()
                        logger.info("Sender connected: %s", addr)
                        data_buffer = b""
                    except socket.timeout:
                        continue

                try:
                    data_buffer = self._recv_exactly(conn, data_buffer, payload_size)
                    msg_size    = struct.unpack(">L", data_buffer[:payload_size])[0]
                    data_buffer = data_buffer[payload_size:]

                    data_buffer = self._recv_exactly(conn, data_buffer, msg_size)
                    frame_data  = data_buffer[:msg_size]
                    data_buffer = data_buffer[msg_size:]

                    np_arr = np.frombuffer(frame_data, dtype=np.uint8)
                    frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if frame is None:
                        continue

                    # Savaşan İHA Arayüzü İçin Ekran Merkezini Hesapla
                    h, w = frame.shape[:2]
                    center_x, center_y = w // 2, h // 2

                    # --- FPS hesapla ---
                    now_t = time.time()
                    dt = now_t - prev_time
                    prev_time = now_t
                    if dt > 0:
                        inst_fps = 1.0 / dt
                        # basit üstel yumuşatma, titremeyi önler
                        fps_smoothed = fps_smoothed * 0.9 + inst_fps * 0.1 if fps_smoothed > 0 else inst_fps

                    # --- Algılama: HER FRAME'de tutarlı şekilde çalışır ---
                    # (Not: eskiden scan_interval ile bazı frame'lerde atlanıyordu,
                    #  bu da davranışı tutarsız gösteriyordu. Artık start aktifse
                    #  her frame işleniyor.)
                    if self._start_event.is_set():
                        mode = self._current_mode()

                        if mode in (PerceptionMode.QR, PerceptionMode.BOTH):
                            qr_seen = self._process_qr(frame, qr_seen)
                        if mode in (PerceptionMode.YOLO, PerceptionMode.BOTH):
                            if yolo_model is None:
                                yolo_model = self._load_yolo_if_needed()
                            self._process_yolo(frame, yolo_model, center_x, center_y)
                    else:
                        self._draw_text(frame, "WAITING FOR START...", (w // 2 - 150, h // 2),
                                         0.8, (0, 0, 255), thickness=2)

                    # --- HUD OVERLAY (Her zaman çizilir) ---
                    self._draw_hud(frame, w, h, fps_smoothed)

                    if getattr(config, "QR_SHOW_WINDOW", False):
                        cv2.imshow("Perception View", frame)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            self._stop_event.set()

                except (socket.error, struct.error) as e:
                    logger.warning("Connection lost: %s  — waiting for reconnect.", e)
                    if conn:
                        conn.close()
                    conn = None

        except KeyboardInterrupt:
            pass
        finally:
            if conn:
                conn.close()
            server_socket.close()
            if getattr(config, "QR_SHOW_WINDOW", False):
                cv2.destroyAllWindows()

    # ------------------------------------------------------------------
    # HUD çizimi
    # ------------------------------------------------------------------
    def _draw_text(self, frame, text, org, scale, color, thickness=2):
        """
        Yazıyı önce siyah, kalın bir kontur ile, sonra üzerine renkli ve
        ince olarak çizer. Bu sayede yazı arka plan ne olursa olsun
        keskin ve okunaklı görünür.
        """
        cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (0, 0, 0), thickness + 2, cv2.LINE_AA)
        cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    color, thickness, cv2.LINE_AA)

    def _draw_hud(self, frame, w: int, h: int, fps: float) -> None:
        mode_str   = f"MOD: {self._current_mode().name}"
        status_str = f"DURUM: {'AKTIF' if self._start_event.is_set() else 'BEKLEMEDE'}"
        time_str   = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-4]
        res_str    = f"COZUNURLUK: {w}x{h}"
        fps_str    = f"FPS: {fps:4.1f}"

        status_color = (0, 255, 0) if self._start_event.is_set() else (0, 165, 255)

        # Sol Üst: Mod + Durum
        self._draw_text(frame, mode_str, (20, 36), 0.75, (0, 255, 255))

        # Sağ Üst: Saat
        (tw, _), _ = cv2.getTextSize(time_str, cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)
        self._draw_text(frame, time_str, (w - tw - 20, 36), 0.75, (0, 255, 255))


        # Sağ Alt: FPS
        (fw, _), _ = cv2.getTextSize(fps_str, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        self._draw_text(frame, fps_str, (w - fw - 20, h - 20), 0.6, (200, 200, 200))

        # --- Kilitlenme Bölgesi: soldan %25, sağdan %25 boşluk;
        #     üstten/alttan yükseklik %10 boşluk bırakan düz dikdörtgen ---
        x1 = w // 6
        x2 = w - w // 6
        y1 = int(h * 0.10)
        y2 = h - int(h * 0.10)

        box_color = (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2, cv2.LINE_AA)

        # Kutu etiketi

        # Merkezde küçük bir nişangah (crosshair)
        cx, cy = w // 2, h // 2
        r = 10
        cv2.line(frame, (cx - r, cy), (cx + r, cy), box_color, 1, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - r), (cx, cy + r), box_color, 1, cv2.LINE_AA)

    # ------------------------------------------------------------------
    # QR işleme
    # ------------------------------------------------------------------
    def _process_qr(self, frame: np.ndarray, seen: set) -> set:
        h, w = frame.shape[:2]
        if w > 800:
            scan_frame = cv2.resize(frame, (w // 2, h // 2), interpolation=cv2.INTER_AREA)
            scale = 2
        else:
            scan_frame = frame
            scale = 1

        gray     = cv2.cvtColor(scan_frame, cv2.COLOR_BGR2GRAY)
        barcodes = decode(gray)

        for barcode in barcodes:
            if barcode.type != "QRCODE":
                continue
            try:
                data = barcode.data.decode("utf-8")
            except Exception:
                continue

            x, y, bw, bh = (v * scale for v in barcode.rect)

            # Anti-aliasing ile QR Çizimi
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 4, cv2.LINE_AA)
            self._draw_text(frame, data[:20], (x, y - 10), 0.7, (0, 255, 0))

            if data and data not in seen:
                seen.add(data)
                logger.info("QR Kilitlendi: %s", data)
                self.result_queue.put_nowait({"type": "qr", "data": data})
                break  # ilk geçerli QR yeterli

        return seen

    # ------------------------------------------------------------------
    # YOLO işleme
    # ------------------------------------------------------------------
    def _process_yolo(self, frame: np.ndarray, model, center_x: int, center_y: int) -> None:
        results     = model(frame, verbose=False)[0]
        detections  = []

        for box in results.boxes:
            label = model.names[int(box.cls[0])]
            conf  = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            # Hedefin tam merkez koordinatları
            target_cx = (x1 + x2) // 2
            target_cy = (y1 + y2) // 2

            detections.append({
                "label": label,
                "conf":  round(conf, 3),
                "box":   [x1, y1, x2 - x1, y2 - y1],
            })

            # Bounding Box ve Metin Çizimleri (Anti-aliased, keskin yazı)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 80, 0), 2, cv2.LINE_AA)
            self._draw_text(frame, f"{label} %{int(conf*100)}", (x1, y1 - 8), 0.6, (255, 80, 0))

            # Hedef merkezine nokta
            cv2.circle(frame, (target_cx, target_cy), 4, (0, 0, 255), -1, cv2.LINE_AA)

            # Ekran merkezinden (İHA) -> Hedefe Kilitlenme Çizgisi
            cv2.line(frame, (center_x, center_y), (target_cx, target_cy), (0, 0, 255), 2, cv2.LINE_AA)

        if detections:
            logger.debug("YOLO algılandı: %d hedef", len(detections))
            self.result_queue.put_nowait({"type": "yolo", "data": detections})

    # ------------------------------------------------------------------
    # Yardımcılar
    # ------------------------------------------------------------------
    def _load_yolo_if_needed(self):
        mode = self._current_mode()
        if mode not in (PerceptionMode.YOLO, PerceptionMode.BOTH):
            return None
        try:
            from ultralytics import YOLO
            model = YOLO(self.yolo_model_path)
            logger.info("YOLO model yüklendi: %s", self.yolo_model_path)
            return model
        except Exception as e:
            logger.error("YOLO model yüklenemedi: %s", e)
            return None

    def _current_mode(self) -> PerceptionMode:
        v = self._mode_value.value
        return [PerceptionMode.QR, PerceptionMode.YOLO, PerceptionMode.BOTH][v]

    @staticmethod
    def _recv_exactly(conn: socket.socket, buf: bytes, size: int) -> bytes:
        """Buffer'da en az `size` bayt olana kadar soketten oku."""
        while len(buf) < size:
            packet = conn.recv(4096)
            if not packet:
                raise socket.error("Connection closed")
            buf += packet
        return buf

    def _drain_commands(self) -> None:
        while True:
            try:
                msg = self.command_queue.get_nowait()
                if not isinstance(msg, dict):
                    continue
                cmd = msg.get("cmd")
                if cmd == "start":
                    self._start_event.set()
                    logger.info("Perception sistemi aktif.")
                elif cmd == "stop":
                    self._start_event.clear()
                    logger.info("Perception sistemi durduruldu.")
                elif cmd == "set_mode":
                    raw = msg.get("mode", "qr")
                    try:
                        new_mode = PerceptionMode(raw)
                    except ValueError:
                        logger.warning("Bilinmeyen mod: %s", raw)
                        continue
                    self._mode_value.value = [PerceptionMode.QR, PerceptionMode.YOLO,
                                               PerceptionMode.BOTH].index(new_mode)
                    logger.info("Mod değiştirildi → %s", new_mode.value)
                elif cmd == "kill":
                    self._stop_event.set()
            except Exception:
                break