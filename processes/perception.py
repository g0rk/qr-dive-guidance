from __future__ import annotations

from enum import Enum
import multiprocessing as mp
import logging
import socket
import struct
import time
import math
import datetime

import numpy as np
import cv2

from pyzbar.pyzbar import decode
from logger import setup_logger
import config

logger = logging.getLogger("Perception")


def polygon_to_corners(polygon, x0: int, y0: int, scale: int):
    """
    pyzbar polygon -> TAM KARE koordinatinda 4 kose, tutarli sirayla.  [Faz 1]

    NEDEN: `barcode.rect` eksen-hizali sinirlayici kutudur. QR 55 derecelik
    bir aciyla ve zeminde donmus olarak goruldugu icin goruntude bir KARE
    degil, bir DORTGEN (yamuk) olusturur. Kutu o dortgeni cevreler, yani
    QR'dan buyuktur - donme aciSina gore %41'e kadar. `barcode.polygon`
    ise QR'in gercek dort kosesini verir ve pyzbar bunu ZATEN dondurmektedir;
    simdiye kadar atiyorduk.

    Donen sira: merkez etrafindaki aciya gore siralanmis (tutarli sarim),
    ardindan sol-uste en yakin koseden baslatilmis. Bu, ayni sahnede ayni
    ciktiyi garanti eder - hata ayiklamayi ve Faz 5'teki poz kestirimini
    kolaylastirir.

    ⚠️ Bu sira TUTARLIDIR ama KANONIK DEGILDIR: hangi kosenin QR'in
       "sol ustu" oldugunu soylemez. solvePnP bunu ister; o is Faz 5'te,
       finder pattern'lardan cikarilacak.

    Dondurur: [[x,y] x 4]  ya da  None (dejenere tespit).
    """
    if polygon is None:
        return None
    # pyzbar bozuk/carpik kodlarda 4'ten farkli sayida nokta dondurebilir.
    if len(polygon) != 4:
        return None

    pts = [(x0 + p.x * scale, y0 + p.y * scale) for p in polygon]

    cx = sum(p[0] for p in pts) / 4.0
    cy = sum(p[1] for p in pts) / 4.0
    pts.sort(key=lambda p: math.atan2(p[1] - cy, p[0] - cx))

    # Deterministik baslangic: sol-uste en yakin kose (x+y en kucuk).
    start = min(range(4), key=lambda i: pts[i][0] + pts[i][1])
    pts = pts[start:] + pts[:start]

    return [[int(round(p[0])), int(round(p[1]))] for p in pts]


def quad_center(corners):
    """
    Dortgenin KOSEGEN KESISIMI = karenin goruntudeki GERCEK merkezi. [Faz 2]

    ⚠️ KOSELERIN ORTALAMASINI ALMAYIN. Perspektifte yanlistir.
       Projektif geometride korunan sey sudur: bir karenin merkezi,
       kosegenlerinin kesisimidir - ve dogrular perspektif altinda dogru
       kaldigi icin, GORUNTUdeki kosegenlerin kesisimi de merkezin
       goruntusudur. Ortalama ise agirlik merkezidir; yamuk bir sekilde
       merkezden kayar.

    corners sirali olmali (polygon_to_corners cikti sirasi): p0-p2 ve
    p1-p3 kosegen olur.

    Dondurur: (x, y) float  ya da  None.
    """
    if not corners or len(corners) != 4:
        return None
    (x1, y1), (x2, y2), (x3, y3), (x4, y4) = (
        corners[0], corners[1], corners[2], corners[3])

    # Kosegen 1: p0->p2   Kosegen 2: p1->p3   (a*x + b*y = c biciminde)
    a1, b1 = y3 - y1, x1 - x3
    c1 = a1 * x1 + b1 * y1
    a2, b2 = y4 - y2, x2 - x4
    c2 = a2 * x2 + b2 * y2

    det = a1 * b2 - a2 * b1
    if abs(det) < 1e-9:          # dejenere: kosegenler paralel/cakisik
        return None
    return ((b2 * c1 - b1 * c2) / det, (a1 * c2 - a2 * c1) / det)


def quad_in_av(corners, av_x1, av_y1, av_x2, av_y2) -> bool:
    """
    QR'in dort kosesi de AV icinde mi.  [Faz 2]

    ⚠️ DURUSTLUK NOTU - bu fonksiyon KARAR DEGISTIRMIYOR.
       Faz 1'de "kutu QR'dan 2 kat buyuk oldugu icin gecerli vurus
       reddedilebilir" diye yazmistim. YANLISTI ve olcumle curutuldu
       (tests/test_faz2_center.py, 41 konum/aci, 0 ayrisma).

       Sebep: AV EKSEN-HIZALI bir dikdortgen. Disbukey bir dortgenin
       boyle bir dikdortgenin icinde olmasi <=> dort kosesinin de icinde
       olmasi <=> koselerin min/max'inin icinde olmasi. Kosellerin
       min/max'i ise sinirlayici kutunun ta kendisidir. Yani iki test
       MATEMATIKSEL OLARAK OZDES; kutunun ALANININ 2 kat olmasi bunu
       degistirmez. Olculdu: pyzbar `rect` == polygon bbox, 0 px fark.

       Yine de duruyor cunku: (a) niyeti aciga vuruyor - test edilen sey
       QR'in siniri, (b) AV bir gun eksen-hizali olmayan bir sekle
       donerse dogru kalir, (c) Faz 5 zaten koselere muhtac.
    """
    return all(av_x1 <= px <= av_x2 and av_y1 <= py <= av_y2
               for px, py in corners)

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

    # Line thickness cap. The competition rulebook limits overlay lines to
    # 3 px; keeping to it costs nothing and a thin overlay hides less of the
    # target, which matters when the QR is only ~70 px across at the decode
    # threshold. A 4 px box used to cover a meaningful slice of it.
    _MAX_LINE_PX = 3

    def _draw_qr_overlay(self, frame, corners, box, center, color,
                         data: str, in_av: bool, source: str) -> None:
        """Draw the detected QR: true quadrilateral, diagonals, centre.

        WHY NOT AN AXIS-ALIGNED BOX
        ---------------------------
        The detector returns the QR's four actual corners. Drawing the
        bounding box instead throws that away, and the box is a poor
        stand-in: measured inflation over the real quadrilateral is 1.02x
        under perspective alone but 2.00x at 45 degrees of rotation. What
        inflates the box is ROTATION, not perspective. At a steep dive the
        drawn box can be twice the area of the thing it claims to outline.

        The centre marker is the DIAGONAL INTERSECTION, not the midpoint of
        the box. Under perspective the far edge is foreshortened, so the
        average of the corners drifts toward it. Measured difference: 0.0 px
        head-on, 2.8 px in a realistic dive, 9.2 px at aggressive angles.

        Falls back to the bounding box when the detector returns a
        degenerate polygon -- rare, but the overlay should degrade rather
        than vanish.
        """
        x, y, bw, bh = box
        cx, cy = center
        t = self._MAX_LINE_PX

        if corners:
            pts = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], True, color, t, cv2.LINE_AA)
            # Diagonals: they show the perspective directly -- on a tilted
            # target the two halves are visibly unequal, which is exactly
            # the information the bounding box destroys.
            cv2.line(frame, tuple(corners[0]), tuple(corners[2]),
                     color, 1, cv2.LINE_AA)
            cv2.line(frame, tuple(corners[1]), tuple(corners[3]),
                     color, 1, cv2.LINE_AA)
            # Corner index labels make the ordering visible. Ordering is not
            # cosmetic: solvePnP needs corners in a known sequence, and a
            # silently rotated ordering yields a plausible-looking but wrong
            # pose.
            for i, (px, py) in enumerate(corners):
                cv2.circle(frame, (int(px), int(py)), 5, color, -1, cv2.LINE_AA)
                self._draw_text(frame, str(i), (int(px) + 8, int(py) - 8),
                                0.45, color, thickness=1)
        else:
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, t, cv2.LINE_AA)

        # Centre cross, drawn thin so it does not obscure the QR modules.
        icx, icy = int(round(cx)), int(round(cy))
        cv2.line(frame, (icx - 12, icy), (icx + 12, icy), color, 1, cv2.LINE_AA)
        cv2.line(frame, (icx, icy - 12), (icx, icy + 12), color, 1, cv2.LINE_AA)

        # Primary label is the STATUS, not the payload.
        #
        # The distinction it carries is the whole mission rule: decoding the
        # QR is not enough, the code must lie ENTIRELY inside the target
        # area. A frame that decodes a QR hanging over the edge looks like
        # success and is not one. Printing the payload as the headline
        # blurred that -- the text read the same either way.
        status = "MISSION COMPLETE" if in_av else "DECODED - OUTSIDE TARGET AREA"
        self._draw_text(frame, status, (x, y - 12), 0.8, color)

        # The decoded payload stays, one line below and smaller. It is the
        # value that goes into the mission packet, so it has to be visible
        # for diagnosis -- just not as the headline.
        #
        # Source tag: "quad" when the four corners were usable, "box" when
        # the fallback ran. Without it a degenerate detection looks exactly
        # like a good one in the recording.
        self._draw_text(frame, "%s   src=%s" % (data[:24], source),
                        (x, y + bh + 22), 0.5, color, thickness=1)

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

        # --- Hedef Vuruş Alanı (AV): soldan/sağdan %25, üstten/alttan %10 ---
        # ⚠️ Eskiden `w // 6` (=%16.7) çiziliyordu ama yorum %25 diyordu.
        #    QR taraması artık BU kutuya kırpıldığı için ikisi aynı yerden
        #    gelmek zorunda — yoksa çizilen kutu ile taranan bölge ayrışır
        #    ve ekranda gördüğün şey `in_av` kararıyla uyuşmaz.
        x1, y1, x2, y2 = self._av_bounds(w, h)

        box_color = (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2, cv2.LINE_AA)
        self._draw_text(frame, "AV", (x1 + 8, y1 + 26), 0.6, box_color, thickness=1)

        # --- QR tarama bölgesi (AV + pay) — gri kesikli ---
        # Görsel hata ayıklama: pyzbar'ın gerçekte NEREYE baktığını gösterir.
        if getattr(config, "QR_SCAN_ENABLED_ROI", True):
            sx1, sy1, sx2, sy2 = self._scan_bounds(w, h)
            dash, gap = 18, 12
            for xx in range(sx1, sx2, dash + gap):
                x_end = min(xx + dash, sx2)
                cv2.line(frame, (xx, sy1), (x_end, sy1), (170, 170, 170), 1, cv2.LINE_AA)
                cv2.line(frame, (xx, sy2), (x_end, sy2), (170, 170, 170), 1, cv2.LINE_AA)
            for yy in range(sy1, sy2, dash + gap):
                y_end = min(yy + dash, sy2)
                cv2.line(frame, (sx1, yy), (sx1, y_end), (170, 170, 170), 1, cv2.LINE_AA)
                cv2.line(frame, (sx2, yy), (sx2, y_end), (170, 170, 170), 1, cv2.LINE_AA)
            self._draw_text(frame, "QR TARAMA", (sx1 + 8, sy1 - 10), 0.5,
                            (170, 170, 170), thickness=1)

        # Kutu etiketi

        # Merkezde küçük bir nişangah (crosshair)
        cx, cy = w // 2, h // 2
        r = 10
        cv2.line(frame, (cx - r, cy), (cx + r, cy), box_color, 1, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - r), (cx, cy + r), box_color, 1, cv2.LINE_AA)

    # ------------------------------------------------------------------
    # QR işleme
    # ------------------------------------------------------------------
    def _av_bounds(self, w: int, h: int):
        """
        Hedef Vuruş Alanı sınırları (şartname Şekil 2/4: %25 yatay, %10 dikey).

        Not: config'ten `getattr` ile okunuyor ki bu dosya, AV sabitleri
        eklenmemiş ESKİ bir config.py ile de çalışsın (şartname değerleri
        varsayılan). Böylece iki dosya birbirinden bağımsız taşınabilir.
        """
        av_x1 = int(w * getattr(config, "AV_MARGIN_X", 0.25))
        av_y1 = int(h * getattr(config, "AV_MARGIN_Y", 0.10))
        return av_x1, av_y1, w - av_x1, h - av_y1

    def _scan_bounds(self, w: int, h: int):
        """
        QR taranacak bölge: AV + pay.

        AV'ye TAM kırpmıyoruz; kenardan taşan bir QR kırpılmış hâliyle
        çözülüp kutusu AV sınırına yapışık görünür, yani AV DIŞINDAKİ bir
        QR "içeride" sanılırdı. Geniş tarayıp katı içerme testi uyguluyoruz.
        """
        av_x1, av_y1, av_x2, av_y2 = self._av_bounds(w, h)
        pad = float(getattr(config, "QR_SCAN_AV_PAD", 0.08))
        pad_x = int(w * pad)
        pad_y = int(h * pad)
        return (max(0, av_x1 - pad_x), max(0, av_y1 - pad_y),
                min(w, av_x2 + pad_x), min(h, av_y2 + pad_y))

    def _process_qr(self, frame: np.ndarray, seen: set) -> set:
        """
        AV bölgesini KIRPARAK QR tara.

        ⚠️ DEĞİŞİKLİK: eskiden `w > 800` ise TÜM KARE yarıya indiriliyordu.
           Bu pyzbar'ı hızlandırır ama QR'ın piksel boyunu da yarılar, yani
           decode menzilini kısaltır (~%30 kayıp ölçüldü). Kırpma aynı
           hızlanmayı verir, çözünürlükten hiç feragat etmez: AV zaten
           karenin %50 x %80'i = piksellerin ~%40'ı.
        """
        h, w = frame.shape[:2]
        av_x1, av_y1, av_x2, av_y2 = self._av_bounds(w, h)

        if getattr(config, "QR_SCAN_ENABLED_ROI", True):
            x0, y0, x3, y3 = self._scan_bounds(w, h)
        else:
            x0, y0, x3, y3 = 0, 0, w, h

        roi = frame[y0:y3, x0:x3]
        if roi.size == 0:
            return seen

        scale = max(1, int(getattr(config, "QR_SCAN_DOWNSCALE", 1)))
        if scale > 1:
            roi = cv2.resize(roi, (roi.shape[1] // scale, roi.shape[0] // scale),
                             interpolation=cv2.INTER_AREA)

        gray     = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        barcodes = decode(gray)

        for barcode in barcodes:
            if barcode.type != "QRCODE":
                continue
            try:
                data = barcode.data.decode("utf-8")
            except Exception:
                continue

            # ROI + downscale'i geri al -> TAM KARE koordinatları
            bx, by, bw, bh = barcode.rect
            x  = x0 + bx * scale
            y  = y0 + by * scale
            bw = bw * scale
            bh = bh * scale

            # Faz 1: QR'in GERCEK dort kosesi. pyzbar bunu zaten dondurur;
            # dejenere tespitte None -> kutuya geri duseriz.
            corners = polygon_to_corners(getattr(barcode, "polygon", None),
                                         x0, y0, scale)

            # --- Faz 2: MERKEZ ve in_av gercek dortgenden ---
            #
            # MERKEZ: kosegen kesisimi kullaniliyor, kutunun ortasi degil.
            # Perspektif altinda ikisi ayrisiyor - olculdu: gercekci dalis
            # perspektifinde 2.8 px, agresif perspektifte 9.2 px.
            # 6mm/2MP ile 2.8 px = 0.075 derece = 30 m'de yerde ~4 cm.
            # Yani DAHA DOGRU ama etkisi kucuk; bedava oldugu icin dogrusu
            # kullaniliyor.
            #
            # in_av: KARAR DEGISTIRMIYOR (bkz. quad_in_av docstring).
            # AV eksen-hizali oldugu icin kutu testiyle ozdes. Niyeti
            # aciga vurdugu ve Faz 5 koselere muhtac oldugu icin duruyor.
            qc = quad_center(corners)
            if corners and qc is not None:
                in_av = quad_in_av(corners, av_x1, av_y1, av_x2, av_y2)
                cx, cy = qc                 # kosegen kesisimi (bkz. quad_center)
                kaynak = "dortgen"
            else:
                # Yedek: dejenere tespit. Kutu kaba ama guvenli.
                in_av = (x >= av_x1 and y >= av_y1
                         and (x + bw) <= av_x2 and (y + bh) <= av_y2)
                cx, cy = x + bw / 2.0, y + bh / 2.0
                kaynak = "kutu"

            # Green when inside the target area, orange when outside, so the
            # distinction is readable at a glance. Orange means "decoded but
            # does NOT count as a hit".
            color = (0, 255, 0) if in_av else (0, 165, 255)
            self._draw_qr_overlay(frame, corners, (x, y, bw, bh), (cx, cy),
                                  color, data, in_av, kaynak)

            # Normalize hata: ex>0 QR sagda, ey>0 QR asagida (goruntu y'si asagi buyur)
            ex = (cx - w / 2.0) / (w / 2.0)
            ey = (cy - h / 2.0) / (h / 2.0)

            # ⚠️ P4: HER KAREDE yayinlanir.
            #    Eskiden yalnizca QR ILK GORULDUGUNDE gonderiliyordu
            #    (`data not in seen` kapisi). O haliyle dalis merkezlemesi
            #    IMKANSIZ - taze konum hic gelmiyor, tek bir eski mesaj var.
            #    `seen` artik yalnizca LOG'u bir kereye indirmek icin.
            self.result_queue.put_nowait({
                "type":    "qr",
                "data":    data,
                "box":     [int(x), int(y), int(bw), int(bh)],
                "corners": corners,          # [[x,y] x 4] ya da None (Faz 1)
                "center":  [int(round(cx)), int(round(cy))],
                "error":   [round(ex, 4), round(ey, 4)],
                "frame":   [int(w), int(h)],
                "in_av":   bool(in_av),
                "src":     kaynak,           # "dortgen" | "kutu" (Faz 2)
            })

            if data not in seen:
                seen.add(data)
                logger.info("QR Kilitlendi: %s  (AV icinde: %s, kaynak: %s)",
                            data, in_av, kaynak)

            # Ilk gecerli QR yeterli. ⚠️ Bu `break` eskiden `data not in seen`
            # blogunun ICINDEydi: ayni QR ikinci kez gorulunce dongu kirilmiyor,
            # kalan barkodlar bosuna taraniyordu.
            break

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