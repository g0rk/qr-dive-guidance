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
    pyzbar polygon -> the QR's 4 corners in FULL-FRAME coordinates, in a
    consistent order.

    WHY: `barcode.rect` is an axis-aligned bounding box. Seen from a 55-degree
    dive, with the code rotated on the ground, the QR is not a SQUARE in the
    image but a QUADRILATERAL. The box encloses that quadrilateral, so it is
    larger than the QR -- by up to 41 % depending on the rotation angle.
    `barcode.polygon` gives the QR's four real corners, and pyzbar ALREADY
    returns it. We were throwing it away.

    Order returned: sorted by angle around the centre (a consistent winding),
    then rotated to start at the corner nearest the top-left. That guarantees
    the same output for the same scene, which makes debugging and pose
    estimation tractable.

    ⚠️ This order is CONSISTENT but not CANONICAL: it does not tell you which
       corner is the QR's own "top left". solvePnP needs that, and it has to
       come from the finder patterns.

    Returns: [[x,y] x 4]  or  None (degenerate detection).
    """
    if polygon is None:
        return None
    # pyzbar can return a point count other than 4 for damaged or badly
    # skewed codes.
    if len(polygon) != 4:
        return None

    pts = [(x0 + p.x * scale, y0 + p.y * scale) for p in polygon]

    cx = sum(p[0] for p in pts) / 4.0
    cy = sum(p[1] for p in pts) / 4.0
    pts.sort(key=lambda p: math.atan2(p[1] - cy, p[0] - cx))

    # Deterministic starting point: the corner nearest the top-left (smallest
    # x+y).
    start = min(range(4), key=lambda i: pts[i][0] + pts[i][1])
    pts = pts[start:] + pts[:start]

    return [[int(round(p[0])), int(round(p[1]))] for p in pts]


def quad_center(corners):
    """
    The DIAGONAL INTERSECTION = the square's true centre as it appears in the
    image.

    ⚠️ DO NOT AVERAGE THE CORNERS. Under perspective that is wrong.
       What projective geometry preserves is this: the centre of a square is
       the intersection of its diagonals -- and because straight lines stay
       straight under perspective, the intersection of the diagonals IN THE
       IMAGE is the image of the centre. The average of the corners is the
       centroid instead, and on a trapezoid the centroid drifts off centre.

    `corners` must be ordered (the output order of polygon_to_corners), so
    that p0-p2 and p1-p3 are the diagonals.

    Returns: (x, y) float  or  None.
    """
    if not corners or len(corners) != 4:
        return None
    (x1, y1), (x2, y2), (x3, y3), (x4, y4) = (
        corners[0], corners[1], corners[2], corners[3])

    # Diagonal 1: p0->p2   Diagonal 2: p1->p3   (as a*x + b*y = c)
    a1, b1 = y3 - y1, x1 - x3
    c1 = a1 * x1 + b1 * y1
    a2, b2 = y4 - y2, x2 - x4
    c2 = a2 * x2 + b2 * y2

    det = a1 * b2 - a2 * b1
    if abs(det) < 1e-9:          # degenerate: diagonals parallel or coincident
        return None
    return ((b2 * c1 - b1 * c2) / det, (a1 * c2 - a2 * c1) / det)


def quad_in_av(corners, av_x1, av_y1, av_x2, av_y2) -> bool:
    """
    Are all four QR corners inside the target area?

    ⚠️ HONEST NOTE - THIS FUNCTION CHANGES NO DECISION.
       An earlier version of this work argued that "a valid hit can be
       rejected because the box is up to 2x larger than the QR". That was
       WRONG, and measurement refuted it (tests/test_faz2_center.py, 41
       positions and angles, zero disagreement).

       The reason: the target area is an AXIS-ALIGNED rectangle. A convex
       quadrilateral lies inside such a rectangle <=> all four of its corners
       do <=> the min/max of those corners does. And the min/max of the
       corners is precisely the bounding box. So the two tests are
       MATHEMATICALLY IDENTICAL; the box having twice the AREA does not change
       that. Measured: pyzbar's `rect` equals the polygon bbox, 0 px apart.

       It stays anyway because: (a) it states the intent -- what is being
       tested is the QR's own boundary, (b) it remains correct if the target
       area ever stops being axis-aligned, (c) pose estimation needs the
       corners regardless.
    """
    return all(av_x1 <= px <= av_x2 and av_y1 <= py <= av_y2
               for px, py in corners)

class PerceptionMode(Enum):
    QR   = "qr"
    YOLO = "yolo"
    BOTH = "both"

class PerceptionProcess(mp.Process):
    """
    QR and/or YOLO detection in a single process, with the HUD overlay.

    Command queue messages:
        {"cmd": "start"}                            -> enable detection
        {"cmd": "stop"}                             -> stop detection
        {"cmd": "set_mode", "mode": "qr|yolo|both"} -> change mode
        {"cmd": "kill"}                             -> terminate the process

    Result queue messages:
        {"type": "qr",   "data": "<text>"}
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
        # Value + Lock so the mode can be shared across processes.
        self._mode_value  = mp.Value("i", mode.value == "yolo" and 1 or
                                         mode.value == "both" and 2 or 0)

    # ------------------------------------------------------------------
    # Process entry point
    # ------------------------------------------------------------------
    def run(self) -> None:
        setup_logger(level=logging.INFO, log_to_file=False)

        # Import YOLO only when it is actually needed.
        yolo_model = self._load_yolo_if_needed()

        qr_seen = set()

        # For the FPS readout.
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

                    # Frame centre, used by the YOLO lock-on line.
                    h, w = frame.shape[:2]
                    center_x, center_y = w // 2, h // 2

                    # --- FPS ---
                    now_t = time.time()
                    dt = now_t - prev_time
                    prev_time = now_t
                    if dt > 0:
                        inst_fps = 1.0 / dt
                        # Simple exponential smoothing, to stop the readout
                        # from flickering.
                        fps_smoothed = fps_smoothed * 0.9 + inst_fps * 0.1 if fps_smoothed > 0 else inst_fps

                    # --- Detection: runs on EVERY frame ---
                    # (An earlier version skipped frames on a scan_interval,
                    #  which made the behaviour look inconsistent. Now every
                    #  frame is processed while start is active.)
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

                    # --- HUD overlay (always drawn) ---
                    self._draw_hud(frame, w, h, fps_smoothed)

                    if getattr(config, "QR_SHOW_WINDOW", False):
                        cv2.imshow("Perception View", frame)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            self._stop_event.set()

                except (socket.error, struct.error) as e:
                    logger.warning("Connection lost: %s  - waiting for reconnect.", e)
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
    # HUD drawing
    # ------------------------------------------------------------------
    def _draw_text(self, frame, text, org, scale, color, thickness=2):
        """
        Draw the text twice: first a thick black outline, then the coloured
        glyphs on top. This keeps it sharp and readable over any background --
        grass, sky or the QR itself.
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
        mode_str = f"MODE: {self._current_mode().name}"
        time_str = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-4]
        fps_str  = f"FPS: {fps:4.1f}"

        # Top left: mode
        self._draw_text(frame, mode_str, (20, 36), 0.75, (0, 255, 255))

        # Top right: clock
        (tw, _), _ = cv2.getTextSize(time_str, cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)
        self._draw_text(frame, time_str, (w - tw - 20, 36), 0.75, (0, 255, 255))

        # Bottom right: FPS
        (fw, _), _ = cv2.getTextSize(fps_str, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        self._draw_text(frame, fps_str, (w - fw - 20, h - 20), 0.6, (200, 200, 200))

        # --- Target area: 25 % clear left and right, 10 % top and bottom ---
        # ⚠️ This used to be drawn as `w // 6` (= 16.7 %) while the comment
        #    beside it claimed 25 %. The QR scan is now cropped to THIS box,
        #    so the two have to come from one place -- otherwise the box you
        #    see on screen and the region actually scanned drift apart, and
        #    what you are looking at stops matching the `in_av` decision.
        x1, y1, x2, y2 = self._av_bounds(w, h)

        box_color = (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2, cv2.LINE_AA)
        self._draw_text(frame, "AV", (x1 + 8, y1 + 26), 0.6, box_color, thickness=1)

        # --- QR scan region (target area + padding), dashed grey ---
        # Visual debugging: it shows where pyzbar is actually looking.
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
            self._draw_text(frame, "QR SCAN", (sx1 + 8, sy1 - 10), 0.5,
                            (170, 170, 170), thickness=1)

        # Small crosshair at the frame centre.
        cx, cy = w // 2, h // 2
        r = 10
        cv2.line(frame, (cx - r, cy), (cx + r, cy), box_color, 1, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - r), (cx, cy + r), box_color, 1, cv2.LINE_AA)

    # ------------------------------------------------------------------
    # QR processing
    # ------------------------------------------------------------------
    def _av_bounds(self, w: int, h: int):
        """
        Target-area bounds (rulebook: 25 % horizontal, 10 % vertical).

        Note: read from config with `getattr` so this file also works against
        an OLDER config.py that predates the target-area constants -- the
        rulebook values are the defaults. That keeps the two files
        independently portable.
        """
        av_x1 = int(w * getattr(config, "AV_MARGIN_X", 0.25))
        av_y1 = int(h * getattr(config, "AV_MARGIN_Y", 0.10))
        return av_x1, av_y1, w - av_x1, h - av_y1

    def _scan_bounds(self, w: int, h: int):
        """
        The region to scan for QRs: the target area plus padding.

        We deliberately do NOT crop exactly to the target area. A QR hanging
        over the edge would still decode from its cropped remains, its box
        would appear flush with the boundary, and a QR that is actually
        OUTSIDE would be reported as inside. So we scan wide and then apply a
        strict containment test.
        """
        av_x1, av_y1, av_x2, av_y2 = self._av_bounds(w, h)
        pad = float(getattr(config, "QR_SCAN_AV_PAD", 0.08))
        pad_x = int(w * pad)
        pad_y = int(h * pad)
        return (max(0, av_x1 - pad_x), max(0, av_y1 - pad_y),
                min(w, av_x2 + pad_x), min(h, av_y2 + pad_y))

    def _process_qr(self, frame: np.ndarray, seen: set) -> set:
        """
        Scan for QRs by CROPPING the target area.

        ⚠️ CHANGED: the old code halved the WHOLE FRAME whenever `w > 800`.
           That speeds pyzbar up but also halves the QR's pixel size, which
           shortens the decode range -- a loss of about 30 % was measured.
           Cropping buys the same speed-up and gives up no resolution at all:
           the target area is already 50 % of the width by 80 % of the height,
           about 40 % of the pixels.
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

            # Undo the ROI offset and the downscale -> FULL-FRAME coordinates.
            bx, by, bw, bh = barcode.rect
            x  = x0 + bx * scale
            y  = y0 + by * scale
            bw = bw * scale
            bh = bh * scale

            # The QR's four REAL corners. pyzbar already returns them; on a
            # degenerate detection this is None and we fall back to the box.
            corners = polygon_to_corners(getattr(barcode, "polygon", None),
                                         x0, y0, scale)

            # --- Centre and in_av, both from the true quadrilateral ---
            #
            # CENTRE: the diagonal intersection, not the midpoint of the box.
            # Under perspective the two diverge -- measured at 2.8 px in a
            # realistic dive perspective, 9.2 px at aggressive angles.
            # With the 6 mm lens and this 2 MP sensor, 2.8 px is 0.075 degrees,
            # which is about 4 cm on the ground at 30 m. So it is MORE correct
            # but the effect is small; it is used because it costs nothing.
            #
            # in_av: CHANGES NO DECISION (see the quad_in_av docstring).
            # Because the target area is axis-aligned it is identical to the
            # box test. It stays because it states the intent, and because
            # pose estimation needs the corners anyway.
            qc = quad_center(corners)
            if corners and qc is not None:
                in_av = quad_in_av(corners, av_x1, av_y1, av_x2, av_y2)
                cx, cy = qc                 # diagonal intersection
                source = "quad"
            else:
                # Fallback: degenerate detection. The box is coarse but safe.
                in_av = (x >= av_x1 and y >= av_y1
                         and (x + bw) <= av_x2 and (y + bh) <= av_y2)
                cx, cy = x + bw / 2.0, y + bh / 2.0
                source = "box"

            # Green when inside the target area, orange when outside, so the
            # distinction is readable at a glance. Orange means "decoded but
            # does NOT count as a hit".
            color = (0, 255, 0) if in_av else (0, 165, 255)
            self._draw_qr_overlay(frame, corners, (x, y, bw, bh), (cx, cy),
                                  color, data, in_av, source)

            # Normalised error: ex>0 means the QR is to the right, ey>0 means
            # below (image y grows downward).
            ex = (cx - w / 2.0) / (w / 2.0)
            ey = (cy - h / 2.0) / (h / 2.0)

            # ⚠️ Published on EVERY frame.
            #    This used to be sent only the FIRST time a QR was seen (a
            #    `data not in seen` gate). With that gate, centering the dive
            #    was IMPOSSIBLE -- no fresh position ever arrived, just one
            #    stale message. `seen` now only exists to log once.
            self.result_queue.put_nowait({
                "type":    "qr",
                "data":    data,
                "box":     [int(x), int(y), int(bw), int(bh)],
                "corners": corners,          # [[x,y] x 4] or None
                "center":  [int(round(cx)), int(round(cy))],
                "error":   [round(ex, 4), round(ey, 4)],
                "frame":   [int(w), int(h)],
                "in_av":   bool(in_av),
                "src":     source,           # "quad" | "box"
            })

            if data not in seen:
                seen.add(data)
                logger.info("QR locked: %s  (inside target area: %s, source: %s)",
                            data, in_av, source)

            # The first valid QR is enough. ⚠️ This `break` used to sit INSIDE
            # the `data not in seen` block: seeing the same QR a second time
            # did not break the loop, so the remaining barcodes were scanned
            # for nothing.
            break

        return seen

    # ------------------------------------------------------------------
    # YOLO processing
    # ------------------------------------------------------------------
    def _process_yolo(self, frame: np.ndarray, model, center_x: int, center_y: int) -> None:
        results     = model(frame, verbose=False)[0]
        detections  = []

        for box in results.boxes:
            label = model.names[int(box.cls[0])]
            conf  = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            # Centre of the detected target.
            target_cx = (x1 + x2) // 2
            target_cy = (y1 + y2) // 2

            detections.append({
                "label": label,
                "conf":  round(conf, 3),
                "box":   [x1, y1, x2 - x1, y2 - y1],
            })

            # Bounding box and label, anti-aliased for a sharp overlay.
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 80, 0), 2, cv2.LINE_AA)
            self._draw_text(frame, f"{label} %{int(conf*100)}", (x1, y1 - 8), 0.6, (255, 80, 0))

            # Dot on the target centre.
            cv2.circle(frame, (target_cx, target_cy), 4, (0, 0, 255), -1, cv2.LINE_AA)

            # Lock-on line from the frame centre (the aircraft) to the target.
            cv2.line(frame, (center_x, center_y), (target_cx, target_cy), (0, 0, 255), 2, cv2.LINE_AA)

        if detections:
            logger.debug("YOLO detections: %d target(s)", len(detections))
            self.result_queue.put_nowait({"type": "yolo", "data": detections})

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _load_yolo_if_needed(self):
        mode = self._current_mode()
        if mode not in (PerceptionMode.YOLO, PerceptionMode.BOTH):
            return None
        try:
            from ultralytics import YOLO
            model = YOLO(self.yolo_model_path)
            logger.info("YOLO model loaded: %s", self.yolo_model_path)
            return model
        except Exception as e:
            logger.error("YOLO model could not be loaded: %s", e)
            return None

    def _current_mode(self) -> PerceptionMode:
        v = self._mode_value.value
        return [PerceptionMode.QR, PerceptionMode.YOLO, PerceptionMode.BOTH][v]

    @staticmethod
    def _recv_exactly(conn: socket.socket, buf: bytes, size: int) -> bytes:
        """Read from the socket until the buffer holds at least `size` bytes."""
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
                    logger.info("Perception active.")
                elif cmd == "stop":
                    self._start_event.clear()
                    logger.info("Perception stopped.")
                elif cmd == "set_mode":
                    raw = msg.get("mode", "qr")
                    try:
                        new_mode = PerceptionMode(raw)
                    except ValueError:
                        logger.warning("Unknown mode: %s", raw)
                        continue
                    self._mode_value.value = [PerceptionMode.QR, PerceptionMode.YOLO,
                                               PerceptionMode.BOTH].index(new_mode)
                    logger.info("Mode changed -> %s", new_mode.value)
                elif cmd == "kill":
                    self._stop_event.set()
            except Exception:
                break
