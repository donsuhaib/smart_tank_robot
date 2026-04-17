"""
Vision pipeline:
    - frame acquisition from the ESP32-CAM MJPEG stream
    - line detection in the lower ROI
    - object detection (MobileNet-SSD)
    - camera-based distance estimation
    - frame annotation for the dashboard
"""

import time
import threading
from collections import deque

import cv2
import numpy as np
import requests

import config


# =====================================================================
# MJPEG stream reader
# =====================================================================
class MJPEGStream:
    """
    Background thread that keeps the latest frame from the ESP32-CAM
    MJPEG stream available without blocking the main loop.
    """

    def __init__(self, url=config.ESP32_STREAM_URL):
        self.url       = url
        self.frame     = None
        self.running   = False
        self.lock      = threading.Lock()
        self.thread    = None
        self.last_ok   = 0.0
        self.last_error = ""
        self._last_error_log_at = 0.0
        self._retry_delay = 0.5

    def start(self):
        self.running = True
        self.thread  = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()
        return self

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

    def _reader(self):
        while self.running:
            try:
                with requests.get(self.url, stream=True, timeout=3.0) as r:
                    r.raise_for_status()
                    bytes_buf = b""
                    for chunk in r.iter_content(chunk_size=4096):
                        if not self.running:
                            break
                        bytes_buf += chunk
                        a = bytes_buf.find(b"\xff\xd8")
                        b = bytes_buf.find(b"\xff\xd9")
                        if a != -1 and b != -1 and b > a:
                            jpg = bytes_buf[a:b + 2]
                            bytes_buf = bytes_buf[b + 2:]
                            img = cv2.imdecode(
                                np.frombuffer(jpg, dtype=np.uint8),
                                cv2.IMREAD_COLOR,
                            )
                            if img is not None:
                                img = cv2.resize(
                                    img,
                                    (config.FRAME_WIDTH, config.FRAME_HEIGHT),
                                )
                                with self.lock:
                                    self.frame = img
                                    self.last_ok = time.time()
                                    self.last_error = ""
                                    self._retry_delay = 0.5
            except Exception as e:
                now = time.time()
                err = str(e)
                with self.lock:
                    self.last_error = err
                if config.VERBOSE and (now - self._last_error_log_at) >= 5.0:
                    print(f"[stream] reconnecting ({self.url}) failed: {err}")
                    self._last_error_log_at = now
                time.sleep(self._retry_delay)
                self._retry_delay = min(5.0, self._retry_delay * 1.6)

    def read(self):
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()

    def status(self):
        with self.lock:
            return {
                "stream_ok": self.frame is not None,
                "last_frame_age_s": (time.time() - self.last_ok) if self.last_ok > 0 else None,
                "last_stream_error": self.last_error,
            }


# =====================================================================
# Line detector
# =====================================================================
class LineDetector:
    def __init__(self):
        self.history = deque(maxlen=config.LINE_SMOOTH_HISTORY)

    def detect(self, frame):
        """
        Returns: (position, confidence, annotated_overlay_data)
            position:   'left' | 'center' | 'right' | 'lost'
            confidence: 0..1
            overlay:    dict with polygons to draw
        """
        h, w = frame.shape[:2]
        y1 = int(h * config.ROI_TOP)
        y2 = int(h * config.ROI_BOTTOM)
        x1 = int(w * config.ROI_LEFT)
        x2 = int(w * config.ROI_RIGHT)
        roi = frame[y1:y2, x1:x2]

        gray    = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(
            blurred, config.LINE_BINARY_THRESHOLD, 255, cv2.THRESH_BINARY_INV
        )

        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN,  kernel, iterations=1)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)

        # Split ROI into thirds
        rh, rw   = binary.shape
        third    = rw // 3
        left_s   = int(np.sum(binary[:, :third]       > 0))
        center_s = int(np.sum(binary[:, third:2*third] > 0))
        right_s  = int(np.sum(binary[:, 2*third:]     > 0))
        total    = left_s + center_s + right_s
        roi_area = rh * rw

        position   = "lost"
        confidence = 0.0

        if total >= config.LINE_MIN_AREA:
            confidence = total / float(roi_area)
            if confidence >= config.LINE_CONFIDENCE_MIN:
                best = max((left_s, "left"), (center_s, "center"), (right_s, "right"))
                position = best[1]

        self.history.append(position)
        smoothed = self._smooth(position)

        # contour points for overlay
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        overlay_contours = []
        for c in contours:
            if cv2.contourArea(c) >= config.LINE_MIN_AREA / 3:
                shifted = c + np.array([[x1, y1]])
                overlay_contours.append(shifted)

        overlay = {
            "roi_rect": (x1, y1, x2, y2),
            "thirds":   [x1 + third, x1 + 2 * third],
            "contours": overlay_contours,
            "position": smoothed,
            "confidence": confidence,
        }
        return smoothed, confidence, overlay

    def _smooth(self, current):
        if not self.history:
            return current
        counts = {}
        for p in self.history:
            counts[p] = counts.get(p, 0) + 1
        return max(counts.items(), key=lambda kv: kv[1])[0]


# =====================================================================
# Object detector (MobileNet-SSD via OpenCV DNN)
# =====================================================================
class ObjectDetector:
    def __init__(self):
        self.net       = None
        self.available = False
        try:
            self.net = cv2.dnn.readNetFromCaffe(
                config.MODEL_PROTOTXT, config.MODEL_WEIGHTS
            )
            self.available = True
            if config.VERBOSE:
                print("[vision] MobileNet-SSD loaded")
        except Exception as e:
            print(f"[vision] Could not load object detector: {e}")
            print("         Dashboard will run without object detection.")

    def detect(self, frame):
        """
        Returns a list of detections: [{label, confidence, box(x1,y1,x2,y2)}...]
        """
        if not self.available or frame is None:
            return []

        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(
            cv2.resize(frame, (300, 300)),
            0.007843, (300, 300), 127.5,
        )
        self.net.setInput(blob)
        detections = self.net.forward()

        results = []
        for i in range(detections.shape[2]):
            conf = float(detections[0, 0, i, 2])
            if conf < config.DETECT_CONFIDENCE:
                continue
            idx = int(detections[0, 0, i, 1])
            if idx < 0 or idx >= len(config.MOBILENET_SSD_CLASSES):
                continue
            label = config.MOBILENET_SSD_CLASSES[idx]
            if label == "background":
                continue
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            x1, y1, x2, y2 = box.astype(int)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w - 1, x2), min(h - 1, y2)
            if x2 <= x1 or y2 <= y1:
                continue
            results.append({
                "label":      label,
                "confidence": conf,
                "box":        (int(x1), int(y1), int(x2), int(y2)),
            })
        return results

    # ------------------------------------------------------------------
    @staticmethod
    def select_primary(detections, frame_shape):
        """Pick the 'most relevant' object - largest near the horizontal centre."""
        if not detections:
            return None
        h, w = frame_shape[:2]
        cx   = w / 2.0

        def score(d):
            x1, y1, x2, y2 = d["box"]
            area      = (x2 - x1) * (y2 - y1)
            obj_cx    = (x1 + x2) / 2.0
            centre_w  = 1.0 - abs(obj_cx - cx) / cx   # 1 at centre, 0 at edge
            return area * max(0.2, centre_w)

        return max(detections, key=score)


# =====================================================================
# Distance estimation
# =====================================================================
def estimate_distance_cm(label, pixel_width):
    """
    Monocular distance using known real-world width and focal length.
    Falls back to the explicit reference calibration when pixel_width is valid.
    """
    if pixel_width is None or pixel_width <= 1:
        return -1.0

    known_w = config.KNOWN_WIDTHS_CM.get(label, config.DEFAULT_KNOWN_WIDTH_CM)

    # Primary estimation using configured focal length
    d1 = (known_w * config.FOCAL_LENGTH_PX) / float(pixel_width)

    # Cross-check with calibration-based ratio method
    if config.REFERENCE_PIXEL_WIDTH > 0:
        d2 = (config.REFERENCE_DISTANCE_CM
              * config.REFERENCE_PIXEL_WIDTH
              / float(pixel_width))
        return round((d1 + d2) / 2.0, 1)
    return round(d1, 1)


# =====================================================================
# Frame annotator
# =====================================================================
def annotate_frame(frame,
                   line_overlay,
                   detections,
                   primary,
                   primary_distance_cm,
                   ultrasonic_center_cm,
                   robot_state,
                   autonomous,
                   fps):
    """Draw every piece of info on the frame for the dashboard stream."""
    img = frame.copy()
    h, w = img.shape[:2]

    # ---- Line ROI
    if line_overlay is not None:
        x1, y1, x2, y2 = line_overlay["roi_rect"]
        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 120, 0), 1)
        for tx in line_overlay["thirds"]:
            cv2.line(img, (tx, y1), (tx, y2), (180, 180, 180), 1)
        # line contours
        if line_overlay["contours"]:
            cv2.drawContours(img, line_overlay["contours"], -1, (0, 255, 0), 2)
        # direction arrow
        pos = line_overlay["position"]
        cy  = int((y1 + y2) / 2)
        cxs = {"left": x1 + 20, "center": (x1 + x2) // 2, "right": x2 - 20, "lost": -1}
        cxp = cxs.get(pos, -1)
        if cxp > 0:
            cv2.arrowedLine(img, ((x1 + x2) // 2, y2 - 5), (cxp, cy),
                            (0, 255, 255), 3, tipLength=0.3)

    # ---- Object detections
    for d in detections or []:
        x1, y1, x2, y2 = d["box"]
        colour = (0, 140, 255) if d is not primary else (0, 0, 255)
        cv2.rectangle(img, (x1, y1), (x2, y2), colour, 2)
        label = f"{d['label']} {d['confidence']*100:.0f}%"
        cv2.putText(img, label, (x1, max(12, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)

    # ---- Primary object: distance label on bbox
    if primary is not None and primary_distance_cm is not None and primary_distance_cm > 0:
        x1, y1, x2, y2 = primary["box"]
        text = f"{primary_distance_cm:.0f} cm"
        cv2.putText(img, text, (x1, min(h - 4, y2 + 16)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

    # ---- HUD (top-left)
    hud_lines = [
        f"State: {robot_state}",
        f"Auto : {'ON' if autonomous else 'OFF'}",
        f"FPS  : {fps:.1f}",
    ]
    if primary is not None:
        hud_lines.append(f"Obj  : {primary['label']}")
        if primary_distance_cm and primary_distance_cm > 0:
            hud_lines.append(f"ObjD : {primary_distance_cm:.0f} cm")
    if ultrasonic_center_cm is not None and ultrasonic_center_cm > 0:
        hud_lines.append(f"US-C : {ultrasonic_center_cm:.0f} cm")
    if line_overlay is not None:
        hud_lines.append(f"Line : {line_overlay['position']}")

    y = 16
    for text in hud_lines:
        cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 1, cv2.LINE_AA)
        y += 16

    return img
