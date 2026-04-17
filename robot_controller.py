"""
Robot controller: talks to the ESP32-CAM over HTTP.
All low-level actuation and sensor reading happens on the ESP32;
this module only issues commands and polls status.
"""

import time
import threading
import requests

import config


class RobotController:
    def __init__(self):
        self._last_cmd      = None
        self._last_cmd_time = 0.0
        self._lock          = threading.Lock()
        self._robot_api_supported = None   # None=unknown, True/False=detected
        self._firmware_hint = "unknown"
        self._status        = {
            "state":          "UNKNOWN",
            "distance_left":   -1.0,
            "distance_center": -1.0,
            "distance_right":  -1.0,
            "autonomous":     False,
            "online":         False,
        }

    # ------------------------------------------------------------------
    # low-level HTTP helper
    # ------------------------------------------------------------------
    def _get(self, path, params=None, silent=False):
        url = f"{config.ESP32_BASE_URL}{path}"
        try:
            r = requests.get(url, params=params, timeout=config.HTTP_TIMEOUT)
            r.raise_for_status()
            return r
        except requests.RequestException as e:
            if config.VERBOSE and not silent:
                print(f"[robot] HTTP error for {url}: {e}")
            return None

    def _probe_robot_api(self):
        """
        Detect whether the ESP32 exposes the custom robot endpoints.
        CameraWebServer returns 404 for these routes.
        """
        if self._robot_api_supported is not None:
            return self._robot_api_supported

        # Probe with a harmless endpoint expected by our custom firmware.
        r = self._get("/scan", silent=True)
        self._robot_api_supported = (r is not None)
        self._firmware_hint = "esp32cam_robot" if r is not None else "camera_web_server_or_unknown"

        if config.VERBOSE and not self._robot_api_supported:
            print("[robot] Robot API endpoints not found on ESP32 (likely CameraWebServer firmware).")
        return self._robot_api_supported

    # ------------------------------------------------------------------
    # commands
    # ------------------------------------------------------------------
    def _send_cmd(self, path):
        if not self._probe_robot_api():
            return False
        # avoid spamming identical commands faster than 50 ms
        now = time.time()
        with self._lock:
            if self._last_cmd == path and (now - self._last_cmd_time) < 0.05:
                return True
            self._last_cmd      = path
            self._last_cmd_time = now
        return self._get(path) is not None

    def forward(self):   return self._send_cmd("/forward")
    def backward(self):  return self._send_cmd("/backward")
    def left(self):      return self._send_cmd("/left")
    def right(self):     return self._send_cmd("/right")
    def stop(self):      return self._send_cmd("/stop")

    def set_servo(self, angle):
        if not self._probe_robot_api():
            return False
        angle = int(max(0, min(180, angle)))
        return self._get("/servo", params={"angle": angle}) is not None

    def scan(self):
        """
        Trigger a 3-angle sweep on the ESP32-CAM and return
        {left, center, right} distances (cm).
        """
        if not self._probe_robot_api():
            return None
        r = self._get("/scan")
        if r is None:
            return None
        try:
            data = r.json()
            return {
                "left":   float(data.get("distance_left",   -1.0)),
                "center": float(data.get("distance_center", -1.0)),
                "right":  float(data.get("distance_right",  -1.0)),
            }
        except Exception as e:
            if config.VERBOSE:
                print(f"[robot] scan parse error: {e}")
            return None

    def fetch_status(self):
        r = self._get("/status")
        if r is None:
            with self._lock:
                self._status["online"] = False
            return self._status
        try:
            data = r.json()
            # Heuristic: stock CameraWebServer /status has camera settings and no robot fields.
            if self._robot_api_supported is None:
                has_robot_fields = any(k in data for k in ("distance_left", "distance_center", "distance_right", "state"))
                self._robot_api_supported = bool(has_robot_fields)
                self._firmware_hint = "esp32cam_robot" if has_robot_fields else "camera_web_server_or_unknown"
            with self._lock:
                self._status.update({
                    "state":           data.get("state", "UNKNOWN"),
                    "distance_left":   float(data.get("distance_left",  -1)),
                    "distance_center": float(data.get("distance_center", -1)),
                    "distance_right":  float(data.get("distance_right", -1)),
                    "autonomous":      bool(data.get("autonomous", False)),
                    "online":          True,
                })
        except Exception as e:
            if config.VERBOSE:
                print(f"[robot] status parse error: {e}")
            with self._lock:
                self._status["online"] = False
        return self._status

    def status_snapshot(self):
        with self._lock:
            return dict(self._status)

    def robot_api_supported(self):
        if self._robot_api_supported is None:
            self._probe_robot_api()
        return bool(self._robot_api_supported)

    def firmware_hint(self):
        if self._robot_api_supported is None:
            self._probe_robot_api()
        return self._firmware_hint


# ----------------------------------------------------------------------
# Ultrasonic filtering helper
# ----------------------------------------------------------------------
class UltrasonicFilter:
    """Simple rolling-median filter that rejects invalid spikes."""

    def __init__(self, n=config.ULTRASONIC_SMOOTH_N):
        self.n = n
        self.buffers = {"left": [], "center": [], "right": []}

    def update(self, scan):
        if scan is None:
            return None
        out = {}
        for k in ("left", "center", "right"):
            v = scan.get(k, -1.0)
            if config.ULTRASONIC_MIN_VALID <= v <= config.ULTRASONIC_MAX_VALID:
                buf = self.buffers[k]
                buf.append(v)
                if len(buf) > self.n:
                    buf.pop(0)
                sorted_buf = sorted(buf)
                out[k] = sorted_buf[len(sorted_buf) // 2]
            else:
                out[k] = -1.0
        return out
