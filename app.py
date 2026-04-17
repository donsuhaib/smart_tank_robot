"""Flask dashboard server - the 'brain' of the robot.

Responsibilities:
    - pull MJPEG frames from the ESP32-CAM
    - run line detection, object detection, distance estimation
    - drive the state machine when autonomous mode is ON
    - send motor commands to the ESP32-CAM
    - serve the annotated video stream and the dashboard UI
"""

import time
import threading

import cv2
from flask import Flask, Response, jsonify, render_template, request

import config
from robot_controller import RobotController, UltrasonicFilter
from vision import (MJPEGStream, LineDetector, ObjectDetector,
                    estimate_distance_cm, annotate_frame)
from state_machine import StateMachine


# ---------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------
app       = Flask(__name__)
robot     = RobotController()
stream    = MJPEGStream().start()
line_det  = LineDetector()
obj_det   = ObjectDetector()
us_filter = UltrasonicFilter()
state_m   = StateMachine(robot)

shared_lock    = threading.Lock()
latest_frame   = None
latest_status  = {
    "state":            "STOPPED",
    "autonomous":       False,
    "line_position":    "lost",
    "line_confidence":  0.0,
    "object_name":      "",
    "object_confidence": 0.0,
    "object_distance_cm": -1.0,
    "ultrasonic_left":  -1.0,
    "ultrasonic_center": -1.0,
    "ultrasonic_right": -1.0,
    "fps":              0.0,
    "esp32_online":     False,
    "esp32_robot_api":  False,
    "firmware_hint":    "unknown",
    "stream_ok":        False,
    "stream_age_s":     -1.0,
    "stream_error":     "",
    "last_update":      0.0,
}
autonomous_enabled = False
last_us_poll       = 0.0


# ---------------------------------------------------------------------
# Vision + decision loop
# ---------------------------------------------------------------------
def processing_loop():
    global latest_frame, latest_status, last_us_poll

    prev_t = time.time()
    fps    = 0.0
    period = 1.0 / max(1, config.CONTROL_LOOP_HZ)

    while True:
        loop_start = time.time()
        frame = stream.read()
        if frame is None:
            time.sleep(0.05)
            continue

        # --- vision
        line_pos, line_conf, line_overlay = line_det.detect(frame)
        detections = obj_det.detect(frame)
        primary    = ObjectDetector.select_primary(detections, frame.shape)

        primary_distance = -1.0
        primary_label    = ""
        primary_conf     = 0.0
        if primary is not None:
            x1, y1, x2, y2 = primary["box"]
            pixel_w = (x2 - x1)
            primary_distance = estimate_distance_cm(primary["label"], pixel_w)
            primary_label    = primary["label"]
            primary_conf     = primary["confidence"]

        # --- ultrasonic (poll a few times per second, not every frame)
        now = time.time()
        raw_scan = None
        if now - last_us_poll > 0.25:
            raw_scan = robot.scan()
            us_clean = us_filter.update(raw_scan) if raw_scan else None
            last_us_poll = now
        else:
            us_clean = {
                "left":   latest_status["ultrasonic_left"],
                "center": latest_status["ultrasonic_center"],
                "right":  latest_status["ultrasonic_right"],
            }

        # --- state machine
        robot_api_ok = robot.robot_api_supported()
        if autonomous_enabled and robot_api_ok:
            cam_d = primary_distance if primary_distance > 0 else None
            state_m.update(line_pos, line_conf, cam_d, us_clean)
        else:
            if state_m.state != "STOPPED":
                state_m.state = "STOPPED"

        # --- fps
        dt = time.time() - prev_t
        prev_t = time.time()
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)

        # --- annotate and store
        stream_state = stream.status()
        annotated = annotate_frame(
            frame,
            line_overlay,
            detections,
            primary,
            primary_distance if primary_distance > 0 else None,
            us_clean["center"] if us_clean and us_clean["center"] > 0 else None,
            state_m.state,
            autonomous_enabled,
            fps,
        )

        with shared_lock:
            latest_frame = annotated
            latest_status.update({
                "state":             state_m.state,
                "autonomous":        autonomous_enabled,
                "line_position":     line_pos,
                "line_confidence":   round(float(line_conf), 3),
                "object_name":       primary_label,
                "object_confidence": round(float(primary_conf), 3),
                "object_distance_cm": float(primary_distance),
                "ultrasonic_left":   float(us_clean["left"])   if us_clean else -1.0,
                "ultrasonic_center": float(us_clean["center"]) if us_clean else -1.0,
                "ultrasonic_right":  float(us_clean["right"])  if us_clean else -1.0,
                "fps":               round(fps, 1),
                "esp32_online":      (raw_scan is not None) if raw_scan is not None else latest_status["esp32_online"],
                "esp32_robot_api":   robot_api_ok,
                "firmware_hint":     robot.firmware_hint(),
                "stream_ok":         bool(stream_state["stream_ok"]),
                "stream_age_s":      float(stream_state["last_frame_age_s"]) if stream_state["last_frame_age_s"] is not None else -1.0,
                "stream_error":      stream_state["last_stream_error"],
                "last_update":       time.time(),
            })

        # pace the loop
        slept = time.time() - loop_start
        if slept < period:
            time.sleep(period - slept)


# ---------------------------------------------------------------------
# Flask routes
# ---------------------------------------------------------------------
@app.route("/")
def index():
    return render_template("index.html")


def _mjpeg_generator():
    while True:
        with shared_lock:
            frame = None if latest_frame is None else latest_frame.copy()
        if frame is None:
            time.sleep(0.03)
            continue
        ok, jpg = cv2.imencode(
            ".jpg", frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), config.JPEG_QUALITY],
        )
        if not ok:
            continue
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n")


@app.route("/video")
def video():
    return Response(_mjpeg_generator(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/status")
def api_status():
    with shared_lock:
        return jsonify(latest_status)


@app.route("/api/control", methods=["POST"])
def api_control():
    global autonomous_enabled
    cmd = (request.json or {}).get("cmd", "").lower()

    # manual commands disable autonomous
    manual_cmds = {"forward", "backward", "left", "right", "stop"}
    if cmd in manual_cmds:
        if autonomous_enabled:
            autonomous_enabled = False
            state_m.stop_autonomous()
        ok = getattr(robot, cmd)()
        return jsonify({"ok": bool(ok), "cmd": cmd,
                        "autonomous": autonomous_enabled})

    if cmd == "auto_on":
        if not robot.robot_api_supported():
            return jsonify({
                "ok": False,
                "error": "Robot firmware endpoints unavailable. Flash esp32cam_robot firmware.",
                "autonomous": False,
            }), 409
        autonomous_enabled = True
        state_m.set_state("FOLLOW_LINE")
        return jsonify({"ok": True, "autonomous": True})

    if cmd == "auto_off":
        autonomous_enabled = False
        state_m.stop_autonomous()
        return jsonify({"ok": True, "autonomous": False})

    if cmd == "servo":
        angle = int((request.json or {}).get("angle", 90))
        robot.set_servo(angle)
        return jsonify({"ok": True, "angle": angle})

    return jsonify({"ok": False, "error": f"unknown cmd: {cmd}"}), 400


# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------
def main():
    t = threading.Thread(target=processing_loop, daemon=True)
    t.start()
    print(f"[app] Flask running on http://{config.FLASK_HOST}:{config.FLASK_PORT}")
    print(f"[app] ESP32-CAM expected at {config.ESP32_BASE_URL}")
    app.run(host=config.FLASK_HOST, port=config.FLASK_PORT,
            debug=config.FLASK_DEBUG, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
