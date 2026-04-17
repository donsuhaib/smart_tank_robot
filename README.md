# Smart Tank Robot Car

Python (laptop) + ESP32-CAM (robot) with a live annotated web dashboard.

## Project structure
```
smart_tank_robot/
├── app.py                 # Flask server + vision+decision loop
├── config.py              # All tunable settings
├── vision.py              # MJPEG reader, line detect, object detect, distance
├── robot_controller.py    # HTTP client to ESP32-CAM + ultrasonic filtering
├── state_machine.py       # FOLLOW_LINE / AVOID_OBSTACLE / SEARCH_LINE / STOPPED
├── requirements.txt
├── models/                # put MobileNetSSD_deploy.prototxt + .caffemodel here
├── templates/
│   └── index.html
├── static/
│   ├── style.css
│   └── app.js
└── firmware/
    └── esp32cam_robot.ino
```

## 1. Install Python dependencies
```
cd smart_tank_robot
python -m venv .venv
.venv\Scripts\activate        (Windows)
pip install -r requirements.txt
```
Linux/macOS activation:
```
source .venv/bin/activate
```

## 2. Download MobileNet-SSD model
Place these two files inside `models/`:
- `MobileNetSSD_deploy.prototxt`
- `MobileNetSSD_deploy.caffemodel`
(Standard Chuanqi305 Caffe MobileNet-SSD files used by OpenCV DNN.)
Dashboard still runs without them — just without object detection.

## 3. Configure ESP32-CAM IP
Edit `config.py` → set `ESP32_IP` to the IP printed by the ESP32 at boot.

## 4. Run the Flask server
```
python app.py
```
Open http://<your-laptop-ip>:5000 in a browser on the same Wi-Fi.

The main video panel is the **annotated** stream (bounding boxes, labels,
camera-estimated distance, line ROI/path, HUD with robot state and FPS).

## 5. Flashing the ESP32-CAM with CH340
Wiring:
- CH340 TX -> ESP32-CAM U0R (GPIO3 / RX)
- CH340 RX -> ESP32-CAM U0T (GPIO1 / TX)
- CH340 5V -> ESP32-CAM 5V
- CH340 GND -> ESP32-CAM GND
Steps:
1. Short **GPIO0 → GND** to enter flash mode.
2. Press the ESP32-CAM reset button.
3. In Arduino IDE: board = AI Thinker ESP32-CAM, pick the CH340 port.
4. Upload `firmware/esp32cam_robot.ino`.
5. After "Done uploading": remove GPIO0↔GND link, press reset.
6. Open Serial Monitor @115200 baud to read the IP.

Required Arduino libraries:
- ESP32 board package (espressif)
- ESP32Servo

## 6. Calibrate camera distance
1. Place a known object of known width (e.g. a bottle = ~7 cm) exactly 50 cm
   in front of the camera.
2. Note the observed bounding-box pixel width in the dashboard.
3. In `config.py`:
   - `REFERENCE_DISTANCE_CM = 50`
   - `REFERENCE_PIXEL_WIDTH = <observed>`
   - `FOCAL_LENGTH_PX = (REFERENCE_PIXEL_WIDTH * REFERENCE_DISTANCE_CM) / KNOWN_WIDTH_CM`
4. Update per-class `KNOWN_WIDTHS_CM` for objects you care about.

## Hardware summary
- ESP32-CAM AI Thinker on the car.
- L298N motor driver, 4 DC motors in tank drive (L-front+L-rear ; R-front+R-rear).
- HC-SR04 on SG90/MG90 servo for scanning.
- External motor battery; external regulated 5V for servo; common ground.
- HC-SR04 ECHO dropped 5V→3.3V via divider before GPIO4.

## Pin map
- GPIO12 -> L298N IN1
- GPIO13 -> L298N IN2
- GPIO14 -> L298N IN3
- GPIO15 -> L298N IN4
- GPIO2  -> Servo signal
- GPIO16 -> HC-SR04 TRIG
- GPIO4  -> HC-SR04 ECHO (via divider)

CAUTION: GPIO12 is boot-sensitive. Firmware drives it LOW first-thing at boot.
GPIO4 may toggle the onboard flash LED on some boards — confirm before relying on it.

## Troubleshooting
- `ModuleNotFoundError: cv2` -> run `pip install -r requirements.txt` inside your activated virtual environment.
- `404` on `/scan` or `/forward` -> your board is likely running `CameraWebServer`; flash `firmware/esp32cam_robot.ino`.
- Stream timeout to `:81/stream` -> check ESP32 IP in `config.py` and verify `http://<esp32-ip>:81/stream` opens in a browser.
