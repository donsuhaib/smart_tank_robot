"""
Central configuration for the smart tank robot.
Adjust these values to your own hardware / environment.
"""

# ----------------------------------------------------------------------
# ESP32-CAM network settings
# ----------------------------------------------------------------------
ESP32_IP         = "192.168.68.111"          # change to your ESP32-CAM IP
ESP32_PORT       = 80
ESP32_STREAM_URL = f"http://{ESP32_IP}:81/stream"   # AI-Thinker default stream
ESP32_BASE_URL   = f"http://{ESP32_IP}:{ESP32_PORT}"
HTTP_TIMEOUT     = 0.8                     # seconds

# ----------------------------------------------------------------------
# Flask server
# ----------------------------------------------------------------------
FLASK_HOST  = "0.0.0.0"
FLASK_PORT  = 5000
FLASK_DEBUG = False

# ----------------------------------------------------------------------
# Vision - frame processing
# ----------------------------------------------------------------------
FRAME_WIDTH   = 320
FRAME_HEIGHT  = 240
JPEG_QUALITY  = 70

# ROI used for line following (fraction of frame)
ROI_TOP    = 0.60
ROI_BOTTOM = 0.95
ROI_LEFT   = 0.05
ROI_RIGHT  = 0.95

# Line detection thresholds
LINE_BINARY_THRESHOLD = 70     # dark-line threshold
LINE_MIN_AREA         = 300    # minimum contour area to accept
LINE_CONFIDENCE_MIN   = 0.35   # minimum fraction of ROI filled
LINE_SMOOTH_HISTORY   = 5      # number of frames used to smooth decision

# ----------------------------------------------------------------------
# Object detection
# ----------------------------------------------------------------------
MODEL_PROTOTXT  = "models/MobileNetSSD_deploy.prototxt"
MODEL_WEIGHTS   = "models/MobileNetSSD_deploy.caffemodel"
DETECT_CONFIDENCE = 0.45

MOBILENET_SSD_CLASSES = [
    "background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
    "dog", "horse", "motorbike", "person", "pottedplant",
    "sheep", "sofa", "train", "tvmonitor",
]

# ----------------------------------------------------------------------
# Camera-based distance estimation
#   distance_cm = (KNOWN_WIDTH_CM * FOCAL_LENGTH_PX) / pixel_width
# ----------------------------------------------------------------------
FOCAL_LENGTH_PX = 525.0       # calibrate for your camera
REFERENCE_DISTANCE_CM = 50.0  # reference used for calibration
REFERENCE_PIXEL_WIDTH = 120.0 # measured pixel width at REFERENCE_DISTANCE

# Per-class real-world widths in centimeters (rough averages)
KNOWN_WIDTHS_CM = {
    "person":       45.0,
    "bottle":        7.0,
    "chair":        45.0,
    "cat":          25.0,
    "dog":          35.0,
    "car":         180.0,
    "bicycle":      60.0,
    "motorbike":    70.0,
    "bus":         250.0,
    "aeroplane":   900.0,
    "tvmonitor":    55.0,
    "sofa":        180.0,
    "diningtable": 120.0,
    "pottedplant":  25.0,
    "bird":         20.0,
    "sheep":        80.0,
    "horse":       200.0,
    "cow":         200.0,
    "train":       300.0,
    "boat":        300.0,
}
DEFAULT_KNOWN_WIDTH_CM = 30.0

# ----------------------------------------------------------------------
# State machine thresholds
# ----------------------------------------------------------------------
OBSTACLE_STOP_CM       = 20.0   # stop/avoid if closer
OBSTACLE_CLEAR_CM      = 35.0   # hysteresis - must be farther than this to clear
CAMERA_OBSTACLE_CM     = 30.0   # cam-distance threshold used for obstacle avoidance
ULTRASONIC_MIN_VALID   = 2.0
ULTRASONIC_MAX_VALID   = 400.0
ULTRASONIC_SMOOTH_N    = 5
CONTROL_LOOP_HZ        = 10     # decision loop frequency

# Logging
VERBOSE = True
