from ament_index_python.packages import get_package_share_directory
import os


# Delivery constants
IS_INDOOR = True
TAKEOFF_ALTITUDE = 2.0  # Meters
TAKEOFF_SLEEP = 5.0 # Seconds
MAX_ALTITUDE = 4.0  # Meters
MIN_CENTERING_ALTITUDE = 1.2  # Meters, final altitude above figure for landing
TARGET_UP_ALTITUDE = 0.5  # Meters, how high the drone will climb if it doesn't find the package
TARGET_DOWN_ALTITUDE = -0.5  # Meters, distancia de descida para para centralizar
DETECTIONS_LOST_TOLERANCE = 3 # Number of missed detections to consider it as lost
PACKAGE_PROPORTION_ALIGN = 1  # How many times the smaller side fits into the larger one to consider it aligned

# Timeouts
SEARCH_TIMEOUT = 30  # seconds
CENTER_TIMEOUT = 30  # seconds
ALIGN_TIMEOUT = 30  # seconds
REACQUIRE_TIMEOUT = 30  # seconds

# Package Info
STARTING_PACKAGE_IDX = -1
PACKAGE_POSITIONS = [
    {"x": 0.0, "y": -1.0},
    {"x": 0.0, "y": -2.0},
    {"x": 0.0, "y": -3.0},
]
DELIVER_POSITIONS = [
    {"x": 2.0, "y": -1.0},
    {"x": 2.0, "y": -2.0},
    {"x": 2.0, "y": -3.0},
]

# YOLO detection parameters
YOLO_MODEL_PATH_PKG = os.path.join(
    get_package_share_directory("delivery"), "models", "yolov11nMKDrone.pt"
)
YOLO_MODEL_PATH_CROSS = os.path.join(
    get_package_share_directory("delivery"), "models", "yolov11nTC.pt"
)
YOLO_CONFIDENCE_THRESHOLD = 0.2 
DETECTION_SAVE_PATH = "detections/"  # save detection images

# Camera
IMAGE_SOURCE = "webcam"
IMAGE_CALCULUS_OFFSET_X = {"x": 0.15}
CAMERA_FOV_HORIZONTAL = 62.2  # DEGREE
CAMERA_FOV_VERTICAL = 48.8  # DEGREE

# Position control parameters
POSITION_CONTROLLER_KP_XY = 0.5  # Proportional gain for XY movement
POSITION_CONTROLLER_KP_YAW = 0.5  # Proportional gain for yaw
POSITION_CONTROLLER_MAX_VELOCITY_XY = 0.8
POSITION_CONTROLLER_MAX_VELOCITY_YAW = 0.3  # rad/s
POSITION_CONTROLLER_TOLERANCE_XY = 0.1  # Meters, XY tolerance for considering the PID at the target
POSITION_CONTROLLER_TOLERANCE_Z = 0.1  # Meters, Z tolerance for considering the PID at the target

# Servo motor value
SERVO_PIN_OUT = 1
SERVO_OPEN_PWM = 1200
SERVO_CLOSE_PWM = 1800
