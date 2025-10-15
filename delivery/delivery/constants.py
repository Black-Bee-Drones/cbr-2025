from ament_index_python.packages import get_package_share_directory
import os


# Delivery constants
IS_INDOOR = True
TAKEOFF_ALTITUDE = 2.0  # Meters
TAKEOFF_SLEEP = 5.0 # Seconds
MAX_ALTITUDE = 2.8  # Meters
MIN_CENTERING_ALTITUDE = 0.8  # Meters, final altitude above figure for landing
TARGET_UP_ALTITUDE = 0.5  # Meters, how high the drone will climb if it doesn't find the package
TARGET_DOWN_ALTITUDE = -0.25  # Meters, distancia de descida para para centralizar
DETECTIONS_LOST_TOLERANCE = 3  # Number of missed detections to consider it as lost
PACKAGE_PROPORTION_ALIGN = 1.7  # How many times the smaller side fits into the larger one to consider it aligned

# Timeouts
SEARCH_TIMEOUT = 30  # seconds
CENTER_TIMEOUT = 60  # seconds
ALIGN_TIMEOUT = 60  # seconds
REACQUIRE_TIMEOUT = 30  # seconds
CENTER_TIMEOUT = 60  # seconds
ALIGN_TIMEOUT = 60  # seconds
REACQUIRE_TIMEOUT = 30  # seconds

# Package Info
STARTING_PACKAGE_IDX = -1
PACKAGE_POSITIONS = [
    {"x": -2.2, "y": 2.25},  # x: (+) frente
    {"x": -2.2, "y": 1.25},  #    (-) back
    {"x": -2.2, "y": 0.25},  # y: (+)
]                           #    (-)
DELIVER_POSITIONS = [
    {"x": -0.20, "y": 2.25},
    {"x": -0.20, "y": 2.25},
    {"x": -0.20, "y": 2.25},
]

# YOLO detection parameters
YOLO_MODEL_PATH_PKG = os.path.join(
    get_package_share_directory("delivery"), "models", "best1000.pt"
)
YOLO_MODEL_PATH_CROSS = os.path.join(
    get_package_share_directory("delivery"), "models", "yolov11nTC.pt"
)
YOLO_CONFIDENCE_THRESHOLD = 0.6 
DETECTION_SAVE_PATH = os.path.join(get_package_share_directory("delivery"), "detections")  # save detection images

# Camera
IMAGE_SOURCE = "imx219"
IMAGE_CALCULUS_OFFSET_Y = -231
CAMERA_FOV_HORIZONTAL = 62.2  # DEGREE
CAMERA_FOV_VERTICAL = 48.8  # DEGREE
IMAGE_CENTER_X = 820
IMAGE_CENTER_Y = 616

# Position control parameters
POSITION_CONTROLLER_KP_XY = 0.00031  # Proportional gain for XY movement
POSITION_CONTROLLER_KP_YAW = 0.11  # Proportional gain for yaw
POSITION_CONTROLLER_MAX_VELOCITY_XY = 0.22
POSITION_CONTROLLER_MIN_VELOCITY_XY = 0.02
POSITION_CONTROLLER_MAX_VELOCITY_YAW = 0.1  # rad/s
POSITION_CONTROLLER_TOLERANCE_XY = 35  # Meters, XY tolerance for considering the PID at the target
POSITION_CONTROLLER_TOLERANCE_Z = 0.1  # Meters, Z tolerance for considering the PID at the target

# Servo motor value
SERVO_PIN_OUT = 4
SERVO_OPEN_PWM = 1495
SERVO_CLOSE_PWM = 2006
SERVO_PIN_OUT = 4
SERVO_OPEN_PWM = 1495
SERVO_CLOSE_PWM = 2006
