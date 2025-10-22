from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os

# Controller constants
TAKEOFF_POSE = 1.0
TAKEOFF_HEIGHT = 2.0
TAKEOFF_TIMEOUT = 10.0
ALTITUDE_TOLERANCE = 0.1
VELOCITY_UP_DOWN = 0.2
VELOCITY_SIDES = 0.2
VELOCITY_IN_OUT = 0.2
VELOCITY_YAW = 0.1

ACTION_TIMEOUT = 0.05
SLEEP_AFTER_TAKEOFF = 2.0
SLEEP_AFTER_LAND = 1.0

YOLO_MODEL_PATH = os.path.join(
    get_package_share_directory("interaction"), "models", "yolo11n-pose.pt"
)
YOLO_CONFIDENCE_THRESHOLD = 0.65
YOLO_IMAGE_SIZE = 640
DETECTION_SAVE_PATH = os.path.join(
    get_package_prefix("interaction"), "share", "detections_phase4"
)

GESTURE_CONFIRMATION_THRESHOLD = 8  # For continuous gestures (move, yaw)
GESTURE_CONFIRMATION_THRESHOLD_SINGLE = 2  # For single actions (takeoff, land)
MAX_GESTURE_FRAMES = 300
