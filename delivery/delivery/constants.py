from ament_index_python.packages import get_package_share_directory
import os

# Delivery constants
TAKEOFF_ALTITUDE = 3.0  # Meters
SEARCH_TIMEOUT = 180  # seconds
CENTER_TIMEOUT = 180  # seconds
ALIGN_TIMEOUT = 180  # seconds
REACQUIRE_TARGET_TIMEOUT = 180  # seconds

# Package Info
STARTING_PACKAGE_IDX = 0
LAST_PACKAGE_IDX = 2
PACKAGE_POSITIONS = [{}]
DELIVER_POSITIONS = [{}]

# YOLO detection parameters
YOLO_MODEL_PATH = os.path.join(
    get_package_share_directory("mapping"), "models", "yolov11n.onnx"
)
YOLO_CONFIDENCE_THRESHOLD = 0.7  
YOLO_IMAGE_SIZE = 640 
DETECTION_SAVE_PATH = "detections/"  # save detection images

# Movement tolerances
VELOCITY_TOLERANCE = 0.1  # m/s - velocity considered as stopped
ALTITUDE_TOLERANCE = 0.1  # meters - altitude precision tolerance
POSITION_TOLERANCE = 0.3  # meters - position precision tolerance
CENTERING_VELOCITY = 0.3  # m/s - velocity during precision centering

# Height reference management
MAINTAIN_ABSOLUTE_HEIGHT = True  # Keep same height above original ground level
TARGET_HEIGHT_ABOVE_GROUND = 3.0  # meters - desired height above original ground
ALTITUDE_COMPENSATION_TIME = 5.0  # seconds - time for drone to auto-compensate height
ALTITUDE_COMPENSATION_GAIN = 0.4  # gain for counteracting auto-compensation
CENTERING_ALTITUDE = 1.2  # final altitude above figure for landing

# Camera
IMAGE_SOURCE = "webcam"
IMAGE_CENTER_X = 640
IMAGE_CENTER_Y = 360

# Position control parameters
POSITION_CONTROLLER_KP_XY = 0.6  # Proportional gain for XY movement
POSITION_CONTROLLER_KP_Z = 0.6  # Proportional gain for Z movement
POSITION_CONTROLLER_KP_YAW = 0.5  # Proportional gain for yaw
MAX_VELOCITY_XY = 0.8
MAX_VELOCITY_Z = 0.5
MAX_VELOCITY_YAW = 0.3  # rad/s
CENTERING_P_GAIN = 0.002 

# Centering and alignment
CENTER_ERROR_TOLERANCE = 0.1 # meters
CENTERING_TOLERANCE_PX = 20 #pixels
ALIGN_X_PROPORTION = 0.7

# Detections lost to restart detection
MIN_DETECTIONS_LOST = 5

# Vertical limit
MAX_ALTITUDE = 5.0  # meters
TARGET_UP_ALTITUDE = 0.5  # meters - how high the drone will climb if it doesn't find the package
