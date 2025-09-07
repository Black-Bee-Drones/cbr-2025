from ament_index_python.packages import get_package_share_directory
import os

# Arena dimensions
ARENA_WIDTH = 8.0
ARENA_HEIGHT = 8.0
SEARCH_AREA_WIDTH = 6.5
SEARCH_AREA_HEIGHT = 6.5

# Flight parameters
TAKEOFF_ALTITUDE = 2.5
SEARCH_ALTITUDE = 2.5
RTL_ALTITUDE = 2.5

# Grid waypoint parameters
GRID_SPACING = 1.5  # meters - spacing between grid points
WAYPOINT_VELOCITY = 0.8  # m/s - movement velocity between waypoints
CENTERING_VELOCITY = 0.3  # m/s - velocity during precision centering

# Landing base detection parameters
LANDING_BASE_SIZE = 1.0  # meters - 1m x 1m landing bases
MIN_LANDING_BASE_HEIGHT = 0.0
MAX_LANDING_BASE_HEIGHT = 1.5
EXPECTED_LANDING_BASES = 6  # number of landing bases to find

# Height reference management
MAINTAIN_ABSOLUTE_HEIGHT = True  # Keep same height above original ground level
TARGET_HEIGHT_ABOVE_GROUND = 2.5  # meters - desired height above original ground

# Position precision
LANDING_PRECISION = 0.2  # meters - precision radius for landing

# Timeouts
SEARCH_TIMEOUT = 300  # seconds - 5 minutes search timeout per attempt
LANDING_TIMEOUT = 30  # seconds - landing timeout
TAKEOFF_TIMEOUT = 20  # seconds - takeoff timeout

# YOLO detection parameters
YOLO_MODEL_PATH = os.path.join(
    get_package_share_directory("mapping"), "models", "yolov11n.onnx"
)
YOLO_CONFIDENCE_THRESHOLD = 0.7  # confidence threshold for detection
YOLO_IMAGE_SIZE = 640  # input image size for YOLO
DETECTION_SAVE_PATH = "detections/"  # path to save detection images

# Centering control parameters
CENTERING_P_GAIN = 0.002  # P controller gain for centering (m/s per pixel)
CENTERING_TOLERANCE_PX = 20  # pixel tolerance for considering centered
CENTERING_TIMEOUT = 15  # seconds - max time for centering operation
LAND_WAIT_TIME = 15  # seconds - wait time after landing before takeoff

# Camera parameters
CAMERA_SOURCE = "webcam"
IMAGE_CENTER_X = 320  # camera image center X (pixels)
IMAGE_CENTER_Y = 240  # camera image center Y (pixels)

# Movement tolerances
VELOCITY_TOLERANCE = 0.1  # m/s - velocity considered as stopped
ALTITUDE_TOLERANCE = 0.2  # meters - altitude precision tolerance
POSITION_TOLERANCE = 0.3  # meters - position precision tolerance

# Position control parameters
POSITION_CONTROLLER_KP_XY = 0.8  # Proportional gain for XY movement
POSITION_CONTROLLER_KP_Z = 0.6  # Proportional gain for Z movement
POSITION_CONTROLLER_KP_YAW = 0.5  # Proportional gain for yaw
MAX_VELOCITY_XY = 0.8
MAX_VELOCITY_Z = 0.5

# Safety parameters
MIN_SAFE_ALTITUDE = 1.0
MAX_SAFE_ALTITUDE = 4.0
WALL_CLEARANCE = 0.5
