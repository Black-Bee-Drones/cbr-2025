from ament_index_python.packages import get_package_share_directory
import os

# Arena dimensions
ARENA_WIDTH = 8.0
ARENA_HEIGHT = 8.0
SEARCH_AREA_WIDTH = 7.0
SEARCH_AREA_HEIGHT = 7.0

# Flight parameters
TAKEOFF_ALTITUDE = 3.0  # meters - takeoff and search altitude
SEARCH_ALTITUDE = 3.0   # meters - maintain during search
CENTERING_ALTITUDE = 1.2  # meters - altitude above figure for landing

# Camera FOV calculations for Arducam v2.3 IMX219
# FOV: 62.2°(H) × 48.8°(V) at 1280x720
# At 3m: coverage ≈ 3.63m x 2.73m
# Grid spacing of 2.0m provides ~45% overlap for robust coverage
GRID_SPACING = 2.0 
WAYPOINT_VELOCITY = 0.8  # m/s - movement velocity between waypoints
CENTERING_VELOCITY = 0.3  # m/s - velocity during precision centering

# Grid pattern configuration
GRID_PATTERN_TYPE = "COLUMNS"  # "COLUMNS" or "ROWS"
GRID_PRIMARY_DIRECTION = "FORWARD"  # Primary movement direction
GRID_TRANSITION_DIRECTION = "RIGHT"  # Transition between columns/rows
GRID_START_OFFSET = (0.0, 0.0)  # (1.0, -0.75)  # Starting offset (forward 1m, right 0.75m)

# Landing base detection parameters
LANDING_BASE_SIZE = 1.0  # meters - 1m x 1m landing bases
MIN_LANDING_BASE_HEIGHT = 0.0
MAX_LANDING_BASE_HEIGHT = 1.5
EXPECTED_LANDING_BASES = 6  # number of landing bases to find


# Duplicate detection
DUPLICATE_BASE_RADIUS = 1.5  # meters - radius to consider as same base

# Timeouts
SEARCH_TIMEOUT = 30  # seconds - timeout per waypoint navigation
TAKEOFF_TIMEOUT = 20  # seconds - takeoff timeout

# YOLO detection parameters
YOLO_MODEL_PATH = os.path.join(
    get_package_share_directory("mapping"), "models", "yolov11n.onnx"
)
YOLO_CONFIDENCE_THRESHOLD = 0.7  
YOLO_IMAGE_SIZE = 640 
DETECTION_SAVE_PATH = "detections/"  # save detection images

# Centering control parameters
CENTERING_P_GAIN = 0.002  # P controller gain for centering (m/s per pixel)
CENTERING_TOLERANCE_PX = 20  # pixel tolerance for considering centered
CENTERING_TIMEOUT = 15  # seconds - max time for centering operation
LAND_WAIT_TIME = 15  # seconds - wait time after landing before takeoff

# Camera parameters
CAMERA_SOURCE = "webcam"
IMAGE_CENTER_X = 640  # camera image center X (pixels)
IMAGE_CENTER_Y = 360  # camera image center Y (pixels)

# Movement tolerances
VELOCITY_TOLERANCE = 0.1  # m/s - velocity considered as stopped
ALTITUDE_TOLERANCE = 0.1  # meters - altitude precision tolerance
POSITION_TOLERANCE = 0.3  # meters - position precision tolerance


