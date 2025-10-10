from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os

# Arena dimensions
ARENA_WIDTH = 8.0
ARENA_HEIGHT = 8.0
SEARCH_AREA_WIDTH = 4.5
SEARCH_AREA_HEIGHT = 4.5

# Flight parameters
TAKEOFF_ALTITUDE = 3.0  # meters - takeoff and search altitud
CENTERING_ALTITUDE = 1.2  # meters - altitude above figure for landing

# Camera FOV calculations for Arducam v2.3 IMX219
# FOV: 62.2°(H) × 48.8°(V) at 1280x720
# At 3m: coverage ≈ 3.63m x 2.73m
# Grid spacing of 1.25m
GRID_SPACING = 1.25
CENTERING_VEL_MAX = 0.21
CENTERING_VEL_MIN = 0.06

# Grid pattern configuration
GRID_PATTERN_TYPE = "COLUMNS"  # "COLUMNS" or "ROWS"
GRID_PRIMARY_DIRECTION = "FORWARD"  # Primary movement direction
GRID_TRANSITION_DIRECTION = "RIGHT"  # Transition between columns/rows
GRID_START_OFFSET = (
    0.75,
    -0.5,
)  # (forward 0.75m, right 0.5m)

# Duplicate detection
DUPLICATE_BASE_RADIUS = 1.2  # meters - radius to consider as same base

# Timeouts
SEARCH_TIMEOUT = 60
TAKEOFF_TIMEOUT = 20  # seconds - takeoff timeout

# YOLO detection parameters
YOLO_MODEL_PATH = os.path.join(
    get_package_share_directory("mapping"), "models", "best.engine"
)
YOLO_CONFIDENCE_THRESHOLD = 0.65
YOLO_IMAGE_SIZE = 640
DETECTION_SAVE_PATH = os.path.join(
    get_package_prefix("mapping").replace("install", "src"), "detections"
)

# Centering control parameters
CENTERING_P_GAIN = 0.00031  # P controller gain for centering (m/s per pixel)
CENTERING_TOLERANCE_PX = 200  # pixel tolerance for considering centered
CENTERING_TIMEOUT = 30  # seconds - max time for centering operation
LAND_WAIT_TIME = 5  # seconds - wait time after landing before takeoff
DESCEND_KP = 0.16

# Camera parameters
CAMERA_SOURCE = "imx219"
IMAGE_CENTER_X = 820  # camera image center X (pixels)
IMAGE_CENTER_Y = 616  # camera image center Y (pixels)
IMAGE_OFFSET_Y = 189
CENTER_DETECTION_THRESHOLD = 40

ALTITUDE_TOLERANCE = 0.1  # meters - altitude precision tolerance
POSITION_TOLERANCE = 0.10  # meters - position precision tolerance
