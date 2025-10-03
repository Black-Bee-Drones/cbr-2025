from ament_index_python.packages import get_package_share_directory
import os


# Delivery constants
TAKEOFF_ALTITUDE = 3.0  # Meters
MAX_ALTITUDE = 5.0  # Meters
MIN_CENTERING_ALTITUDE = 1.2  # Meters, final altitude above figure for landing
TARGET_UP_ALTITUDE = 0.5  # Meters, how high the drone will climb if it doesn't find the package
TARGET_DOWN_ALTITUDE = 0.5  # Meters, distancia de descida para para centralizar
DETECTIONS_LOST_TOLERANCE = 5 # Number of missed detections to consider it as lost
PACKAGE_PROPORTION_ALIGN = 2  # How many times the smaller side fits into the larger one to consider it aligned

# Timeouts
SEARCH_TIMEOUT = 180  # seconds
CENTER_TIMEOUT = 180  # seconds
ALIGN_TIMEOUT = 180  # seconds
REACQUIRE_TIMEOUT = 180  # seconds

# Package Info
STARTING_PACKAGE_IDX = 0
PACKAGE_POSITIONS = [{}]
DELIVER_POSITIONS = [{}]

# YOLO detection parameters
YOLO_MODEL_PATH = os.path.join(
    get_package_share_directory("delivery"), "models", "best.pt"
)
YOLO_CONFIDENCE_THRESHOLD = 0.2  
YOLO_IMAGE_SIZE = 640 
DETECTION_SAVE_PATH = "detections/"  # save detection images

# Camera
IMAGE_SOURCE = "webcam"
IMAGE_CALCULUS_OFFSET_X = 0.1,
CAMERA_PIXELS_PER_DEGREE = 20,
CAMERA_RESOLUTION_WIDTH = 1280,
CAMERA_RESOLUTION_HEIGHT = 720,
IMAGE_CENTER_X = CAMERA_RESOLUTION_WIDTH // 2
IMAGE_CENTER_Y = CAMERA_RESOLUTION_HEIGHT // 2

# Position control parameters
POSITION_CONTROLLER_KP_XY = 0.6  # Proportional gain for XY movement
POSITION_CONTROLLER_KP_Z = 0.6  # Proportional gain for Z movement
POSITION_CONTROLLER_KP_YAW = 0.5  # Proportional gain for yaw
POSITION_CONTROLLER_MAX_VELOCITY_XY = 0.8
POSITION_CONTROLLER_MAX_VELOCITY_Z = 0.5
POSITION_CONTROLLER_MAX_VELOCITY_YAW = 0.3  # rad/s
POSITION_CONTROLLER_TOLERANCE_XY = 0.1  # Meters, XY tolerance for considering the PID at the target 
POSITION_CONTROLLER_TOLERANCE_Z = 0.1  # Meters, Z tolerance for considering the PID at the target
POSITION_CONTROLLER_TOLERANCE_YAW = 0.05  # Rad, Yaw tolerance for considering the PID at the target    # I guess never used because use pkg proportion on AlignPkg

# Servo motor value
SERVO_PIN_OUT = 1
SERVO_PICK_PWM = 1
SERVO_DROP_PWM = 1
