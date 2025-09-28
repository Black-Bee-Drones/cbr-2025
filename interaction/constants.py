from ament_index_python.packages import get_package_share_directory
import os

TAKEOFF_ALTITUDE = 1.5
RTL_ALTITUDE = 1.5
SEARCH_DISTANCE = 4.0

# Timeouts
SEARCH_TIMEOUT = 300  # seconds - 5 minutes search timeout per attempt
LANDING_TIMEOUT = 30  # seconds - landing timeout
TAKEOFF_TIMEOUT = 20  # seconds - takeoff timeout

# Safety parameters
MIN_SAFE_ALTITUDE = 1.0
MAX_SAFE_ALTITUDE = 4.0
WALL_CLEARANCE = 0.5

#Gestures/Human Following Parameters 
GESTURE_CONTROLLER_PROCESS = "gesture_ctrl_process"
GESTURE_RECOGNIZER_PROCESS = "gesture_recog_process"
INTERACTION_PKG_NAME = "interaction"
GESTURE_CONTROLLER_NODE = "mav_gesture_controller" 
GESTURE_RECOGNIZER_NODE = "mav_gesture_recognizer"

# Movement tolerances
VELOCITY_TOLERANCE = 0.1  # m/s - velocity considered as stopped
ALTITUDE_TOLERANCE = 0.1  # meters - altitude precision tolerance
POSITION_TOLERANCE = 0.3  # meters - position precision tolerance