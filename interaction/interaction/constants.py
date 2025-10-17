from ament_index_python.packages import get_package_share_directory
import os

# Controller constants
TAKEOFF_POSE = 1.0
TAKEOFF_HEIGHT = 2.0
TAKEOFF_TIMEOUT = 10.0
ALTITUDE_TOLERANCE = 0.1
VELOCITY_UP_DOWN = 0.2
VELOCITY_SIDES = 0.2
VELOCITY_IN_OUT = -0.2
VELOCITY_YAW = 0.1

ACTION_TIMEOUT = 0.05
SLEEP_AFTER_TAKEOFF = 2.0
SLEEP_AFTER_LAND = 1.0

#Gestures/Human Following Parameters 
GESTURE_CONTROLLER_PROCESS = "gesture_ctrl_process"
GESTURE_RECOGNIZER_PROCESS = "gesture_recog_process"
INTERACTION_PKG_NAME = "interaction"
GESTURE_CONTROLLER_NODE = "mav_gesture_controller" 
GESTURE_RECOGNIZER_NODE = "mav_gesture_recognizer"

# GESTURE TRIGGER / RTL COUNTING 
RTL_COUNT_TOPIC = "/drone/land_count_trigger"
LAND_GESTURE_ID = 1
RTL_REQUIRED_COUNT = 6