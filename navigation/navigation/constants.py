# Mission constants

TAKEOFF_HEIGHT = 50

WAYPOINT_DISTANCE = 100

COMMAND_SLEEP = 2

RED = "\033[91m"
YELLOW = "\033[93m"
GREEN = "\033[92m"
RESET = "\033[0m"

CONNECTION_TIMEOUT = 30

YOLO_CONFIDENCE = 0.6

HIGH_WINDOW = 1.2
LOW_WINDOW = 0.4
NULL_WINDOW = -1


WAYPOINTS_HEIGHT_YAW = [
                {"height": LOW_WINDOW, "yaw": -90}, 
                {"height": NULL_WINDOW, "yaw": 0}, 
                {"height": NULL_WINDOW, "yaw": 90}, 
                {"height": NULL_WINDOW, "yaw": 0}, 
                {"height": LOW_WINDOW, "yaw": -90}, 
                {"height": HIGH_WINDOW, "yaw": 0}, 
                {"height": LOW_WINDOW, "yaw": 90}, 
                {"height": HIGH_WINDOW, "yaw": 0}, 
                {"height": NULL_WINDOW, "yaw": -90}, 
                {"height": LOW_WINDOW, "yaw": 0}, 
                {"height": NULL_WINDOW, "yaw": 90}, 
                {"height": HIGH_WINDOW, "yaw": 90}, 
            ]