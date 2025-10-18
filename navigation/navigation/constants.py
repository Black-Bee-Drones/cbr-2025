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

SEARCH_POSITIONS = [
                {"name": "Position 1", "x": -200, "y": 150, "z": 0},  # 200 LEFT, 150 FORWARD
                {"name": "Position 2", "x": -300, "y": 0, "z": 0},    # 300 LEFT
                {"name": "Position 3", "x": 0, "y": 200, "z": 0},     # 200 FORWARD
                {"name": "Position 4", "x": 300, "y": 0, "z": 0}      # 300 RIGHT
            ]