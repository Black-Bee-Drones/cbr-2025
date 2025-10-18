import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

import time

from navigation.constants import (
    WAYPOINT_DISTANCE,
    GREEN, RED, RESET,
    COMMAND_SLEEP
)

class Navigate(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "FINAL_SUCCEED"])

    def execute(self, blackboard: Blackboard):
        try:
            tello = blackboard["tello"]
            
            tello.move("forward", WAYPOINT_DISTANCE)
            blackboard["current_waypoint"] += 1
            waypoint = blackboard["current_waypoint"]
            if waypoint > 12:
                return "FINAL_SUCCEED"
            yasmin.YASMIN_LOG_INFO(f"{GREEN}Current waypoint: {waypoint}{RESET}")
            time.sleep(COMMAND_SLEEP)
            if waypoint == 1 and tello.get_height() < 70:
                tello.move("up", 120 - tello.get_height())
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"{RED}Tello could not handle movement. {e}{RESET}")
            return ABORT

        
