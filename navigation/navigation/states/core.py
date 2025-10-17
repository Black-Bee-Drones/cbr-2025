import rclpy

import yasmin
from yasmin import Blackboard
from yasmin import State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from navigation.constants import (
    # Log formatting constants
    RED, GREEN, YELLOW, RESET,
)

class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        try:
            ...
            yasmin.YASMIN_LOG_INFO(f"{GREEN}Phase 4 initialized.{RESET}")
            return SUCCEED
    
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"{RED}Phase 4 could not be initialized. {e}{RESET}")
            return ABORT


class Takeoff(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        try:
            ...
            yasmin.YASMIN_LOG_INFO(f"{GREEN}Tello TAKE OFF was successful.{RESET}")
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"{RED}Tello could not takeoff. {e}{RESET}")
            return ABORT


class Land(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        try:
            yasmin.YASMIN_LOG_INFO(f"{GREEN}Tello LAND was successful.{RESET}")            
            ...
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"{RED}Tello could not land. {e}{RESET}")
            yasmin.YASMIN_LOG_WARN(f"{YELLOW}Human intervention is needed.{RESET}")
            return ABORT

