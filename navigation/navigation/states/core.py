import rclpy

import time

import yasmin
from yasmin import Blackboard
from yasmin import State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

import djitellopy

from navigation.constants import (
    # Log formatting constants
    RED, GREEN, YELLOW, RESET,
    # Timeout settings
    CONNECTION_TIMEOUT,
    TAKEOFF_HEIGHT,
    COMMAND_SLEEP
)

class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        try:
            tello = djitellopy.Tello()
            blackboard["tello"] = tello

            blackboard["current_waypoint"] = 0
            
            yasmin.YASMIN_LOG_INFO(f"{YELLOW}Waiting for Tello to be fully ready...{RESET}")

            tello.connect(wait_for_state=True) 

            max_attempts = CONNECTION_TIMEOUT
            for attempt in range(max_attempts):
                try:
                    battery = tello.get_battery()
                    if battery > 0: 
                        yasmin.YASMIN_LOG_INFO(f"{GREEN}Tello is ready! Battery: {battery}%{RESET}")
                        break
                except:
                    time.sleep(1)
                    yasmin.YASMIN_LOG_INFO(f"{YELLOW}Waiting... ({attempt+1}/{max_attempts}){RESET}")
            else:
                raise Exception("Tello did not respond within timeout period")
            
            tello.streamon() 
            input(f"{YELLOW}Tello connected. Press Enter to continue when ready...{RESET}")

            
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
            tello = blackboard["tello"]
            tello.takeoff() 
            time.sleep(COMMAND_SLEEP)
            yasmin.YASMIN_LOG_INFO(f"{GREEN}Tello TAKE OFF was successful.{RESET}")
            current_height = tello.get_height()
            tello.move(direction="down", x = abs(TAKEOFF_HEIGHT-current_height))
            yasmin.YASMIN_LOG_INFO(f"{YELLOW}Adjusted to desired take off attitude.{RESET}")
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

