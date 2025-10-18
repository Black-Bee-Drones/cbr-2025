import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

import cv2
import os
import time

from navigation.constants import (
    COMMAND_SLEEP, RED, RESET, GREEN, RESET, YELLOW
)

class Photoshoot(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
    def execute(self, blackboard: Blackboard):
        try:
            tello = blackboard["tello"]

            is_fast_forwarding = blackboard["is_fast_forwarding"]

            if is_fast_forwarding: 
                return SUCCEED

            for i in range (0,3):
                time.sleep(1) 
                read = tello.get_frame_read()
                current_waypoint = blackboard["current_waypoint"]
                cv2.imwrite(f"Waypoint_{current_waypoint}_{i}.png",read.frame)

                tello.rotate_counter_clockwise(90)

                height_error = 100 - tello.get_height()                     
                while abs(height_error) >= 10:
                    tello.send_rc_control(
                            left_right_velocity = 0, 
                            forward_backward_velocity = 0, 
                            up_down_velocity = 10 if height_error > 0 else -10,
                            yaw_velocity = 0                                            
                        )
                    time.sleep(tello.TIME_BTW_RC_CONTROL_COMMANDS)
                    height_error = 100 - tello.get_height()                     

            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"{RED}Tello could not complete photoshoot. {e}{RESET}")
            return ABORT

        
