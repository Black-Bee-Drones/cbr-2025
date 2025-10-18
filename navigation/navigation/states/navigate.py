import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

import time

from navigation.constants import (
    WAYPOINT_DISTANCE,
    GREEN, RED, RESET,
    COMMAND_SLEEP,
    WAYPOINTS_HEIGHT_YAW
)

class Navigate(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "FINAL_SUCCEED"])
        self.waypoints = WAYPOINTS_HEIGHT_YAW
    def execute(self, blackboard: Blackboard):
        try:
            tello = blackboard["tello"]

            blackboard["current_waypoint"] += 1
            current_waypoint = blackboard["current_waypoint"]

            current_height = tello.get_height()

            if current_waypoint == 1:
                if current_height <= 100:
                    tello.move("up", 120 - current_height)
                else:
                    while abs(120 - current_height) <= 20:
                        tello.send_rc_control(
                                                left_right_velocity = 0, 
                                                forward_backward_velocity = 0, 
                                                up_down_velocity = 10 if height_error > 0 else -10,
                                                yaw_velocity = 0
                                            )
                        current_height = tello.get_height()
                        time.sleep(tello.TIME_BTW_RC_CONTROL_COMMANDS)
                tello.move("forward", WAYPOINT_DISTANCE)
                time.sleep(COMMAND_SLEEP)
            elif current_waypoint <= 12:
                height_error = self.waypoints[current_waypoint]["height"] - tello.get_height()
                yaw_error = (self.waypoints[current_waypoint]["yaw"] - tello.get_yaw()) % 360
                
                while abs(height_error) <= 10:
                    tello.send_rc_control(
                                            left_right_velocity = 0, 
                                            forward_backward_velocity = 0, 
                                            up_down_velocity = 10 if height_error > 0 else -10,
                                            yaw_velocity = 0                                            
                                        )
                    time.sleep(tello.TIME_BTW_RC_CONTROL_COMMANDS)

                    height_error = self.waypoints[current_waypoint]["height"] - tello.get_height()


                tello.send_rc_control(0, 0, 0, 0)

                if yaw_error > 180:
                    tello.rotate_counter_clockwise(360 - yaw_error)
                else:
                    tello.rotate_clockwise(yaw_error)
                time.sleep(COMMAND_SLEEP)

                tello.move("forward", WAYPOINT_DISTANCE)

            if current_waypoint > 12:
                return "FINAL_SUCCEED"
            yasmin.YASMIN_LOG_INFO(f"{GREEN}Current waypoint: {current_waypoint}{RESET}")
            time.sleep(COMMAND_SLEEP)

            return SUCCEED
        
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"{RED}Tello could not handle movement. {e}{RESET}")
            return ABORT

        
