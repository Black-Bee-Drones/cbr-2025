import rclpy
import time 

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode
from mirela_sdk import position_controller
from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.utils.process import ProcessUtils

from interaction.constants import (
    TAKEOFF_ALTITUDE,
    TAKEOFF_TIMEOUT,
    ALTITUDE_TOLERANCE,
    GESTURE_CONTROLLER_PROCESS,
    GESTURE_RECOGNIZER_PROCESS,
)

class Initialize(State):
    """Initializes the drone connection and checks system status."""

    def __init__(self, outcomes):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        
        try:
            blackboard["mavdrone"] = MavDrone(node=YasminNode.get_instance())
            mavdrone : MavDrone = blackboard["mavdrone"]

            rclpy.spin_once(YasminNode.get_instance(), timeout_sec=0.5)
            initial_position = (
                mavdrone.get_local_pos.pose.position.x,
                mavdrone.get_local_pos.pose.position.y,
                mavdrone.get_local_pos.pose.position.z,
            )
            blackboard["initial_position"] = initial_position

            ground_altitude = mavdrone.get_rng_alt.data
            blackboard["ground_reference_altitude"] = ground_altitude

            blackboard["rtl_land_counter"] = 0

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Error at Initialize: {e}")


class Takeoff(State):
    """Arms the drone and takes off to search altitude."""

    def __init__(self, outcomes):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):

        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in Takeoff state.")
            return ABORT
        

        mavdrone: MavDrone = blackboard["mavdrone"]
        yasmin.YASMIN_LOG_INFO(f"Taking off to altitude: {TAKEOFF_ALTITUDE}m...")


        # Store takeoff position for RTL
        takeoff_position = {
            "local_x": mavdrone.get_local_pos.pose.position.x,
            "local_y": mavdrone.get_local_pos.pose.position.y,
            "local_z": mavdrone.get_local_pos.pose.position.z,
        }
        blackboard["takeoff_position"] = takeoff_position

        try:
            mavdrone.arm_takeoff(TAKEOFF_ALTITUDE)

            time.sleep(3)

            start_time = time.time()
            while time.time() - start_time < TAKEOFF_TIMEOUT:
                rclpy.spin_once(self.node, timeout_sec=0.1)

                current_alt = mavdrone.get_rng_alt.data
                yasmin.YASMIN_LOG_INFO(f"Current altitude: {current_alt:.2f}m")

                altitude_error = TAKEOFF_ALTITUDE - current_alt

                if abs(altitude_error) < ALTITUDE_TOLERANCE:
                    yasmin.YASMIN_LOG_INFO(
                        "Takeoff altitude reached. Ready to start search pattern."
                    )

                    mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                    time.sleep(2)

                    return SUCCEED

                correction_velocity = max(-0.5, min(0.5, 0.3 * altitude_error))
                mavdrone.offboard_velocity(0.0, 0.0, correction_velocity, 0.0)

                time.sleep(0.1)

            yasmin.YASMIN_LOG_ERROR("Takeoff timeout reached.")
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Takeoff failed: {e}")
            return ABORT
        

class FindHuman(State):
    """Starting the movement to find human."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])


    def execute(self, blackboard: Blackboard):

        mavdrone: MavDrone = blackboard["mavdrone"]
        
        try:
            print()
            mavdrone.offboard_position(3.0, -4.0, 0.0, 0.0)
            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Find Human failed: {e}")
            return ABORT


class StartGesture(State):
    """Starting Human Follow mode. Activating gesture control systems."""
    
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])


    def execute(self, blackboard: Blackboard):
        mavdrone: MavDrone = blackboard["mavdrone"]
        

        ProcessUtils.kill_process(GESTURE_CONTROLLER_PROCESS)

        gesture_controller_cmd = (
            "ros2 run interaction mav_gesture_controller "
            "--ros-args "
        )

        if not ProcessUtils.start_process(gesture_controller_cmd, GESTURE_CONTROLLER_PROCESS):
            yasmin.YASMIN_LOG_ERROR("Failed to start control node.")
            return ABORT
        
        yasmin.YASMIN_LOG_INFO("Gesture Controller node started successfully")


        ProcessUtils.kill_process(GESTURE_RECOGNIZER_PROCESS)

        gesture_recognizer_cmd = (
            "ros2 run interaction mav_gesture_controller "
            "--ros-args "
        )

        if not ProcessUtils.start_process(gesture_recognizer_cmd, GESTURE_RECOGNIZER_PROCESS):
            yasmin.YASMIN_LOG_ERROR("Failed to start recognizer node.")

            yasmin.YASMIN_LOG_INFO("Killing Controller due to Recognizer failure.")
            ProcessUtils.kill_process(GESTURE_CONTROLLER_PROCESS)

            return ABORT
        
        yasmin.YASMIN_LOG_INFO("Gesture Recognizer node started successfully")

        return SUCCEED



class CheckCount(State):
    pass

class ReturnToLaunch(State):
    """Returns the drone to the takeoff position using local coordinates and lands."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in ReturnToLaunch state.")
            return ABORT

        mavdrone: MavDrone = blackboard["mavdrone"]
        position_controller = blackboard.get("position_controller")
        takeoff_position = blackboard.get("takeoff_position")

        if not position_controller:
            yasmin.YASMIN_LOG_ERROR("Position controller not available.")
            return ABORT

        if not takeoff_position:
            yasmin.YASMIN_LOG_ERROR("Takeoff position not stored.")
            return ABORT
        
        yasmin.YASMIN_LOG_INFO("Returning to takeoff base using local coordinates...")


class End(State):
    """Finalizes the mission and performs cleanup."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Phase 1 Mission completed. Cleanup...")

        if "mavdrone" in blackboard:
            mavdrone: MavDrone = blackboard["mavdrone"]

            if mavdrone.get_state.armed:
                yasmin.YASMIN_LOG_INFO("Drone still armed, ensuring safe landing...")
                try:
                    mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                    mavdrone.land()
                except Exception as e:
                    yasmin.YASMIN_LOG_ERROR(f"Failed to land during cleanup: {e}")

        return SUCCEED