import rclpy
import time 

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode
from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.utils.process import ProcessUtils
from std_msgs.msg import Int16 

from interaction.constants import (
    TAKEOFF_HEIGHT,
    TAKEOFF_TIMEOUT,
    ALTITUDE_TOLERANCE,
    GESTURE_CONTROLLER_PROCESS,
    GESTURE_RECOGNIZER_PROCESS,
    RTL_COUNT_TOPIC,
    RTL_REQUIRED_COUNT
)

class Initialize(State):
    """Initializes the drone connection and checks system status."""

    def __init__(self, outcomes):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        
        try:
            blackboard["mavdrone"] = MavDrone(node=YasminNode.get_instance())
            mavdrone : MavDrone = blackboard["mavdrone"]


            # Store takeoff position for RTL
            mavdrone.delay(0.1) # Callback processing delay
            takeoff_position = mavdrone.get_position_as_target
            blackboard["takeoff_position"] = takeoff_position

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
        yasmin.YASMIN_LOG_INFO(f"Taking off to altitude: {TAKEOFF_HEIGHT}m...")

        try:
            mavdrone.arm_takeoff(TAKEOFF_HEIGHT)

            start_time = time.time()
            while time.time() - start_time < TAKEOFF_TIMEOUT:
                rclpy.spin_once(self.node, timeout_sec=0.1)

                current_alt = mavdrone.get_rng_alt.data
                yasmin.YASMIN_LOG_INFO(f"Current altitude: {current_alt:.2f}m")

                altitude_error = TAKEOFF_HEIGHT - current_alt

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
            yasmin.YASMIN_LOG_INFO("Navigating towards human position...")
            mavdrone.offboard_position(
                x=3.0,
                y=-4.0,
                z=0.0,
                ground_reference=False,
                precision_radius=0.15,
                timeout=30,
                strategy="default"
                )
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
    """ Monitors the 'rtl_land_counter' and triggers transition to RTL when the count reaches the limit (6). """

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()
        self.land_count_sub = None

        self.land_count_sub = self.node.create_subscription(Int16, RTL_COUNT_TOPIC, self._count_callback, 10)
        yasmin.YASMIN_LOG_INFO(f"RTL Count Subscriber configured on topic: {RTL_COUNT_TOPIC}")

    def _count_callback(self, msg: Int16):
        """Updates the 'rtl_land_counter' variable in the blackboard."""
        
        blackboard = self.node.get_blackboard()
        current_count = blackboard.get("rtl_land_counter", 0)
        
        # O Recognizer publica '1' por trigger, então somamos o valor da msg
        new_count = current_count + msg.data 
        blackboard["rtl_land_counter"] = new_count

        yasmin.YASMIN_LOG_INFO(f"RTL Count: {new_count}/{RTL_REQUIRED_COUNT}")

    def execute(self, blackboard: Blackboard):
        mavdrone: MavDrone = blackboard["mavdrone"]

        while rclpy.ok():
            current_count = blackboard.get("rtl_land_counter", 0)

            if current_count >= RTL_REQUIRED_COUNT:
                yasmin.YASMIN_LOG_INFO("RTL count reached 6. Preparing for next state.")
                
                if self.land_count_sub:
                    self.node.destroy_subscription(self.land_count_sub)
                    self.land_count_sub = None

                if not ProcessUtils.kill_process(GESTURE_CONTROLLER_PROCESS):
                    yasmin.YASMIN_LOG_ERROR("Failed to kill Gesture Controller process.")
                    return ABORT
                if not ProcessUtils.kill_process(GESTURE_RECOGNIZER_PROCESS):
                    yasmin.YASMIN_LOG_ERROR("Failed to kill Gesture Recognizer process.")
                    return ABORT
                
                return SUCCEED

            rclpy.spin_once(self.node, timeout_sec=0.1)

        if self.land_count_sub:
            self.node.destroy_subscription(self.land_count_sub)
            self.land_count_sub = None
            
        return ABORT

class ReturnToLaunch(State):
    """Returns the drone to the takeoff position using local coordinates and lands."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in ReturnToLaunch state.")
            return ABORT

        mavdrone: MavDrone = blackboard["mavdrone"]

        while mavdrone.get_state.armed:
            yasmin.YASMIN_LOG_INFO("Drone is still armed, waiting for finishing landing...")
            mavdrone.delay(1)

        mavdrone.arm_takeoff(TAKEOFF_HEIGHT)

        # Set initial position as takeoff position for RTL
        takeoff_position = blackboard.get("takeoff_position")
        mavdrone.set_takeoff_position(takeoff_position)
        mavdrone.rtl(
            rtl_alt=None,
            precision_radius=0.3,
            rtl_strategy="default",
            land=True
        )

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