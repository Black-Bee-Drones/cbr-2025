import rclpy
import time

import yasmin
from yasmin import Blackboard
from yasmin import State
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

import time

from mirela_sdk.control.mavros import MavDrone
#from mirela_sdk.image_processing import ImageHandler
from mirela_sdk.image_processing.camera import ImageHandler

from delivery.utils import YOLODetector

from delivery.constants import (
    TAKEOFF_ALTITUDE,
    STARTING_PACKAGE_IDX,
    PACKAGE_POSITIONS,
    DELIVER_POSITIONS,
    IMAGE_SOURCE
)

class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard : Blackboard):
        try:
            yasmin.YASMIN_LOG_INFO("Initializing mission...")
            blackboard["mavdrone"] = MavDrone(
                node=YasminNode.get_instance(),
                mavros=False,
                indoor=True
                )
            mavdrone: MavDrone = blackboard["mavdrone"]


            rclpy.spin_once(YasminNode.get_instance(), timeout_sec=0.5)
            initial_position = (
                mavdrone.get_local_pos.pose.position.x,
                mavdrone.get_local_pos.pose.position.y,
                mavdrone.get_local_pos.pose.position.z,
            )
            blackboard["initial_position"] = initial_position
            
            # Vai dar problema porque não armamos ainda
            ground_altitude = mavdrone.get_rng_alt.range
            blackboard["ground_reference_altitude"] = ground_altitude

            yasmin.YASMIN_LOG_INFO(
                f"Initial drone position: ({initial_position[0]:.2f}, {initial_position[1]:.2f}, {initial_position[2]:.2f})"
            )
            yasmin.YASMIN_LOG_INFO(
                f"Ground reference altitude: {ground_altitude:.2f}m, Target search altitude: {ground_altitude + TAKEOFF_ALTITUDE:.2f}m"
            )

            blackboard["current_package"] = STARTING_PACKAGE_IDX
            blackboard["packages_positions"] = PACKAGE_POSITIONS
            blackboard["deliver_positions"] = DELIVER_POSITIONS
            blackboard["visited_bases"] = []

            blackboard["current_detection"] = None

            #Flag to define if next target is a package or a base
            blackboard["target_is_package"] = True

            if not blackboard["packages_positions"]:
                yasmin.YASMIN_LOG_WARN("Package positions not declared.")
            if not blackboard["deliver_positions"]:
                yasmin.YASMIN_LOG_WARN("Deliver positions not declared.")

            yolo_detector = YOLODetector()
            blackboard["yolo_detector"] = yolo_detector

            image_handler = ImageHandler(
                node=YasminNode.get_instance(), image_source=IMAGE_SOURCE
                )
            blackboard["image_handler"] = image_handler

            yasmin.YASMIN_LOG_INFO("Mission successfully initialized.")
            return SUCCEED
        
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Failed to initialize: {e}")
            return ABORT

class Takeoff(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard : Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Mavdrone not initialized.")
            return ABORT

        mavdrone: MavDrone = blackboard["mavdrone"]
        yasmin.YASMIN_LOG_INFO(f"Taking off to {TAKEOFF_ALTITUDE} meters...")
        try:
            mavdrone.arm_takeoff(TAKEOFF_ALTITUDE)
            time.sleep(5)
            yasmin.YASMIN_LOG_INFO("Takeoff successful.")
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Takeoff failed: {e}")
            return ABORT
        
class Land(State):
    def __init__(self, ):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.mavdrone : MavDrone = None

    def execute(self, blackboard : Blackboard):
        
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Mandrone not initialized.")
            return ABORT
    
        self.mavdrone = blackboard["mavdrone"]

        try:
            self.mavdrone.land()
            time.sleep(5) #wait for landing to complete
            yasmin.YASMIN_LOG_INFO("Landed successfully.")
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Landing failed: {e}")
            return ABORT
            
        
class ReturnToLaunch(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.mavdrone : MavDrone = None

    def execute(self, blackboard : Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Mavdrone not initialized.")
            return ABORT
        
        self.mavdrone: MavDrone = blackboard["mavdrone"]

        yasmin.YASMIN_LOG_INFO("Returning to launch...")
        try:
            self.mavdrone.rtl(rtl_alt=TAKEOFF_ALTITUDE, rtl_strategy="gps_return")
            yasmin.YASMIN_LOG_INFO("Return to launch initiated.")
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Return to launch failed: {e}")
            return ABORT
        
class End(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED])
        self.mavdrone : MavDrone = None

    def execute(self, blackboard : Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Mavdrone not initialized.")
            return SUCCEED

        self.mavdrone: MavDrone = blackboard["mavdrone"]

        if self.mavdrone and self.mavdrone.get_state.armed:
            yasmin.YASMIN_LOG_INFO("Drone is armed, attempting to land...")
            try:
                self.mavdrone.land()
                yasmin.YASMIN_LOG_INFO("Drone landed successfully.")
            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(f"Landing failed: {e}")

        return SUCCEED