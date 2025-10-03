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

            mavdrone.delay(0.1) # Process callbacks
            blackboard["initial_position"] = mavdrone.get_position_as_target

            blackboard["current_package"] = STARTING_PACKAGE_IDX
            blackboard["packages_positions"] = PACKAGE_POSITIONS
            blackboard["deliver_positions"] = DELIVER_POSITIONS
            blackboard["visited_bases"] = []

            blackboard["current_detection"] = None

            #Flag to define if next target is a package or a base
            blackboard["target_is_package"] = True  # I guess it's depreciated, because it's set in Class.__init__

            if not blackboard["packages_positions"]:
                yasmin.YASMIN_LOG_WARN("Package positions not declared.")
            if not blackboard["deliver_positions"]:
                yasmin.YASMIN_LOG_WARN("Deliver positions not declared.")

            blackboard["image_handler"] = ImageHandler(
                node=YasminNode.get_instance(), 
                image_source=IMAGE_SOURCE,
                image_processing_callback=lambda frame: frame
            )

            blackboard["yolo_detector"] = YOLODetector()

            yasmin.YASMIN_LOG_INFO("Take photo for first Yolo detection")
            frame = blackboard["image_handler"].take_photo()
            yasmin.YASMIN_LOG_INFO("Run first Yolo detection...")
            blackboard["yolo_detector"].detect(frame)

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
            # Adjust initial position for correct RTL and Ground Reference
            mavdrone.set_takeoff_position(blackboard["initial_position"])
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
        mavdrone: MavDrone = blackboard.get("mavdrone")
        if not mavdrone:
            yasmin.YASMIN_LOG_ERROR("Mavdrone not available in DescendToTarget state.")
            return ABORT

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
            self.mavdrone.set_takeoff_position(blackboard["initial_position"])
            self.mavdrone.rtl(
                rtl_alt=TAKEOFF_ALTITUDE,
                precision_radius=0.25,
                rtl_strategy="PID",
                land=True
            )
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