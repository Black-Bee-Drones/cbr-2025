import rclpy
import time

import yasmin
from yasmin import Blackboard
from yasmin import State
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

import time

from mirela_sdk.control.mavros import MavDrone

from delivery.utils import PositionController, YOLOPackageDetector, YOLODeliverDetector

from delivery.constants import (
    TAKEOFF_ALTITUDE,
    STARTING_PACKAGE_IDX,
    PACKAGE_POSITIONS,
    DELIVER_POSITIONS
)

class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard : Blackboard):
        try:
            yasmin.YASMIN_LOG_INFO("Initializing mission...")
            blackboard["mavdrone"] = MavDrone(node=YasminNode.get_instance())
            mavdrone: MavDrone = blackboard["mavdrone"]


            rclpy.spin_once(YasminNode.get_instance(), timeout_sec=0.5)
            initial_position = (
                mavdrone.get_local_pos.pose.position.x,
                mavdrone.get_local_pos.pose.position.y,
                mavdrone.get_local_pos.pose.position.z,
            )
            blackboard["initial_position"] = initial_position
            
            ground_altitude = mavdrone.get_rng_alt.data
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

            if not blackboard["packages_positions"]:
                yasmin.YASMIN_LOG_WARN("Package positions not declared.")
            if not blackboard["deliver_positions"]:
                yasmin.YASMIN_LOG_WARN("Deliver positions not declared.")

            yolo_pkg_detector = YOLOPackageDetector()
            blackboard["yolo_pkg_detector"] = yolo_pkg_detector

            yolo_deliver_detector = YOLODeliverDetector()
            blackboard["yolo_deliver_detector"] = yolo_deliver_detector

            position_controller = PositionController(mavdrone)

            blackboard["position_controller"] = position_controller

            yasmin.YASMIN_LOG_INFO("Mission successfully initialized.")
            return SUCCEED
        
        except Exception as e:
            import traceback
            print("Failed to initialize:", traceback.format_exc())
            raise

class Takeoff(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard : Blackboard):
        if "drone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Drone not initialized.")
            return ABORT

        drone: MavDrone = blackboard["drone"]
        yasmin.YASMIN_LOG_INFO(f"Taking off to {TAKEOFF_ALTITUDE} meters...")
        try:
            drone.arm_takeoff(TAKEOFF_ALTITUDE)
            time.sleep(5)
            yasmin.YASMIN_LOG_INFO("Takeoff successful.")
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Takeoff failed: {e}")
            return ABORT
        
class ReturnToLaunch(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.drone : MavDrone = None

    def execute(self, blackboard : Blackboard):
        if "drone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Drone not initialized.")
            return ABORT
        
        self.drone: MavDrone = blackboard["drone"]

        yasmin.YASMIN_LOG_INFO("Returning to launch...")
        try:
            self.drone.rtl(rtl_alt=TAKEOFF_ALTITUDE, rtl_strategy="gps_return")
            yasmin.YASMIN_LOG_INFO("Return to launch initiated.")
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Return to launch failed: {e}")
            return ABORT
        
class End(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED])
        self.drone : MavDrone = None

    def execute(self, blackboard : Blackboard):
        if "drone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Drone not initialized.")
            return SUCCEED

        self.drone: MavDrone = blackboard["drone"]

        if self.drone and self.drone.get_state.armed:
            yasmin.YASMIN_LOG_INFO("Drone is armed, attempting to land...")
            try:
                self.drone.land()
                yasmin.YASMIN_LOG_INFO("Drone landed successfully.")
            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(f"Landing failed: {e}")

        return SUCCEED