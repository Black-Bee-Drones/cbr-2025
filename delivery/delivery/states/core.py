import yasmin
from yasmin import Blackboard
from yasmin import State
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

import time

from mirela_sdk.control.mavros import MavDrone

from delivery.constants import (
    TAKEOFF_ALTITUDE
)

class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard : Blackboard):
        try:
            yasmin.YASMIN_LOG_INFO("Initializing mission...")
            blackboard["drone"] = MavDrone(node=YasminNode.get_instance())
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