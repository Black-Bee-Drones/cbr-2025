import yasmin
import time

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from mirela_sdk.control.mavros.mavros_api import MavDrone

from delivery.constants import (
    TAKEOFF_ALTITUDE
)

class MarkBaseAndTakeoff(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "next_pkg"])

    def execute(self, blackboard : Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in MarkBaseAndTakeoff state.")
            return ABORT

        mavdrone : MavDrone = blackboard["mavdrone"]

        yasmin.YASMIN_LOG_INFO("Marking base as visited...")

        try:

            landing_position = {
                "x": mavdrone.get_local_pos.pose.position.x,
                "y": mavdrone.get_local_pos.pose.position.y,
                "z": mavdrone.get_local_pos.pose.position.z,
                "timestamp": time.time(),
            }
            visited_bases = blackboard.get("visited_bases", [])
            visited_bases.append(landing_position)
            blackboard["visited_bases"] = visited_bases

            
            yasmin.YASMIN_LOG_INFO(f"Total bases visited: {len(visited_bases)}/3")

            
            yasmin.YASMIN_LOG_INFO(f"Taking off to {TAKEOFF_ALTITUDE} meters...")
            mavdrone.takeoff(TAKEOFF_ALTITUDE)
            time.sleep(5)
            yasmin.YASMIN_LOG_INFO("Takeoff successful.")

            # Updates current package index
            current_package = blackboard.get("current_package")
            if current_package < 2:
                blackboard["current_package"] = current_package + 1
                return "next_pkg"

            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Takeoff failed: {e}")
            return ABORT
