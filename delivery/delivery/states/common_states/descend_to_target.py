import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.control.mavros.mavros_api import MavDrone

from constants import (
    CENTERING_ALTITUDE,
    TAKEOFF_ALTITUDE,
)


class DescendToTarget(State):
    """
    Descer um pouco e realinhar o drone até chegar em uma boa altura para dar land
    """
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "height_limit"])

    def execute(self, blackboard: Blackboard):

        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR(
                "Mavdrone not available in DescendPkg state."
            )
            return ABORT
        
        mavdrone : MavDrone = blackboard["mavdrone"]

        try:
            current_altitude = mavdrone.get_rng_alt.range

            if current_altitude <= CENTERING_ALTITUDE:
                yasmin.YASMIN_LOG_INFO("Height limit achieved. Descending finished.")
                return "height_limit"
            
            max_distance = TAKEOFF_ALTITUDE - CENTERING_ALTITUDE
            kp = (current_altitude - CENTERING_ALTITUDE) / max_distance 
            descending_distance = 0.60 * kp 
            mavdrone.offboard_position(0.0, 0.0, descending_distance)
            
            yasmin.YASMIN_LOG_INFO(f"Descending: {descending_distance} m")

            return SUCCEED

        
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"DescendPkg failed: {e}")