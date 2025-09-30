import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.control.mavros.mavros_api import MavDrone

from delivery.constants import (
    TAKEOFF_ALTITUDE,
    SEARCH_TIMEOUT,
)


class GoToTarget(State):
    """
    Sends drone to current target (base or package) location.
    """

    def __init__(self):
        super().__init__(outcomes = [SUCCEED, ABORT])

    def execute(self, blackboard : Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR(
                "MavDrone not available in GoToTarget state."
            )
            return ABORT

        mavdrone: MavDrone = blackboard["mavdrone"]

        target_is_package = blackboard.get("target_is_package")

        #Checks if target is package or base
        if target_is_package:
            target_positions = blackboard.get("packages_positions")
        else:
            target_positions = blackboard.get("deliver_positions")
        
        if not target_positions:
            yasmin.YASMIN_LOG_ERROR(f"Target positions not available.")
            return ABORT

        current_target = blackboard.get("current_package")
        current_target_pos = target_positions[current_target]

        yasmin.YASMIN_LOG_INFO(f"Next target position: {current_target_pos}.")

        try:

            #Alguma coisa nisso aqui vai mudar pra configurar que as posições enviadas
            #são em relação ao takeoff (modificação aguarda commit no mirela_sdk)
            mavdrone.offboard_position(
                x=current_target_pos["y"], y=current_target_pos["x"], 
                z=0.0, timeout_sec=SEARCH_TIMEOUT,
            )

            yasmin.YASMIN_LOG_INFO("Target point reached successfully.")
            return SUCCEED

            
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Navigation failed: {e}")
            return ABORT
