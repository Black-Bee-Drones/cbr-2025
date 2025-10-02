import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.control.mavros.mavros_api import MavDrone

from delivery.constants import (
    TAKEOFF_ALTITUDE,
    SEARCH_TIMEOUT,
)

#current package pode ser atualizado aqui
class GoToTarget(State):
    """
    Sends drone to current target (base or package) location.
    """

    def __init__(self, desired_class):
        '''
        Args:
            desired_class: Class ID to filter detections ("base" or "package")
        '''
        super().__init__(outcomes = [SUCCEED, ABORT])
        self._desired_class = desired_class.lower()
        if self._desired_class not in ("base" or "package"):
            raise TypeError("Parameter desired_class should be 'base' or 'package'.")

    def execute(self, blackboard : Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR(
                "MavDrone not available in GoToTarget state."
            )
            return ABORT

        mavdrone: MavDrone = blackboard["mavdrone"]

        #Checks if target is package or base
        if self._desired_class == "base":
            target_positions = blackboard.get("deliver_positions")
        elif self._desired_class == "package":
            target_positions = blackboard.get("packages_positions")

        if not target_positions:
            yasmin.YASMIN_LOG_ERROR(f"Target positions not available.")
            return ABORT

        current_target = blackboard["current_package"]
        current_target_pos = target_positions[current_target]

        yasmin.YASMIN_LOG_INFO(f"Next target position: {current_target_pos}.")

        try:
            mavdrone.offboard_position(
                x=current_target_pos["y"], 
                y=current_target_pos["x"], 
                z=0.0, 
                timeout_sec=SEARCH_TIMEOUT, 
                ground_reference=True
            )

            yasmin.YASMIN_LOG_INFO("Target point reached successfully.")
            return SUCCEED

            
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Navigation failed: {e}")
            return ABORT
