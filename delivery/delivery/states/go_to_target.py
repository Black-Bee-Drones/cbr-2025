import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.control.mavros.mavros_api import MavDrone

from delivery.constants import (
    SEARCH_TIMEOUT,
)


class GoToTarget(State):
    """
    State that sends the drone to the current target position (base or package).

    Outcome of the state:
        - SUCCEED: Drone successfully reached the target position.
        - ABORT: Navigation failed or required data is missing.
    """

    def __init__(self, desired_class):
        '''
        Args:
            desired_class: Class ID to filter detections ("base" or "package")

        Raises:
            TypeError: If desired_class is not "base" or "package".
        '''
        super().__init__(outcomes=[SUCCEED, ABORT])

        self._desired_class = desired_class.lower()
        if self._desired_class not in ("base", "package"):
            raise TypeError("Parameter desired_class should be 'base' or 'package'.")

    def execute(self, blackboard : Blackboard):
        if ("mavdrone" not in blackboard) or not blackboard["mavdrone"]:
            yasmin.YASMIN_LOG_ERROR(f"Mavdrone not available in {self.__class__.__name__} state.")
            return ABORT
        mavdrone: MavDrone = blackboard["mavdrone"]

        if self._desired_class == "base":
            all_positions = blackboard["deliver_positions"]
            
        elif self._desired_class == "package":
            all_positions = blackboard["packages_positions"]
            blackboard["next_package"] += 1

        if not all_positions:
            yasmin.YASMIN_LOG_ERROR("Target positions not available.")
            return ABORT

        package_id = blackboard["next_package"]
        if package_id >= len(all_positions):
            yasmin.YASMIN_LOG_INFO("Finish package delivery.")
            return ABORT

        target_position = all_positions[package_id]

        yasmin.YASMIN_LOG_INFO(f"Next target position: {target_position}.")

        try:
            mavdrone.offboard_position(
                x=target_position["y"],
                y=target_position["x"],
                z=0.0,
                timeout_sec=SEARCH_TIMEOUT,
            )
            yasmin.YASMIN_LOG_INFO("Target point reached successfully.")
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Navigation failed: {e}")
            return ABORT
