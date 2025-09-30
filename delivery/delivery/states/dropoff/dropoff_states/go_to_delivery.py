import yasmin

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from mirela_sdk.control.mavros.mavros_api import MavDrone

from delivery.constants import (
    TAKEOFF_ALTITUDE,
)

class GoToDelivery(State):
    """
    Controller: Mandar drone para coordenada na base de entrega baseada no index na blackboard.
    Detector: Node de Yolo para reconhecer a cruz
    """
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
    
    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR(
                "Mavdrone not available in GoToDelivery state."
            )
            return ABORT
        
        mavdrone: MavDrone = blackboard["mavdrone"]

        deliver_positions = blackboard.get("deliver_positions")

        packages_positions = blackboard.get("packages_positions")
        current_package = blackboard.get("current_package")
        current_position = packages_positions[current_package]

        current_target = deliver_positions[current_package]

        target_dx = current_target["x"] - current_position["x"]
        target_dy = -current_target["y"] + current_position["y"]

        mavdrone.offboard_position(target_dx, target_dy, TAKEOFF_ALTITUDE)
