import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT


from delivery.constants import (
    TAKEOFF_ALTITUDE,
    SEARCH_TIMEOUT,
)


class GoToTarget(State):
    """
    Controller: Mandar drone para coordenada na base de entrega baseada no index na blackboard.
    Detector: Node de Yolo para reconhecer o pacote
    """

    def __init__(self):
        super().__init__(outcomes = [SUCCEED, ABORT])

    def execute(self, blackboard : Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR(
                "MavDrone not available in NavigateToWaypoint state."
            )
            return ABORT

        # mavdrone: MavDrone = blackboard.get("mavdrone")
        
        packages_positions = blackboard.get("packages_positions")
        if not packages_positions:
            yasmin.YASMIN_LOG_ERROR(f"Next package position not avaible.")
            return ABORT

        yasmin.YASMIN_LOG_INFO(f"Next package position: {packages_positions}.")
        
        position_controller : PositionController = blackboard.get("position_controller")
        if not position_controller:
            yasmin.YASMIN_LOG_ERROR("Position controller not avaible.")
            return ABORT

        try:
            idx = blackboard.get("current_package")
            success = position_controller.goto_position_ground_relative(
                packages_positions[idx]["x"],
                packages_positions[idx]["y"],
                TAKEOFF_ALTITUDE,
                blackboard.get("ground_reference_altitude", 0.0),
                timeout = SEARCH_TIMEOUT
            )

            if success:
                yasmin.YASMIN_LOG_INFO("Package point reached successfully.")
                return SUCCEED
            else:
                yasmin.YASMIN_LOG_ERROR("Failed to reach package point.")
                return ABORT
            
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Navigation failed: {e}")
            return ABORT
