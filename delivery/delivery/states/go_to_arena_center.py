import yasmin
from yasmin import State, StateMachine, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.control.mavros.mavros_api import MavDrone

class GoToArenaCenter(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Going to arena center...")

        if ("mavdrone" not in blackboard) or not blackboard["mavdrone"]:
            yasmin.YASMIN_LOG_ERROR(f"Mavdrone not available in {self.__class__.__name__} state.")
            return ABORT
        mavdrone: MavDrone = blackboard["mavdrone"]

        try:
            mavdrone.offboard_position(
                x = 2.0,
                y = 0.0,
                z = 0.0,
                timeout_sec=30,
                precision_radius=0.3,
                disable_altitude_control=True,
            )
            yasmin.YASMIN_LOG_INFO("Reached arena's center.")
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Reaching arena's center failed: {e}")
            return ABORT
