from mirela_sdk.control.mavros.mavros_api import MavDrone

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, ABORT

from delivery.constants import (
    REACQUIRE_TIMEOUT,
    MAX_ALTITUDE,
    MIN_CENTERING_ALTITUDE,
    TARGET_UP_ALTITUDE,
    TARGET_DOWN_ALTITUDE,
    POSITION_CONTROLLER_TOLERANCE_Z,
    SEARCH_TIMEOUT,
)


class HeightController(State):
    """
    State to adjust the drone's altitude.

    Outcome of the state:
        - SUCCEED: Target altitude reached successfully or timeout.
        - FAIL: Target altitude not reached within allowed time.
        - ABORT: Required components (e.g., `mavdrone`) not available.
    """
    def __init__(self, direction: str):
        """
        Args:
            direction (str): "up" or "down".

        Raises:
            TypeError: If `direction` is not "up" or "down".
        """
        super().__init__(outcomes=[SUCCEED, FAIL, ABORT])

        self._direction = direction.lower()
        if self._direction not in ("up", "down"):
            raise TypeError("Parameter direction should be 'up' or 'down'.")

    def execute(self, blackboard : Blackboard):
        if ("mavdrone" not in blackboard) or not blackboard["mavdrone"]:
            yasmin.YASMIN_LOG_ERROR(f"Mavdrone not available in {self.__class__.__name__} state.")
            return ABORT
        mavdrone: MavDrone = blackboard["mavdrone"]

        current_alt = mavdrone.get_height

        if self._direction == 'up':
            if current_alt >= MAX_ALTITUDE or (current_alt + TARGET_UP_ALTITUDE) >= MAX_ALTITUDE:
                yasmin.YASMIN_LOG_INFO("Altitude exceeds maximum allowed limit.")
                return FAIL
        elif self._direction == 'down':
            if current_alt <= MIN_CENTERING_ALTITUDE or (current_alt + TARGET_DOWN_ALTITUDE) <= MIN_CENTERING_ALTITUDE:
                yasmin.YASMIN_LOG_INFO("Altitude exceeds minimum allowed limit.")
                return FAIL

        yasmin.YASMIN_LOG_INFO("Starting vertical correction.")

        try:
            if self._direction == 'up':
                target_altitude = TARGET_UP_ALTITUDE

            elif self._direction == 'down': 
                target_altitude = TARGET_DOWN_ALTITUDE

                if current_alt > 1.6:
                    target_altitude *= 2

            mavdrone.offboard_position(
                x=0.0,
                y=0.0,
                z=target_altitude,
                timeout_sec=REACQUIRE_TIMEOUT,
                precision_radius=POSITION_CONTROLLER_TOLERANCE_Z,
            )
            yasmin.YASMIN_LOG_INFO("Target point reached successfully.")
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Navigation failed: {e}")
            return ABORT
