import time

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
    POSITION_CONTROLLER_KP_Z,
    POSITION_CONTROLLER_MAX_VELOCITY_Z,
    POSITION_CONTROLLER_TOLERANCE_Z,
)


class ReacquireTarget(State):
    """
    State to reacquire a lost target by adjusting the drone's altitude.

    Outcome of the state:
        - SUCCEED: Target altitude reached successfully.
        - FAIL: Target altitude not reached within allowed time or timeout.
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

        current_alt = mavdrone.get_rng_alt.range

        if self._direction == 'up':
            new_alt = current_alt + TARGET_UP_ALTITUDE

            if new_alt >= MAX_ALTITUDE:
                yasmin.YASMIN_LOG_INFO("Target altitude exceeds maximum allowed limit.")
                return FAIL

        elif self._direction == 'down':
            new_alt = current_alt - TARGET_DOWN_ALTITUDE

            if new_alt <= MIN_CENTERING_ALTITUDE:
                new_alt = MIN_CENTERING_ALTITUDE

        yasmin.YASMIN_LOG_INFO("Starting vertical correction.")
        start = time.time()
        while (time.time() - start) < REACQUIRE_TIMEOUT:
            current_alt = mavdrone.get_rng_alt.range

            if (self._direction == 'up') and (new_alt - current_alt <= POSITION_CONTROLLER_TOLERANCE_Z):
                    yasmin.YASMIN_LOG_WARN(f"Ascending height limit reached: current altitude {current_alt:.2f}m >= max limit {MAX_ALTITUDE}m")
                    return FAIL
            elif (self._direction == 'down') and (current_alt - new_alt <= POSITION_CONTROLLER_TOLERANCE_Z):
                    yasmin.YASMIN_LOG_INFO(f"Descending height limit reached: current altitude {current_alt:.2f}m <= max limit {MIN_CENTERING_ALTITUDE}m")
                    return FAIL

            error_z = new_alt - current_alt

            if abs(error_z) <= POSITION_CONTROLLER_TOLERANCE_Z:
                yasmin.YASMIN_LOG_INFO(f"Target altitude reached successfully: {current_alt:.2f}m (error={error_z:.2f}m)")
                return SUCCEED

            vel_z = error_z * POSITION_CONTROLLER_KP_Z

            vel_z = max(-POSITION_CONTROLLER_MAX_VELOCITY_Z, min(POSITION_CONTROLLER_MAX_VELOCITY_Z, vel_z))

            yasmin.YASMIN_LOG_INFO(f"Ascent correction: current_alt={current_alt:.2f}m, target_alt={new_alt:.2f}m, error={error_z:.2f}m, linear_z={vel_z:.2f}m/s")
            mavdrone.offboard_velocity(
                linear_z=vel_z
            )

        yasmin.YASMIN_LOG_ERROR(f"Timeout ({REACQUIRE_TIMEOUT:.1f}s) without reaching target altitude {new_alt:.2f}m. Last altitude={current_alt:.2f}m")
        return FAIL
