import time

from mirela_sdk.control.mavros.mavros_api import MavDrone

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT


from delivery.constants import (
    CENTERING_TOLERANCE_PX,
    MAX_ALTITUDE,
    TARGET_UP_ALTITUDE,
    POSITION_CONTROLLER_KP_Z,
    MAX_VELOCITY_Z,
    REACQUIRE_TARGET_TIMEOUT,
)


# Não faz sentido só subir
class ReacquireTarget(State):
    """
    Up to search package
    """
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "next_pkg"])

    def execute(self, blackboard : Blackboard):
        mavdrone: MavDrone = blackboard["mavdrone"]

        current_alt = mavdrone.get_rng_alt.range

        new_alt = current_alt + TARGET_UP_ALTITUDE

        if new_alt >= MAX_ALTITUDE:
            yasmin.YASMIN_LOG_ERROR("Target altitude exceeds maximum allowed limit.")
            return "next_pkg"

        yasmin.YASMIN_LOG_INFO("Starting ascent.")
        start = time.time()
        while (time.time() - start) < REACQUIRE_TARGET_TIMEOUT:
            current_alt = mavdrone.get_rng_alt.range

            if current_alt >= MAX_ALTITUDE:
                yasmin.YASMIN_LOG_ERROR(f"Aborting: current altitude {current_alt:.2f}m >= max limit {MAX_ALTITUDE:.2f}m")
                return "next_pkg"

            error_z = new_alt - current_alt

            if abs(error_z) < CENTERING_TOLERANCE_PX:
                yasmin.YASMIN_LOG_INFO(f"Target altitude reached successfully: {current_alt:.2f}m (error={error_z:.2f}m)")
                return SUCCEED

            vel_z = error_z * POSITION_CONTROLLER_KP_Z

            vel_z = max(-MAX_VELOCITY_Z, min(MAX_VELOCITY_Z, vel_z))

            yasmin.YASMIN_LOG_INFO(f"Ascent correction: current_alt={current_alt:.2f}m, target_alt={new_alt:.2f}m, error={error_z:.2f}m, linear_z={vel_z:.2f}m/s")
            mavdrone.offboard_velocity(
                linear_z=vel_z
            )

        yasmin.YASMIN_LOG_ERROR(f"Timeout ({REACQUIRE_TARGET_TIMEOUT:.1f}s) without reaching target altitude {new_alt:.2f}m. Last altitude={current_alt:.2f}m")
        return ABORT
