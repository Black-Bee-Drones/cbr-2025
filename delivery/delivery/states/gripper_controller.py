import time

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, ABORT

from mirela_sdk.control.mavros.mavros_api import MavDrone

from delivery.constants import (
    SERVO_PIN_OUT,
    SERVO_CLOSE_PWM,
    SERVO_OPEN_PWM,
)


class GripperController(State):
    """
    State to control the drone's gripper mechanism.

    Outcome of the state:
        SUCCEED: Gripper action executed successfully.
        FAIL: Gripper control command failed.
        ABORT: `mavdrone` not available in the blackboard.
    """
    def __init__(self, action: str):
        """
        Args:
            action (str): Gripper action. Must be "close" or "open"

        Raises:
            TypeError: If `action` is not "close" or "open".
        """
        super().__init__(outcomes=[SUCCEED, FAIL, ABORT])
        self._action = action.lower()
        if self._action not in ("close", "open"):
            raise TypeError("Parameter action should be 'close' or 'open'.")

    def execute(self, blackboard: Blackboard):
        if ("mavdrone" not in blackboard) or not blackboard["mavdrone"]:
            yasmin.YASMIN_LOG_ERROR(f"Mavdrone not available in {self.__class__.__name__} state.")
            return ABORT
        mavdrone: MavDrone = blackboard["mavdrone"]

        if self._action=="close":
            pwm_value = SERVO_CLOSE_PWM
        elif self._action=="open":
            pwm_value = SERVO_OPEN_PWM

        try:
            yasmin.YASMIN_LOG_INFO(f"{self._action} package, pwm_value={pwm_value}")
            mavdrone.do_servo(
                aux_out=SERVO_PIN_OUT,
                pwm_value=pwm_value,
            )
            time.sleep(3)
            yasmin.YASMIN_LOG_INFO("GripperController executado.")
            return SUCCEED
        except:
            yasmin.YASMIN_LOG_INFO('Fail to control gripper')
            return FAIL
