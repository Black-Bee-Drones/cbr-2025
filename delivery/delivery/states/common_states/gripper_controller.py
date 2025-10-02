import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL

from mirela_sdk.control.mavros.mavros_api import MavDrone

from delivery.constants import (
    SERVO_PIN_OUT,
    SERVO_PICK_PWM,
    SERVO_DROP_PWM,
)


class GripperController(State):
    def __init__(self, action: str):
        """
        Args:
            action (str): "pick" or "drop"
        """
        super().__init__(outcomes=[SUCCEED, ABORT])
        self._action = action.lower()
        if self._action not in ("pick", "drop"):
            raise TypeError("Parameter action should be 'pick' or 'drop'.")


    def execute(self, blackboard: Blackboard):
        mavdrone: MavDrone = blackboard.get("mavdrone")

        if not mavdrone:
            yasmin.YASMIN_LOG_ERROR("Mavdrone not available in DescendPkg state.")
            return ABORT

        if self._action=="pick":
            pwm_value = SERVO_PICK_PWM
        elif self._action=="drop":
            pwm_value = SERVO_DROP_PWM

        try:
            yasmin.YASMIN_LOG_INFO(f"{self._action} package, pwm_value={pwm_value}")
            mavdrone.do_servo(
                aux_out=SERVO_PIN_OUT,
                pwm_value=pwm_value,
            )
            yasmin.YASMIN_LOG_INFO("GripperController executado.")
            return SUCCEED
        except:
            yasmin.YASMIN_LOG_INFO('Fail to control gripper')
            return FAIL
