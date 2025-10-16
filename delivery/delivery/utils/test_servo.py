from time import sleep
from mirela_sdk.control.mavros import MavDrone

import rclpy
from rclpy.node import Node

from delivery.constants import (
    SERVO_PIN_OUT,
    SERVO_CLOSE_PWM,
    SERVO_OPEN_PWM,
)


class TestServo(Node):
    def __init__(self):
        super().__init__("test_SERVO")
        self.mavdrone = MavDrone(self, indoor=True)

        self.mavdrone.do_servo(
            aux_out=SERVO_PIN_OUT,
            pwm_value=SERVO_OPEN_PWM,
        )
        sleep(6)
        self.mavdrone.do_servo(
            aux_out=SERVO_PIN_OUT,
            pwm_value=SERVO_CLOSE_PWM,
        )


if __name__ == "__main__":
    rclpy.init()

    node = TestServo()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
