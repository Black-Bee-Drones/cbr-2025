# Main State Machine for Delivery

import rclpy

import yasmin
from yasmin_ros import set_ros_loggers
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin import Blackboard
from yasmin import State
from mirela_sdk.control.mavros import MavDrone


from delivery.states import (
    Initialize,
    Takeoff,
)


class WaitAndLand(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        if ("mavdrone" not in blackboard) or not blackboard["mavdrone"]:
            yasmin.YASMIN_LOG_ERROR(f"Mavdrone not available in {self.__class__.__name__} state.")
            return ABORT
        mavdrone: MavDrone = blackboard["mavdrone"]

        try:
            yasmin.YASMIN_LOG_INFO("Waiting 60 seconds.")
            mavdrone.delay(60)
            yasmin.YASMIN_LOG_INFO("Landing...")
            mavdrone.land()
            mavdrone.delay(5) 
            yasmin.YASMIN_LOG_INFO("Landed successfully.")
            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Landing failed: {e}")
            return ABORT



class Delivery(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])


        self.add_state(
            "TAKEOFF",
            Takeoff(True),
            transitions={SUCCEED:"WAIT_AND_LAND", ABORT:ABORT},
        )

        self.add_state(
            "WAIT_AND_LAND",
            WaitAndLand(),
            transitions={SUCCEED:SUCCEED, ABORT:ABORT},
        )
        

        self.set_start_state("TAKEOFF")


def main():
    rclpy.init()

    set_ros_loggers()

    delivery_sm = Delivery()

    YasminViewerPub("YASMIN_DEMO", delivery_sm)

    try:
        yasmin.YASMIN_LOG_ERROR(delivery_sm.validate())
        final_outcome = delivery_sm()
        yasmin.YASMIN_LOG_INFO(final_outcome)
    except KeyboardInterrupt:
        if delivery_sm.is_running():
            delivery_sm.cancel_state()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
