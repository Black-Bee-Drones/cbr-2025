# Main State Machine for Delivery

import rclpy

import yasmin
from yasmin_ros import set_ros_loggers
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from delivery.states import (
    Initialize,
    Takeoff,
    Land,
    GoToTarget,
    GripperController
)

from delivery.state_machines import (
    PickupSM,
)



class Delivery(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "INITIALIZE",
            Initialize(),
            transitions={SUCCEED:"TAKEOFF", ABORT:ABORT},
        )
        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={SUCCEED:"PICKUP", ABORT:"RETURN_TO_LAUNCH"},
        )
        self.add_state(
            "PICKUP",
            PickupSM(),
            transitions={SUCCEED:"GO_TO_NEXT_CROSS", ABORT:"RETURN_TO_LAUNCH"},
        )
        self.add_state(
            "GO_TO_NEXT_CROSS",
            GoToTarget(desired_class="base"),
            transitions={
                SUCCEED : "LAND",
                ABORT   : "RETURN_TO_LAUNCH"
            },
        )
        self.add_state(
            "LAND",
            Land(),
            transitions={
                SUCCEED : "DROP_PKG",
                ABORT   : "RETURN_TO_LAUNCH"
            }
        )
        self.add_state(
            "DROP_PKG",
            GripperController(action="open"),
            transitions={
                SUCCEED: "RETURN_TO_LAUNCH",
                ABORT: "RETURN_TO_LAUNCH"
            },
        )
        self.add_state(
            "RETURN_TO_LAUNCH",
            Land(rtl=True),
            transitions={SUCCEED:SUCCEED, ABORT:ABORT},
        )

        self.set_start_state("INITIALIZE")


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
