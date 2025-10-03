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
    ReturnToLaunch,
    End,
)

from delivery.state_machines import (
    DropoffSM,
    PickupSM,
)


class Delivery(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "next_pkg"])

        self.add_state(
            "INITIALIZE",
            Initialize(),
            transitions={SUCCEED:"TAKEOFF", ABORT:"END"},
        )
        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={SUCCEED:"PICKUP", ABORT:"RETURN_TO_LAUNCH"},
        )
        self.add_state(
            "PICKUP",
            PickupSM(),
            transitions={SUCCEED:"DROPOFF", ABORT:"RETURN_TO_LAUNCH"},
        )
        self.add_state(
            "DROPOFF",
            DropoffSM(),
            transitions={SUCCEED:"RETURN_TO_LAUNCH", ABORT:"RETURN_TO_LAUNCH", "next_pkg": "PICKUP"},
        )
        self.add_state(
            "RETURN_TO_LAUNCH",
            ReturnToLaunch(),
            transitions={SUCCEED:"END", ABORT:"END"},
        )
        self.add_state(
            "END",
            End(),
            transitions={SUCCEED: SUCCEED},
        )

        self.set_start_state("INITIALIZE")


def main():
    rclpy.init()
    set_ros_loggers()
    sm = Delivery()

    YasminViewerPub("YASMIN_DEMO", sm)

    # Execute the FSM
    try:
        yasmin.YASMIN_LOG_ERROR(sm.validate())
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()