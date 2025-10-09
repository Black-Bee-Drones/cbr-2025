# Main State Machine for Dropoff

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, ABORT

from delivery.states import (
    Takeoff,
    Land,
    CenterOnDetection,
    GoToTarget,
    ReacquireTarget,
    GripperController,
    HeightController
)


class DropoffSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "GO_TO_NEXT_CROSS",
            GoToTarget(desired_class="base"),
            transitions={
                SUCCEED : "CENTER",
                ABORT   : ABORT,
            },
        )
        self.add_state(
            "CENTER",
            CenterOnDetection(desired_class='cross'),
            transitions={
                SUCCEED : "DESCEND",
                FAIL    : "REACQUIRE_CROSS",  # Target not detected for too long.
                ABORT   : ABORT,
            },
        )
        self.add_state(
            "REACQUIRE_CROSS",
            ReacquireTarget(desired_class='cross'),
            transitions={
                SUCCEED        : "CENTER",
                FAIL           : "GO_TO_NEXT_CROSS",
                ABORT          : ABORT,
            },
        )
        self.add_state (
            "DESCEND",
            HeightController(direction="down"),
            transitions={
                SUCCEED        : "CENTER",
                FAIL           : "LAND",
                ABORT          : ABORT,
            },
        )
        self.add_state(
            "LAND",
            Land(),
            transitions={
                SUCCEED : "DROP_PKG",
                ABORT   : ABORT,
            },
        )
        self.add_state(
            "DROP_PKG",
            GripperController(action="open"),
            transitions={
                SUCCEED: "TAKEOFF",
                ABORT: ABORT,
            },
        )
        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={
                SUCCEED    : SUCCEED,
                ABORT      : ABORT,
            },
        )

        self.set_start_state("GO_TO_NEXT_CROSS")
