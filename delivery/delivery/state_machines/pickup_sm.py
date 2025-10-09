# Main State Machine for Pickup

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, CANCEL, ABORT

from delivery.states import (
    Takeoff,
    Land,
    CenterOnDetection,
    GoToTarget,
    ReacquireTarget,
    GripperController,
    AlignPkg,
    CheckPkg,
    HeightController
)


class PickupSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "GO_TO_NEXT_PACKAGE",
            GoToTarget(desired_class="package"),
            transitions={
                SUCCEED : "CENTER",
                ABORT   : ABORT,
            },
        )
        self.add_state(
            "CENTER",
            CenterOnDetection(desired_class='package'),
            transitions={
                SUCCEED : "ALIGN_PKG",
                FAIL    : "REACQUIRE_PACKAGE",  # Target not detected for too long.
                ABORT   : ABORT,
            },
        )
        self.add_state(
            "REACQUIRE_PACKAGE",
            ReacquireTarget(desired_class='package'),
            transitions={
                SUCCEED : "CENTER",
                FAIL    : "GO_TO_NEXT_PACKAGE",
                ABORT   : ABORT,
            },
        )
        self.add_state(
            "ALIGN_PKG",
            AlignPkg(),
            transitions={
                SUCCEED : "DESCEND",
                FAIL    : "REACQUIRE_PACKAGE",
                ABORT   : ABORT,
            },
        )
        self.add_state(
            "DESCEND",
            HeightController(direction="down"),
            transitions={
                SUCCEED : "CENTER",
                FAIL    : "OPEN_GRIPPER",
                ABORT   : ABORT,
            },
        )
        self.add_state(
            "OPEN_GRIPPER",
            GripperController(action="open"),
            transitions={
                SUCCEED : "LAND",
                ABORT   : ABORT,
            },
        )
        self.add_state(
            "LAND",
            Land(),
            transitions={
                SUCCEED :"PICK_PKG",
                ABORT   : ABORT,
            },
        )
        self.add_state(
            "PICK_PKG",
            GripperController(action="close"),
            transitions={
                SUCCEED : "TAKEOFF",
                ABORT   : ABORT,
            },
        )
        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={
                SUCCEED : "CHECK_PKG",
                ABORT   : ABORT,
            },
        )
        self.add_state(
            "CHECK_PKG",
            CheckPkg(),
            transitions={
                SUCCEED : SUCCEED,
                FAIL    : "CENTER",
                CANCEL  : "GO_TO_NEXT_PACKAGE",
                ABORT   : ABORT,
            },
        )

        self.set_start_state("GO_TO_NEXT_PACKAGE")
