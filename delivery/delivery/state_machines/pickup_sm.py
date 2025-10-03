# Main State Machine for Pickup

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL, TIMEOUT

from delivery.states import (
    Takeoff,
    Land,
    CenterOnDetection,
    GoToTarget,
    ReacquireTarget,
    GripperController,
    AlignPkg,
    CheckPkg,
)


class PickupSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, FAIL, "next_pkg"])

        self.add_state(
            "GO_TO_PACKAGE",
            GoToTarget(desired_class="package"),
            transitions={
                SUCCEED : "CENTER_ON_DETECTION", 
                ABORT   : ABORT
            },
        )
        self.add_state(
            "CENTER_ON_DETECTION",
            CenterOnDetection(desired_class='package'),
            transitions={    
                SUCCEED : "ALIGN_PKG",
                FAIL    : "ASCEND_TO_TARGET",  # Target not detected for too long.
                TIMEOUT : "ALIGN_PKG",
                ABORT   : ABORT,
            },
        )
        self.add_state(
            "ASCEND_TO_TARGET",
            ReacquireTarget(direction="up"),
            transitions={
                SUCCEED        : "CENTER_ON_DETECTION",
                TIMEOUT        : "CENTER_ON_DETECTION",
                "height_limit" : 'next_pkg',
                ABORT          : ABORT,
            },
        )
        self.add_state(
            "ALIGN_PKG",
            AlignPkg(),
            transitions={
                SUCCEED:"DESCEND_TO_TARGET", 
                FAIL: "ASCEND_TO_TARGET",  # Target not detected for too long.
                TIMEOUT : "ASCEND_TO_TARGET",
                ABORT: ABORT, 
            },
        )
        self.add_state(
            "DESCEND_TO_TARGET",
            ReacquireTarget(direction="down"),
            transitions={
                SUCCEED        : "CENTER_ON_DETECTION",
                TIMEOUT        : "CENTER_ON_DETECTION",
                "height_limit" : 'LAND',
                ABORT          : ABORT,
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
            GripperController(action="pick"),
            transitions={
                SUCCEED : "TAKEOFF", 
                FAIL    : "TAKEOFF",  # if error in mavdrone.do_servo()
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
                FAIL    : "CENTER_ON_DETECTION",
                ABORT   : ABORT, 
            },
        )

        self.set_start_state("GO_TO_PACKAGE")