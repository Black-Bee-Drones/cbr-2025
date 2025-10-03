# Main State Machine for Dropoff

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL, TIMEOUT

from delivery.states import (
    Takeoff,
    Land,
    CenterOnDetection,
    GoToTarget,
    ReacquireTarget,
    GripperController,
)


class DropoffSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "next_pkg"])

        self.add_state(
            "GO_TO_BASE",
            GoToTarget(desired_class="base"),
            transitions={
                SUCCEED : "CENTER_ON_DETECTION", 
                ABORT   : ABORT,
            },
        )
        self.add_state(
            "CENTER_ON_DETECTION",
            CenterOnDetection(desired_class='base'),
            transitions={
                SUCCEED : "DESCEND_TO_TARGET",
                FAIL    : "ASCEND_TO_TARGET",  # Target not detected for too long.
                TIMEOUT : "DESCEND_TO_TARGET",
                ABORT   : ABORT,
            },
        )
        self.add_state(
            "ASCEND_TO_TARGET",
            ReacquireTarget(direction="up"),
            transitions={
                SUCCEED        : "CENTER_ON_DETECTION",
                TIMEOUT        : "CENTER_ON_DETECTION",
                "height_limit" : 'next_pkg', ###################################
                ABORT          : ABORT,
            },
        )
        self.add_state (
            "DESCEND_TO_TARGET",
            DescendToTarget(),
            transitions={
                SUCCEED        : "CENTER_ON_DETECTION", 
                TIMEOUT        : "CENTER_ON_DETECTION",
                "height_limit" : "LAND", 
                ABORT          : "next_pkg", 
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
            GripperController(action='drop'),
            transitions={
                SUCCEED: "TAKEOFF", 
                FAIL: "TAKEOFF",  # if error in mavdrone.do_servo()
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
        
        self.set_start_state("GO_TO_BASE")