# Main State Machine for Delivery

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL, TIMEOUT

from delivery.states import (
    Land,
    Takeoff,
    CenterOnDetection, 
    GoToTarget,
    DescendToTarget,
    GripperController,
)


class DropoffSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "next_pkg"])

        self.add_state(
            "GO_TO_TARGET",
            GoToTarget(desired_class="base"),
            transitions={
                SUCCEED:"CENTER_ON_DETECTION", 
                ABORT: ABORT
            },
        )
        self.add_state(
            "CENTER_ON_DETECTION",
            CenterOnDetection(desired_class='base'),
            transitions={
                SUCCEED: "DESCEND_TO_TARGET",  # Target centered successfully.
                FAIL: ABORT,  # Target not detected for too long.
                ABORT: ABORT,  # Required components not available.
                TIMEOUT: ABORT,  # Could not center target within allowed time.
            },
        )
        self.add_state (
            "DESCEND_TO_TARGET",
            DescendToTarget(),
            transitions={SUCCEED: "LAND", ABORT: "next_pkg", "height_limit": "LAND"},
        )
        self.add_state(
            "LAND",
            Land(),
            transitions={SUCCEED: "RELEASE_PKG", ABORT: ABORT}
        )
        self.add_state(
            "RELEASE_PKG",
            GripperController(action='drop'),
            transitions={SUCCEED: "TAKEOFF", ABORT: ABORT, FAIL: "TAKEOFF"},  # FAIL -> if error in mavdrone.do_servo()
        )
        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={SUCCEED: SUCCEED, ABORT : ABORT, "next_pkg" : "next_pkg"},
        )
        
        self.set_start_state("GO_TO_TARGET")