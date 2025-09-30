# Main State Machine for Delivery

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL

from pickup.pickup_states import (
    GoToPkg, 
    CenterPkg, 
    ReacquireTarget, 
    AlignPkg, 
    DescendPkg, 
    PickPkg, 
    CenterPkg, 
    CheckPkg,
)

from delivery.states import (
    Takeoff,
    Land,
)


class PickupSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "height_limit", "pkg_detected", FAIL])

        self.add_state(
            "GO_TO_PKG",
            GoToPkg(),
            transitions={SUCCEED:"CENTER_PKG", ABORT: ABORT},
        )
        self.add_state(
            "CENTER_PKG",
            CenterPkg(),
            transitions={SUCCEED:"ALIGN_PKG", ABORT: ABORT, FAIL: "REACQUIRE_TARGET"},
        )
        self.add_state(
            "REACQUIRE_TARGET",
            ReacquireTarget(),
            transitions={SUCCEED:"CENTER_PKG", ABORT: ABORT},
        )
        self.add_state(
            "ALIGN_PKG",
            AlignPkg(),
            transitions={SUCCEED:"DESCEND_PKG", ABORT: ABORT, FAIL: "REACQUIRE_TARGET"},
        )
        self.add_state(
            "DESCEND_PKG",
            DescendPkg(),
            transitions={SUCCEED:"CENTER_PKG", ABORT: ABORT, "height_limit": "LAND"},
        )
        self.add_state(
            "LAND",
            Land(),
            transitions={SUCCEED:"PICK_PKG", ABORT: ABORT},
        )
        self.add_state(
            "PICK_PKG",
            PickPkg(),
            transitions={SUCCEED:"TAKEOFF", ABORT: ABORT},
        )
        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={SUCCEED: "CHECK_PKG", ABORT : ABORT},
        )
        self.add_state(
            "CHECK_PKG",
            CheckPkg(),
            transitions={SUCCEED: SUCCEED, ABORT : ABORT, "pkg_detected" : "CENTER_PKG"},
        )

        self.set_start_state("GO_TO_PKG")