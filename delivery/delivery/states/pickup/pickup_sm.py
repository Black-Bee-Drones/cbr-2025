# Main State Machine for Delivery

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL

from .pickup_states import (
    ReacquireTarget, 
    AlignPkg, 
    PickPkg, 
    CheckPkg,
)

from common_states import (
    CenterOnDetection,
    GoToTarget,
    DescendToTarget, 
)

from delivery.states import (
    Takeoff,
    Land,
)


class PickupSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, FAIL, "height_limit", "pkg_detected", "next_pkg"])

        self.add_state(
            "GO_TO_TARGET",
            GoToTarget(),
            transitions={SUCCEED:"CENTER_ON_DETECTION", ABORT: ABORT},
        )
        self.add_state(
            "CENTER_ON_DETECTION",
            CenterOnDetection(desired_class='package'),
            transitions={SUCCEED:"ALIGN_PKG", ABORT: ABORT, FAIL: "REACQUIRE_TARGET"},
        )
        self.add_state(
            "REACQUIRE_TARGET",
            ReacquireTarget(),
            transitions={SUCCEED:"CENTER_ON_DETECTION", ABORT: ABORT, "next_pkg": "next_pkg"},
        )
        self.add_state(
            "ALIGN_PKG",
            AlignPkg(),
            transitions={SUCCEED:"DESCEND_TO_TARGET", ABORT: ABORT, FAIL: "REACQUIRE_TARGET"},
        )
        self.add_state(
            "DESCEND_TO_TARGET",
            DescendToTarget(),
            transitions={SUCCEED:"CENTER_ON_DETECTION", ABORT: ABORT, "height_limit": "LAND"},
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
            transitions={SUCCEED: SUCCEED, ABORT : ABORT, "pkg_detected" : "CENTER_ON_DETECTION"},
        )

        self.set_start_state("GO_TO_TARGET")