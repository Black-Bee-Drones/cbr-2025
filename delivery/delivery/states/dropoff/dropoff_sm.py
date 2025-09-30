# Main State Machine for Delivery

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from .dropoff_states import (
    MarkBaseAndTakeoff, 
    ReleasePkg, 
)

from common_states import (
    CenterOnDetection, 
    GoToTarget,
)

from delivery.states import (
    Land,
)


class DropoffSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "next_pkg"])

        self.add_state(
            "GO_TO_TARGET",
            GoToTarget(),
            transitions={SUCCEED:"CENTER_ON_DETECTION", ABORT: ABORT},
        )
        self.add_state(
            "CENTER_ON_DETECTION",
            CenterOnDetection(),
            transitions={SUCCEED: "LAND", ABORT: ABORT},
        )
        self.add_state(
            "LAND",
            Land(),
            transitions={SUCCEED: "RELEASE_PKG", ABORT: ABORT}
        )
        self.add_state(
            "RELEASE_PKG",
            ReleasePkg(),
            transitions={SUCCEED: "MARK_BASE_AND_TAKEOFF", ABORT: ABORT},
        )
        self.add_state(
            "MARK_BASE_AND_TAKEOFF",
            MarkBaseAndTakeoff(),
            transitions={SUCCEED: SUCCEED, ABORT : ABORT, "next_pkg" : "next_pkg"},
        )
        
        self.set_start_state("GO_TO_TARGET")