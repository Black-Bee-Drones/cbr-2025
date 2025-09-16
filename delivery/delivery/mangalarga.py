# Main State Machine for Delivery

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from delivery.states import (
    Initialize,
    Takeoff,
    PickupSM,
    DropoffSM,
    ReturnToLaunch,
    End,
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