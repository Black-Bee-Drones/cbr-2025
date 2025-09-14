# Main State Machine for Delivery

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from delivery.states import (
    Initialize,
    Takeoff
)

class Delivery(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.add_state(
            "INITIALIZE",
            Initialize(),
            transitions={SUCCEED:"TAKEOFF", ABORT:"END"},
        )
        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={SUCCEED:"", ABORT:"RETURN_TO_LAUNCH"},
        )
        self.set_start_state("INITIALIZE")