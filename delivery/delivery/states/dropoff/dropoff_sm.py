# Main State Machine for Delivery

from yasmin import StateMachine, State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from delivery.states import (
    Takeoff,
)

class GoToDelivery(State):
    """
    Controller: Mandar drone para coordenada na base de entrega baseada no index na blackboard.
    Detector: Node de Yolo para reconhecer a cruz
    """
class ReleasePkg(State):
    """
    Soltar pacote
    """

class Delivery(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "height_limit"])
        self.add_state(
            "GO_TO_DELIVERY",
            GoToDelivery(),
            transitions={SUCCEED:"...", ABORT: ABORT},
        )
        self.add_state(
            "REALEASE_PKG",
            ReleasePkg(),
            transitions={SUCCEED:"TAKEOFF", ABORT: ABORT},
        )
        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={SUCCEED: SUCCEED, ABORT : ABORT},
        )
        
        self.set_start_state("GO_TO_DELIVERY")