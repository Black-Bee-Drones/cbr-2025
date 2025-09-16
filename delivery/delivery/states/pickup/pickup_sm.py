# Main State Machine for Delivery

from yasmin import StateMachine, State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from delivery.states import (
    Takeoff,
    Land,
)

class GoToPkg(State):
    """
    Controller: Mandar drone para coordenada na base de entrega baseada no index na blackboard.
    Detector: Node de Yolo para reconhecer o pacote
    """
class CenterPkg(State):
    """
    Movimentação X, Y para centralizar o drone e o pacote
    """
class AllingPkg(State):
    """
    Movimentação dw Yw no drone até atingir as proporções laterais corretas do bounding box
    """
class DescendPkg(State):
    """
    Descer um pouco e realinhar o drone até chegar em uma boa altura para dar land
    """
class PickPkg(State):
    """
    Ativar garra
    """

class Delivery(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "height_limit"])
        self.add_state(
            "GO_TO_PKG",
            GoToPkg(),
            transitions={SUCCEED:"CENTER_PKG", ABORT: ABORT},
        )
        self.add_state(
            "CENTER_PKG",
            CenterPkg(),
            transitions={SUCCEED:"ALLING_PKG", ABORT: ABORT},
        )
        self.add_state(
            "ALLING_PKG",
            AllingPkg(),
            transitions={SUCCEED:"DESCEND_PKG", ABORT: ABORT},
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
            transitions={SUCCEED: SUCCEED, ABORT : ABORT},
        )
        
        self.set_start_state("GO_TO_PKG")