import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT


# Falta implementar
class DescendPkg(State):
    """
    Descer um pouco e realinhar o drone até chegar em uma boa altura para dar land
    """
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "height_limit"])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("DescendPkg  executado.")
        return SUCCEED