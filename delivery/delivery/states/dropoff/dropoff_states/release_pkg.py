import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT


class ReleasePkg(State):
    """
    Soltar pacote
    """
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("RealeasePkg executado.")
        return SUCCEED
