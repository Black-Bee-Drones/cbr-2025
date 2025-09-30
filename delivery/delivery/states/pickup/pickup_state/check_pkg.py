import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT


class CheckPkg(State):
    """
    Checar pela yolo se o pacote ainda esta na base
    """
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "pkg_detected"])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("CheckPkg (stub) executado.")
        return SUCCEED