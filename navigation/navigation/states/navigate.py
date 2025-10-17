import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT



class Navigate(State):
    def __init__(self, action: str):
        super().__init__(outcomes=[SUCCEED, ABORT, "FINAL_SUCCEED"])

    def execute(self, blackboard: Blackboard):
        try:
            ...
            return SUCCEED
        except Exception as e:
            return ABORT

        
