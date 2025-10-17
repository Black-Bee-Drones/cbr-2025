import rclpy

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from .states import (
    Initialize,
    Takeoff,
    Navigate,
    Photoshoot,
    BaseSearch,
    Land
)

class NavigationSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "INITIALIZE",
            Initialize(),
            transitions={SUCCEED:"TAKEOFF", ABORT:ABORT},
        )
        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={SUCCEED:"NAVIGATE", ABORT:"LAND"},
        )
        self.add_state(
            "NAVIGATE",
            Navigate(),
            transitions={SUCCEED:"PHOTOSHOOT", ABORT:"LAND", "FINAL_SUCCEED": "BASE_SEARCH"},
        )
        self.add_state(
            "PHOTOSHOOT",
            Photoshoot(),
            transitions={SUCCEED:"NAVIGATE", ABORT:"NAVIGATE"},
        )
        self.add_state(
            "BASE_SEARCH",
            BaseSearch(),
            transitions={SUCCEED:"LAND", ABORT:"LAND"},
        )
        self.add_state(
            "LAND",
            Land(),
            transitions={SUCCEED:SUCCEED, ABORT:ABORT},
        )
