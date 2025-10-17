import rclpy

import yasmin
from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from navigation.states import (
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
            transitions={SUCCEED:"PHOTOSHOOT", ABORT:"LAND", "FINAL_SUCCEED": "LAND"},
        )
        self.add_state(
            "PHOTOSHOOT",
            Photoshoot(),
            transitions={SUCCEED:"NAVIGATE", ABORT:"LAND"},
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

        self.set_start_state("INITIALIZE")

def main():
    rclpy.init()

    navigation_sm = NavigationSM()

    try:
        yasmin.YASMIN_LOG_ERROR(navigation_sm.validate())
        final_outcome = navigation_sm()
        yasmin.YASMIN_LOG_DEBUG(final_outcome)
    except KeyboardInterrupt:
        if navigation_sm.is_running():
            navigation_sm.cancel_state()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
