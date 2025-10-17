import rclpy
from rclpy.executors import MultiThreadedExecutor

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from interaction.states import (
    Initialize,
    Takeoff,
    AdjustPosition,
    ReturnToLaunch,
    End,
    PoseControl,
)

from interaction.constants import (
    TAKEOFF_HEIGHT,
    TAKEOFF_POSE,
)


class CBRPhase3StateMachine(StateMachine):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "INITIALIZE",
            Initialize(),
            transitions={SUCCEED: "TAKEOFF", ABORT: "END"},
        )

        self.add_state(
            "TAKEOFF", Takeoff(TAKEOFF_POSE), transitions={SUCCEED: "ADJUST_POSITION", ABORT: "END"}
        )

        self.add_state(
            "ADJUST_POSITION",
            AdjustPosition(),
            transitions={SUCCEED: "POSE_CONTROL", ABORT: "RETURN_TO_LAUNCH"},
        )

        self.add_state(
            "POSE_CONTROL",
            PoseControl(),
            transitions={SUCCEED: "RETURN_TO_LAUNCH", ABORT: "RETURN_TO_LAUNCH"},
        )

        self.add_state(
            "RETURN_TO_LAUNCH",
            ReturnToLaunch(),
            transitions={SUCCEED: "END", ABORT: "END"},
        )

        self.add_state("END", End(), transitions={SUCCEED: SUCCEED})


def main() -> None:
    print("CBR 2025 - PHASE 3: HUMAN INTERACTION")

    rclpy.init()

    try:
        phase3_sm = CBRPhase3StateMachine()

        print(phase3_sm())

    except Exception as e:
        print(f"Mission failed with exception: {e}")

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
