import rclpy

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from interaction.states.basic_states import (
    Initialize, 
    Takeoff,
    FindHuman,
    StartGesture,
    CheckCount,
    ReturnToLaunch,
    End
)

class CBRPhase3StateMachine(StateMachine):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "INITIALIZE", Initialize(), transitions={SUCCEED: "TAKEOFF", ABORT: "END"}
        )

        self.add_state(
            "TAKEOFF", Takeoff(), transitions={SUCCEED: "FIND_HUMAN", ABORT: "END"}
        )

        self.add_state(
            "FIND_HUMAN", FindHuman(), transitions={SUCCEED: "START_GESTURE", ABORT: "RETURN_TO_LAUNCH"}
        )

        self.add_state(
            "START_GESTURE", StartGesture(), transitions={SUCCEED: "CHECK_COUNT", ABORT: "RETURN_TO_LAUNCH"}
        )

        self.add_state(
            "CHECK_COUNT", CheckCount(), transitions={SUCCEED: "RETURN_TO_LAUNCH", ABORT: "RETURN_TO_LAUNCH"}
        )

        self.add_state(
            "RETURN_TO_LAUNCH", ReturnToLaunch(), transitions={SUCCEED: "END", ABORT:"END"}
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