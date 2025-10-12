import rclpy
from rclpy.executors import MultiThreadedExecutor

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from interaction.states.basic_states import (
    Initialize, 
    Takeoff,
    FindHuman,
    CheckCount,
    ReturnToLaunch,
    End,
    GestureControl
)

class CBRPhase3StateMachine(StateMachine):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "INITIALIZE", Initialize(), transitions={SUCCEED: "START_GESTURE", ABORT: "END"}
        )

        self.add_state(
            "TAKEOFF", Takeoff(), transitions={SUCCEED: "FIND_HUMAN", ABORT: "END"}
        )

        self.add_state(
            "FIND_HUMAN", FindHuman(), transitions={SUCCEED: "START_GESTURE", ABORT: "RETURN_TO_LAUNCH"}
        )

        self.add_state(
            "START_GESTURE", GestureControl(), transitions={SUCCEED: "RETURN_TO_LAUNCH", ABORT: "RETURN_TO_LAUNCH"}
        )

        self.add_state(
            "RETURN_TO_LAUNCH", ReturnToLaunch(), transitions={SUCCEED: "END", ABORT:"END"}
        )

        self.add_state("END", End(), transitions={SUCCEED: SUCCEED})



def main() -> None:
    print("CBR 2025 - PHASE 3: HUMAN INTERACTION")

    rclpy.init()

    try:
        # Inicializa o nó yasmin
        yasmin_node = YasminNode.get_instance()
        
        # Usa MultiThreadedExecutor para suportar callback groups
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(yasmin_node)
        
        phase3_sm = CBRPhase3StateMachine()

        print(phase3_sm())

    except Exception as e:
        print(f"Mission failed with exception: {e}")

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()