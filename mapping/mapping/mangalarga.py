#!/usr/bin/env python3

import rclpy
import time

from yasmin import StateMachine, State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, TIMEOUT
from yasmin_ros.yasmin_node import YasminNode

from mapping.states import (
    Initialize,
    Takeoff,
    NavigateToWaypoint,
    CaptureAndDetect,
    CenterOnDetection,
    LandAndWait,
    ReturnToLaunch,
    End,
)


class CBRPhase1StateMachine(StateMachine):
    """
    CBR 2025 Phase 1 State Machine - Location and Mapping

    Mission: Detect 6 landing bases in 8x8m arena, land on each, and return to takeoff base.
    Uses boustrophedon search pattern with odometry for navigation.
    """

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "INITIALIZE", Initialize(), transitions={SUCCEED: "TAKEOFF", ABORT: "END"}
        )

        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={SUCCEED: "NAVIGATE_TO_WAYPOINT", ABORT: "END"},
        )

        self.add_state(
            "NAVIGATE_TO_WAYPOINT",
            NavigateToWaypoint(),
            transitions={
                SUCCEED: "CAPTURE_AND_DETECT",
                "ALL_COMPLETE": "RETURN_TO_LAUNCH",
                ABORT: "RETURN_TO_LAUNCH",
            },
        )

        self.add_state(
            "CAPTURE_AND_DETECT",
            CaptureAndDetect(),
            transitions={
                SUCCEED: "NAVIGATE_TO_WAYPOINT",
                "DETECTION_FOUND": "CENTER_ON_DETECTION",
                ABORT: "RETURN_TO_LAUNCH",
            },
        )

        self.add_state(
            "CENTER_ON_DETECTION",
            CenterOnDetection(),
            transitions={
                SUCCEED: "LAND_AND_WAIT",
                ABORT: "NAVIGATE_TO_WAYPOINT",
                TIMEOUT: "NAVIGATE_TO_WAYPOINT",
            },
        )

        self.add_state(
            "LAND_AND_WAIT",
            LandAndWait(),
            transitions={SUCCEED: "TAKEOFF_AFTER_LANDING", ABORT: "RETURN_TO_LAUNCH"},
        )

        self.add_state(
            "TAKEOFF_AFTER_LANDING",
            Takeoff(),
            transitions={
                SUCCEED: "NAVIGATE_TO_WAYPOINT",
                ABORT: "RETURN_TO_LAUNCH",
            },
        )

        self.add_state(
            "RETURN_TO_LAUNCH",
            ReturnToLaunch(),
            transitions={SUCCEED: "END", ABORT: "END"},
        )

        self.add_state("END", End(), transitions={SUCCEED: SUCCEED})


def main() -> None:
    print("CBR 2025 - PHASE 1: LOCATION AND MAPPING")

    rclpy.init()

    try:
        phase1_sm = CBRPhase1StateMachine()

        print(phase1_sm())

    except Exception as e:
        print(f"Mission failed with exception: {e}")

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
