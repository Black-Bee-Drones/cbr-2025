#!/usr/bin/env python3
import rclpy
from yasmin import StateMachine, State, Blackboard
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
    End
)


class CentralizeTestStateMachine(StateMachine):
    """Test state machine for one mission cycle."""
    
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        
        self.add_state("INITIALIZE", Initialize(),
                      transitions={SUCCEED: "TAKEOFF", ABORT: "END"})
        
        self.add_state("TAKEOFF", Takeoff(),
                      transitions={SUCCEED: "NAVIGATE", ABORT: "END"})

        self.add_state("NAVIGATE", NavigateToWaypoint(),
                      transitions={
                          SUCCEED: "CAPTURE",
                          "ALL_COMPLETE": "LAND",
                          ABORT: "LAND"
                      })
        
        self.add_state("CAPTURE", CaptureAndDetect(),
                      transitions={
                          SUCCEED: "LAND",
                          "DETECTION_FOUND": "CENTER",
                          ABORT: "LAND"
                      })

        self.add_state("CENTER", CenterOnDetection(),
                      transitions={
                          SUCCEED: "LAND",
                          ABORT: "LAND",
                          TIMEOUT: "LAND"
                      })

        self.add_state("LAND", LandAndWait(),
                      transitions={
                          SUCCEED: "END",
                          ABORT: "END"
                      })

        self.add_state("END", End(),
                      transitions={SUCCEED: SUCCEED})


def main():
    rclpy.init()
    
    try:
        sm = CentralizeTestStateMachine()
        print(sm())
        
    except Exception as e:
        print(f"Centralization test failed: {e}")
    
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
