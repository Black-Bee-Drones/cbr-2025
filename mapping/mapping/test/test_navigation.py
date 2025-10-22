#!/usr/bin/env python3
import rclpy
from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from mapping.states import Initialize, Takeoff, NavigateToWaypoint, ReturnToLaunch, End
from mapping.constants import SEARCH_AREA_WIDTH, SEARCH_AREA_HEIGHT, GRID_SPACING


class NavigationTestStateMachine(StateMachine):
    """Test state machine for navigation only."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state("INITIALIZE", Initialize(), 
                      transitions={SUCCEED: "TAKEOFF", ABORT: "END"})
        
        self.add_state("TAKEOFF", Takeoff(),
                      transitions={SUCCEED: "NAVIGATE", ABORT: "END"})

        self.add_state("NAVIGATE", NavigateToWaypoint(),
                      transitions={
                          SUCCEED: "NAVIGATE", 
                          "ALL_COMPLETE": "RTL",
                          ABORT: "RTL"
                      })
        
        self.add_state("RTL", ReturnToLaunch(),
                      transitions={SUCCEED: "END", ABORT: "END"})
        
        self.add_state("END", End(), 
                      transitions={SUCCEED: SUCCEED})


def main():
    print("NAVIGATION TEST")
    print(f"- Grid: {SEARCH_AREA_WIDTH}x{SEARCH_AREA_HEIGHT}m with {GRID_SPACING}m spacing")
    print("="*60 + "\n")

    rclpy.init()
    
    try:
        sm = NavigationTestStateMachine()
        outcome = sm()
        print(f"\nNavigation test completed with outcome: {outcome}")
        
    except Exception as e:
        print(f"Navigation test failed: {e}")
    
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
