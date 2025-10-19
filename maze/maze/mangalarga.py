#!/usr/bin/env python3
import rclpy
import signal
import sys

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode
from yasmin.blackboard import Blackboard

from maze.states import (
    Initialize,
    Takeoff,
    EnterMaze,
    ScanPosition,
    AnalyzePassages,
    NavigateToPassage,
    Land,
    EmergencyStop,
)
from maze.constants import SUCCESS, FAILURE, COMPLETE, MAX_POINTS_TO_VISIT


class MazeNavigationStateMachine(StateMachine):
    """
    State machine for CBR 2025 Phase 4 - Maze Navigation.

    Mission: Navigate through maze, detect QR codes, visit 12 points.
    Uses Tello drone with attached lidar sensor.
    """

    def __init__(self, use_mock_lidar: bool = False):
        """
        Initialize maze navigation state machine.

        Parameters
        ----------
        use_mock_lidar : bool
            Use mock lidar for testing without hardware
        """
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "INITIALIZE",
            Initialize(use_mock_lidar=use_mock_lidar),
            transitions={SUCCESS: "TAKEOFF", FAILURE: "EMERGENCY_STOP"},
        )

        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={SUCCESS: "ENTER_MAZE", FAILURE: "EMERGENCY_STOP"},
        )

        self.add_state(
            "ENTER_MAZE",
            EnterMaze(),
            transitions={SUCCESS: "SCAN_POSITION", FAILURE: "EMERGENCY_STOP"},
        )

        self.add_state(
            "SCAN_POSITION",
            ScanPosition(),
            transitions={SUCCESS: "ANALYZE_PASSAGES", FAILURE: "EMERGENCY_STOP"},
        )

        self.add_state(
            "ANALYZE_PASSAGES",
            AnalyzePassages(),
            transitions={SUCCESS: "NAVIGATE_TO_PASSAGE", COMPLETE: "LAND"},
        )

        self.add_state(
            "NAVIGATE_TO_PASSAGE",
            NavigateToPassage(),
            transitions={SUCCESS: "SCAN_POSITION", FAILURE: "EMERGENCY_STOP"},
        )

        self.add_state(
            "LAND", Land(), transitions={SUCCESS: SUCCEED, FAILURE: "EMERGENCY_STOP"}
        )

        self.add_state("EMERGENCY_STOP", EmergencyStop(), transitions={SUCCESS: ABORT})


def cleanup(blackboard: Blackboard):
    """Cleanup function to safely disconnect all components."""
    node = blackboard["node"] if "node" in blackboard else None
    if node:
        node.get_logger().info("Cleaning up...")

    tello = blackboard["tello"] if "tello" in blackboard else None
    if tello:
        try:
            if tello.is_flying:
                tello.land()
            tello.stop_video_stream()
            tello.disconnect()
        except Exception as e:
            if node:
                node.get_logger().error(f"Tello cleanup error: {e}")

    # Disconnect lidar
    lidar_client = blackboard["lidar_client"] if "lidar_client" in blackboard else None
    if lidar_client:
        try:
            lidar_client.disconnect()
        except Exception as e:
            if node:
                node.get_logger().error(f"Lidar cleanup error: {e}")


def signal_handler(sig, frame, blackboard):
    """Handle Ctrl+C gracefully."""
    print("\n\nInterrupt received, landing drone...")
    cleanup(blackboard)
    sys.exit(0)


def main(args=None) -> None:
    """Main entry point for maze navigation."""

    print("=" * 60)
    print("CBR 2025 - PHASE 4: MAZE NAVIGATION")
    print("=" * 60)
    print(f"Mission: Explore maze and detect QR codes")
    print(f"Target: Visit {MAX_POINTS_TO_VISIT} points")
    print("=" * 60)

    rclpy.init(args=args)

    blackboard = Blackboard()

    try:
        yasmin_node = YasminNode.get_instance()
        yasmin_node.declare_parameter("max_points", MAX_POINTS_TO_VISIT)

        max_points = (
            yasmin_node.get_parameter("max_points").get_parameter_value().integer_value
        )

        blackboard["node"] = yasmin_node
        blackboard["max_points"] = max_points

        print("Using hardcoded maze navigation (Competition Mode)")
        print(f"Maximum points to visit: {max_points}")
        print("-" * 60)

        signal.signal(signal.SIGINT, lambda s, f: signal_handler(s, f, blackboard))

        # Always use mock lidar for competition
        maze_sm = MazeNavigationStateMachine(use_mock_lidar=True)

        outcome = maze_sm(blackboard)

        print("-" * 60)
        print(f"Mission completed with outcome: {outcome}")

        if "maze_map" in blackboard and blackboard["maze_map"]:
            summary = blackboard["maze_map"].get_exploration_summary()
            print("\nFINAL MISSION SUMMARY:")
            print(f"  Points visited: {summary['visited_points']}")
            print(f"  QR codes found: {summary['qr_codes_found']}")
            print(f"  Total scans: {summary['total_scans']}")

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
    except Exception as e:
        print(f"Mission failed with exception: {e}")
        import traceback

        traceback.print_exc()
    finally:
        cleanup(blackboard)
        rclpy.shutdown()
        print("Mission terminated")


if __name__ == "__main__":
    main()
