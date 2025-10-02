import rclpy
import time

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from mirela_sdk.control.mavros.mavros_api import MavDrone

from mapping.constants import (
    TAKEOFF_ALTITUDE,
    SEARCH_ALTITUDE,
    TAKEOFF_TIMEOUT,
    ALTITUDE_TOLERANCE,
    SEARCH_AREA_WIDTH,
    SEARCH_AREA_HEIGHT,
    GRID_SPACING,
    GRID_PATTERN_TYPE,
    GRID_PRIMARY_DIRECTION,
    GRID_TRANSITION_DIRECTION,
    GRID_START_OFFSET,
)
from mapping.utils import Grid, YOLODetector


class Initialize(State):
    """Initializes the drone connection and checks system status."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):

        try:
            blackboard["mavdrone"] = MavDrone(node=YasminNode.get_instance(), indoor=True)
            mavdrone: MavDrone = blackboard["mavdrone"]

            rclpy.spin_once(YasminNode.get_instance(), timeout_sec=0.5)

            pose = mavdrone.get_vision_pos.pose.pose.position
            initial_position = (
                pose.x,
                pose.y,
                pose.z
            )
            blackboard["initial_position"] = initial_position
            blackboard["takeoff_position"] = initial_position 
            
            blackboard["target_search_altitude"] = SEARCH_ALTITUDE

            yasmin.YASMIN_LOG_INFO(
                f"Initial drone position: ({initial_position[0]:.2f}, {initial_position[1]:.2f}, {initial_position[2]:.2f})"
            )
            yasmin.YASMIN_LOG_INFO(
                f"Target search altitude: {SEARCH_ALTITUDE:.2f}m"
            )

            grid = Grid(
                search_width=SEARCH_AREA_WIDTH,
                search_height=SEARCH_AREA_HEIGHT,
                grid_spacing=GRID_SPACING,
                initial_position=initial_position,
                start_offset=GRID_START_OFFSET,
                pattern_type=GRID_PATTERN_TYPE,
                primary_direction=GRID_PRIMARY_DIRECTION,
                transition_direction=GRID_TRANSITION_DIRECTION,
            )
            blackboard["grid_waypoints"] = grid

            yolo_detector = YOLODetector()
            blackboard["yolo_detector"] = yolo_detector

            blackboard["visited_bases"] = []
            blackboard["current_target_waypoint"] = None
            blackboard["current_detection"] = None

            pattern_summary = grid.get_pattern_summary()
            yasmin.YASMIN_LOG_INFO("Phase 1 Mission Initialized")
            yasmin.YASMIN_LOG_INFO(
                f"Pattern: {pattern_summary['pattern_type']} - {pattern_summary['primary_direction']}"
            )
            yasmin.YASMIN_LOG_INFO(
                f"Grid waypoints: {pattern_summary['total_waypoints']} points"
            )
            yasmin.YASMIN_LOG_INFO(f"Search area: {pattern_summary['search_area']}")
            yasmin.YASMIN_LOG_INFO(
                f"Search origin: ({pattern_summary['search_origin'][0]:.2f}, {pattern_summary['search_origin'][1]:.2f})"
            )

            print(grid.visualize_pattern())

            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Initialization failed: {e}")
            return ABORT


class Takeoff(State):
    """Arms the drone and takes off to search altitude."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in Takeoff state.")
            return ABORT

        mavdrone: MavDrone = blackboard["mavdrone"]

        yasmin.YASMIN_LOG_INFO(f"Taking off to altitude: {TAKEOFF_ALTITUDE}m...")

        if blackboard["takeoff_position"] is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            pose = mavdrone.get_vision_pos.pose.pose.position
            takeoff_position = (
                pose.x,
                pose.y,
                pose.z
            )
            blackboard["takeoff_position"] = takeoff_position
            yasmin.YASMIN_LOG_INFO(f"Stored takeoff position: ({takeoff_position[0]:.2f}, {takeoff_position[1]:.2f})")

        try:
            mavdrone.arm_takeoff(TAKEOFF_ALTITUDE-1)

            mavdrone.delay(3)

            start_time = time.time()
            while time.time() - start_time < TAKEOFF_TIMEOUT:
                rclpy.spin_once(self.node, timeout_sec=0.1)

                current_alt = mavdrone.get_rng_alt.range
                yasmin.YASMIN_LOG_INFO(f"Current altitude: {current_alt:.2f}m")

                altitude_error = TAKEOFF_ALTITUDE - current_alt

                if abs(altitude_error) < ALTITUDE_TOLERANCE:
                    yasmin.YASMIN_LOG_INFO(
                        "Takeoff altitude reached. Ready to start search pattern."
                    )

                    mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                    time.sleep(2)

                    return SUCCEED

                correction_velocity = max(-0.5, min(0.5, 0.3 * altitude_error))
                mavdrone.offboard_velocity(0.0, 0.0, correction_velocity, 0.0)

                time.sleep(0.1)

            yasmin.YASMIN_LOG_ERROR("Takeoff timeout reached.")
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Takeoff failed: {e}")
            return ABORT


class ReturnToLaunch(State):
    """Returns the drone to the takeoff position using local coordinates and lands."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in ReturnToLaunch state.")
            return ABORT

        mavdrone: MavDrone = blackboard["mavdrone"]
        takeoff_position = blackboard["takeoff_position"]

        if not takeoff_position:
            yasmin.YASMIN_LOG_ERROR("Takeoff position not stored.")
            return ABORT

        yasmin.YASMIN_LOG_INFO("Returning to takeoff base using local coordinates...")

        # Log mission summary
        visited_bases = blackboard["visited_bases"]
        grid_waypoints = blackboard["grid_waypoints"]

        yasmin.YASMIN_LOG_INFO(f"Mission Summary:")
        if grid_waypoints:
            progress = grid_waypoints.get_progress()
            yasmin.YASMIN_LOG_INFO(
                f"  - Waypoints visited: {progress['visited_waypoints']}/{progress['total_waypoints']}"
            )
        yasmin.YASMIN_LOG_INFO(f"  - Landing bases visited: {len(visited_bases)}")
        yasmin.YASMIN_LOG_INFO(f"  - Mission completion: {len(visited_bases)}/6 bases")

        try:
            # Get current position
            rclpy.spin_once(YasminNode.get_instance(), timeout_sec=0.1)
            current_pos = mavdrone.get_vision_pos.pose.pose.position
            
            yasmin.YASMIN_LOG_INFO(f"Returning to position ({takeoff_position[0]:.2f}, {takeoff_position[1]:.2f})")
        
            mavdrone.offboard_position(
                x=takeoff_position[0] - current_pos.x,
                y=takeoff_position[1] - current_pos.y,
                z=0.0, 
                precision_radius=0.3,
                timeout_sec=60.0,
                strategy="PID"
            )

            # Land at takeoff position
            yasmin.YASMIN_LOG_INFO("Landing at takeoff position...")
            mavdrone.land()
            time.sleep(10) 

            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"RTL failed: {e}")
            return ABORT


class End(State):
    """Finalizes the mission and performs cleanup."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Phase 1 Mission completed. Cleanup...")

        if "mavdrone" in blackboard:
            mavdrone: MavDrone = blackboard["mavdrone"]

            if mavdrone.get_state.armed:
                yasmin.YASMIN_LOG_INFO("Drone still armed, ensuring safe landing...")
                try:
                    mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                    mavdrone.land()
                except Exception as e:
                    yasmin.YASMIN_LOG_ERROR(f"Failed to land during cleanup: {e}")

        visited_bases = blackboard["visited_bases"]
        grid_waypoints = blackboard["grid_waypoints"]

        yasmin.YASMIN_LOG_INFO("FINAL MISSION REPORT")

        if grid_waypoints:
            progress = grid_waypoints.get_progress()
            yasmin.YASMIN_LOG_INFO(
                f"Grid coverage: {progress['progress_percent']:.1f}%"
            )
            yasmin.YASMIN_LOG_INFO(
                f"Waypoints visited: {progress['visited_waypoints']}/{progress['total_waypoints']}"
            )
        yasmin.YASMIN_LOG_INFO(f"Landing bases visited: {len(visited_bases)}")
        yasmin.YASMIN_LOG_INFO(
            f"Mission success rate: {len(visited_bases)}/6 ({100*len(visited_bases)/6:.1f}%)"
        )

        if len(visited_bases) == 6:
            yasmin.YASMIN_LOG_INFO("MISSION COMPLETED SUCCESSFULLY!")
        else:
            yasmin.YASMIN_LOG_INFO("Mission partially completed.")

        return SUCCEED
