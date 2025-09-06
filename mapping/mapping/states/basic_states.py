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
    RTL_ALTITUDE,
    TAKEOFF_TIMEOUT,
    ALTITUDE_TOLERANCE,
    SEARCH_AREA_WIDTH,
    SEARCH_AREA_HEIGHT,
    GRID_SPACING,
)
from mapping.utils import GridWaypoints, YOLODetector, PositionController


class Initialize(State):
    """Initializes the drone connection and checks system status."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):

        try:
            blackboard["mavdrone"] = MavDrone(node=YasminNode.get_instance())
            mavdrone: MavDrone = blackboard["mavdrone"]

            grid_waypoints = GridWaypoints(
                search_width=SEARCH_AREA_WIDTH,
                search_height=SEARCH_AREA_HEIGHT,
                grid_spacing=GRID_SPACING,
            )
            blackboard["grid_waypoints"] = grid_waypoints

            yolo_detector = YOLODetector()
            blackboard["yolo_detector"] = yolo_detector

            # Initialize position controller
            position_controller = PositionController(mavdrone)
            blackboard["position_controller"] = position_controller

            blackboard["visited_bases"] = []
            blackboard["takeoff_position"] = None
            blackboard["current_target_waypoint"] = None
            blackboard["current_detection"] = None
            blackboard["pre_center_position"] = None

            time.sleep(2)

            grid_summary = grid_waypoints.get_summary()

            yasmin.YASMIN_LOG_INFO(
                f"Grid waypoints: {grid_summary['total_waypoints']} points"
            )
            yasmin.YASMIN_LOG_INFO(f"Search area: {grid_summary['search_area']}")
            yasmin.YASMIN_LOG_INFO(f"Grid spacing: {grid_summary['grid_spacing']}")

            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Initialization failed: {e}")
            return ABORT


class Takeoff(State):
    """Arms the drone and takes off to search altitude."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in Takeoff state.")
            return ABORT

        mavdrone: MavDrone = blackboard["mavdrone"]
        node = YasminNode.get_instance()

        yasmin.YASMIN_LOG_INFO(f"Taking off to search altitude: {TAKEOFF_ALTITUDE}m...")

        try:
            takeoff_position = {
                "local_x": mavdrone.get_local_pos.pose.position.x,
                "local_y": mavdrone.get_local_pos.pose.position.y,
                "local_z": mavdrone.get_local_pos.pose.position.z,
            }
            blackboard["takeoff_position"] = takeoff_position

            mavdrone.set_home(current_gps=True)
            mavdrone.arm_takeoff(TAKEOFF_ALTITUDE)

            time.sleep(3)

            start_time = time.time()
            while time.time() - start_time < TAKEOFF_TIMEOUT:
                rclpy.spin_once(node)

                current_alt = mavdrone.get_rel_alt.data
                yasmin.YASMIN_LOG_INFO(f"Current altitude: {current_alt:.2f}m")

                altitude_diff = abs(current_alt - TAKEOFF_ALTITUDE)
                if altitude_diff < ALTITUDE_TOLERANCE:
                    yasmin.YASMIN_LOG_INFO(
                        "Takeoff altitude reached. Ready to start search pattern."
                    )

                    mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                    time.sleep(2)

                    return SUCCEED

                altitude_error = TAKEOFF_ALTITUDE - current_alt
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
        position_controller = blackboard.get("position_controller")
        takeoff_position = blackboard.get("takeoff_position")

        if not position_controller:
            yasmin.YASMIN_LOG_ERROR("Position controller not available.")
            return ABORT

        if not takeoff_position:
            yasmin.YASMIN_LOG_ERROR("Takeoff position not stored.")
            return ABORT

        yasmin.YASMIN_LOG_INFO("Returning to takeoff base using local coordinates...")

        # Log mission summary
        visited_bases = blackboard.get("visited_bases", [])
        grid_waypoints = blackboard.get("grid_waypoints")

        yasmin.YASMIN_LOG_INFO(f"Mission Summary:")
        if grid_waypoints:
            progress = grid_waypoints.get_progress()
            yasmin.YASMIN_LOG_INFO(
                f"  - Waypoints visited: {progress['visited_waypoints']}/{progress['total_waypoints']}"
            )
        yasmin.YASMIN_LOG_INFO(f"  - Landing bases visited: {len(visited_bases)}")
        yasmin.YASMIN_LOG_INFO(f"  - Mission completion: {len(visited_bases)}/6 bases")

        try:
            # First, climb to RTL altitude for safe navigation
            yasmin.YASMIN_LOG_INFO(f"Climbing to RTL altitude: {RTL_ALTITUDE}m")
            current_pos = mavdrone.get_local_pos.pose.position
            success = position_controller.goto_position(
                current_pos.x, current_pos.y, RTL_ALTITUDE, timeout=30.0
            )

            if not success:
                yasmin.YASMIN_LOG_ERROR("Failed to climb to RTL altitude")
                return ABORT

            # Navigate back to takeoff position
            success = position_controller.return_to_takeoff_position(
                takeoff_position, approach_altitude=RTL_ALTITUDE
            )

            if not success:
                yasmin.YASMIN_LOG_ERROR("Failed to return to takeoff position")
                return ABORT

            # Land at takeoff position
            yasmin.YASMIN_LOG_INFO("Landing at takeoff position...")
            mavdrone.land()

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

        visited_bases = blackboard.get("visited_bases", [])
        grid_waypoints = blackboard.get("grid_waypoints")

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
