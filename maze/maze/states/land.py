"""Land state for maze navigation"""

import time
from yasmin import State
from yasmin.blackboard import Blackboard

from maze.constants import SUCCESS, FAILURE


class Land(State):
    """
    Land state - moves to arena center and lands the drone.
    """

    def __init__(self):
        """Initialize land state."""
        super().__init__(outcomes=[SUCCESS, FAILURE])

    def execute(self, blackboard: Blackboard) -> str:
        """Execute land state - move to arena center and land."""

        node = blackboard["node"] if "node" in blackboard else None
        tello = blackboard["tello"] if "tello" in blackboard else None
        qr_detector = blackboard["qr_detector"] if "qr_detector" in blackboard else None
        maze_map = blackboard["maze_map"] if "maze_map" in blackboard else None

        node.get_logger().info("========================================")
        node.get_logger().info("MOVING TO ARENA CENTER AND LANDING")
        node.get_logger().info("========================================")

        if maze_map:
            summary = maze_map.get_exploration_summary()
            node.get_logger().info("Mission Summary:")
            node.get_logger().info(f"  Points visited: {summary['visited_points']}")
            node.get_logger().info(f"  Total scans: {summary['total_scans']}")
            node.get_logger().info(f"  QR codes found: {summary['qr_codes_found']}")

        if "qr_codes_found" in blackboard and blackboard["qr_codes_found"]:
            node.get_logger().info(f"QR codes detected: {blackboard['qr_codes_found']}")

        try:
            if tello and tello.is_flying:
                
                # go 1m forward
                tello.go_xyz_speed(100, 0, 0, 30)

                # Move to arena center before landing
                # 1.5m forward and 3.5m to the left relative to drone's orientation
                node.get_logger().info("Moving to arena center...")
                node.get_logger().info(
                    "Moving 1.5m forward and 3.5m left to reach landing platform"
                )

                # Convert meters to centimeters for go_xyz_speed
                x = 150  # 1.5 meters forward
                y = -350  # 3.5 meters to the left (negative = left)k
                z = 0  # No vertical movement
                speed = 30  # Speed in cm/s

                try:
                    # Use go_xyz_speed to move to the center
                    # This moves relative to the drone's current orientation
                    if tello.go_xyz_speed(x, y, z, speed):
                        node.get_logger().info("Successfully moved to arena center")
                        # Wait for drone to stabilize
                        time.sleep(2)
                    else:
                        node.get_logger().warn("Failed to move to exact center")
                        node.get_logger().info(
                            "Proceeding with landing at current position"
                        )

                except Exception as move_error:
                    node.get_logger().warn(
                        f"Failed to move to exact center: {move_error}"
                    )
                    node.get_logger().info(
                        "Proceeding with landing at current position"
                    )

                # Stop video stream before landing
                node.get_logger().info("Stopping video stream...")
                tello.stop_video_stream()

                # Land the drone
                node.get_logger().info("Landing drone...")
                if not tello.land():
                    node.get_logger().error("Landing failed")
                    return FAILURE

                node.get_logger().info("Landing complete - Mission accomplished!")
            else:
                node.get_logger().info("Drone already on ground")

            return SUCCESS

        except Exception as e:
            node.get_logger().error(f"Landing sequence error: {e}")
            # Try emergency landing if something goes wrong
            if tello and tello.is_flying:
                node.get_logger().info("Attempting emergency landing...")
                try:
                    tello.land()
                except:
                    pass
            return FAILURE
