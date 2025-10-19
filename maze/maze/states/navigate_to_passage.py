"""Navigate to passage state for maze navigation"""

from yasmin import State
from yasmin.blackboard import Blackboard

from maze.constants import SUCCESS, FAILURE, ADVANCE_DISTANCE
from maze.utils.maze_data import Direction


class NavigateToPassage(State):
    """
    Navigate to the next unvisited passage.
    Rotates to face the passage and moves forward.
    """

    def __init__(self):
        """Initialize navigate to passage state."""
        super().__init__(outcomes=[SUCCESS, FAILURE])

    def execute(self, blackboard: Blackboard) -> str:
        """Execute navigate to passage state."""

        node = blackboard["node"]
        tello = blackboard["tello"]
        maze_map = blackboard["maze_map"]

        next_direction = blackboard["next_direction"]
        next_position = blackboard["next_position"]

        node.get_logger().info("========================================")
        node.get_logger().info(f"NAVIGATING TO {next_position}")
        node.get_logger().info("========================================")

        try:
            # Calculate rotation needed
            current_dir = maze_map.current_direction
            rotation_needed = next_direction.to_relative(current_dir)

            if rotation_needed > 0:
                node.get_logger().info(
                    f"Rotating from {current_dir.name} to {next_direction.name}"
                )

                if rotation_needed <= 180:
                    if not tello.rotate_clockwise(rotation_needed):
                        node.get_logger().error("Failed to rotate")
                        return FAILURE
                else:
                    # Rotate counter-clockwise is shorter
                    if not tello.rotate_counter_clockwise(360 - rotation_needed):
                        node.get_logger().error("Failed to rotate")
                        return FAILURE

                maze_map.update_direction(next_direction)

            # Move forward to next grid position
            node.get_logger().info(f"Moving forward {ADVANCE_DISTANCE}m")

            if not tello.move_forward(ADVANCE_DISTANCE):
                node.get_logger().error("Failed to move forward")
                return FAILURE

            # Update maze map
            maze_map.mark_visited(
                next_position[0],
                next_position[1],
                entry_direction=self._get_opposite_direction(next_direction),
            )

            # Update blackboard
            blackboard["points_visited"] += 1

            # Notify mock lidar about position advance
            lidar_client = blackboard["lidar_client"]
            if hasattr(lidar_client, "advance_position"):
                lidar_client.advance_position()

            node.get_logger().info(f"Arrived at position {next_position}")
            node.get_logger().info(f"Points visited: {blackboard['points_visited']}")

            return SUCCESS

        except Exception as e:
            node.get_logger().error(f"Navigation error: {e}")
            return FAILURE

    def _get_opposite_direction(self, direction: Direction) -> Direction:
        """Get opposite direction."""
        opposite_map = {
            Direction.NORTH: Direction.SOUTH,
            Direction.SOUTH: Direction.NORTH,
            Direction.EAST: Direction.WEST,
            Direction.WEST: Direction.EAST,
        }
        return opposite_map[direction]
