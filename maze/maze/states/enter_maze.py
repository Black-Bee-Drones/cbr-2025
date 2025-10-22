"""Enter maze state for maze navigation"""

from yasmin import State
from yasmin.blackboard import Blackboard

from maze.constants import SUCCESS, FAILURE, ENTRY_DISTANCE
from maze.utils.maze_data import Direction


class EnterMaze(State):
    """
    Enter maze state - moves drone through the entrance gate.
    """

    def __init__(self):
        """Initialize enter maze state."""
        super().__init__(outcomes=[SUCCESS, FAILURE])

    def execute(self, blackboard: Blackboard) -> str:
        """Execute enter maze state."""

        node = blackboard["node"]
        tello = blackboard["tello"]
        maze_map = blackboard["maze_map"]

        node.get_logger().info("========================================")
        node.get_logger().info("ENTERING MAZE")
        node.get_logger().info("========================================")

        try:
            # Move forward through entrance gate
            node.get_logger().info(f"Moving forward {ENTRY_DISTANCE}m through entrance")

            if not tello.move_forward(ENTRY_DISTANCE):
                node.get_logger().error("Failed to enter maze")
                return FAILURE

            # Mark first position as visited
            maze_map.mark_visited(0, 0, entry_direction=Direction.SOUTH)
            blackboard["points_visited"] = 1

            node.get_logger().info("Successfully entered maze")
            node.get_logger().info(f"Current position: {maze_map.current_position}")

            return SUCCESS

        except Exception as e:
            node.get_logger().error(f"Enter maze error: {e}")
            return FAILURE
