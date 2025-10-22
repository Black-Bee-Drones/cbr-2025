"""Analyze passages state for maze navigation"""

from yasmin import State
from yasmin.blackboard import Blackboard

from maze.constants import SUCCESS, COMPLETE


class AnalyzePassages(State):
    """
    Analyze scan data to find next unvisited passage.
    Determines if mission is complete or continues exploration.
    """

    def __init__(self):
        """Initialize analyze passages state."""
        super().__init__(outcomes=[SUCCESS, COMPLETE])

    def execute(self, blackboard: Blackboard) -> str:
        """Execute analyze passages state."""

        node = blackboard["node"]
        maze_map = blackboard["maze_map"]
        points_visited = blackboard["points_visited"]
        max_points = blackboard["max_points"]

        node.get_logger().info("========================================")
        node.get_logger().info("ANALYZING PASSAGES")
        node.get_logger().info("========================================")

        # Check if mission complete
        if points_visited >= max_points:
            node.get_logger().info(f"Mission complete! Visited {points_visited} points")
            blackboard["mission_complete"] = True
            return COMPLETE

        # Find next unvisited passage
        next_passage = maze_map.get_next_unvisited_passage()

        if next_passage:
            direction, target_pos = next_passage
            blackboard["next_direction"] = direction
            blackboard["next_position"] = target_pos

            node.get_logger().info(f"Next passage found:")
            node.get_logger().info(f"  Direction: {direction.name}")
            node.get_logger().info(f"  Target position: {target_pos}")

            return SUCCESS
        else:
            # No more passages from current position
            # Try to backtrack or complete mission
            node.get_logger().info("No unvisited passages from current position")

            # For now, consider mission complete if no more passages
            node.get_logger().info(
                f"Exploration complete. Visited {points_visited} points"
            )
            blackboard["mission_complete"] = True
            return COMPLETE
