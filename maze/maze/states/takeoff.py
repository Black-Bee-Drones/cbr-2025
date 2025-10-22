"""Takeoff state for maze navigation"""

from yasmin import State
from yasmin.blackboard import Blackboard

from maze.constants import SUCCESS, FAILURE, ENTRY_HEIGHT


class Takeoff(State):
    """
    Takeoff state - takes off and reaches entry height.
    """

    def __init__(self):
        """Initialize takeoff state."""
        super().__init__(outcomes=[SUCCESS, FAILURE])

    def execute(self, blackboard: Blackboard) -> str:
        """Execute takeoff state."""

        node = blackboard["node"]
        tello = blackboard["tello"]

        node.get_logger().info("========================================")
        node.get_logger().info("TAKEOFF")
        node.get_logger().info("========================================")

        try:
            # Start video stream for QR detection
            node.get_logger().info("Starting video stream...")
            tello.start_video_stream()

            # Takeoff to entry height
            if not tello.takeoff(ENTRY_HEIGHT):
                node.get_logger().error("Takeoff failed")
                return FAILURE

            # Update blackboard
            blackboard["current_height"] = ENTRY_HEIGHT

            node.get_logger().info(f"Takeoff complete at {ENTRY_HEIGHT}m")
            return SUCCESS

        except Exception as e:
            node.get_logger().error(f"Takeoff error: {e}")
            return FAILURE
