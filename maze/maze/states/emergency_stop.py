"""Emergency stop state for maze navigation"""

from yasmin import State
from yasmin.blackboard import Blackboard

from maze.constants import SUCCESS


class EmergencyStop(State):
    """
    Emergency stop state - immediately stops all operations.
    """

    def __init__(self):
        """Initialize emergency stop state."""
        super().__init__(outcomes=[SUCCESS])

    def execute(self, blackboard: Blackboard) -> str:
        """Execute emergency stop state."""

        node = blackboard["node"] if "node" in blackboard else None
        tello = blackboard["tello"] if "tello" in blackboard else None
        lidar_client = (
            blackboard["lidar_client"] if "lidar_client" in blackboard else None
        )

        if node:
            node.get_logger().error("========================================")
            node.get_logger().error("EMERGENCY STOP ACTIVATED")
            node.get_logger().error("========================================")
        else:
            print("EMERGENCY STOP ACTIVATED")

        try:
            # Emergency stop drone
            if tello:
                tello.land()
                tello.disconnect()

            # Disconnect lidar
            if lidar_client:
                lidar_client.disconnect()

            if node:
                node.get_logger().error("Emergency stop complete")
            else:
                print("Emergency stop complete")

        except Exception as e:
            error_msg = f"Emergency stop error: {e}"
            if node:
                node.get_logger().error(error_msg)
            else:
                print(error_msg)

        return SUCCESS
