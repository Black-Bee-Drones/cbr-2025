"""Initialize state for maze navigation"""

import rclpy
from yasmin import State
from yasmin.blackboard import Blackboard

from maze.utils.tello_wrapper import TelloWrapper
from maze.utils.lidar_client import LidarClient, MockLidarClient
from maze.utils.qr_detector import QRCodeDetector
from maze.utils.maze_data import MazeMap
from maze.constants import (
    SUCCESS,
    FAILURE,
    LIDAR_WEBSOCKET_URL,
    CONNECTION_TIMEOUT,
    MAX_POINTS_TO_VISIT,
)


class Initialize(State):
    """
    Initialize all components for maze navigation.
    Sets up Tello connection, lidar client, QR detector, and maze map.
    """

    def __init__(self, use_mock_lidar: bool = False):
        """
        Initialize the initialization state.

        Parameters
        ----------
        use_mock_lidar : bool
            Use mock lidar client for testing
        """
        super().__init__(outcomes=[SUCCESS, FAILURE])
        self.use_mock_lidar = use_mock_lidar

    def execute(self, blackboard: Blackboard) -> str:
        """Execute initialization state."""

        node = blackboard["node"]
        node.get_logger().info("========================================")
        node.get_logger().info("CBR 2025 - PHASE 4: MAZE NAVIGATION")
        node.get_logger().info("========================================")
        node.get_logger().info("Initializing components...")

        try:
            # Initialize Tello drone
            node.get_logger().info("Connecting to Tello...")
            tello = TelloWrapper(node)
            if not tello.connect():
                node.get_logger().error("Failed to connect to Tello")
                return FAILURE

            blackboard["tello"] = tello

            # Initialize lidar client - Always use mock for competition
            node.get_logger().info("Initializing lidar client...")
            node.get_logger().info("Using hardcoded maze navigation (Competition Mode)")
            lidar_client = MockLidarClient(node)

            if not lidar_client.connect(timeout=CONNECTION_TIMEOUT):
                node.get_logger().error("Failed to initialize mock lidar client")
                return FAILURE

            blackboard["lidar_client"] = lidar_client

            # Initialize QR detector
            node.get_logger().info("Initializing QR detector...")
            qr_detector = QRCodeDetector(node)
            blackboard["qr_detector"] = qr_detector

            # Initialize maze map
            node.get_logger().info("Initializing maze map...")
            maze_map = MazeMap(grid_size=1.0)
            blackboard["maze_map"] = maze_map

            # Initialize mission parameters
            blackboard["max_points"] = MAX_POINTS_TO_VISIT
            blackboard["points_visited"] = 0
            blackboard["current_height"] = 0.0
            blackboard["mission_complete"] = False
            blackboard["qr_codes_found"] = []

            # Check battery
            battery = tello.get_battery()
            if battery < 30:
                node.get_logger().warn(f"Low battery: {battery}%")
                if battery < 20:
                    node.get_logger().error("Battery too low for mission")
                    return FAILURE

            node.get_logger().info("Initialization complete")
            node.get_logger().info(f"Battery: {battery}%")
            node.get_logger().info(f"Mission: Visit {MAX_POINTS_TO_VISIT} points")

            return SUCCESS

        except Exception as e:
            node.get_logger().error(f"Initialization failed: {e}")
            return FAILURE
