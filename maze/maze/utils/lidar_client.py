"""WebSocket client for receiving lidar data from ESP module"""

import asyncio
import json
import threading
from typing import Optional, Callable
import websocket
from rclpy.node import Node


class LidarClient:
    """
    WebSocket client for receiving lidar distance measurements
    from ESP module attached to Tello drone.
    """

    def __init__(self, node: Node, websocket_url: str):
        """
        Initialize lidar client.

        Parameters
        ----------
        node : Node
            ROS2 node for logging
        websocket_url : str
            WebSocket server URL (e.g., "ws://192.168.10.1:8765")
        """
        self.node = node
        self.websocket_url = websocket_url
        self.ws = None
        self.is_connected = False
        self.latest_distance = None
        self.distance_lock = threading.Lock()
        self.ws_thread = None
        self.running = False
        self.on_distance_callback = None

    def connect(self, timeout: float = 5.0) -> bool:
        """
        Connect to WebSocket server.

        Parameters
        ----------
        timeout : float
            Connection timeout in seconds

        Returns
        -------
        bool
            True if connected successfully
        """
        try:
            self.node.get_logger().info(
                f"Connecting to lidar WebSocket at {self.websocket_url}"
            )

            # Create WebSocket connection
            self.ws = websocket.WebSocketApp(
                self.websocket_url,
                on_message=self._on_message,
                on_error=self._on_error,
                on_close=self._on_close,
                on_open=self._on_open,
            )

            # Start WebSocket in separate thread
            self.running = True
            self.ws_thread = threading.Thread(target=self._run_ws)
            self.ws_thread.daemon = True
            self.ws_thread.start()

            # Wait for connection
            start_time = (
                asyncio.get_event_loop().time() if asyncio.get_event_loop() else 0
            )
            while (
                not self.is_connected
                and (asyncio.get_event_loop().time() if asyncio.get_event_loop() else 0)
                - start_time
                < timeout
            ):
                threading.Event().wait(0.1)

            if self.is_connected:
                self.node.get_logger().info("Connected to lidar WebSocket")
                return True
            else:
                self.node.get_logger().error("Failed to connect to lidar WebSocket")
                return False

        except Exception as e:
            self.node.get_logger().error(f"WebSocket connection error: {e}")
            return False

    def disconnect(self):
        """Disconnect from WebSocket server."""
        try:
            self.running = False
            if self.ws:
                self.ws.close()
            if self.ws_thread:
                self.ws_thread.join(timeout=2)
            self.is_connected = False
            self.node.get_logger().info("Disconnected from lidar WebSocket")
        except Exception as e:
            self.node.get_logger().error(f"Error disconnecting: {e}")

    def _run_ws(self):
        """Run WebSocket client in thread."""
        while self.running:
            try:
                self.ws.run_forever(reconnect=1)
            except Exception as e:
                self.node.get_logger().error(f"WebSocket error: {e}")
                if self.running:
                    threading.Event().wait(1)  # Wait before reconnecting

    def _on_open(self, ws):
        """WebSocket opened callback."""
        self.is_connected = True
        self.node.get_logger().debug("WebSocket connection opened")

    def _on_message(self, ws, message):
        """
        WebSocket message received callback.

        Expected message format: {"distance": 1.23} (distance in meters)
        """
        try:
            data = json.loads(message)
            if "distance" in data:
                with self.distance_lock:
                    self.latest_distance = float(data["distance"])

                # Callback if registered
                if self.on_distance_callback:
                    self.on_distance_callback(self.latest_distance)

                self.node.get_logger().debug(
                    f"Lidar distance: {self.latest_distance:.2f}m"
                )
            else:
                self.node.get_logger().warn(f"Invalid lidar message format: {message}")

        except json.JSONDecodeError as e:
            self.node.get_logger().error(f"Failed to decode lidar message: {e}")
        except Exception as e:
            self.node.get_logger().error(f"Error processing lidar message: {e}")

    def _on_error(self, ws, error):
        """WebSocket error callback."""
        self.node.get_logger().error(f"WebSocket error: {error}")

    def _on_close(self, ws, close_status_code, close_msg):
        """WebSocket closed callback."""
        self.is_connected = False
        self.node.get_logger().debug(f"WebSocket closed: {close_msg}")

    def get_distance(self, timeout: float = 2.0) -> Optional[float]:
        """
        Get latest distance reading.

        Parameters
        ----------
        timeout : float
            Timeout for getting valid reading

        Returns
        -------
        Optional[float]
            Distance in meters or None if no valid reading
        """
        if not self.is_connected:
            self.node.get_logger().warn("Lidar not connected")
            return None

        # Wait for valid reading with timeout
        start_time = 0
        while start_time < timeout:
            with self.distance_lock:
                if self.latest_distance is not None:
                    distance = self.latest_distance
                    self.latest_distance = None  # Clear after reading
                    return distance
            threading.Event().wait(0.1)
            start_time += 0.1

        self.node.get_logger().warn("Lidar reading timeout")
        return None

    def set_distance_callback(self, callback: Callable[[float], None]):
        """
        Set callback function for distance updates.

        Parameters
        ----------
        callback : Callable[[float], None]
            Function to call with distance value when received
        """
        self.on_distance_callback = callback

    def request_reading(self):
        """Request a new distance reading from the sensor."""
        if self.ws and self.is_connected:
            try:
                self.ws.send(json.dumps({"command": "read"}))
            except Exception as e:
                self.node.get_logger().error(f"Failed to request reading: {e}")


class MockLidarClient:
    """
    Mock lidar client for competition maze navigation.
    Returns hardcoded distance values that guide the drone through
    the predefined serpentine path in the maze.
    """

    def __init__(self, node: Node, websocket_url: str = None):
        """Initialize mock lidar client with maze-specific values."""
        self.node = node
        self.is_connected = False
        self.position_index = 0
        self.scan_index = 0

        # Hardcoded lidar readings for each position in the maze
        # Each position has [North, East, South, West] readings at high and low altitudes
        # Distance > 0.5m means passage, < 0.5m means wall
        # Positions are numbered in order of visitation
        self.maze_layout = {
            # Position 0: Entry point at (2,1.5) - need to go SOUTH to (2,0.5)
            0: {
                "high": [0.3, 0.3, 0.3, 0.3],  # All walls at high level
                "low": [0.3, 0.3, 0.8, 0.3],  # Gate on SOUTH at low level
            },
            # Position 1: At (2,0.5) - need to go EAST to (3,0.5)
            1: {
                "high": [0.3, 0.8, 0.3, 0.3],  # Free passage EAST
                "low": [0.3, 0.8, 0.3, 0.3],  # Same on low
            },
            # Position 2: At (3,0.5) - need to go NORTH to (3,1.5)
            2: {
                "high": [0.8, 0.3, 0.3, 0.3],  # Passage NORTH
                "low": [0.8, 0.3, 0.3, 0.3],
            },
            # Position 3: At (3,1.5) - need to go EAST to (4,1.5)
            3: {
                "high": [0.3, 0.8, 0.3, 0.3],  # Passage EAST
                "low": [0.3, 0.8, 0.3, 0.3],
            },
            # Position 4: At (4,1.5) - need to go SOUTH to (4,0.5)
            4: {
                "high": [0.3, 0.3, 0.3, 0.3],  # SOUTH at bottom only
                "low": [0.3, 0.3, 0.8, 0.3],
            },
            # Position 5: At (4,0.5) - need to go EAST to (5,0.5)
            5: {
                "high": [0.3, 0.8, 0.3, 0.3],  # EAST at top only
                "low": [0.3, 0.3, 0.3, 0.3],
            },
            # Position 6: At (5,0.5) - need to go NORTH to (5,1.5)
            6: {
                "high": [0.3, 0.3, 0.3, 0.3],  # NORTH at bottom only
                "low": [0.8, 0.3, 0.3, 0.3],
            },
            # Position 7: At (5,1.5) - need to go EAST to (6,1.5)
            7: {
                "high": [0.3, 0.8, 0.3, 0.3],  # EAST at top only
                "low": [0.3, 0.3, 0.3, 0.3],
            },
            # Position 8: At (6,1.5) - need to go SOUTH to (6,0.5)
            8: {
                "high": [0.3, 0.3, 0.8, 0.3],  # SOUTH free (any height)
                "low": [0.3, 0.3, 0.8, 0.3],
            },
            # Position 9: At (6,0.5) - need to go EAST to (7,0.5)
            9: {
                "high": [0.3, 0.3, 0.3, 0.3],  # EAST at bottom only
                "low": [0.3, 0.8, 0.3, 0.3],
            },
            # Position 10: At (7,0.5) - need to go NORTH to (7,1.5)
            10: {
                "high": [0.8, 0.3, 0.3, 0.3],  # NORTH free
                "low": [0.8, 0.3, 0.3, 0.3],
            },
            # Position 11: At (7,1.5) - EXIT (forward/NORTH leads out)
            11: {
                "high": [0.8, 0.3, 0.3, 0.3],  # Forward/North at top (exit)
                "low": [0.3, 0.3, 0.3, 0.3],
            },
        }

        self.current_position = 0
        self.current_height = "high"
        self.current_direction = 0  # Index for [N, E, S, W]

    def connect(self, timeout: float = 5.0) -> bool:
        """Simulate connection."""
        self.node.get_logger().info("Mock lidar client connected (Competition Mode)")
        self.node.get_logger().info(f"Starting at position {self.current_position}")
        self.is_connected = True
        return True

    def disconnect(self):
        """Simulate disconnection."""
        self.node.get_logger().info("Mock lidar client disconnected")
        self.is_connected = False

    def set_height_level(self, height_m: float):
        """
        Update current height level based on drone altitude.
        Called by states when changing altitude.
        """
        if height_m > 0.5:  # Above 0.5m is considered high
            self.current_height = "high"
        else:
            self.current_height = "low"
        self.node.get_logger().debug(f"Height level set to: {self.current_height}")

    def set_direction(self, direction_index: int):
        """
        Set current scanning direction.
        0=North, 1=East, 2=South, 3=West
        """
        self.current_direction = direction_index % 4
        self.node.get_logger().debug(
            f"Direction set to: {['North', 'East', 'South', 'West'][self.current_direction]}"
        )

    def advance_position(self):
        """Move to next position in the maze."""
        if self.current_position < len(self.maze_layout) - 1:
            self.current_position += 1
            self.node.get_logger().info(
                f"Advanced to maze position {self.current_position}"
            )
        else:
            self.node.get_logger().info("Reached end of maze")

    def get_distance(self, timeout: float = 2.0) -> Optional[float]:
        """
        Return simulated distance based on current position and direction.
        This simulates the lidar reading for the current scan direction.
        """
        if not self.is_connected:
            return None

        # Get current position data
        if self.current_position in self.maze_layout:
            position_data = self.maze_layout[self.current_position]
            distances = position_data.get(self.current_height, [0.3, 0.3, 0.3, 0.3])

            # Get distance for current direction
            distance = distances[self.current_direction]

            # Log the reading
            direction_names = ["North", "East", "South", "West"]
            self.node.get_logger().debug(
                f"Mock lidar at pos {self.current_position} "
                f"({self.current_height}, {direction_names[self.current_direction]}): "
                f"{distance:.2f}m {'[PASSAGE]' if distance > 0.5 else '[WALL]'}"
            )

            # Don't auto-increment - direction is explicitly set by scan_position
            return distance
        else:
            # Default wall reading if position not defined
            self.node.get_logger().warn(
                f"No data for position {self.current_position}, returning wall"
            )
            return 0.3

    def request_reading(self):
        """Simulate reading request."""
        pass

    def set_distance_callback(self, callback):
        """Mock callback setter."""
        pass
