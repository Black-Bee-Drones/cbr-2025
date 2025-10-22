"""Tello drone wrapper for ROS2 integration"""

import time
from typing import Optional
import numpy as np
from djitellopy import Tello
from rclpy.node import Node


class TelloWrapper:
    """
    Wrapper class for Tello drone using djitellopy library.
    Provides high-level control interface for the maze navigation mission.
    """

    def __init__(self, node: Node):
        """
        Initialize Tello wrapper.

        Parameters
        ----------
        node : Node
            ROS2 node for logging
        """
        self.node = node
        self.tello = None
        self.is_connected = False
        self.is_flying = False
        self.current_height = 0.0
        self.current_yaw = 0.0
        self.frame_read = None
        self.stream_on = False

    def connect(self, max_retries: int = 3) -> bool:
        """
        Connect to Tello drone.

        Parameters
        ----------
        max_retries : int
            Maximum number of connection attempts

        Returns
        -------
        bool
            True if connected successfully
        """
        for attempt in range(max_retries):
            try:
                self.node.get_logger().info(
                    f"Attempting to connect to Tello (attempt {attempt + 1}/{max_retries})"
                )
                self.tello = Tello()
                self.tello.connect()

                # Check battery level
                battery = self.tello.get_battery()
                self.node.get_logger().info(f"Connected to Tello. Battery: {battery}%")

                if battery < 20:
                    self.node.get_logger().warn(f"Low battery warning: {battery}%")

                self.is_connected = True
                return True

            except Exception as e:
                self.node.get_logger().error(
                    f"Connection attempt {attempt + 1} failed: {e}"
                )
                time.sleep(2)

        return False

    def disconnect(self):
        """Disconnect from Tello drone."""
        if self.tello and self.is_connected:
            try:
                if self.stream_on:
                    self.stop_video_stream()
                self.tello.end()
                self.is_connected = False
                self.node.get_logger().info("Disconnected from Tello")
            except Exception as e:
                self.node.get_logger().error(f"Error disconnecting: {e}")

    def takeoff(self, target_height: float = 0.5) -> bool:
        """
        Takeoff and reach target height.

        Parameters
        ----------
        target_height : float
            Target height in meters

        Returns
        -------
        bool
            True if takeoff successful
        """
        if not self.is_connected:
            self.node.get_logger().error("Not connected to Tello")
            return False

        try:
            self.node.get_logger().info(f"Taking off to {target_height}m")
            self.tello.takeoff()
            self.is_flying = True
            time.sleep(4)  # Wait for stabilization

            # Adjust to target height
            current_height = self.tello.get_height()
            print(f"Current Height: {current_height}")
            height_diff = (target_height * 100) - current_height  # Convert to cm

            if abs(height_diff) > 20:
                if height_diff > 0:
                    self.tello.move_up(int(abs(height_diff)))
                else:
                    self.tello.move_down(int(abs(height_diff)))
                time.sleep(2)

            self.current_height = target_height
            self.node.get_logger().info(
                f"Takeoff complete at height: {self.current_height}m"
            )
            return True

        except Exception as e:
            self.node.get_logger().error(f"Takeoff failed: {e}")
            return False

    def land(self) -> bool:
        """
        Land the drone.

        Returns
        -------
        bool
            True if landing successful
        """
        if not self.is_flying:
            self.node.get_logger().warn("Drone is not flying")
            return True

        try:
            self.node.get_logger().info("Landing...")
            self.tello.land()
            self.is_flying = False
            self.current_height = 0.0
            self.node.get_logger().info("Landing complete")
            return True

        except Exception as e:
            self.node.get_logger().error(f"Landing failed: {e}")
            return False

    def move_forward(self, distance_m: float) -> bool:
        """Move forward by specified distance in meters."""
        try:
            distance_cm = int(distance_m * 100)
            if distance_cm < 20:  # Minimum movement is 20cm
                distance_cm = 20
            self.node.get_logger().info(f"Moving forward {distance_cm}cm")
            self.tello.move_forward(distance_cm)
            time.sleep(1)  # Stabilization
            return True
        except Exception as e:
            self.node.get_logger().error(f"Move forward failed: {e}")
            return False

    def move_back(self, distance_m: float) -> bool:
        """Move backward by specified distance in meters."""
        try:
            distance_cm = int(distance_m * 100)
            if distance_cm < 20:
                distance_cm = 20
            self.node.get_logger().info(f"Moving back {distance_cm}cm")
            self.tello.move_back(distance_cm)
            time.sleep(1)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Move back failed: {e}")
            return False

    def move_left(self, distance_m: float) -> bool:
        """Move left by specified distance in meters."""
        try:
            distance_cm = int(distance_m * 100)
            if distance_cm < 20:
                distance_cm = 20
            self.node.get_logger().info(f"Moving left {distance_cm}cm")
            self.tello.move_left(distance_cm)
            time.sleep(1)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Move left failed: {e}")
            return False

    def move_right(self, distance_m: float) -> bool:
        """Move right by specified distance in meters."""
        try:
            distance_cm = int(distance_m * 100)
            if distance_cm < 20:
                distance_cm = 20
            self.node.get_logger().info(f"Moving right {distance_cm}cm")
            self.tello.move_right(distance_cm)
            time.sleep(1)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Move right failed: {e}")
            return False

    def rotate_clockwise(self, degrees: int) -> bool:
        """Rotate clockwise by specified degrees."""
        try:
            self.node.get_logger().info(f"Rotating clockwise {degrees}°")
            self.tello.rotate_clockwise(degrees)
            self.current_yaw = (self.current_yaw + degrees) % 360
            time.sleep(1)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Rotation failed: {e}")
            return False

    def rotate_counter_clockwise(self, degrees: int) -> bool:
        """Rotate counter-clockwise by specified degrees."""
        try:
            self.node.get_logger().info(f"Rotating counter-clockwise {degrees}°")
            self.tello.rotate_counter_clockwise(degrees)
            self.current_yaw = (self.current_yaw - degrees) % 360
            time.sleep(1)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Counter rotation failed: {e}")
            return False

    def go_xyz_speed(self, x_cm: int, y_cm: int, z_cm: int, speed: int) -> bool:
        """
        Move to x, y, z relative to current position.

        Parameters
        ----------
        x_cm : int
            Forward (+) or backward (-) distance in cm (20-500)
        y_cm : int
            Left (-) or right (+) distance in cm (20-500)
        z_cm : int
            Up (+) or down (-) distance in cm (20-500)
        speed : int
            Speed in cm/s (10-100)

        Returns
        -------
        bool
            True if successful
        """
        try:
            self.node.get_logger().info(
                f"Moving relative: x={x_cm}cm, y={y_cm}cm, z={z_cm}cm at {speed}cm/s"
            )
            self.tello.go_xyz_speed(x_cm, y_cm, z_cm, speed)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Go xyz speed failed: {e}")
            return False

    def set_height(self, target_height_m: float) -> bool:
        """
        Set drone to specific height.

        Parameters
        ----------
        target_height_m : float
            Target height in meters

        Returns
        -------
        bool
            True if height adjustment successful
        """
        try:
            current_height_cm = self.tello.get_height()
            current_height_m = current_height_cm / 100.0
            height_diff_m = target_height_m - current_height_m
            height_diff_cm = int(abs(height_diff_m * 100))

            if height_diff_cm < 20:  # Skip small adjustments
                return True

            if height_diff_m > 0:
                self.node.get_logger().info(f"Moving up {height_diff_cm}cm")
                self.tello.move_up(height_diff_cm)
            else:
                self.node.get_logger().info(f"Moving down {height_diff_cm}cm")
                self.tello.move_down(height_diff_cm)

            time.sleep(2)
            self.current_height = target_height_m
            return True

        except Exception as e:
            self.node.get_logger().error(f"Height adjustment failed: {e}")
            return False

    def start_video_stream(self):
        """Start video streaming from Tello."""
        try:
            if not self.stream_on:
                self.tello.streamon()
                self.stream_on = True
                # Get frame reader object
                self.frame_read = self.tello.get_frame_read()
                self.node.get_logger().info("Video stream started")
                time.sleep(2)  # Wait for stream to stabilize
        except Exception as e:
            self.node.get_logger().error(f"Failed to start video stream: {e}")

    def stop_video_stream(self):
        """Stop video streaming from Tello."""
        try:
            if self.stream_on:
                if self.frame_read:
                    self.frame_read.stop()
                    self.frame_read = None
                self.tello.streamoff()
                self.stream_on = False
                self.node.get_logger().info("Video stream stopped")
        except Exception as e:
            self.node.get_logger().error(f"Failed to stop video stream: {e}")

    def take_photo(self) -> Optional[np.ndarray]:
        """
        Take a photo from the drone camera in RGB format.

        Returns
        -------
        Optional[np.ndarray]
            Captured image in RGB format or None if failed
        """
        try:
            if not self.stream_on:
                self.start_video_stream()
                time.sleep(2)  # Wait for stream to stabilize

            if self.frame_read is not None:
                # Get the current frame (in BGR from djitellopy)
                frame = self.frame_read.frame
                if frame is not None:
                    # Convert BGR to RGB before returning
                    import cv2

                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    return frame_rgb.copy()
                else:
                    self.node.get_logger().warn("Frame is None")
                    return None
            else:
                self.node.get_logger().error("Frame reader not initialized")
                return None
        except Exception as e:
            # Log error but don't crash
            self.node.get_logger().debug(f"Frame capture exception: {e}")
            return None

    def get_battery(self) -> int:
        """Get battery percentage."""
        if self.is_connected:
            return self.tello.get_battery()
        return 0

    def get_height(self) -> float:
        """Get current height in meters."""
        if self.is_connected:
            return self.tello.get_height() / 100.0  # Convert cm to m
        return 0.0

    def emergency_stop(self):
        """Emergency stop - immediately stop all motors."""
        if self.tello:
            try:
                self.tello.emergency()
                self.is_flying = False
                self.node.get_logger().warn("EMERGENCY STOP ACTIVATED")
            except Exception as e:
                self.node.get_logger().error(f"Emergency stop failed: {e}")
