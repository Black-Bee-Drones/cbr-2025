import rclpy
import time
import math
from typing import Dict, Tuple, Optional

import yasmin
from yasmin_ros.yasmin_node import YasminNode

from mapping.constants import (
    POSITION_TOLERANCE,
    ALTITUDE_TOLERANCE,
    POSITION_CONTROLLER_KP_XY,
    POSITION_CONTROLLER_KP_Z,
    POSITION_CONTROLLER_KP_YAW,
    MAX_VELOCITY_XY,
    MAX_VELOCITY_Z,
)


class PositionController:
    """
    Generic position controller for indoor flight using local pose feedback.

    Provides proportional control to reach target positions using offboard_velocity
    commands with body reference frame.
    """

    def __init__(self, mavdrone, node=None):
        """
        Initialize position controller.

        Args:
            mavdrone: MavDrone instance for control commands
            node: ROS2 node (optional, will use YasminNode if not provided)
        """
        self.mavdrone = mavdrone
        self.node = node or YasminNode.get_instance()

        # Control gains from constants
        self.kp_xy = POSITION_CONTROLLER_KP_XY
        self.kp_z = POSITION_CONTROLLER_KP_Z
        self.kp_yaw = POSITION_CONTROLLER_KP_YAW

        # Velocity limits from constants
        self.max_vel_xy = MAX_VELOCITY_XY
        self.max_vel_z = MAX_VELOCITY_Z
        self.max_vel_yaw = 0.3  # rad/s

    def goto_position(
        self,
        target_x: float,
        target_y: float,
        target_z: float = None,
        target_yaw: float = None,
        tolerance_xy: float = POSITION_TOLERANCE,
        tolerance_z: float = ALTITUDE_TOLERANCE,
        timeout: float = 30.0,
    ) -> bool:
        """
        Navigate to target position using proportional control.

        Args:
            target_x: Target X position (meters, local frame)
            target_y: Target Y position (meters, local frame)
            target_z: Target Z position (meters, local frame, optional)
            target_yaw: Target yaw angle (radians, optional)
            tolerance_xy: XY position tolerance (meters)
            tolerance_z: Z position tolerance (meters)
            timeout: Maximum time to reach position (seconds)

        Returns:
            True if position reached successfully, False if timeout or error
        """
        start_time = time.time()

        yasmin.YASMIN_LOG_INFO(
            f"Navigating to position ({target_x:.2f}, {target_y:.2f})"
        )
        if target_z is not None:
            yasmin.YASMIN_LOG_INFO(f"Target altitude: {target_z:.2f}m")

        while time.time() - start_time < timeout:
            # Get current position
            rclpy.spin_once(self.node, timeout_sec=0.01)
            current_pos = self.mavdrone.get_local_pos.pose.position
            current_alt = self.mavdrone.get_rel_alt.data

            # Calculate position errors
            error_x = target_x - current_pos.x
            error_y = target_y - current_pos.y
            error_z = (target_z - current_alt) if target_z is not None else 0.0

            distance_xy = math.sqrt(error_x**2 + error_y**2)
            altitude_ok = abs(error_z) < tolerance_z if target_z is not None else True

            if distance_xy < tolerance_xy and altitude_ok:
                yasmin.YASMIN_LOG_INFO("Target position reached successfully")
                self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                time.sleep(0.5)
                return True

            # Calculate proportional velocities
            vel_x = self._limit_velocity(error_x * self.kp_xy, self.max_vel_xy)
            vel_y = self._limit_velocity(error_y * self.kp_xy, self.max_vel_xy)
            vel_z = (
                self._limit_velocity(error_z * self.kp_z, self.max_vel_z)
                if target_z is not None
                else 0.0
            )

            vel_yaw = 0.0
            if target_yaw is not None:
                current_yaw = self._get_current_yaw()
                yaw_error = self._normalize_angle(target_yaw - current_yaw)
                vel_yaw = self._limit_velocity(
                    yaw_error * self.kp_yaw, self.max_vel_yaw
                )

            # Send velocity commands (body reference frame)
            self.mavdrone.offboard_velocity(
                vel_x, vel_y, vel_z, vel_yaw, ground_reference=False
            )

            if int(time.time() * 4) % 4 == 0:  # Log every 1 second
                yasmin.YASMIN_LOG_DEBUG(
                    f"Position error: ({error_x:.2f}, {error_y:.2f}, {error_z:.2f})"
                )
                yasmin.YASMIN_LOG_DEBUG(
                    f"Velocity cmd: ({vel_x:.2f}, {vel_y:.2f}, {vel_z:.2f})"
                )

            time.sleep(0.05)  # 20Hz

        yasmin.YASMIN_LOG_ERROR("Position control timeout reached")
        self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
        return False

    def goto_relative_position(
        self,
        delta_x: float,
        delta_y: float,
        delta_z: float = 0.0,
        tolerance: float = POSITION_TOLERANCE,
        timeout: float = 20.0,
    ) -> bool:
        """
        Move to position relative to current position.

        Args:
            delta_x: Relative X movement (meters)
            delta_y: Relative Y movement (meters)
            delta_z: Relative Z movement (meters)
            tolerance: Position tolerance (meters)
            timeout: Timeout (seconds)

        Returns:
            True if successful, False otherwise
        """
        # Get current position
        rclpy.spin_once(self.node, timeout_sec=0.1)
        current_pos = self.mavdrone.get_local_pos.pose.position
        current_alt = self.mavdrone.get_rel_alt.data

        # Calculate target position
        target_x = current_pos.x + delta_x
        target_y = current_pos.y + delta_y
        target_z = current_alt + delta_z if delta_z != 0.0 else None

        return self.goto_position(
            target_x,
            target_y,
            target_z,
            tolerance_xy=tolerance,
            tolerance_z=tolerance,
            timeout=timeout,
        )

    def return_to_takeoff_position(
        self, takeoff_position: Dict[str, float], approach_altitude: float = None
    ) -> bool:
        """
        Return to takeoff position using local coordinates.

        Args:
            takeoff_position: Dict with takeoff local coordinates
            approach_altitude: Altitude to maintain during return (optional)

        Returns:
            True if successful, False otherwise
        """
        yasmin.YASMIN_LOG_INFO(
            "Returning to takeoff position using local coordinates..."
        )

        target_x = takeoff_position.get("local_x", 0.0)
        target_y = takeoff_position.get("local_y", 0.0)
        target_z = approach_altitude  # Can be None for altitude hold

        yasmin.YASMIN_LOG_INFO(f"Takeoff position: ({target_x:.2f}, {target_y:.2f})")

        return self.goto_position(target_x, target_y, target_z, timeout=60.0)

    def hover_at_current_position(self, duration: float = 2.0):
        """
        Hover at current position for specified duration.

        Args:
            duration: Hover duration (seconds)
        """
        yasmin.YASMIN_LOG_INFO(f"Hovering for {duration} seconds...")

        # Get current position
        rclpy.spin_once(self.node, timeout_sec=0.1)
        current_pos = self.mavdrone.get_local_pos.pose.position
        current_alt = self.mavdrone.get_rel_alt.data

        # Hold position using position control
        end_time = time.time() + duration
        while time.time() < end_time:
            self.goto_position(
                current_pos.x,
                current_pos.y,
                current_alt,
                tolerance_xy=0.1,
                tolerance_z=0.1,
                timeout=0.2,
            )
            time.sleep(0.1)

        # Stop movement
        self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)

    def set_control_gains(
        self, kp_xy: float = None, kp_z: float = None, kp_yaw: float = None
    ):
        """
        Update control gains.

        Args:
            kp_xy: Proportional gain for XY movement
            kp_z: Proportional gain for Z movement
            kp_yaw: Proportional gain for yaw
        """
        if kp_xy is not None:
            self.kp_xy = kp_xy
        if kp_z is not None:
            self.kp_z = kp_z
        if kp_yaw is not None:
            self.kp_yaw = kp_yaw

        yasmin.YASMIN_LOG_INFO(
            f"Control gains updated: kp_xy={self.kp_xy}, kp_z={self.kp_z}, kp_yaw={self.kp_yaw}"
        )

    def set_velocity_limits(
        self,
        max_vel_xy: float = None,
        max_vel_z: float = None,
        max_vel_yaw: float = None,
    ):
        """
        Update velocity limits.

        Args:
            max_vel_xy: Maximum XY velocity (m/s)
            max_vel_z: Maximum Z velocity (m/s)
            max_vel_yaw: Maximum yaw velocity (rad/s)
        """
        if max_vel_xy is not None:
            self.max_vel_xy = max_vel_xy
        if max_vel_z is not None:
            self.max_vel_z = max_vel_z
        if max_vel_yaw is not None:
            self.max_vel_yaw = max_vel_yaw

        yasmin.YASMIN_LOG_INFO(
            f"Velocity limits updated: xy={self.max_vel_xy}, z={self.max_vel_z}, yaw={self.max_vel_yaw}"
        )

    def _limit_velocity(self, velocity: float, max_velocity: float) -> float:
        """Limit velocity to maximum value."""
        return max(-max_velocity, min(max_velocity, velocity))

    def _get_current_yaw(self) -> float:
        """Get current yaw angle from quaternion (simplified)."""
        # might use tf_transformations
        orientation = self.mavdrone.get_local_pos.pose.orientation

        # Simple yaw extraction from quaternion
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        return yaw

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi] range."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
