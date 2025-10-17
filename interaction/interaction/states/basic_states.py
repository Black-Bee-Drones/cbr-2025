import rclpy
import time

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode
from mirela_sdk.control.mavros.mavros_api import MavDrone
from interaction.constants import (
    TAKEOFF_HEIGHT,
    TAKEOFF_POSE,
    TAKEOFF_TIMEOUT,
    ALTITUDE_TOLERANCE,
    TAKEOFF_POSE,
    YOLO_IMAGE_SIZE,
)

from mirela_sdk.utils.position_utils import PositionUtils
import math
import numpy as np

from interaction.utils.yolo_detector import YoloDetector


class Initialize(State):
    """Initializes the drone connection and checks system status."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):

        try:
            blackboard["mavdrone"] = MavDrone(
                node=YasminNode.get_instance(), mavros=False, indoor=True
            )
            mavdrone: MavDrone = blackboard["mavdrone"]

            yasmin.YASMIN_LOG_INFO("Initializing YOLO detector...")
            yolo_detector = YoloDetector()
            if yolo_detector.load_model():
                blackboard["yolo_detector"] = yolo_detector
                yasmin.YASMIN_LOG_INFO("YOLO detector initialized successfully.")
                yasmin.YASMIN_LOG_INFO("Running random detection...")
                yolo_detector.detect_gesture(
                    np.zeros((YOLO_IMAGE_SIZE, YOLO_IMAGE_SIZE, 3), dtype=np.uint8)
                )

            else:
                yasmin.YASMIN_LOG_ERROR("Failed to load YOLO model.")
                return ABORT

            takeoff_position = mavdrone.get_position_as_target

            blackboard["takeoff_position"] = takeoff_position
            blackboard["initial_orientation"] = PositionUtils.get_yaw_from_pose(
                mavdrone.get_position
            )
            blackboard["rtl_land_counter"] = 0

            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Error at Initialize: {e}")

            return ABORT


class Takeoff(State):
    """Arms the drone and takes off to search altitude."""

    def __init__(self, altitude: float = TAKEOFF_HEIGHT):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.altitude = altitude

    def execute(self, blackboard: Blackboard):

        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in Takeoff state.")
            return ABORT

        mavdrone: MavDrone = blackboard["mavdrone"]
        yasmin.YASMIN_LOG_INFO(f"Taking off to altitude: {self.altitude}m...")

        try:
            mavdrone.arm_takeoff(self.altitude)

            start_time = time.time()
            while time.time() - start_time < TAKEOFF_TIMEOUT:
                rclpy.spin_once(YasminNode.get_instance(), timeout_sec=0.1)

                current_alt = mavdrone.get_height
                yasmin.YASMIN_LOG_INFO(f"Current altitude: {current_alt:.2f}m")

                altitude_error = self.altitude - current_alt

                if abs(altitude_error) < ALTITUDE_TOLERANCE:
                    yasmin.YASMIN_LOG_INFO(
                        "Takeoff altitude reached. Ready to start search pattern."
                    )

                    mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                    time.sleep(2)
                    mavdrone.set_takeoff_position(blackboard["takeoff_position"])

                    return SUCCEED

                correction_velocity = max(-0.5, min(0.5, 0.3 * altitude_error))
                mavdrone.offboard_velocity(0.0, 0.0, correction_velocity, 0.0)

                time.sleep(0.1)

            yasmin.YASMIN_LOG_ERROR("Takeoff timeout reached.")
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Takeoff failed: {e}")
            return ABORT


class AdjustPosition(State):
    def __init__(self, adjust_yaw: bool = True):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.adjust = adjust_yaw

    def execute(self, blackboard: Blackboard):
        if ("mavdrone" not in blackboard) or not blackboard["mavdrone"]:
            yasmin.YASMIN_LOG_ERROR(
                f"Mavdrone not available in {self.__class__.__name__} state."
            )
            return ABORT
        mavdrone: MavDrone = blackboard["mavdrone"]

        try:

            mavdrone.offboard_position(
                x=-2.0,
                y=0.0,
                z=TAKEOFF_POSE,
                timeout_sec=30,
                ground_reference=True,
                precision_radius=0.15,
                disable_altitude_control=True,
            )
            mavdrone.offboard_position(
                x=-1.5,
                y=2.10,
                z=TAKEOFF_POSE,
                timeout_sec=30,
                ground_reference=True,
                precision_radius=0.15,
                disable_altitude_control=True,
            )

            if self.adjust:
                yasmin.YASMIN_LOG_INFO("Adjust in position completed.")

                position = mavdrone.get_position
                orientation = PositionUtils.get_yaw_from_pose(position)

                initial_orientation = blackboard["initial_orientation"]

                initial_orientation = math.degrees(initial_orientation)

                current_orientation = math.degrees(orientation)

                yaw_error = initial_orientation - current_orientation
                print(
                    f"os carai {yaw_error} | inicial {initial_orientation} | atual {current_orientation}"
                )
                while yaw_error < 87:
                    rclpy.spin_once(mavdrone.node, timeout_sec=0.1)
                    mavdrone.node.get_logger().info(
                        f"Yaw: {yaw_error}/90", throttle_duration_sec=0.1
                    )
                    mavdrone.offboard_velocity(angular_z=-0.1)
                    position = mavdrone.get_position
                    orientation = PositionUtils.get_yaw_from_pose(position)
                    current_orientation = math.degrees(orientation)
                    yaw_error = initial_orientation - current_orientation

                yasmin.YASMIN_LOG_INFO("Adjust Yaw completed")

            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Landing failed: {e}")
            return ABORT


class ReturnToLaunch(State):
    """Returns the drone to the takeoff position using local coordinates and lands."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in ReturnToLaunch state.")
            return ABORT

        mavdrone: MavDrone = blackboard["mavdrone"]
        takeoff_position = blackboard["takeoff_position"]

        if not takeoff_position:
            yasmin.YASMIN_LOG_ERROR("Takeoff position not stored.")
            return ABORT

        try:
            mavdrone.set_takeoff_position(blackboard["initial_position"])
            mavdrone.offboard_position(
                x=mavdrone.get_position.pose.pose.position.x,
                y=0.0,
                z=TAKEOFF_HEIGHT,
                timeout_sec=40,
                ground_reference=True,
                precision_radius=0.15,
                disable_altitude_control=True,
            )
            mavdrone.rtl(
                rtl_alt=None, precision_radius=0.1, rtl_strategy="default", land=False
            )
            mavdrone.offboard_position(
                x=mavdrone.get_position.pose.pose.position.x + 0.2,
                y=mavdrone.get_position.pose.pose.position.y,
                z=mavdrone.get_position.pose.pose.position.z,
                ground_reference=True,
                precision_radius=0.1,
                timeout_sec=30,
                strategy="default",
                disable_altitude_control=True,
            )
            mavdrone.land()
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"RTL failed: {e}")
            return ABORT

        return SUCCEED


class End(State):
    """Finalizes the mission and performs cleanup."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Phase 1 Mission completed. Cleanup...")

        if "mavdrone" in blackboard:
            mavdrone: MavDrone = blackboard["mavdrone"]

            if mavdrone.get_state.armed:
                yasmin.YASMIN_LOG_INFO("Drone still armed, ensuring safe landing...")
                try:
                    mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                    mavdrone.land()
                except Exception as e:
                    yasmin.YASMIN_LOG_ERROR(f"Failed to land during cleanup: {e}")

        return SUCCEED
