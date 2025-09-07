import rclpy
import time
import math
import cv2
import numpy as np

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, TIMEOUT
from yasmin_ros.yasmin_node import YasminNode

from mirela_sdk.image_processing.camera.image_handler import ImageHandler

from mapping.constants import (
    CENTERING_P_GAIN,
    CENTERING_TOLERANCE_PX,
    CENTERING_TIMEOUT,
    CENTERING_VELOCITY,
    LAND_WAIT_TIME,
    CAMERA_SOURCE,
    TAKEOFF_TIMEOUT,
    SEARCH_ALTITUDE,
    ALTITUDE_TOLERANCE,
)


class CenterOnDetection(State):
    """Center drone on detected landing base using P controller."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, TIMEOUT])
        self.image_handler = None
        self.current_frame = None
        self.centering_active = False

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR(
                "MavDrone not available in CenterOnDetection state."
            )
            return ABORT

        mavdrone = blackboard["mavdrone"]
        node = YasminNode.get_instance()
        yolo_detector = blackboard.get("yolo_detector")
        current_detection = blackboard.get("current_detection")

        if not yolo_detector or not current_detection:
            yasmin.YASMIN_LOG_ERROR("YOLO detector or detection not available.")
            return ABORT

        yasmin.YASMIN_LOG_INFO(
            "Starting precision centering on detected landing base..."
        )

        pre_center_position = {
            "x": mavdrone.get_local_pos.pose.position.x,
            "y": mavdrone.get_local_pos.pose.position.y,
            "z": mavdrone.get_local_pos.pose.position.z,
        }
        blackboard["pre_center_position"] = pre_center_position

        try:
            self._start_camera_stream(node)

            success = self._center_on_target(mavdrone, node, yolo_detector)

            self._stop_camera_stream()

            if success:
                yasmin.YASMIN_LOG_INFO("- Successfully centered on landing base")
                return SUCCEED
            else:
                yasmin.YASMIN_LOG_ERROR(" x Failed to center on landing base")
                return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Centering failed: {e}")
            self._stop_camera_stream()
            return ABORT

    def _start_camera_stream(self, node):
        self.centering_active = True
        self.image_handler = ImageHandler(
            node=node,
            image_source=CAMERA_SOURCE,
            image_processing_callback=self._process_frame,
            show_result=None,
        )
        self.image_handler.run()

    def _stop_camera_stream(self):
        """Stop camera stream."""
        self.centering_active = False
        if self.image_handler:
            self.image_handler.cleanup()
            self.image_handler = None

    def _process_frame(self, frame):
        """Process camera frame for detection."""
        if self.centering_active:
            self.current_frame = frame.copy()

    def _center_on_target(self, mavdrone, node, yolo_detector):
        """Center drone on detected target using P controller."""
        start_time = time.time()

        while time.time() - start_time < CENTERING_TIMEOUT:
            if self.current_frame is None:
                time.sleep(0.1)
                continue

            detections = yolo_detector.detect_landing_bases(
                self.current_frame, save_image=False
            )

            if not detections:
                yasmin.YASMIN_LOG_WARN("Lost detection during centering")
                time.sleep(0.1)
                continue

            best_detection = yolo_detector.get_best_detection(detections)

            if yolo_detector.is_centered(best_detection, CENTERING_TOLERANCE_PX):
                yasmin.YASMIN_LOG_INFO("Target centered successfully")
                mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                return True

            error_x, error_y = yolo_detector.calculate_centering_error(best_detection)

            vel_x = -error_y * CENTERING_P_GAIN
            vel_y = -error_x * CENTERING_P_GAIN

            vel_x = max(-CENTERING_VELOCITY, min(CENTERING_VELOCITY, vel_x))
            vel_y = max(-CENTERING_VELOCITY, min(CENTERING_VELOCITY, vel_y))

            mavdrone.offboard_velocity(vel_x, vel_y, 0.0, 0.0, ground_reference=False)

            yasmin.YASMIN_LOG_DEBUG(
                f"Centering: error=({error_x}, {error_y})px, vel=({vel_x:.2f}, {vel_y:.2f})m/s"
            )

            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.05)

        yasmin.YASMIN_LOG_ERROR("Centering timeout reached")
        mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
        return False


class LandAndWait(State):
    """Land on detected base and wait before takeoff."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in LandAndWait state.")
            return ABORT

        mavdrone = blackboard["mavdrone"]
        node = YasminNode.get_instance()

        yasmin.YASMIN_LOG_INFO("Landing on detected base...")

        try:
            landing_position = {
                "x": mavdrone.get_local_pos.pose.position.x,
                "y": mavdrone.get_local_pos.pose.position.y,
                "z": mavdrone.get_local_pos.pose.position.z,
                "timestamp": time.time(),
            }

            visited_bases = blackboard.get("visited_bases", [])
            visited_bases.append(landing_position)
            blackboard["visited_bases"] = visited_bases

            mavdrone.land()

            yasmin.YASMIN_LOG_INFO(
                f"- Landed successfully! Waiting {LAND_WAIT_TIME} seconds..."
            )
            yasmin.YASMIN_LOG_INFO(f"Total bases visited: {len(visited_bases)}/6")

            time.sleep(LAND_WAIT_TIME)

            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Landing failed: {e}")
            return ABORT


class TakeoffAndReturn(State):
    """Takeoff from base and return to pre-centering position."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, TIMEOUT])

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in TakeoffAndReturn state.")
            return ABORT

        mavdrone = blackboard["mavdrone"]
        node = YasminNode.get_instance()
        pre_center_position = blackboard.get("pre_center_position")

        if not pre_center_position:
            yasmin.YASMIN_LOG_ERROR("Pre-centering position not available.")
            return ABORT

        yasmin.YASMIN_LOG_INFO("Taking off and returning to grid position...")

        # Get position controller
        position_controller = blackboard.get("position_controller")
        if not position_controller:
            yasmin.YASMIN_LOG_ERROR("Position controller not available.")
            return ABORT

        try:
            # Use height-aware takeoff to maintain altitude above original ground
            success = position_controller.takeoff_to_maintain_height()

            if not success:
                yasmin.YASMIN_LOG_ERROR("Height-aware takeoff failed")
                return ABORT

            yasmin.YASMIN_LOG_INFO("Takeoff completed. Ready to continue mission.")

            # Return to pre-centering position
            success = position_controller.goto_position(
                pre_center_position["x"],
                pre_center_position["y"],
                pre_center_position["z"],
                timeout=30.0,
            )

            if success:
                yasmin.YASMIN_LOG_INFO("- Successfully returned to grid position")
                return SUCCEED
            else:
                yasmin.YASMIN_LOG_ERROR("x Failed to return to grid position")
                return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Takeoff and return failed: {e}")
            return ABORT

    def _return_to_position(self, mavdrone, node, target_position):
        """Return to target position using velocity control."""
        start_time = time.time()

        while time.time() - start_time < 30:
            current_pos = mavdrone.get_local_pos.pose.position

            distance = math.sqrt(
                (target_position["x"] - current_pos.x) ** 2
                + (target_position["y"] - current_pos.y) ** 2
            )

            if distance < 0.3:
                mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                time.sleep(0.5)
                return True

            velocity_scale = min(0.5, distance * 0.3)
            vel_x = (target_position["x"] - current_pos.x) / distance * velocity_scale
            vel_y = (target_position["y"] - current_pos.y) / distance * velocity_scale

            mavdrone.offboard_velocity(vel_x, vel_y, 0.0, 0.0, ground_reference=False)

            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.1)

        return False
