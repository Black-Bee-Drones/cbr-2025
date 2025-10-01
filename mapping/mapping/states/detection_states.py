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

from mapping.utils import YOLODetector
from mapping.constants import (
    CENTERING_P_GAIN,
    CENTERING_TOLERANCE_PX,
    CENTERING_TIMEOUT,
    CENTERING_VELOCITY,
    CENTERING_ALTITUDE,
    LAND_WAIT_TIME,
    CAMERA_SOURCE,
)


class CenterOnDetection(State):
    """Two-phase centering: Phase 1 at search altitude, Phase 2 descending to landing altitude."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, TIMEOUT])
        self.image_handler = None
        self.node = YasminNode.get_instance()
        self.image_handler = ImageHandler(
            node=self.node,
            image_source=CAMERA_SOURCE,
        )

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in CenterOnDetection state.")
            return ABORT

        self.mavdrone = blackboard["mavdrone"]
        self.yolo_detector: YOLODetector = blackboard["yolo_detector"]
        current_detection = blackboard["current_detection"]

        if not self.yolo_detector or not current_detection:
            yasmin.YASMIN_LOG_ERROR("YOLO detector or detection not available.")
            return ABORT

        yasmin.YASMIN_LOG_INFO("Starting two-phase centering on detected landing base...")

        try:
            yasmin.YASMIN_LOG_INFO("Phase 1: Centering at current altitude...")
            phase1_success = self._phase1_center_at_altitude()
            
            if not phase1_success:
                yasmin.YASMIN_LOG_ERROR("Phase 1 centering failed")
                return ABORT

            yasmin.YASMIN_LOG_INFO("Phase 2: Descending to landing altitude...")
            phase2_success = self._phase2_descend_and_center()
            
            if phase2_success:
                yasmin.YASMIN_LOG_INFO("Successfully centered and ready to land")
                return SUCCEED
            else:
                yasmin.YASMIN_LOG_ERROR("Phase 2 centering failed")
                return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Centering failed: {e}")
            self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
            return ABORT

    def _phase1_center_at_altitude(self) -> bool:
        """Phase 1: Center on target while maintaining current altitude."""
        start_time = time.time()
        centered = False
        lost_detections = 0

        while time.time() - start_time < CENTERING_TIMEOUT:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            
            detection = self.yolo_detector.detect(
                self.image_handler.take_photo(), save_image=False
            )

            if not detection:
                lost_detections += 1
                if lost_detections > 10:
                    yasmin.YASMIN_LOG_ERROR("Lost detection too many times in Phase 1")
                    break
                continue

            if self.yolo_detector.is_centered(detection, CENTERING_TOLERANCE_PX):
                yasmin.YASMIN_LOG_INFO("Phase 1: Target centered")
                self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                centered = True
                break

            error_x, error_y = self.yolo_detector.calculate_centering_error(detection)
            vel_x = -error_y * CENTERING_P_GAIN
            vel_y = -error_x * CENTERING_P_GAIN
            vel_x = max(-CENTERING_VELOCITY, min(CENTERING_VELOCITY, vel_x))
            vel_y = max(-CENTERING_VELOCITY, min(CENTERING_VELOCITY, vel_y))

            self.mavdrone.offboard_velocity(vel_x, vel_y, 0.0, 0.0, ground_reference=False)
            
            yasmin.YASMIN_LOG_DEBUG(
                f"Phase 1: error=({error_x:.0f},{error_y:.0f})px, vel=({vel_x:.2f},{vel_y:.2f})m/s"
            )

        self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
        return centered

    def _phase2_descend_and_center(self) -> bool:
        """Phase 2: Descend to landing altitude while maintaining center."""
        start_time = time.time()
        
        yasmin.YASMIN_LOG_INFO(f"Descending to {CENTERING_ALTITUDE:.1f}m above figure...")

        while time.time() - start_time < CENTERING_TIMEOUT:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            
            current_lidar = self.mavdrone.get_rng_alt.range
            
            if current_lidar < CENTERING_ALTITUDE + 0.2:
                yasmin.YASMIN_LOG_INFO("Phase 2: Landing altitude reached")
                self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                time.sleep(1)
                return True

            detection = self.yolo_detector.detect(
                self.image_handler.take_photo(), save_image=False
            )

            vel_x, vel_y = 0.0, 0.0
            if detection:
                error_x, error_y = self.yolo_detector.calculate_centering_error(detection)
                vel_x = -error_y * CENTERING_P_GAIN * 0.7  # Reduced gain during descent
                vel_y = -error_x * CENTERING_P_GAIN * 0.7
                vel_x = max(-0.2, min(0.2, vel_x))
                vel_y = max(-0.2, min(0.2, vel_y))

            vel_z = -0.2 
            self.mavdrone.offboard_velocity(vel_x, vel_y, vel_z, 0.0, ground_reference=False)
            
            yasmin.YASMIN_LOG_DEBUG(
                f"Phase 2: alt={current_lidar:.2f}m, vel=({vel_x:.2f},{vel_y:.2f},{vel_z:.2f})m/s"
            )

        self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
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

        yasmin.YASMIN_LOG_INFO("Landing on detected base...")

        try:
            mavdrone.land()
            time.sleep(8) 
            
            rclpy.spin_once(YasminNode.get_instance(), timeout_sec=0.1)
            landing_position = {
                "x": mavdrone.get_visual_pos.pose.pose.position.x,
                "y": mavdrone.get_visual_pos.pose.pose.position.y,
                "z": mavdrone.get_visual_pos.pose.pose.position.z,
                "timestamp": time.time(),
            }
            visited_bases = blackboard["visited_bases"]
            visited_bases.append(landing_position)
            blackboard["visited_bases"] = visited_bases

            yasmin.YASMIN_LOG_INFO(
                f"- Landed successfully! Waiting {LAND_WAIT_TIME} seconds..."
            )
            yasmin.YASMIN_LOG_INFO(f"Total bases visited: {len(visited_bases)}/6")

            time.sleep(LAND_WAIT_TIME)

            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Landing failed: {e}")
            return ABORT
