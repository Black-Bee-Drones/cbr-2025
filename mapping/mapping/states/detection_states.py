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

from mapping.utils import YOLODetector, PositionController
from mapping.constants import (
    CENTERING_P_GAIN,
    CENTERING_TOLERANCE_PX,
    CENTERING_TIMEOUT,
    CENTERING_VELOCITY,
    CENTERING_ALTITUDE,
    LAND_WAIT_TIME,
    CAMERA_SOURCE,
    TAKEOFF_TIMEOUT,
    SEARCH_ALTITUDE,
    ALTITUDE_TOLERANCE,
    ALTITUDE_COMPENSATION_GAIN,
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
        self.yolo_detector: YOLODetector = blackboard.get("yolo_detector")
        current_detection = blackboard.get("current_detection")
        ground_reference = blackboard.get("ground_reference_altitude", 0.0)
        target_search_altitude = blackboard.get("target_search_altitude", SEARCH_ALTITUDE)

        if not self.yolo_detector or not current_detection:
            yasmin.YASMIN_LOG_ERROR("YOLO detector or detection not available.")
            return ABORT

        yasmin.YASMIN_LOG_INFO("Starting two-phase centering on detected landing base...")

        # Store pre-centering position
        pre_center_position = {
            "x": self.mavdrone.get_local_pos.pose.position.x,
            "y": self.mavdrone.get_local_pos.pose.position.y,
            "z": self.mavdrone.get_local_pos.pose.position.z,
        }
        blackboard["pre_center_position"] = pre_center_position

        try:
            # PHASE 1: Center at search altitude maintaining ground reference
            yasmin.YASMIN_LOG_INFO("Phase 1: Centering at search altitude...")
            phase1_success = self._phase1_center_at_altitude(
                target_search_altitude, ground_reference
            )
            
            if not phase1_success:
                yasmin.YASMIN_LOG_ERROR("Phase 1 centering failed")
                return ABORT

            # Measure figure altitude after centering
            current_lidar_reading = self.mavdrone.get_rng_alt.data
            figure_altitude = target_search_altitude - current_lidar_reading
            blackboard["figure_altitude"] = figure_altitude
            yasmin.YASMIN_LOG_INFO(f"Figure detected at altitude: {figure_altitude:.2f}m above ground")

            # PHASE 2: Descend while maintaining center to landing altitude
            yasmin.YASMIN_LOG_INFO("Phase 2: Descending to landing altitude...")
            phase2_success = self._phase2_descend_and_center(figure_altitude)
            
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

    def _phase1_center_at_altitude(self, target_altitude: float, ground_reference: float) -> bool:
        """Phase 1: Center on target while maintaining search altitude above ground."""
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

            current_lidar = self.mavdrone.get_rng_alt.data
            altitude_error = (target_altitude - ground_reference) - current_lidar
            vel_z = altitude_error * ALTITUDE_COMPENSATION_GAIN
            vel_z = max(-0.2, min(0.2, vel_z))

            self.mavdrone.offboard_velocity(vel_x, vel_y, vel_z, 0.0, ground_reference=False)
            
            yasmin.YASMIN_LOG_DEBUG(
                f"Phase 1: error=({error_x:.0f},{error_y:.0f})px, vel=({vel_x:.2f},{vel_y:.2f},{vel_z:.2f})m/s"
            )

        self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
        return centered

    def _phase2_descend_and_center(self, figure_altitude: float) -> bool:
        """Phase 2: Descend to landing altitude while maintaining center."""
        target_landing_altitude = figure_altitude + CENTERING_ALTITUDE
        start_time = time.time()
        
        yasmin.YASMIN_LOG_INFO(f"Descending to {CENTERING_ALTITUDE:.1f}m above figure...")

        while time.time() - start_time < CENTERING_TIMEOUT:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            
            current_lidar = self.mavdrone.get_rng_alt.data
            
            if abs(current_lidar - CENTERING_ALTITUDE) < 0.1:
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

            altitude_error = current_lidar - CENTERING_ALTITUDE
            vel_z = -min(0.3, max(0.1, altitude_error * 0.5))  

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
            time.sleep(5)  # Wait for landing to complete

            landing_position = {
                "x": mavdrone.get_local_pos.pose.position.x,
                "y": mavdrone.get_local_pos.pose.position.y,
                "z": mavdrone.get_local_pos.pose.position.z,
                "timestamp": time.time(),
            }
            visited_bases = blackboard.get("visited_bases", [])
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


class TakeoffAndReturn(State):
    """Takeoff from base and return to pre-centering position at search altitude."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, TIMEOUT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in TakeoffAndReturn state.")
            return ABORT

        mavdrone = blackboard["mavdrone"]
        pre_center_position = blackboard.get("pre_center_position")
        figure_altitude = blackboard.get("figure_altitude", 0.0)
        ground_reference = blackboard.get("ground_reference_altitude", 0.0)
        target_search_altitude = blackboard.get("target_search_altitude", SEARCH_ALTITUDE)

        if not pre_center_position:
            yasmin.YASMIN_LOG_ERROR("Pre-centering position not available.")
            return ABORT

        # Calculate required takeoff altitude to reach search altitude above ground
        # We're currently on a figure at 'figure_altitude' above ground
        # We need to reach 'target_search_altitude' above ground
        required_takeoff_altitude = target_search_altitude - figure_altitude
        
        yasmin.YASMIN_LOG_INFO(
            f"Taking off from figure at {figure_altitude:.1f}m to reach {SEARCH_ALTITUDE:.1f}m above ground"
        )
        yasmin.YASMIN_LOG_INFO(f"Required takeoff altitude: {required_takeoff_altitude:.1f}m")

        position_controller: PositionController = blackboard.get("position_controller")
        if not position_controller:
            yasmin.YASMIN_LOG_ERROR("Position controller not available.")
            return ABORT

        try:
            safe_takeoff_altitude = max(0.5, required_takeoff_altitude)
            mavdrone.arm_takeoff(safe_takeoff_altitude)
            time.sleep(3)

            start_time = time.time()
            while time.time() - start_time < TAKEOFF_TIMEOUT:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                current_alt = mavdrone.get_rng_alt.data
                
                if abs(current_alt - safe_takeoff_altitude) < ALTITUDE_TOLERANCE:
                    yasmin.YASMIN_LOG_INFO(f"Takeoff complete at {current_alt:.2f}m above figure")
                    mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                    time.sleep(2)
                    break

                altitude_error = safe_takeoff_altitude - current_alt
                correction_velocity = max(-0.5, min(0.5, 0.3 * altitude_error))
                mavdrone.offboard_velocity(0.0, 0.0, correction_velocity, 0.0)
                time.sleep(0.1)

            yasmin.YASMIN_LOG_INFO("Returning to grid position...")
            success = position_controller.goto_position_ground_relative(
                pre_center_position["x"],
                pre_center_position["y"],
                target_search_altitude,
                ground_reference,
                timeout=30.0
            )

            if success:
                yasmin.YASMIN_LOG_INFO("Successfully returned to grid position at search altitude")
                return SUCCEED
            else:
                yasmin.YASMIN_LOG_ERROR("Failed to return to grid position")
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
