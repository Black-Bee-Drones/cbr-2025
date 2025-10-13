import rclpy
import time
import math

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, TIMEOUT
from yasmin_ros.yasmin_node import YasminNode

from mirela_sdk.image_processing.camera.image_handler import ImageHandler
from mirela_sdk.image_processing.camera import IMX219Config

from mapping.utils import YOLODetector
from mapping.constants import (
    CENTERING_P_GAIN,
    CENTERING_TIMEOUT,
    CENTERING_VEL_MAX,
    CENTERING_VEL_MIN,
    CENTERING_ALTITUDE,
    LAND_WAIT_TIME,
    CAMERA_SOURCE,
    CENTER_DETECTION_THRESHOLD,
    ALTITUDE_TOLERANCE,
    DESCEND_KP,
    IMAGE_CENTER_X,
    IMAGE_CENTER_Y,
    IMAGE_OFFSET_Y,
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
            config=IMX219Config(sensor_id=0, width=1640, height=1232, flip=2),
        )

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR(
                "MavDrone not available in CenterOnDetection state."
            )
            return ABORT

        self.mavdrone = blackboard["mavdrone"]
        self.yolo_detector: YOLODetector = blackboard["yolo_detector"]
        current_detection = blackboard["current_detection"]

        if not self.yolo_detector or not current_detection:
            yasmin.YASMIN_LOG_ERROR("YOLO detector or detection not available.")
            return ABORT

        yasmin.YASMIN_LOG_INFO(
            f"Starting centering after pre-positioning. "
            f"Initial detection at pixel ({current_detection['center'][0]}, {current_detection['center'][1]}), "
            f"confidence={current_detection['confidence']:.2f}"
        )

        try:
            self.image_handler.open()
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Failed to open camera: {e}")
            self.image_handler.close()
            return ABORT

        yasmin.YASMIN_LOG_INFO(
            "Starting two-phase centering on detected landing base..."
        )

        try:
            yasmin.YASMIN_LOG_INFO("Phase 1: Centering at current altitude...")
            phase1_success = self._phase1_center_at_altitude()

            if not phase1_success:
                yasmin.YASMIN_LOG_ERROR("Phase 1 centering failed")
                self.image_handler.close()
                return ABORT

            yasmin.YASMIN_LOG_INFO("Phase 2: Descending to landing altitude...")
            phase2_success = self._phase2_descend_and_center()

            if phase2_success:
                yasmin.YASMIN_LOG_INFO("Successfully centered and ready to land")
                return SUCCEED
            else:
                yasmin.YASMIN_LOG_ERROR("Phase 2 centering failed")
                self.image_handler.close()
                return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Centering failed: {e}")
            self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
            return ABORT

    @staticmethod
    def saturate_abs(value):
        if value == 0:
            return 0.0
        return max(CENTERING_VEL_MIN, min(CENTERING_VEL_MAX, abs(value))) * (
            1 if value > 0 else -1
        )

    def _get_closest_to_center(self, all_detections):
        """
        Get detection closest to image center.
        Since we pre-centered in CaptureAndDetect, the correct base should be near center.

        Args:
            all_detections: List of all current detections from YOLO

        Returns:
            Detection dict closest to center or None
        """
        if not all_detections:
            return None

        min_distance = float("inf")
        best_detection = None

        for detection in all_detections:
            center_x, center_y = detection["center"]
            error_x = center_x - IMAGE_CENTER_X
            error_y = center_y - IMAGE_CENTER_Y
            distance = math.sqrt(error_x**2 + error_y**2)

            if distance < min_distance:
                min_distance = distance
                best_detection = detection

        if best_detection:
            yasmin.YASMIN_LOG_DEBUG(
                f"Selected detection at {best_detection['center']} "
                f"(distance from center: {min_distance:.1f}px)"
            )

        return best_detection

    def _phase1_center_at_altitude(self) -> bool:
        """Phase 1: Center on target while maintaining current altitude."""
        start_time = time.time()
        centered = False
        lost_detections = 0

        while time.time() - start_time < CENTERING_TIMEOUT:
            all_detections = self.yolo_detector.detect(
                self.image_handler.take_photo(), save_image=False, return_all=True
            )

            detection = self._get_closest_to_center(all_detections)

            if not detection:
                lost_detections += 1
                if lost_detections > 150:
                    self.image_handler.close()
                    yasmin.YASMIN_LOG_ERROR(
                        "Lost target detection too many times in Phase 1"
                    )
                    break
                continue

            lost_detections = 0

            error_x, error_y = self.yolo_detector.calculate_centering_error(detection)

            if (
                abs(error_x) < CENTER_DETECTION_THRESHOLD
                and abs(error_y) < CENTER_DETECTION_THRESHOLD
            ):
                yasmin.YASMIN_LOG_INFO("Phase 1: Target centered")
                self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                centered = True
                break

            vel_x = self.saturate_abs(error_y * CENTERING_P_GAIN)
            vel_y = self.saturate_abs(error_x * CENTERING_P_GAIN)

            self.mavdrone.offboard_velocity(
                vel_x, vel_y, 0.0, 0.0, ground_reference=False
            )

            yasmin.YASMIN_LOG_INFO(
                f"Phase 1: error=({error_x:.0f},{error_y:.0f})px, vel=({vel_x:.2f},{vel_y:.2f})m/s"
            )

        self.image_handler.close()
        self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
        return centered

    def _phase2_descend_and_center(self) -> bool:
        """Phase 2: Descend to landing altitude while maintaining center."""
        self.image_handler.open()
        
        start_time = time.time()
        phase2_threshold = 100

        yasmin.YASMIN_LOG_INFO(
            f"Descending to {CENTERING_ALTITUDE:.1f}m above figure..."
        )

        while time.time() - start_time < CENTERING_TIMEOUT:
            rclpy.spin_once(self.node, timeout_sec=0.01)

            current_lidar = self.mavdrone.get_rng_alt.range
            error_z = current_lidar - CENTERING_ALTITUDE

            if current_lidar < CENTERING_ALTITUDE + 0.40:
                phase2_threshold = 40

            if error_z < ALTITUDE_TOLERANCE:
                yasmin.YASMIN_LOG_INFO("Phase 2: Landing altitude reached")
                self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                time.sleep(1)
                self.image_handler.close()
                self.mavdrone.delay(1.5)
                return True

            all_detections = self.yolo_detector.detect(
                self.image_handler.take_photo(), save_image=False, return_all=True
            )
            detection = self._get_closest_to_center(all_detections)

            vel_x, vel_y = 0.0, 0.0
            error_x, error_y = 0, 0

            if detection:
                error_x, error_y = self.yolo_detector.calculate_centering_error(
                    detection
                )
                vel_x = error_y * CENTERING_P_GAIN * 0.7  # Reduced gain during descent
                vel_y = error_x * CENTERING_P_GAIN * 0.7
                vel_x = (
                    self.saturate_abs(vel_x) if abs(error_y) > phase2_threshold else 0.0
                )
                vel_y = (
                    self.saturate_abs(vel_y) if abs(error_x) > phase2_threshold else 0.0
                )

            vel_z = -DESCEND_KP * error_z
            vel_z = max(0.05, min(0.3, abs(vel_z))) * (1 if vel_z > 0 else -1)

            yasmin.YASMIN_LOG_INFO(
                f"Current Lidar: {current_lidar}; Value: {-DESCEND_KP * error_z}"
            )
            self.mavdrone.offboard_velocity(
                vel_x, vel_y, vel_z, 0.0, ground_reference=False
            )

            yasmin.YASMIN_LOG_INFO(
                f"Phase 2: alt={current_lidar:.2f}m, error=({error_x},{error_y},{error_z}) vel=({vel_x:.2f},{vel_y:.2f},{vel_z:.2f})m/s"
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

            # # Use the estimated position from detection
            # current_detection = blackboard["current_detection"]
            # if current_detection and "estimated_position" in current_detection:
            #     est_x, est_y = current_detection["estimated_position"]
            #     landing_position = {
            #         "x": est_x,
            #         "y": est_y,
            #         "z": mavdrone.get_vision_pos.pose.pose.position.z,
            #         "timestamp": time.time(),
            #     }
            #     yasmin.YASMIN_LOG_INFO(
            #         f"Saving base position: ({est_x:.2f}, {est_y:.2f})"
            #     )
            # else:
            landing_position = {
                "x": mavdrone.get_vision_pos.pose.pose.position.x,
                "y": mavdrone.get_vision_pos.pose.pose.position.y,
                "z": mavdrone.get_vision_pos.pose.pose.position.z,
                "timestamp": time.time(),
            }
            yasmin.YASMIN_LOG_INFO(
                f"Using drone position: ({landing_position['x']:.2f}, {landing_position['y']:.2f})"
            )

            visited_bases = blackboard["visited_bases"]
            visited_bases.append(landing_position)
            blackboard["visited_bases"] = visited_bases

            yasmin.YASMIN_LOG_INFO(
                f"Base {len(visited_bases)} landed at ({landing_position['x']:.2f}, {landing_position['y']:.2f})"
            )

            yasmin.YASMIN_LOG_INFO(
                f"- Landed successfully! Waiting {LAND_WAIT_TIME} seconds..."
            )
            yasmin.YASMIN_LOG_INFO(f"Total bases visited: {len(visited_bases)}/6")

            time.sleep(LAND_WAIT_TIME)

            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Landing failed: {e}")
            return ABORT
