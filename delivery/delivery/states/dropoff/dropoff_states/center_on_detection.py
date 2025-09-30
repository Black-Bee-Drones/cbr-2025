import yasmin
import time
import rclpy

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode
from delivery.utils.yolo_detection import YOLODetector
from mirela_sdk.control.mavros.mavros_api import MavDrone

from delivery.constants import (
    CENTER_TIMEOUT,
    CENTERING_TOLERANCE_PX,
    CENTERING_P_GAIN,
    CENTERING_VELOCITY,
    ALTITUDE_COMPENSATION_GAIN,
    CENTERING_ALTITUDE,
    TAKEOFF_ALTITUDE
)


# REVER OUTCOMES
class CenterOnDetection(State):
    """Two-phase centering: Phase 1 at search altitude, Phase 2 descending to landing altitude."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.image_handler = None
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        self.image_handler = blackboard.get("image_handler")
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in CenterOnDetection state.")
            return ABORT

        self.mavdrone : MavDrone = blackboard["mavdrone"]
        self.yolo_deliver_detector: YOLODetector = blackboard.get("yolo_detector")
        current_detection = blackboard.get("current_detection")
        ground_reference = blackboard.get("ground_reference_altitude", 0.0)
        #target_search_altitude = blackboard.get("target_search_altitude", SEARCH_ALTITUDE)
        target_search_altitude = TAKEOFF_ALTITUDE

        if not self.yolo_deliver_detector or not current_detection:
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
            current_lidar_reading = self.mavdrone.get_rng_alt.range
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

        while time.time() - start_time < CENTER_TIMEOUT:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            
            detection = self.yolo_deliver_detector.detect(
                self.image_handler.take_photo(), save_image=False
            )

            if not detection:
                lost_detections += 1
                if lost_detections > 10:
                    yasmin.YASMIN_LOG_ERROR("Lost detection too many times in Phase 1")
                    break
                continue

            if self.yolo_deliver_detector.is_centered(detection, CENTERING_TOLERANCE_PX):
                yasmin.YASMIN_LOG_INFO("Phase 1: Target centered")
                self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                centered = True
                break

            error_x, error_y = self.yolo_deliver_detector.calculate_centering_error(detection)
            vel_x = -error_y * CENTERING_P_GAIN
            vel_y = -error_x * CENTERING_P_GAIN
            vel_x = max(-CENTERING_VELOCITY, min(CENTERING_VELOCITY, vel_x))
            vel_y = max(-CENTERING_VELOCITY, min(CENTERING_VELOCITY, vel_y))

            current_lidar = self.mavdrone.get_rng_alt.range
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

        while time.time() - start_time < CENTER_TIMEOUT:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            
            current_lidar = self.mavdrone.get_rng_alt.data
            
            if abs(current_lidar - CENTERING_ALTITUDE) < 0.1:
                yasmin.YASMIN_LOG_INFO("Phase 2: Landing altitude reached")
                self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                time.sleep(1)
                return True

            detection = self.yolo_deliver_detector.detect(
                self.image_handler.take_photo(), save_image=False
            )

            vel_x, vel_y = 0.0, 0.0
            if detection:
                error_x, error_y = self.yolo_deliver_detector.calculate_centering_error(detection)
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
