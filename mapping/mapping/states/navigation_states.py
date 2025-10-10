import rclpy
import time
import math
import cv2
import os

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, TIMEOUT
from yasmin_ros.yasmin_node import YasminNode

from mirela_sdk.image_processing.camera.image_handler import ImageHandler

from mapping.constants import (
    POSITION_TOLERANCE,
    SEARCH_TIMEOUT,
    CAMERA_SOURCE,
    DETECTION_SAVE_PATH,
    DUPLICATE_BASE_RADIUS,
)
from mapping.utils import YOLODetector, DetectionPositionCalculator


class NavigateToWaypoint(State):
    """Navigate drone to next grid waypoint and mark previous as visited."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, "ALL_COMPLETE", ABORT])

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR(
                "MavDrone not available in NavigateToWaypoint state."
            )
            return ABORT

        mavdrone = blackboard["mavdrone"]
        grid_waypoints = blackboard["grid_waypoints"]
        if not grid_waypoints:
            yasmin.YASMIN_LOG_ERROR("Grid waypoints not available.")
            return ABORT

        # Mark previous waypoint as visited if exists
        current_waypoint = blackboard["current_target_waypoint"]
        if current_waypoint:
            grid_waypoints.mark_waypoint_visited(current_waypoint["index"])
            progress = grid_waypoints.get_progress()
            yasmin.YASMIN_LOG_INFO(
                f"Waypoint {current_waypoint['index']} completed. Progress: {progress['progress_percent']:.1f}%"
            )

        # Check if mission complete
        visited_bases = blackboard["visited_bases"]
        if len(visited_bases) >= 6:
            yasmin.YASMIN_LOG_INFO("All 6 landing bases visited! Mission complete.")
            return "ALL_COMPLETE"

        target_waypoint = grid_waypoints.get_next_waypoint()
        if not target_waypoint:
            yasmin.YASMIN_LOG_INFO("All waypoints completed!")
            return "ALL_COMPLETE"

        yasmin.YASMIN_LOG_INFO(
            f"Navigating to waypoint {target_waypoint['index']} at ({target_waypoint['x']:.1f}, {target_waypoint['y']:.1f})"
        )
        blackboard["current_target_waypoint"] = target_waypoint
        grid_waypoints.advance_to_next()

        try:
            rclpy.spin_once(YasminNode.get_instance(), timeout_sec=0.1)
            current_pos = mavdrone.get_vision_pos.pose.pose.position

            yasmin.YASMIN_LOG_INFO(f"Current pos: {current_pos}")
            yasmin.YASMIN_LOG_INFO(
                f"Target: (x: {target_waypoint['x']}, y: {target_waypoint['y']})"
            )

            mavdrone.offboard_position(
                x=target_waypoint["x"] - current_pos.x,
                y=target_waypoint["y"] - current_pos.y,
                z=0.0,
                precision_radius=POSITION_TOLERANCE,
                timeout_sec=SEARCH_TIMEOUT,
                strategy="PID",
            )

            yasmin.YASMIN_LOG_INFO("Waypoint reached successfully")

            mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
            time.sleep(0.5)

            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Navigation failed: {e}")
            return ABORT


class CaptureAndDetect(State):
    """Capture image at waypoint and run YOLO detection."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, "DETECTION_FOUND", ABORT])
        self.image_handler = ImageHandler(
            node=YasminNode.get_instance(), image_source=CAMERA_SOURCE
        )
        self.position_calculator = DetectionPositionCalculator()

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in CaptureAndDetect state.")
            return ABORT

        yolo_detector: YOLODetector = blackboard["yolo_detector"]
        if not yolo_detector:
            yasmin.YASMIN_LOG_ERROR("YOLO detector not available.")
            return ABORT

        current_waypoint = blackboard["current_target_waypoint"]
        if not current_waypoint:
            yasmin.YASMIN_LOG_ERROR("Current waypoint not available.")
            return ABORT

        try:
            self.image_handler.open()
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Failed to open camera: {e}")
            return ABORT

        yasmin.YASMIN_LOG_INFO(
            f"Capturing image and detecting at waypoint {current_waypoint['index']}"
        )

        os.makedirs(DETECTION_SAVE_PATH, exist_ok=True)

        try:
            frame = self.image_handler.take_photo()
            self.image_handler.close()

            if frame is None:
                yasmin.YASMIN_LOG_ERROR("Failed to capture image")
                return ABORT

            timestamp = int(time.time() * 1000)
            image_path = f"{DETECTION_SAVE_PATH}/waypoint_{current_waypoint['index']:03d}_{timestamp}.jpg"
            cv2.imwrite(image_path, frame)

            # Get ALL detections from YOLO
            all_detections = yolo_detector.detect(
                frame, save_image=True, timestamp=timestamp, return_all=True
            )

            if all_detections:
                yasmin.YASMIN_LOG_INFO(
                    f"Found {len(all_detections)} detection(s) in image"
                )

                mavdrone = blackboard["mavdrone"]
                rclpy.spin_once(YasminNode.get_instance(), timeout_sec=0.1)

                current_pos = mavdrone.get_vision_pos.pose.pose.position
                current_orientation = mavdrone.get_vision_pos.pose.pose.orientation
                altitude = mavdrone.get_rng_alt.range

                drone_position = (current_pos.x, current_pos.y, current_pos.z)
                drone_orientation = (
                    current_orientation.x,
                    current_orientation.y,
                    current_orientation.z,
                    current_orientation.w,
                )

                visited_bases = blackboard["visited_bases"]

                yasmin.YASMIN_LOG_INFO(
                    f"Filtering detections using world position calculation..."
                )
                yasmin.YASMIN_LOG_INFO(
                    f"Drone position: ({drone_position[0]:.2f}, {drone_position[1]:.2f}), "
                    f"altitude: {altitude:.2f}m"
                )

                best_detection = (
                    self.position_calculator.filter_and_select_best_detection(
                        detections=all_detections,
                        drone_position=drone_position,
                        drone_orientation_quaternion=drone_orientation,
                        altitude=altitude,
                        visited_bases=visited_bases,
                        duplicate_radius=DUPLICATE_BASE_RADIUS,
                    )
                )

                if best_detection:
                    world_x, world_y = best_detection["world_position"]

                    blackboard["current_detection"] = best_detection
                    blackboard["detection_image"] = frame

                    yasmin.YASMIN_LOG_INFO(
                        f"- NEW landing base detected at world position "
                        f"({world_x:.2f}, {world_y:.2f})!"
                    )
                    yasmin.YASMIN_LOG_INFO(
                        f"  Confidence: {best_detection['confidence']:.2f}, "
                        f"Distance to drone: {best_detection['distance_to_drone']:.2f}m"
                    )
                    return "DETECTION_FOUND"
                else:
                    yasmin.YASMIN_LOG_INFO(
                        "All detections filtered as duplicates, continuing search"
                    )
                    return SUCCEED
            else:
                yasmin.YASMIN_LOG_INFO("No landing base detected at this waypoint")
                return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Capture and detect failed: {e}")
            import traceback

            yasmin.YASMIN_LOG_ERROR(traceback.format_exc())
            return ABORT
        finally:
            self.image_handler.close()
