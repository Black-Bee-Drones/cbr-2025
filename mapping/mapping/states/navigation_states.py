import rclpy
import time
import math
import cv2
import os
from typing import List, Dict, Tuple

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from mirela_sdk.image_processing.camera.image_handler import ImageHandler
from mirela_sdk.image_processing.camera import IMX219Config

from mapping.constants import (
    TAKEOFF_ALTITUDE,
    POSITION_TOLERANCE,
    SEARCH_TIMEOUT,
    CAMERA_SOURCE,
    DETECTION_SAVE_PATH,
    DUPLICATE_BASE_RADIUS,
    CAMERA_FOV_HORIZONTAL,
    CAMERA_FOV_VERTICAL,
    IMAGE_CENTER_X,
    IMAGE_CENTER_Y,
)
from mapping.utils import YOLODetector


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

        # Check if we need to return to current waypoint for more detections
        return_to_waypoint = blackboard["return_to_waypoint"]

        if return_to_waypoint:
            current_waypoint = blackboard["current_target_waypoint"]
            yasmin.YASMIN_LOG_INFO(
                f"Returning to waypoint {current_waypoint['index']} to check for additional bases"
            )
            target_waypoint = current_waypoint
            # Reset the flag - will be set again if more valid detections found
            blackboard["return_to_waypoint"] = False
        else:
            # Normal flow - advance to next waypoint
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
                x=target_waypoint["x"],
                y=target_waypoint["y"],
                z=TAKEOFF_ALTITUDE,
                precision_radius=POSITION_TOLERANCE,
                timeout_sec=SEARCH_TIMEOUT,
                strategy="PID",
                ground_reference=True,
                disable_altitude_control=True
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
            node=YasminNode.get_instance(),
            image_source=CAMERA_SOURCE,
            config=IMX219Config(sensor_id=0, width=1640, height=1232, flip=2),
        )

    @staticmethod
    def calculate_meters_per_pixel(altitude: float) -> Tuple[float, float]:
        """
        Calculate meters per pixel based on altitude and camera FOV.

        Args:
            altitude: Height above ground in meters

        Returns:
            Tuple of (meters_per_pixel_x, meters_per_pixel_y)
        """
        coverage_width = (
            2 * altitude * math.tan(math.radians(CAMERA_FOV_HORIZONTAL / 2))
        )
        coverage_height = 2 * altitude * math.tan(math.radians(CAMERA_FOV_VERTICAL / 2))
        meters_per_pixel_x = coverage_width / (2 * IMAGE_CENTER_X)
        meters_per_pixel_y = coverage_height / (2 * IMAGE_CENTER_Y)

        return meters_per_pixel_x, meters_per_pixel_y

    @staticmethod
    def estimate_detection_position(
        detection: Dict, drone_x: float, drone_y: float, altitude: float
    ) -> Tuple[float, float]:
        """
        Estimate world position of detection using simple pixel-to-meter conversion.

        Args:
            detection: Detection dict with 'center' key
            drone_x: Current drone X position
            drone_y: Current drone Y position
            altitude: Current altitude above ground

        Returns:
            Tuple of (estimated_x, estimated_y) in world coordinates
        """
        pixel_x, pixel_y = detection["center"]

        offset_x = pixel_x - IMAGE_CENTER_X
        offset_y = pixel_y - IMAGE_CENTER_Y

        meters_per_px_x, meters_per_px_y = CaptureAndDetect.calculate_meters_per_pixel(
            altitude
        )

        offset_meters_x = offset_y * meters_per_px_y
        offset_meters_y = -offset_x * meters_per_px_x  # Left/right

        estimated_x = drone_x + offset_meters_x
        estimated_y = drone_y + offset_meters_y

        return estimated_x, estimated_y

    @staticmethod
    def is_duplicate_detection(
        estimated_pos: Tuple[float, float], visited_bases: List[Dict]
    ) -> bool:
        """
        Check if detection is near a visited base.

        Args:
            estimated_pos: (x, y) estimated position of detection
            visited_bases: List of visited base positions

        Returns:
            True if detection is a duplicate, False otherwise
        """
        est_x, est_y = estimated_pos

        for base in visited_bases:
            dx = abs(est_x - base["x"])
            dy = abs(est_y - base["y"])

            if dx < DUPLICATE_BASE_RADIUS and dy < DUPLICATE_BASE_RADIUS:
                yasmin.YASMIN_LOG_INFO(
                    f"  Detection at ({est_x:.2f}, {est_y:.2f}) is duplicate "
                    f"(dx={dx:.2f}m, dy={dy:.2f}m from visited base)"
                )
                return True

        return False

    @staticmethod
    def order_detections(
        detections: List[Dict], moving_forward: bool = True
    ) -> List[Dict]:
        """
        Order detections by Y-axis (bottom-first if forward, top-first if backward)
        and then by X-axis (left to right).

        Args:
            detections: List of detection dicts
            moving_forward: True if drone is moving forward in grid

        Returns:
            Ordered list of detections
        """
        # Sort by Y first (reverse if moving forward - larger Y = bottom of image = closer)
        # (left to right)
        return sorted(
            detections,
            key=lambda d: (
                -d["center"][1] if moving_forward else d["center"][1],  # Y-axis
                d["center"][0],  # X-axis
            ),
        )

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
                altitude = mavdrone.get_rng_alt.range
                drone_x, drone_y = current_pos.x, current_pos.y

                visited_bases = blackboard["visited_bases"]

                # Determine movement direction (forward if positive X movement in grid)
                current_waypoint = blackboard["current_target_waypoint"]
                moving_forward = True  # Default
                if current_waypoint and "direction" in current_waypoint:
                    moving_forward = current_waypoint["direction"] in [
                        "FORWARD",
                        "RIGHT",
                    ]

                ordered_detections = self.order_detections(
                    all_detections, moving_forward
                )

                yasmin.YASMIN_LOG_INFO(
                    f"Drone position: ({drone_x:.2f}, {drone_y:.2f}), altitude: {altitude:.2f}m"
                )
                yasmin.YASMIN_LOG_INFO(
                    f"Processing {len(ordered_detections)} ordered detections..."
                )

                # Filter detections and find valid ones
                valid_detections = []
                for detection in ordered_detections:
                    estimated_pos = self.estimate_detection_position(
                        detection, drone_x, drone_y, altitude
                    )

                    if not self.is_duplicate_detection(estimated_pos, visited_bases):
                        detection["estimated_position"] = estimated_pos
                        valid_detections.append(detection)
                        yasmin.YASMIN_LOG_INFO(
                            f"  Valid detection at pixel ({detection['center'][0]}, {detection['center'][1]}), "
                            f"estimated position ({estimated_pos[0]:.2f}, {estimated_pos[1]:.2f})"
                        )

                if valid_detections:
                    blackboard["current_detection"] = valid_detections[0]
                    blackboard["detection_image"] = frame

                    # Set flag to return to waypoint if there are multiple valid detections
                    # When we return, we'll run detection again and the just-visited base
                    # will be filtered out as duplicate
                    blackboard["return_to_waypoint"] = len(valid_detections) > 1

                    yasmin.YASMIN_LOG_INFO(
                        f"Found {len(valid_detections)} valid detection(s)"
                    )
                    if len(valid_detections) > 1:
                        yasmin.YASMIN_LOG_INFO(
                            "Multiple bases detected - will return to waypoint after landing"
                        )

                    first_detection = valid_detections[0]
                    est_x, est_y = first_detection["estimated_position"]
                    yasmin.YASMIN_LOG_INFO(
                        f"- Processing detection at ({est_x:.2f}, {est_y:.2f})"
                    )

                    yasmin.YASMIN_LOG_INFO(
                        f"Pre-centering on detection at ({est_x:.2f}, {est_y:.2f})"
                    )
                    mavdrone.offboard_position(
                        x=est_x,
                        y=est_y,
                        z=TAKEOFF_ALTITUDE,
                        ground_reference=True,
                        precision_radius=0.16,
                        timeout_sec=10.0,
                        strategy="PID",
                        disable_altitude_control=True
                    )
                    yasmin.YASMIN_LOG_INFO("Pre-centering complete")

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
