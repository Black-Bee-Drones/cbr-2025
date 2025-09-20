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
)
from mapping.utils import PositionController, YOLODetector   


class NavigateToWaypoint(State):
    """Navigate drone to a specific grid waypoint."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, TIMEOUT])

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR(
                "MavDrone not available in NavigateToWaypoint state."
            )
            return ABORT

        mavdrone = blackboard["mavdrone"]
        grid_waypoints = blackboard.get("grid_waypoints")
        if not grid_waypoints:
            yasmin.YASMIN_LOG_ERROR("Grid waypoints not available.")
            return ABORT

        target_waypoint = grid_waypoints.get_next_waypoint()
        if not target_waypoint:
            yasmin.YASMIN_LOG_INFO("All waypoints completed!")
            return SUCCEED

        yasmin.YASMIN_LOG_INFO(
            f"Navigating to waypoint ({target_waypoint['x']:.1f}, {target_waypoint['y']:.1f})"
        )

        blackboard["current_target_waypoint"] = target_waypoint

        position_controller: PositionController = blackboard.get("position_controller")
        if not position_controller:
            yasmin.YASMIN_LOG_ERROR("Position controller not available.")
            return ABORT

        # Get target altitude (maintain constant height above ground)
        target_search_altitude = blackboard.get("target_search_altitude", SEARCH_ALTITUDE)

        try:
            # Navigate with ground-relative altitude control
            success = position_controller.goto_position_ground_relative(
                target_waypoint["x"], 
                target_waypoint["y"], 
                target_search_altitude,
                blackboard.get("ground_reference_altitude", 0.0),
                timeout=SEARCH_TIMEOUT
            )

            if success:
                yasmin.YASMIN_LOG_INFO("Waypoint reached successfully")
                return SUCCEED
            else:
                yasmin.YASMIN_LOG_ERROR("Failed to reach waypoint")
                return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Navigation failed: {e}")
            return ABORT


class CaptureAndDetect(State):
    """Capture image at waypoint and run YOLO detection."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, "DETECTION_FOUND", ABORT])
        self.image_handler = ImageHandler(node=YasminNode.get_instance(), image_source=CAMERA_SOURCE)


    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in CaptureAndDetect state.")
            return ABORT

        yolo_detector: YOLODetector = blackboard.get("yolo_detector")
        if not yolo_detector:
            yasmin.YASMIN_LOG_ERROR("YOLO detector not available.")
            return ABORT

        current_waypoint = blackboard.get("current_target_waypoint")
        if not current_waypoint:
            yasmin.YASMIN_LOG_ERROR("Current waypoint not available.")
            return ABORT

        yasmin.YASMIN_LOG_INFO(
            f"Capturing image and detecting at waypoint {current_waypoint['index']}"
        )

        os.makedirs(DETECTION_SAVE_PATH, exist_ok=True)

        try:
            frame = self.image_handler.take_photo()

            if frame is None:
                yasmin.YASMIN_LOG_ERROR("Failed to capture image")
                return ABORT

            timestamp = int(time.time() * 1000)
            image_path = f"{DETECTION_SAVE_PATH}/waypoint_{current_waypoint['index']:03d}_{timestamp}.jpg"
            cv2.imwrite(image_path, frame)

            detection = yolo_detector.detect(
                frame, save_image=True, timestamp=timestamp
            )

            if detection:
                # Check if this detection is near a previously visited base
                mavdrone = blackboard["mavdrone"]
                current_pos = mavdrone.get_local_pos.pose.position
                visited_bases = blackboard.get("visited_bases", [])
                
                is_duplicate = False
                for base in visited_bases:
                    distance = math.sqrt(
                        (current_pos.x - base["x"]) ** 2 + 
                        (current_pos.y - base["y"]) ** 2
                    )
                    if distance < 1.5:  # Within 1.5m is considered same base
                        yasmin.YASMIN_LOG_INFO(
                            f"Detection appears to be already visited base at ({base['x']:.1f}, {base['y']:.1f})"
                        )
                        is_duplicate = True
                        break
                
                if not is_duplicate:
                    blackboard["current_detection"] = detection
                    blackboard["detection_image"] = frame

                    yasmin.YASMIN_LOG_INFO(
                        f"- NEW landing base detected! Confidence: {detection['confidence']:.2f}"
                    )
                    return "DETECTION_FOUND"
                else:
                    yasmin.YASMIN_LOG_INFO("Detection is a duplicate, continuing search")
                    return SUCCEED
            else:
                yasmin.YASMIN_LOG_INFO("No landing base detected at this waypoint")
                return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Capture and detect failed: {e}")
            return ABORT


class AdvanceToNextWaypoint(State):
    """Mark current waypoint as visited and advance to next."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, "ALL_COMPLETE"])

    def execute(self, blackboard: Blackboard):
        grid_waypoints = blackboard.get("grid_waypoints")
        current_waypoint = blackboard.get("current_target_waypoint")

        if grid_waypoints and current_waypoint:
            grid_waypoints.mark_waypoint_visited(current_waypoint["index"])
            grid_waypoints.advance_to_next()

            progress = grid_waypoints.get_progress()
            yasmin.YASMIN_LOG_INFO(
                f"Waypoint {current_waypoint['index']} completed. Progress: {progress['progress_percent']:.1f}%"
            )

            if grid_waypoints.is_complete():
                yasmin.YASMIN_LOG_INFO("All waypoints completed!")
                return "ALL_COMPLETE"

        return SUCCEED
