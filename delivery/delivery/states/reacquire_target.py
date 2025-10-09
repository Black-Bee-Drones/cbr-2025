from mirela_sdk.control.mavros.mavros_api import MavDrone

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, ABORT

from delivery.constants import (
    REACQUIRE_TIMEOUT,
    MAX_ALTITUDE,
    TARGET_UP_ALTITUDE,
    POSITION_CONTROLLER_TOLERANCE_Z,
    SEARCH_TIMEOUT,
    SEARCH_TIMEOUT,
)

from delivery.utils import YoloDetector
from mirela_sdk.image_processing.camera.image_handler import ImageHandler

import time

class ReacquireTarget(State):
    """
    State to reacquire a lost target by adjusting the drone's altitude.

    Outcome of the state:
        - SUCCEED: Target altitude reached successfully or timeout.
        - FAIL: Target altitude not reached within allowed time.
        - ABORT: Required components (e.g., `mavdrone`) not available.
    """
    def __init__(self,  desired_class: str):
        """
        Args:
            desired_class (str): "cross" or "package".

        Raises:
            TypeError: If `desired_class` is not "cross" or "package".
        """
        super().__init__(outcomes=[SUCCEED, FAIL, ABORT])

        self._desired_class = desired_class.lower()
        if self._desired_class not in ("cross", "package"):
            raise TypeError("Parameter desired_class should be 'cross' or 'package'.")

    def execute(self, blackboard : Blackboard):
        if ("mavdrone" not in blackboard) or not blackboard["mavdrone"]:
            yasmin.YASMIN_LOG_ERROR(f"Mavdrone not available in {self.__class__.__name__} state.")
            return ABORT
        mavdrone: MavDrone = blackboard["mavdrone"]

        if ("yolo_detector" not in blackboard) or not blackboard["yolo_detector"]:
            yasmin.YASMIN_LOG_ERROR(f"yolo_detector not available in {self.__class__.__name__} state.")
            return ABORT
        yolo_detector: YoloDetector = blackboard["yolo_detector"]


        if ("image_handler" not in blackboard) or not blackboard["image_handler"]:
            yasmin.YASMIN_LOG_ERROR(f"image_handler not available in {self.__class__.__name__} state.")
            return ABORT
        image_handler: ImageHandler = blackboard["image_handler"]


        current_alt = mavdrone.get_height
        
        package_id = blackboard["next_package"]

        if self._desired_class == "cross":
            all_positions = blackboard["deliver_positions"]
        else:
            all_positions = blackboard["packages_positions"]
        target_position = all_positions[package_id]

        try:
            mavdrone.offboard_position(
                x=target_position["x"],
                y=target_position["y"],
                z=0.0,
                timeout_sec=SEARCH_TIMEOUT,
                ground_reference=True,
            )
            yasmin.YASMIN_LOG_INFO("Target point reached successfully.")
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Navigation failed: {e}")
            return ABORT
        
        succeeded_detections = 0

        start = time.time()
        while (time.time() - start) < REACQUIRE_TIMEOUT:
            frame = image_handler.take_photo()

            detection = yolo_detector.detect(
                frame = frame,
                desired_class = [self._desired_class],
            )
            
            if self._desired_class not in detection.keys():
                succeeded_detections += 1
            else:
                succeeded_detections = 0

            if succeeded_detections >= 2:
                return SUCCEED
        
        if current_alt >= MAX_ALTITUDE or (current_alt + TARGET_UP_ALTITUDE) >= MAX_ALTITUDE:
            yasmin.YASMIN_LOG_INFO("Altitude exceeds maximum allowed limit.")
            return FAIL

        yasmin.YASMIN_LOG_INFO("Starting vertical correction.")

        try:
            mavdrone.offboard_position(
                x=0.0,
                y=0.0,
                z=TARGET_UP_ALTITUDE,
                timeout_sec=REACQUIRE_TIMEOUT,
                precision_radius=POSITION_CONTROLLER_TOLERANCE_Z,
            )
            yasmin.YASMIN_LOG_INFO("Target point reached successfully.")
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Navigation failed: {e}")
            return ABORT
