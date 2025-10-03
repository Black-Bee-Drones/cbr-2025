import time

from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.image_processing.camera.image_handler import ImageHandler
from mirela_sdk.image_processing.camera.image_calculus import ImageCalculus

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, ABORT

from delivery.utils import YOLODetector

from delivery.constants import (
    POSITION_CONTROLLER_KP_XY,
    POSITION_CONTROLLER_TOLERANCE_XY,
    POSITION_CONTROLLER_MAX_VELOCITY_XY,
    CENTER_TIMEOUT,
    DETECTIONS_LOST_TOLERANCE,
)


class CenterOnDetection(State):
    """
    State to control drone movement in X and Y to center on a detected target using YOLO.

    Outcome of the state:
        - SUCCEED: Target centered successfully.
        - FAIL: Target not detected for too long or timeout.
        - ABORT: Required components not available.
    """

    def __init__(self, desired_class: str):
        """
        Args:
            desired_class (str): Class ID to filter detections. Must be "base" or "package".

        Raises:
            TypeError: If `desired_class` is not "base" or "package".
        """
        super().__init__(outcomes=[SUCCEED, FAIL, ABORT])
        self._desired_class = desired_class.lower()
        if self._desired_class not in ("base", "package"):
            raise TypeError("Parameter desired_class should be 'base' or 'package'.")

    def execute(self, blackboard: Blackboard):
        if ("mavdrone" not in blackboard) or not blackboard["mavdrone"]:
            yasmin.YASMIN_LOG_ERROR(f"Mavdrone not available in {self.__class__.__name__} state.")
            return ABORT
        mavdrone: MavDrone = blackboard["mavdrone"]

        if ("image_handler" not in blackboard) or not blackboard["image_handler"]:
            yasmin.YASMIN_LOG_ERROR(f"image_handler not available in {self.__class__.__name__} state.")
            return ABORT
        image_handler: ImageHandler = blackboard["image_handler"]

        if ("yolo_detector" not in blackboard) or not blackboard["yolo_detector"]:
            yasmin.YASMIN_LOG_ERROR(f"yolo_detector not available in {self.__class__.__name__} state.")
            return ABORT
        yolo_detector: YOLODetector = blackboard["yolo_detector"]

        if ("image_calculus" not in blackboard) or not blackboard["image_calculus"]:
            yasmin.YASMIN_LOG_ERROR(f"image_calculus not available in {self.__class__.__name__} state.")
            return ABORT
        image_calculus: ImageCalculus = blackboard["image_calculus"]

        yasmin.YASMIN_LOG_INFO("Starting centering procedure using YOLO detector...")
        detections_lost = 0
        start = time.time()
        while (time.time() - start) < CENTER_TIMEOUT:
            frame = image_handler.take_photo()

            detection = yolo_detector.detect(
                image = frame,
                desired_class = self._desired_class,
            )

            if 'center' not in detection.keys():
                detections_lost += 1

                # Tirou varias fotos e nenhuma tinha deteccao -> FAIL
                if detections_lost > DETECTIONS_LOST_TOLERANCE:
                    yasmin.YASMIN_LOG_ERROR("No target detected in image. Fail centering.")
                    return FAIL

            else:
                error_x, error_y, error_z = image_calculus.calculate_vector_from_drone_to_ground(
                    altura = mavdrone.get_rng_alt.range,
                    target_pixel = detection['center'],
                )

                if (error_x**2 + error_y**2) <= (POSITION_CONTROLLER_TOLERANCE_XY**2):
                    yasmin.YASMIN_LOG_INFO(f"Target centered successfully (error_x={error_x:.2f}, error_y={error_y:.2f}).")
                    return SUCCEED

                vel_x = error_x * POSITION_CONTROLLER_KP_XY
                vel_y = error_y * POSITION_CONTROLLER_KP_XY

                vel_x = max(-POSITION_CONTROLLER_MAX_VELOCITY_XY, min(POSITION_CONTROLLER_MAX_VELOCITY_XY, vel_x))
                vel_y = max(-POSITION_CONTROLLER_MAX_VELOCITY_XY, min(POSITION_CONTROLLER_MAX_VELOCITY_XY, vel_y))

                yasmin.YASMIN_LOG_INFO(f"Adjusting position: error_x={error_x:.2f}, error_y={error_y:.2f}, linear_x={vel_x:.2f}, linear_y={vel_y:.2f}")
                mavdrone.offboard_velocity(
                    linear_x = vel_x,
                    linear_y = vel_y,
                    linear_z = 0.0,
                    angular_z = 0.0,
                )

                detections_lost = 0
        yasmin.YASMIN_LOG_ERROR("CenterOnDetection timeout.")
        return FAIL
