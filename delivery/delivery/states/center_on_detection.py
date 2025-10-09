import time

from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.image_processing.camera.image_handler import ImageHandler

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, ABORT

from delivery.utils import YoloDetector, ImageCalculus

from delivery.constants import (
    POSITION_CONTROLLER_KP_XY,
    POSITION_CONTROLLER_TOLERANCE_XY,
    POSITION_CONTROLLER_MAX_VELOCITY_XY,
    POSITION_CONTROLLER_MIN_VELOCITY_XY,
    CENTER_TIMEOUT,
    DETECTIONS_LOST_TOLERANCE,
    IMAGE_CENTER_X,
    IMAGE_CENTER_Y,
    IMAGE_CALCULUS_OFFSET_Y
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
            desired_class (str): Class ID to filter detections. Must be "cross" or "package".

        Raises:
            TypeError: If `desired_class` is not "cross" or "package".
        """
        super().__init__(outcomes=[SUCCEED, FAIL, ABORT])
        self._desired_class = desired_class.lower()
        if self._desired_class not in ("cross", "package"):
            raise TypeError("Parameter desired_class should be 'cross' or 'package'.")

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
        yolo_detector: YoloDetector = blackboard["yolo_detector"]

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
                frame = frame,
                desired_class = [self._desired_class],
            )

            yasmin.YASMIN_LOG_WARN(f'{detection}')
            
            if self._desired_class not in detection.keys():
                detections_lost += 1

                # Tirou varias fotos e nenhuma tinha deteccao -> FAIL
                if detections_lost > DETECTIONS_LOST_TOLERANCE:
                    yasmin.YASMIN_LOG_ERROR("No target detected in image. Fail centering.")
                    return FAIL

            else:
                detections_lost = 0

                # error_x, error_y, error_z = image_calculus.calculate_vector_from_drone_to_ground(
                #     altura = mavdrone.get_height,
                #     target_pixel = detection[self._desired_class]["center"],
                # )
                center = detection[self._desired_class]["center"]

                error_x = center[0] - IMAGE_CENTER_X
                error_y = center[1] - (IMAGE_CENTER_Y - IMAGE_CALCULUS_OFFSET_Y)

                if (abs(error_x) <= POSITION_CONTROLLER_TOLERANCE_XY) and (abs(error_y) <= POSITION_CONTROLLER_TOLERANCE_XY):
                    yasmin.YASMIN_LOG_INFO(f"Target centered successfully (error_x={error_x:.2f}, error_y={error_y:.2f}).")
                    return SUCCEED

                vel_x = error_y * POSITION_CONTROLLER_KP_XY
                vel_y = error_x * POSITION_CONTROLLER_KP_XY

                vel_x = self.saturate_abs(vel_x)
                vel_y = self.saturate_abs(vel_y)

                yasmin.YASMIN_LOG_INFO(f"Adjusting position: \nerror_x={error_x:.2f}, \nerror_y={error_y:.2f}, \nlinear_x={vel_x:.2f}, \nlinear_y={vel_y:.2f}")
                print(f"Adjusting position: \nerror_x={error_x:.2f}, \nerror_y={error_y:.2f}, \nlinear_x={vel_x:.2f}, \nlinear_y={vel_y:.2f}")
                mavdrone.offboard_velocity(
                    linear_x = vel_x,
                    linear_y = vel_y,
                    linear_z = 0.0,
                    angular_z = 0.0,
                )
        yasmin.YASMIN_LOG_ERROR("CenterOnDetection timeout.")
        return FAIL

    def saturate_abs(self, value):
        if value == 0: return 0.0
        return max(POSITION_CONTROLLER_MIN_VELOCITY_XY, min(POSITION_CONTROLLER_MAX_VELOCITY_XY, abs(value))) * (1 if value > 0 else -1)