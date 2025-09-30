import time
from typing import Tuple

from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.image_processing.camera.image_handler import ImageHandler

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL
from delivery.utils import YOLODetector


from delivery.constants import (
    CENTERING_TOLERANCE_PX,
    CENTER_TIMEOUT,
    POSITION_CONTROLLER_KP_XY,
    MAX_VELOCITY_XY,
)


class CenterOnDetection(State):
    """
    Movimentação X, Y para centralizar o drone e o pacote
    """

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, FAIL])

    def execute(self, blackboard : Blackboard):
        mavdrone: MavDrone = blackboard["mavdrone"]
        image_handler: ImageHandler = blackboard["image_handler"]
        self.yolo_detector: YOLODetector = blackboard["yolo_detector"]

        image_handler.image_processing_callback = self.image_processing_callback

        yasmin.YASMIN_LOG_INFO("Starting centering procedure using YOLO detector...")
        start = time.time()
        while (time.time() - start) < CENTER_TIMEOUT:
            error_x, error_y = image_handler.take_photo()

            if error_x is None or error_y is None:
                yasmin.YASMIN_LOG_ERROR("No target detected in image. Aborting centering.")
                return FAIL

            if (error_x <= CENTERING_TOLERANCE_PX) and (error_y <= CENTERING_TOLERANCE_PX):
                yasmin.YASMIN_LOG_INFO(f"Target centered successfully (error_x={error_x:.2f}, error_y={error_y:.2f}).")
                return SUCCEED
            
            vel_x = error_x * POSITION_CONTROLLER_KP_XY
            vel_y = error_y * POSITION_CONTROLLER_KP_XY

            vel_x = max(-MAX_VELOCITY_XY, min(MAX_VELOCITY_XY, vel_x))
            vel_y = max(-MAX_VELOCITY_XY, min(MAX_VELOCITY_XY, vel_y))

            yasmin.YASMIN_LOG_INFO(f"Adjusting position: error_x={error_x:.2f}, error_y={error_y:.2f}, linear_x={vel_x:.2f}, linear_y={vel_y:.2f}")
            mavdrone.offboard_velocity(
                linear_x = vel_x,
                linear_y = vel_y,
                linear_z = 0.0,
                angular_z = 0.0,
            )
        yasmin.YASMIN_LOG_ERROR(f"Timeout ({CENTER_TIMEOUT:.1f}s) while trying to center target.")
        return FAIL

    def image_processing_callback(self, img) -> Tuple[float, float]:
        """
        ImageHandler callback
        return: if error == None: there isn't Yolo detection
        """
        detection = self.yolo_detector.detect(img)

        if detection:
            error_x, error_y = self.yolo_detector.calculate_centering_error(detection)
        else:
            error_x, error_y = None

        return error_x, error_y
