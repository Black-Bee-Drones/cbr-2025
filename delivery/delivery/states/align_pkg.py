import time

from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.image_processing.camera.image_handler import ImageHandler

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL, TIMEOUT
from delivery.utils import YOLODetector


from delivery.constants import (
    ALIGN_TIMEOUT,
    PACKAGE_PROPORTION_ALIGN,
    DETECTIONS_LOST_TOLERANCE, 
    POSITION_CONTROLLER_KP_YAW, 
    POSITION_CONTROLLER_MAX_VELOCITY_YAW, 
)


class AlignPkg(State):
    """
    State to control drone movement in YAW to align on a detected target using YOLO.

    Outcome of the state:
        - SUCCEED: Target aligned successfully.
        - FAIL: Target not detected for too long.
        - ABORT: Required components not available.
        - TIMEOUT: Could not align target within allowed time.
    """

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, FAIL])

    def execute(self, blackboard : Blackboard):
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

        yasmin.YASMIN_LOG_INFO("Starting aligning procedure using YOLO detector...")
        detections_lost = 0
        start = time.time()
        while time.time() - start < ALIGN_TIMEOUT:
            frame = image_handler.take_photo()

            detection = yolo_detector.detect(
                image = frame,
                desired_class = "package",
            )

            if 'bbox' not in detection.keys():
                detections_lost += 1

                # Tirou varias fotos e nenhuma tinha deteccao -> FAIL
                if detections_lost > DETECTIONS_LOST_TOLERANCE:
                    yasmin.YASMIN_LOG_ERROR("No target detected in image. Fail aligning.")
                    return FAIL

            else:
                detections_lost = 0

                side_x = abs(detection["bbox"][2] - detection["bbox"][0])
                side_y = abs(detection["bbox"][3] - detection["bbox"][1])

                side_max = max(side_x, side_y)
                side_min = min(side_x, side_y)

                package_proportion = side_max/side_min

                if package_proportion >= PACKAGE_PROPORTION_ALIGN:
                    yasmin.YASMIN_LOG_INFO("Succeed, drone aligned with package.")
                    return SUCCEED

                error_yaw = 0.5  # rad

                vel_yaw = error_yaw * POSITION_CONTROLLER_KP_YAW

                vel_yaw = max(-POSITION_CONTROLLER_MAX_VELOCITY_YAW, min(POSITION_CONTROLLER_MAX_VELOCITY_YAW, vel_yaw))

                yasmin.YASMIN_LOG_INFO(f"Adjusting position: error_yaw={error_yaw:.2f}, angular_z={vel_yaw:.2f}")
                mavdrone.offboard_velocity(
                    linear_x = 0.0,
                    linear_y = 0.0,
                    linear_z = 0.0,
                    angular_z = vel_yaw,
                )
        yasmin.YASMIN_LOG_ERROR("Align timeout.")
        return TIMEOUT

    