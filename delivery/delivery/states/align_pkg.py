import time
import numpy as np

from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.image_processing.camera.image_handler import ImageHandler
from mirela_sdk.utils.position_utils import PositionUtils

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, ABORT
from delivery.utils import YoloDetector

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
        - FAIL: Lost packet detection or timeout.
        - ABORT: Required components not available.
    """

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, FAIL, ABORT])

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
        yolo_detector: YoloDetector = blackboard["yolo_detector"]

        yasmin.YASMIN_LOG_INFO("Starting aligning procedure using YOLO detector...")
        detections_lost = 0

        vel_yaw = POSITION_CONTROLLER_KP_YAW

        position = mavdrone.get_position
        orientation = PositionUtils.get_yaw_from_pose(position)

        initial_orientation = blackboard["initial_orientation"]
        orientation_limit = initial_orientation - np.pi/2
        if((orientation - orientation_limit) > (initial_orientation - orientation)):
            vel_yaw *= -1


        start = time.time()
        while time.time() - start < ALIGN_TIMEOUT:
            frame = image_handler.take_photo()
# Draw line from center to target
            detection = yolo_detector.detect(
                frame = frame,
                desired_class = ["package"],
            )

            if "package" not in detection.keys():
                detections_lost += 1

                # Tirou varias fotos e nenhuma tinha deteccao -> FAIL
                if detections_lost > DETECTIONS_LOST_TOLERANCE:
                    yasmin.YASMIN_LOG_ERROR("No target detected in image. Failed aligning.")
                    return FAIL

            else:
                detections_lost = 0

                side_x = abs(detection["package"]["bbox"][2] - detection["package"]["bbox"][0])
                side_y = abs(detection["package"]["bbox"][3] - detection["package"]["bbox"][1])
                mavdrone.node.get_logger().info(f"Bounding box: side_x = {side_x}, side_y = {side_y}", throttle_duration_sec = 0.1                       )
                # yasmin.YASMIN_LOG_INFO(f"Bounding box: side_x = {side_x}, side_y = {side_y}")   
                package_proportion = side_y/side_x

                if package_proportion > PACKAGE_PROPORTION_ALIGN:
                    # position = mavdrone.get_position
                    # orientation = PositionUtils.get_yaw_from_pose(position)
                    # blackboard["orientation_aligned"] = orientation
                    yasmin.YASMIN_LOG_INFO("Succeed, drone aligned with package.")
                    return SUCCEED

                yasmin.YASMIN_LOG_INFO(f"Adjusting position: proportion={package_proportion:.2f}, angular_z={vel_yaw:.2f}")
                mavdrone.offboard_velocity(
                    linear_x = 0.0,
                    linear_y = 0.0,
                    linear_z = 0.0,
                    angular_z = vel_yaw,
                )
        yasmin.YASMIN_LOG_ERROR("Align timeout.")
        return FAIL
