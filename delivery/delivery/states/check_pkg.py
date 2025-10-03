import time

from mirela_sdk.image_processing.camera.image_handler import ImageHandler

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, ABORT

from delivery.utils import YOLODetector

from delivery.constants import (
    DETECTIONS_LOST_TOLERANCE,
)


class CheckPkg(State):
    """
    Status to check if the package was picked up.

    Outcome of the state:
        - SUCCEED: No package detected after multiple attempts (package is gone).
        - FAIL: Package detected at the base (package still present).
        - ABORT: Required components not available (e.g., ImageHandler, YOLODetector).
    """
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, FAIL, ABORT])

    def execute(self, blackboard: Blackboard):
        if ("image_handler" not in blackboard) or not blackboard["image_handler"]:
            yasmin.YASMIN_LOG_ERROR(f"image_handler not available in {self.__class__.__name__} state.")
            return ABORT
        image_handler: ImageHandler = blackboard["image_handler"]

        if ("yolo_detector" not in blackboard) or not blackboard["yolo_detector"]:
            yasmin.YASMIN_LOG_ERROR(f"yolo_detector not available in {self.__class__.__name__} state.")
            return ABORT
        yolo_detector: YOLODetector = blackboard["yolo_detector"]

        for _ in range(DETECTIONS_LOST_TOLERANCE):
            time.sleep(1)
            frame = image_handler.take_photo()

            detection = yolo_detector.detect(
                image = frame,
                desired_class = "package",
                inside_base = True,
            )

            if detection:
                yasmin.YASMIN_LOG_INFO("CheckPkg: package detected!!!")
                return FAIL

        yasmin.YASMIN_LOG_INFO(f"CheckPkg: no package was detected after {DETECTIONS_LOST_TOLERANCE} attempts.")
        return SUCCEED
