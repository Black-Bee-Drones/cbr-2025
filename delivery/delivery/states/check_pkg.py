import time

from mirela_sdk.image_processing.camera.image_handler import ImageHandler

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL

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
        image_handler: ImageHandler = blackboard.get("image_handler")
        if not image_handler:
            yasmin.YASMIN_LOG_ERROR("ImageHandler not available in CenterOnDetection state.")
            return ABORT

        self.yolo_detector: YOLODetector = blackboard.get("yolo_detector")
        if not self.yolo_detector:
            yasmin.YASMIN_LOG_ERROR("YoloDetector not available in CenterOnDetection state.")
            return ABORT

        for _ in range(DETECTIONS_LOST_TOLERANCE):
            time.sleep(1)
            frame = image_handler.take_photo()

            detection = self.yolo_detector.detect(
                image = frame,
                desired_class = "package",
            )

            if detection:
                yasmin.YASMIN_LOG_INFO("CheckPkg: pacote detectado!!!")
                return FAIL

        yasmin.YASMIN_LOG_INFO("CheckPkg: Nenhum pacote detectado após 3 tentativas.")
        return SUCCEED
