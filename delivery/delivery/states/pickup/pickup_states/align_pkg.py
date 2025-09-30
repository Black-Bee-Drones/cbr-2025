import time
from typing import Tuple, Dict

from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.image_processing.camera.image_handler import ImageHandler

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL
from delivery.utils import YOLODetector


from delivery.constants import (
    ALIGN_TIMEOUT,
    ALIGN_X_PROPORTION,
    MIN_DETECTIONS_LOST,
)


class AlignPkg(State):
    """
    Movimentação Yaw no drone até atingir as proporções laterais corretas do bounding box
    """

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, FAIL])

    def execute(self, blackboard : Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR(
                "MavDrone not available in AlignPkg state."
            )
            return ABORT
    
        mavdrone: MavDrone = blackboard["mavdrone"]
        self.image_handler: ImageHandler = blackboard["image_handler"]
        self.yolo_pkg_detector : YOLOPackageDetector = blackboard.get["yolo_pkg_detector"]

        if not self.image_handler:
            yasmin.YASMIN_LOG_ERROR(
                "ImageHandler not available in AlignPkg state."
            )
            return ABORT
        
        if not self.yolo_pkg_detector:
            yasmin.YASMIN_LOG_ERROR(
                "YoloPackageDetector not available in AlignPkg state."
            )
            return ABORT
        
        
        start = time.time()
        lost_detections = 0

        while time.time() - start < ALIGN_TIMEOUT:
            detection = self.yolo_pkg_detector.detect(
                self.image_handler.take_photo(), save_image=False
            )
            side_x, side_y = self.detection_boundingbox_size(detection)
            if not detection:
                lost_detections += 1
                yasmin.YASMIN_LOG_ERROR(f"Missed {lost_detections} detections.")
                if lost_detections > MIN_DETECTIONS_LOST:
                    yasmin.YASMIN_LOG_ERROR(F"Lost package, restarting package detection.")
                    return FAIL
            elif side_y < (side_x * ALIGN_X_PROPORTION):
                mavdrone.offboard_velocity(0, 0, 0, 0.1, True)
                lost_detections = 0
            else:
                yasmin.YASMIN_LOG_INFO(
                    "Drone fully aligned with package."
                )
                return SUCCEED
        
        yasmin.YASMIN_LOG_ERROR("Align timeout.")
        return ABORT

    def detection_boundingbox_size(best_detection : Dict[str, any]) -> Tuple[float, float]:

        """
        Processing detection bounding box
        Returns sides of the detected bounding box
        [side x, side y]
        """

        side_x = best_detection["bbox"][2] - best_detection["bbox"][0]
        side_y = best_detection["bbox"][3] - best_detection["bbox"][1]

        return [side_x, side_y]
    