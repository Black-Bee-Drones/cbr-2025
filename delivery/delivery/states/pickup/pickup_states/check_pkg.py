import time
import yasmin
from yasmin import State, Blackboard
from delivery.utils.yolo_detection import YOLODetector
from mirela_sdk.image_processing.camera.image_handler import ImageHandler

from yasmin_ros.basic_outcomes import (
    SUCCEED,
    ABORT,
)

class CheckPkg(State):
    """
    Checar pela YOLO se o pacote ainda está na base.
    """
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        image_handler: ImageHandler = blackboard.get("image_handler")
        detector: YOLODetector = blackboard.get("yolo_detector")

        if not detector:
            yasmin.YASMIN_LOG_ERROR("Yolo detector not available in CheckPkgState")
            return ABORT
        
        if not image_handler:
            yasmin.YASMIN_LOG_ERROR("Image Handler not available in CheckPkgState")
            return ABORT

        try:
            for _ in range(3):
                time.sleep(1)
                image = image_handler.take_photo()

                detection = detector.detect(desired_class="package", image=image, save_image=False)

                if detection:
                    yasmin.YASMIN_LOG_INFO("CheckPkg: package detected!")
                    return SUCCEED
                
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"CheckPkg failed: {e}")
            return ABORT        




    
        