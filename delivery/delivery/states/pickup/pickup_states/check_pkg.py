import time
import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED
from delivery.utils.yolo_detection import YOLODetector
from mirela_sdk.image_processing.camera.image_handler import ImageHandler

class CheckPkg(State):
    """
    Checar pela YOLO se o pacote ainda está na base.
    """
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, "pkg_detected"])
        self.detector = YOLODetector()

    def execute(self, blackboard: Blackboard):
        image_handler: ImageHandler = blackboard.get("image_handler")

        for _ in range(3):
            time.sleep(1)
            image = image_handler.take_photo()

            detection = self.detector.detect(desired_class="package", image=image, save_image=False)

            if detection:
                yasmin.YASMIN_LOG_INFO("CheckPkg: pacote detectado!!!")
                return "pkg_detected"

        yasmin.YASMIN_LOG_INFO("CheckPkg: Nenhum pacote detectado após 3 tentativas.")
        return SUCCEED




    
        