# Main State Machine for Delivery

import rclpy
import time
import math
import cv2
import os
from typing import Tuple, List, Dict

from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.image_processing.camera.image_handler import ImageHandler

import yasmin
from yasmin import StateMachine, State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL
from delivery.utils import PositionController, YOLOPackageDetector, YOLODeliverDetector


from delivery.states import (
    Takeoff,
    Land,
)

from delivery.constants import (
    TAKEOFF_ALTITUDE,
    SEARCH_TIMEOUT,
    IMAGE_SOURCE,
    CENTER_ERROR_TOLERANCE,
    CENTER_TIMEOUT,
    ALING_TIMEOUT,
    ALIGN_X_PROPOTION,
    MIN_DETECTIONS_LOST
)

class GoToPkg(State):
    """
    Controller: Mandar drone para coordenada na base de entrega baseada no index na blackboard.
    Detector: Node de Yolo para reconhecer o pacote
    """

    def __init__(self):
        super().__init__(outcomes = [SUCCEED, ABORT])

    def execute(self, blackboard : Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR(
                "MavDrone not available in NavigateToWaypoint state."
            )
            return ABORT

        mavdrone: MavDrone = blackboard.get("mavdrone")
        
        package_position = blackboard.get("package_position")
        if not package_position:
            yasmin.YASMIN_LOG_ERROR(f"Next package position not avaible.")
            return ABORT

        yasmin.YASMIN_LOG_INFO(f"Next package position: {package_position}.")
        
        position_controller : PositionController = blackboard.get("position_controller")
        if not position_controller:
            yasmin.YASMIN_LOG_ERROR("Position controller not avaible.")
            return ABORT

        try:
            success = position_controller.goto_position_ground_relative(
                package_position["x"],
                package_position["y"],
                TAKEOFF_ALTITUDE,
                blackboard.get("ground_reference_altitude", 0.0),
                timeout = SEARCH_TIMEOUT
            )

            if success:
                yasmin.YASMIN_LOG_INFO("Package point reached successfully.")
                return SUCCEED
            else:
                yasmin.YASMIN_LOG_ERROR("Failed to reach package point.")
                return ABORT
            
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Navigation failed: {e}")
            return ABORT


class CenterPkg(State):
    """
    Movimentação X, Y para centralizar o drone e o pacote
    """

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, FAIL])

    def execute(self, blackboard : Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR(
                "MavDrone not available in CenterPkg state."
            )
            return ABORT
    
        mavdrone: MavDrone = blackboard["mavdrone"]

        image_handler: ImageHandler = blackboard["image_handler"]

        if not image_handler:
            yasmin.YASMIN_LOG_ERROR(
                "ImageHandler not available in CenterPkg state."
            )
            return ABORT

        image_handler.image_processing_callback = self.image_processing_callback

        start = time.time()
        while (time.time() - start) < CENTER_TIMEOUT:
            error_x, error_y = image_handler.take_photo()

            if error_x is None or error_y is None:
                return FAIL

            if ((error_x ** 2) + (error_y ** 2)) < CENTER_ERROR_TOLERANCE**2:
                return SUCCEED

            mavdrone.offboard_velocity(
                linear_x = error_x,
                linear_y = error_y,
                linear_z = 0.0,
                angular_z = 0.0,
            )
        return ABORT


    def image_processing_callback(self, img) -> Tuple[float, float]:
        """
        ImageHandler callback
        return: if error == None: there isn't Yolo detection
        """
        error_x, error_y = None
        return error_x, error_y


class UpPKG(State):
    """
    Up to search package
    """
class AlingPkg(State):
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

        while time.time() - start < ALING_TIMEOUT:
            detection = self.yolo_pkg_detector(
                self.image_handler.take_photo(), save_imgae=False
            )
            side_x, side_y = self.detection_boundingbox_size(detection)
            if not detection:
                lost_detections += 1
                yasmin.YASMIN_LOG_ERROR(f"Missed {lost_detections} detections.")
                if lost_detections > MIN_DETECTIONS_LOST:
                    yasmin.YASMIN_LOG_ERROR(F"Lost package, restarting package detection.")
                    return FAIL
            elif side_y < (side_x * ALIGN_X_PROPOTION):
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
    

class DescendPkg(State):
    """
    Descer um pouco e realinhar o drone até chegar em uma boa altura para dar land
    """
class PickPkg(State):
    """
    Ativar garra
    """
class CheckPkg(State):
    """
    Checar pela yolo se o pacote ainda esta na base
    """

class PickupSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "height_limit", "pkg_detected"])

        self.add_state(
            "GO_TO_PKG",
            GoToPkg(),
            transitions={SUCCEED:"CENTER_PKG", ABORT: ABORT},
        )
        self.add_state(
            "CENTER_PKG",
            CenterPkg(),
            transitions={SUCCEED:"ALING_PKG", ABORT: ABORT, FAIL: "UP_PKG"},
        )
        self.add_state(
            "UP_PKG",
            UpPKG(),
            transitions={SUCCEED:"CENTER_PKG", ABORT: ABORT, FAIL: "UP_PKG"},
        )
        self.add_state(
            "ALING_PKG",
            AlingPkg(),
            transitions={SUCCEED:"DESCEND_PKG", ABORT: ABORT},
        )
        self.add_state(
            "DESCEND_PKG",
            DescendPkg(),
            transitions={SUCCEED:"CENTER_PKG", ABORT: ABORT, "height_limit": "LAND"},
        )
        self.add_state(
            "LAND",
            Land(),
            transitions={SUCCEED:"PICK_PKG", ABORT: ABORT},
        )
        self.add_state(
            "PICK_PKG",
            PickPkg(),
            transitions={SUCCEED:"TAKEOFF", ABORT: ABORT},
        )
        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={SUCCEED: "CHECK_PKG", ABORT : ABORT},
        )
        self.add_state(
            "CHECK_PKG",
            CheckPkg(),
            transitions={SUCCEED: SUCCEED, ABORT : ABORT, "pkg_detected" : "CENTER_PKG"},
        )

        self.set_start_state("GO_TO_PKG")