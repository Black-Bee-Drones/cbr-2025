# Main State Machine for Delivery

import rclpy
import time
import math
import cv2
import os
from typing import Tuple, List, Dict, Optional

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
    CENTERING_TOLERANCE_PX,
    CENTER_TIMEOUT,
    ALIGN_TIMEOUT,
    ALIGN_X_PROPORTION,
    MIN_DETECTIONS_LOST,
    POSITION_CONTROLLER_KP_XY,
    MAX_VELOCITY_XY,
    MAX_ALTITUDE,
    TARGET_UP_ALTITUDE,
    POSITION_CONTROLLER_KP_Z,
    MAX_VELOCITY_Z,
    REACQUIRE_TARGET_TIMEOUT,
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

        # mavdrone: MavDrone = blackboard.get("mavdrone")
        
        packages_positions = blackboard.get("packages_positions")
        if not packages_positions:
            yasmin.YASMIN_LOG_ERROR(f"Next package position not avaible.")
            return ABORT

        yasmin.YASMIN_LOG_INFO(f"Next package position: {packages_positions}.")
        
        position_controller : PositionController = blackboard.get("position_controller")
        if not position_controller:
            yasmin.YASMIN_LOG_ERROR("Position controller not avaible.")
            return ABORT

        try:
            idx = blackboard.get("current_package")
            success = position_controller.goto_position_ground_relative(
                packages_positions[idx]["x"],
                packages_positions[idx]["y"],
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
        mavdrone: MavDrone = blackboard["mavdrone"]
        image_handler: ImageHandler = blackboard["image_handler"]
        self.yolo_pkg_detector: YOLODeliverDetector = blackboard["yolo_pkg_detector"]

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
        detection = self.yolo_pkg_detector.detect(img)

        if detection:
            error_x, error_y = self.yolo_pkg_detector.calculate_centering_error(detection)
        else:
            error_x, error_y = None

        return error_x, error_y

# Não faz sentido só subir
class ReacquireTarget(State):
    """
    Up to search package
    """
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard : Blackboard):
        mavdrone: MavDrone = blackboard["mavdrone"]

        current_alt = mavdrone.get_rng_alt.range

        new_alt = current_alt + TARGET_UP_ALTITUDE

        if new_alt >= MAX_ALTITUDE:
            yasmin.YASMIN_LOG_ERROR("Target altitude exceeds maximum allowed limit.")
            return ABORT

        yasmin.YASMIN_LOG_INFO("Starting ascent.")
        start = time.time()
        while (time.time() - start) < REACQUIRE_TARGET_TIMEOUT:
            current_alt = mavdrone.get_rel_alt.data

            if current_alt >= MAX_ALTITUDE:
                yasmin.YASMIN_LOG_ERROR(f"Aborting: current altitude {current_alt:.2f}m >= max limit {MAX_ALTITUDE:.2f}m")
                return ABORT

            error_z = new_alt - current_alt

            if abs(error_z) < CENTERING_TOLERANCE_PX:
                yasmin.YASMIN_LOG_INFO(f"Target altitude reached successfully: {current_alt:.2f}m (error={error_z:.2f}m)")
                return SUCCEED

            vel_z = error_z * POSITION_CONTROLLER_KP_Z

            vel_z = max(-MAX_VELOCITY_Z, min(MAX_VELOCITY_Z, vel_z))

            yasmin.YASMIN_LOG_INFO(f"Ascent correction: current_alt={current_alt:.2f}m, target_alt={new_alt:.2f}m, error={error_z:.2f}m, linear_z={vel_z:.2f}m/s")
            mavdrone.offboard_velocity(
                linear_z=vel_z
            )

        yasmin.YASMIN_LOG_ERROR(f"Timeout ({REACQUIRE_TARGET_TIMEOUT:.1f}s) without reaching target altitude {new_alt:.2f}m. Last altitude={current_alt:.2f}m")
        return ABORT

# Pode se perder durante o align pq não faz a troca rápida de estados, aling
# no pior caso vai dar quase um 360
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
    
# Falta implementar
class DescendPkg(State):
    """
    Descer um pouco e realinhar o drone até chegar em uma boa altura para dar land
    """
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "height_limit"])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("DescendPkg  executado.")
        return SUCCEED
    

class PickPkg(State):
    """
    Ativar garra
    """
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("PickPkg (stub) executado.")
        return SUCCEED

class CheckPkg(State):
    """
    Checar pela yolo se o pacote ainda esta na base
    """
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "pkg_detected"])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("CheckPkg (stub) executado.")
        return SUCCEED

class PickupSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "height_limit", "pkg_detected", FAIL])

        self.add_state(
            "GO_TO_PKG",
            GoToPkg(),
            transitions={SUCCEED:"CENTER_PKG", ABORT: ABORT},
        )
        self.add_state(
            "CENTER_PKG",
            CenterPkg(),
            transitions={SUCCEED:"ALING_PKG", ABORT: ABORT, FAIL: "REACQUIRE_TARGET"},
        )
        self.add_state(
            "REACQUIRE_TARGET",
            ReacquireTarget(),
            transitions={SUCCEED:"CENTER_PKG", ABORT: ABORT},
        )
        self.add_state(
            "ALING_PKG",
            AlingPkg(),
            transitions={SUCCEED:"DESCEND_PKG", ABORT: ABORT, FAIL: "REACQUIRE_TARGET"},
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