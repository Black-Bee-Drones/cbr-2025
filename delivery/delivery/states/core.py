import time

import yasmin
from yasmin import Blackboard
from yasmin import State
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.control.mavros import MavDrone
from mirela_sdk.image_processing.camera import ImageHandler

from delivery.utils import YoloDetector, ImageCalculus

from delivery.constants import (
    IS_INDOOR,
    TAKEOFF_ALTITUDE,
    TAKEOFF_SLEEP,
    STARTING_PACKAGE_IDX,
    PACKAGE_POSITIONS,
    DELIVER_POSITIONS,
    IMAGE_SOURCE,
    IMAGE_CALCULUS_OFFSET_X,
    CAMERA_FOV_HORIZONTAL,
    CAMERA_FOV_VERTICAL,
)


class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        try:
            yasmin.YASMIN_LOG_INFO("Initializing mission...")

            yasmin.YASMIN_LOG_INFO("Initializing blackboard constants...")
            blackboard["next_package"] = STARTING_PACKAGE_IDX
            blackboard["packages_positions"] = PACKAGE_POSITIONS
            blackboard["deliver_positions"] = DELIVER_POSITIONS
            blackboard["visited_bases"] = []  # I guess it's deprecated

            if not blackboard["packages_positions"]:
                yasmin.YASMIN_LOG_ERROR("Package positions not declared.")
            if not blackboard["deliver_positions"]:
                yasmin.YASMIN_LOG_ERROR("Deliver positions not declared.")

            yasmin.YASMIN_LOG_INFO("Initializing MavDrone...")
            blackboard["mavdrone"] = MavDrone(
                node=YasminNode.get_instance(),
                mavros=False,
                indoor=IS_INDOOR,
            )

            yasmin.YASMIN_LOG_INFO("Initializing ImageHandler...")
            blackboard["image_handler"] = ImageHandler(
                node=YasminNode.get_instance(),
                image_source=IMAGE_SOURCE,
            )
            blackboard["image_handler"].open()
            frame = blackboard["image_handler"].take_photo()
            height, width, _ = frame.shape
            camera_pixels_per_degree = (width / CAMERA_FOV_HORIZONTAL + height / CAMERA_FOV_VERTICAL) / 2

            yasmin.YASMIN_LOG_INFO("Initializing ImageCalculus...")
            blackboard["image_calculus"] = ImageCalculus()
            blackboard["image_calculus"].update_camera_offset(**IMAGE_CALCULUS_OFFSET_X)
            blackboard["image_calculus"].update_pixels_per_degree(pixels_per_degree=camera_pixels_per_degree)
            blackboard["image_calculus"].update_camera_resolution(
                width=width,
                height=height,
            )

            yasmin.YASMIN_LOG_INFO("Initializing YoloDetector and run first detection...")
            blackboard["yolo_detector_cross"] = YoloDetector()
            blackboard["yolo_detector_cross"].detect(frame=frame)

            yasmin.YASMIN_LOG_INFO("Mission successfully initialized.")
            return SUCCEED
    
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Failed to initialize: {e}")
            return ABORT


class Takeoff(State):
    def __init__(self, update_initial_position: bool = False):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self._update_initial_position = update_initial_position

    def execute(self, blackboard: Blackboard):
        if ("mavdrone" not in blackboard) or not blackboard["mavdrone"]:
            yasmin.YASMIN_LOG_ERROR(f"Mavdrone not available in {self.__class__.__name__} state.")
            return ABORT
        mavdrone: MavDrone = blackboard["mavdrone"]

        if self._update_initial_position:
            yasmin.YASMIN_LOG_INFO('Update inicial position.')
            blackboard["initial_position"] = mavdrone.get_position_as_target

        yasmin.YASMIN_LOG_INFO(f"Taking off to {TAKEOFF_ALTITUDE} meters...")
        try:
            mavdrone.arm_takeoff(TAKEOFF_ALTITUDE)
            time.sleep(TAKEOFF_SLEEP)
            mavdrone.set_takeoff_position(blackboard["initial_position"])
            yasmin.YASMIN_LOG_INFO("Takeoff successful.")
            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Takeoff failed: {e}")
            return ABORT


class Land(State):
    def __init__(self, rtl: bool = False):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self._rtl = rtl

    def execute(self, blackboard: Blackboard):
        if ("mavdrone" not in blackboard) or not blackboard["mavdrone"]:
            yasmin.YASMIN_LOG_ERROR(f"Mavdrone not available in {self.__class__.__name__} state.")
            return ABORT
        mavdrone: MavDrone = blackboard["mavdrone"]

        if self._rtl:
            yasmin.YASMIN_LOG_INFO("Returning to launch...")

            try:
                mavdrone.set_takeoff_position(blackboard["initial_position"])
                mavdrone.rtl(
                    rtl_alt=TAKEOFF_ALTITUDE,
                    precision_radius=0.25,
                    rtl_strategy="PID",
                    land=True
                )
                yasmin.YASMIN_LOG_INFO("Return to launch initiated.")
                return SUCCEED

            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(f"Return to launch failed: {e}")
                yasmin.YASMIN_LOG_INFO('Try to normal land.')

        try:
            mavdrone.land()
            time.sleep(5)  # wait for landing to complete
            yasmin.YASMIN_LOG_INFO("Landed successfully.")
            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Landing failed: {e}")
            return ABORT
