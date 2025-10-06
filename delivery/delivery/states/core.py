import time

import yasmin
from yasmin import Blackboard
from yasmin import State
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.control.mavros import MavDrone
from mirela_sdk.image_processing.camera import ImageHandler
from mirela_sdk.image_processing.camera.image_calculus import ImageCalculus

from delivery.utils import YOLODetectorCross, YOLODetectorPkg

from delivery.constants import (
    TAKEOFF_ALTITUDE,
    STARTING_PACKAGE_IDX,
    PACKAGE_POSITIONS,
    DELIVER_POSITIONS,
    IMAGE_SOURCE,
    IMAGE_CALCULUS_OFFSET_X,
    CAMERA_RESOLUTION_WIDTH,
    CAMERA_RESOLUTION_HEIGHT,
    CAMERA_PIXELS_PER_DEGREE,
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
                indoor=False,
            )

            yasmin.YASMIN_LOG_INFO("Initializing ImageCalculus...")
            blackboard["image_calculus"] = ImageCalculus()
            blackboard["image_calculus"].update_camera_offset(x=IMAGE_CALCULUS_OFFSET_X)
            blackboard["image_calculus"].update_pixels_per_degree(pixels_per_degree=CAMERA_PIXELS_PER_DEGREE)
            blackboard["image_calculus"].update_camera_resolution(
                width=CAMERA_RESOLUTION_WIDTH,
                height=CAMERA_RESOLUTION_HEIGHT,
            )

            yasmin.YASMIN_LOG_INFO("Initializing ImageHandler...")
            blackboard["image_handler"] = ImageHandler(
                node=YasminNode.get_instance(),
                image_source=IMAGE_SOURCE,
            )
            blackboard["image_handler"].open()
            frame = blackboard["image_handler"].take_photo()

            yasmin.YASMIN_LOG_INFO("Initializing YOLODetectorCross and run first detection...")
            blackboard["yolo_detector_cross"] = YOLODetectorCross()
            blackboard["yolo_detector_cross"].detect(image=frame)

            yasmin.YASMIN_LOG_INFO("Initializing YOLODetectorPkg and run first detection...")
            blackboard["yolo_detector_pkg"] = YOLODetectorPkg()
            blackboard["yolo_detector_pkg"].detect(image=frame, desired_class="base")

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
