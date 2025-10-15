import time
import math
import rclpy

import yasmin
from yasmin import Blackboard
from yasmin import State
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.control.mavros import MavDrone
from mirela_sdk.image_processing.camera import ImageHandler
from mirela_sdk.image_processing.camera.imx219_cam import IMX219Config
from mirela_sdk.utils.position_utils import PositionUtils

from delivery.utils import YoloDetector, ImageCalculus
from delivery.utils.center_with_arena import CenterWithArena

from delivery.constants import (
    IS_INDOOR,
    TAKEOFF_ALTITUDE,
    TAKEOFF_SLEEP,
    STARTING_PACKAGE_IDX,
    PACKAGE_POSITIONS,
    DELIVER_POSITIONS,
    IMAGE_SOURCE,
    IMAGE_CALCULUS_OFFSET_Y,
    CAMERA_FOV_HORIZONTAL,
    CAMERA_FOV_VERTICAL,
    POSITION_CONTROLLER_KP_YAW
)


class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        try:
            yasmin.YASMIN_LOG_INFO("Initializing mission...")

            yasmin.YASMIN_LOG_INFO("Initializing blackboard constants...")
            blackboard["next_package"] = STARTING_PACKAGE_IDX
            package_positions = PACKAGE_POSITIONS
            blackboard["packages_positions"] = CenterWithArena.calc_arena_position(package_positions)
            deliver_positions = DELIVER_POSITIONS
            blackboard["deliver_positions"] = CenterWithArena.calc_arena_position(deliver_positions)

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
            mavdrone: MavDrone = blackboard["mavdrone"]

            mavdrone.delay(0.1)
            position = mavdrone.get_position
            initial_orientation = PositionUtils.get_yaw_from_pose(position)
            blackboard["initial_orientation"] = initial_orientation
            blackboard["initial_position"] = mavdrone.get_position_as_target
            initial_position =  blackboard["initial_position"]
            yasmin.YASMIN_LOG_INFO(f"Initial position set to: {initial_position}")

            yasmin.YASMIN_LOG_INFO("Initializing ImageHandler...")
            blackboard["image_handler"] = ImageHandler(
                node=YasminNode.get_instance(),
                image_source=IMAGE_SOURCE,
                config=IMX219Config(sensor_id=0, width=1640, height=1232, flip=2)
            )
            image_handler: ImageHandler = blackboard["image_handler"]
            mavdrone.delay(1)
            image_handler.open()
            mavdrone.delay(1)
            frame = image_handler.take_photo()
            height, width, _ = frame.shape
            camera_pixels_per_degree = (width / CAMERA_FOV_HORIZONTAL + height / CAMERA_FOV_VERTICAL) / 2


            yasmin.YASMIN_LOG_INFO("Initializing YoloDetector and run first detection...")
            blackboard["yolo_detector"] = YoloDetector()
            detector: YoloDetector = blackboard["yolo_detector"]
            detector.detect(frame=frame, save_image=True)

            yasmin.YASMIN_LOG_INFO("Mission successfully initialized.")
            return SUCCEED
    
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Failed to initialize: {e}")
            return ABORT


class Takeoff(State):
    def __init__(self,):
        super().__init__(outcomes=[SUCCEED, ABORT])
    
    def execute(self, blackboard: Blackboard):
        blackboard["mavdrone"] = MavDrone(
            node=YasminNode.get_instance(),
            mavros=False,
            indoor=IS_INDOOR,
        )

        if ("mavdrone" not in blackboard) or not blackboard["mavdrone"]:
            yasmin.YASMIN_LOG_ERROR(f"Mavdrone not available in {self.__class__.__name__} state.")
            return ABORT
        mavdrone: MavDrone = blackboard["mavdrone"]

        yasmin.YASMIN_LOG_INFO(f"Taking off to {TAKEOFF_ALTITUDE} meters...")
        try:
            mavdrone.arm_takeoff(TAKEOFF_ALTITUDE)
            mavdrone.delay(TAKEOFF_SLEEP)
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
                mavdrone.delay(3)
                yasmin.YASMIN_LOG_INFO("Return to launch initiated.")
                return SUCCEED

            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(f"Return to launch failed: {e}")
                yasmin.YASMIN_LOG_INFO('Try to normal land.')

        try:
            mavdrone.land()
            mavdrone.delay(10) 
            yasmin.YASMIN_LOG_INFO("Landed successfully.")
            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Landing failed: {e}")
            return ABORT

class AdjustYaw(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        if ("mavdrone" not in blackboard) or not blackboard["mavdrone"]:
            yasmin.YASMIN_LOG_ERROR(f"Mavdrone not available in {self.__class__.__name__} state.")
            return ABORT
        mavdrone: MavDrone = blackboard["mavdrone"]

        yasmin.YASMIN_LOG_INFO("Executing Adjust Yaw...")
        try:
            mavdrone.offboard_position(
                x=-1.0,
                y=0.0, 
                z=TAKEOFF_ALTITUDE,
                timeout_sec=30,
                ground_reference=True,
                precision_radius=0.15
            )
            mavdrone.offboard_position(
                x=-1.0,
                y=1.0, 
                z=TAKEOFF_ALTITUDE,
                timeout_sec=30,
                ground_reference=True,
                precision_radius=0.15
            )

            yasmin.YASMIN_LOG_INFO("Adjust in position completed.")

            position = mavdrone.get_position
            orientation = PositionUtils.get_yaw_from_pose(position)

            initial_orientation = blackboard["initial_orientation"]
            
            initial_orientation = math.degrees(initial_orientation)

            current_orientation = math.degrees(orientation)

            yaw_error = initial_orientation - current_orientation
            print(f"os carai {yaw_error} | inicial {initial_orientation} | atual {current_orientation}")
            while yaw_error < 87:
                rclpy.spin_once(mavdrone.node, timeout_sec=0.1)
                mavdrone.node.get_logger().info(f"Yaw: {yaw_error}/90", throttle_duration_sec=0.1)
                mavdrone.offboard_velocity(
                    angular_z=-POSITION_CONTROLLER_KP_YAW
                )
                position = mavdrone.get_position
                orientation = PositionUtils.get_yaw_from_pose(position)
                current_orientation = math.degrees(orientation)
                yaw_error = initial_orientation - current_orientation

            yasmin.YASMIN_LOG_INFO("Adjust Yaw completed")

            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Landing failed: {e}")
            return ABORT