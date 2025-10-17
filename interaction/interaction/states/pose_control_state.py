import rclpy
import cv2
import time
import numpy as np
from mirela_sdk.image_processing.camera.image_handler import ImageHandler
from mirela_sdk.image_processing.camera import IMX219Config
import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from interaction.constants import (
    TAKEOFF_POSE,
    VELOCITY_UP_DOWN,
    VELOCITY_SIDES,
    VELOCITY_IN_OUT,
    VELOCITY_YAW,
    ACTION_TIMEOUT,
    YOLO_IMAGE_SIZE,
    GESTURE_CONFIRMATION_THRESHOLD,
    GESTURE_CONFIRMATION_THRESHOLD_SINGLE,
    MAX_GESTURE_FRAMES,
)


class PoseControl(State):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.yolo_detector = None

        # Control variables
        self.image_handler = None
        self.current_gesture = None
        self.previous_gesture = None
        self.gesture_start_time = 0.0
        self.command_sent = False
        self.processing_width = YOLO_IMAGE_SIZE
        self.processing_height = YOLO_IMAGE_SIZE

        # Gesture confirmation variables (bouncing threshold)
        self.gesture_detection_count = 0
        self.confirmed_gesture = None
        self.total_gesture_frames = 0

        self.mavdrone = None
        self.node = None
        self.blackboard = None
        self.land_count = 0

        self.gesture_commands = {
            "double_biceps": lambda: self.mavdrone.arm_takeoff(TAKEOFF_POSE),
            "cross_arms": lambda: self.land_and_count(),
            "right_arm_up_left_arm_side": lambda: self.mavdrone.offboard_velocity(
                linear_y=-VELOCITY_SIDES
            ),
            "left_arm_up_right_arm_side": lambda: self.mavdrone.offboard_velocity(
                linear_y=VELOCITY_SIDES
            ),
            "both_arms_down": lambda: self.mavdrone.offboard_velocity(
                linear_z=-VELOCITY_UP_DOWN
            ),
            "both_arms_up": lambda: self.mavdrone.offboard_velocity(
                linear_z=VELOCITY_UP_DOWN
            ),
            "right_arm_biceps_left_arm_down": lambda: self.mavdrone.offboard_velocity(
                linear_x=VELOCITY_IN_OUT
            ),
            "left_arm_biceps_right_arm_down": lambda: self.mavdrone.offboard_velocity(
                linear_x=-VELOCITY_IN_OUT
            ),
            "left_arm_down_right_arm_side": lambda: self.mavdrone.offboard_velocity(
                angular_z=-VELOCITY_YAW
            ),
            "right_arm_down_left_arm_side": lambda: self.mavdrone.offboard_velocity(
                angular_z=VELOCITY_YAW
            ),
            "neutral": lambda: self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0),
        }

    def land_and_count(self):
        """Land and count for exit condition."""
        self.mavdrone.land()
        self.mavdrone.delay(8)
        self.land_count += 1

    def execute(self, blackboard: Blackboard):
        """Execute the pose control state."""

        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in PoseControl state.")
            return ABORT

        if "yolo_detector" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("YoloDetector not available in PoseControl state.")
            return ABORT

        self.mavdrone = blackboard["mavdrone"]
        self.yolo_detector = blackboard["yolo_detector"]
        self.node = YasminNode.get_instance()
        self.blackboard = blackboard

        try:
            if not self._initialize_camera():
                return ABORT

            yasmin.YASMIN_LOG_INFO("Starting pose-based control...")

            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)

                if self.land_count >= 6:
                    yasmin.YASMIN_LOG_INFO(
                        "Land count reached 6, exiting PoseControl state."
                    )
                    self._cleanup()
                    return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Error in PoseControl state: {e}")
            return ABORT

        finally:
            self._cleanup()

    def _initialize_camera(self):
        """Initialize camera for image capture."""
        try:
            self.image_handler = ImageHandler(
                node=self.node,
                image_source="imx219",
                image_processing_callback=self.process_image,
                config=IMX219Config(sensor_id=0, width=1640, height=1232, flip=2),
            )

            time.sleep(1.0)
            yasmin.YASMIN_LOG_INFO("Camera initialized.")

            self.image_handler.open()
            time.sleep(1.0)
            yasmin.YASMIN_LOG_INFO("Camera opened.")

            self.image_handler.run()
            yasmin.YASMIN_LOG_INFO("Image processing started.")

            return True

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Error initializing camera: {e}")
            return False

    def process_image(self, img: np.array) -> None:
        """Process image to detect poses and execute commands."""
        try:

            gesture = self.yolo_detector.detect_gesture(img)

            self.process_gesture_action(gesture)

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Error processing image: {e}")

    def process_gesture_action(self, gesture: str) -> None:
        """
        Process the action for detected gesture with confirmation threshold.

        Implements a bouncing threshold mechanism:
        - Single actions (takeoff, land) require GESTURE_CONFIRMATION_THRESHOLD_SINGLE frames
        - Continuous gestures require GESTURE_CONFIRMATION_THRESHOLD frames
        - Only confirmed gestures trigger drone commands
        - Prevents false positives and transitional movements
        """

        self.previous_gesture = self.current_gesture
        self.current_gesture = gesture

        # Determine which threshold to use based on gesture type
        single_gestures = ["double_biceps", "cross_arms"]
        required_threshold = (
            GESTURE_CONFIRMATION_THRESHOLD_SINGLE
            if gesture in single_gestures
            else GESTURE_CONFIRMATION_THRESHOLD
        )

        if self.previous_gesture != self.current_gesture:
            self.gesture_detection_count = 0
            self.confirmed_gesture = None
            self.command_sent = False
            self.total_gesture_frames = 0

            if gesture and gesture != "neutral":
                yasmin.YASMIN_LOG_INFO(
                    f"New gesture detected: {gesture}, confirming... (0/{required_threshold})"
                )
            return

        if self.current_gesture and self.current_gesture != "neutral":
            self.gesture_detection_count += 1

            if self.gesture_detection_count <= required_threshold:
                yasmin.YASMIN_LOG_INFO(
                    f"Confirming gesture '{self.current_gesture}': "
                    f"{self.gesture_detection_count}/{required_threshold}"
                )

        # Check if gesture is confirmed (reached threshold)
        if (
            self.gesture_detection_count >= required_threshold
            and self.confirmed_gesture != self.current_gesture
        ):
            self.confirmed_gesture = self.current_gesture
            self.gesture_start_time = time.time()
            yasmin.YASMIN_LOG_INFO(f"✓ Gesture CONFIRMED: {self.confirmed_gesture}")

        if not self.confirmed_gesture or self.confirmed_gesture == "neutral":
            if self.previous_gesture and self.previous_gesture != "neutral":
                self.gesture_commands["neutral"]()
            return

        self.total_gesture_frames += 1

        # # Reset if gesture held too long
        # if self.total_gesture_frames > MAX_GESTURE_FRAMES:
        #     yasmin.YASMIN_LOG_WARN(
        #         f"Gesture '{self.confirmed_gesture}' held too long, resetting..."
        #     )
        #     self.gesture_detection_count = 0
        #     self.confirmed_gesture = None
        #     self.total_gesture_frames = 0
        #     self.gesture_commands["neutral"]()
        #     return

        time_elapsed = time.time() - self.gesture_start_time

        if time_elapsed >= ACTION_TIMEOUT:
            continuous_gestures = [
                "right_arm_up_left_arm_side",
                "left_arm_up_right_arm_side",
                "both_arms_down",
                "both_arms_up",
                "right_arm_biceps_left_arm_down",
                "left_arm_biceps_right_arm_down",
                "left_arm_down_right_arm_side",
                "right_arm_down_left_arm_side",
            ]

            single_gestures = ["double_biceps", "cross_arms"]

            if self.confirmed_gesture in continuous_gestures:
                self.gesture_commands[self.confirmed_gesture]()

            elif self.confirmed_gesture in single_gestures and not self.command_sent:
                yasmin.YASMIN_LOG_INFO(
                    f"Executing single action: {self.confirmed_gesture}"
                )
                self.gesture_commands[self.confirmed_gesture]()
                self.command_sent = True

    def _cleanup(self):
        """Clean up resources."""
        try:
            if self.image_handler:
                self.image_handler.cleanup()

            yasmin.YASMIN_LOG_INFO("PoseControl cleanup completed.")

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Error during cleanup: {e}")
