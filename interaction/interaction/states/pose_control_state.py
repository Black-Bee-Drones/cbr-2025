import rclpy
import cv2
import time
import numpy as np
from ultralytics import YOLO
from mirela_sdk.image_processing.camera.image_handler import ImageHandler
from mirela_sdk.image_processing.camera import IMX219Config
from mirela_sdk.control.mavros.mavros_api import MavDrone

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
    SLEEP_AFTER_TAKEOFF,
    SLEEP_AFTER_LAND,
)


class PoseControl(State):
    # Gesture confirmation threshold (minimum consecutive detections required)
    GESTURE_CONFIRMATION_THRESHOLD = 8
    # Maximum frames allowed for same gesture to avoid stuck states
    MAX_GESTURE_FRAMES = 300

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        # YOLO model
        self.model = None

        # Control variables
        self.image_handler = None
        self.current_gesture = None
        self.previous_gesture = None
        self.gesture_start_time = 0.0
        self.command_sent = False
        self.processing_width = 640
        self.processing_height = 480

        # Gesture confirmation variables (bouncing threshold)
        self.gesture_detection_count = 0  # Counter for consecutive detections
        self.confirmed_gesture = None  # Only set after threshold is reached
        self.total_gesture_frames = 0  # Total frames of confirmed gesture

        # Drone control
        self.mavdrone = None
        self.node = None
        self.blackboard = None
        self.land_count = 0

        # Gesture to command mapping
        self.gesture_commands = {
            "double_biceps": lambda: self.mavdrone.arm_takeoff(TAKEOFF_POSE),
            "cross_arms": lambda: self.land_and_count(),
            "right_arm_up_left_arm_side": lambda: self.mavdrone.offboard_velocity(
                linear_y=VELOCITY_SIDES
            ),
            "left_arm_up_right_arm_side": lambda: self.mavdrone.offboard_velocity(
                linear_y=-VELOCITY_SIDES
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
                angular_z=VELOCITY_YAW
            ),
            "right_arm_down_left_arm_side": lambda: self.mavdrone.offboard_velocity(
                angular_z=-VELOCITY_YAW
            ),
            "neutral": lambda: self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0),
        }

    def land_and_count(self):
        self.mavdrone.land()
        self.land_count += 1

    def detect_gesture(self, keypoints):
        """
        Analyze keypoints to determine the gesture.

        Parameters
        ----------
        keypoints : np.array
            Array of keypoints [17, 2] in COCO format

        Returns
        -------
        str or None
            Detected gesture name or None if no gesture detected
        """
        if keypoints is None or len(keypoints) < 17:
            return None

        # Extract relevant keypoints (indices in COCO format)
        left_shoulder = keypoints[5]
        right_shoulder = keypoints[6]
        left_elbow = keypoints[7]
        right_elbow = keypoints[8]
        left_wrist = keypoints[9]
        right_wrist = keypoints[10]

        # Check if all required keypoints are valid (not zero)
        required_points = [
            left_shoulder,
            right_shoulder,
            left_elbow,
            right_elbow,
            left_wrist,
            right_wrist,
        ]
        if any(point[0] == 0 and point[1] == 0 for point in required_points):
            return None

        # Calculate angles for arms
        def calculate_arm_angle(shoulder, elbow, wrist):
            """Calculate the angle of the arm relative to horizontal"""
            # Vector from shoulder to wrist
            arm_vector = wrist - shoulder
            # Angle in degrees (0° is horizontal right, positive is down)
            angle = np.arctan2(arm_vector[1], arm_vector[0]) * 180 / np.pi
            return angle

        left_angle = calculate_arm_angle(left_shoulder, left_elbow, left_wrist)
        right_angle = calculate_arm_angle(right_shoulder, right_elbow, right_wrist)

        # Calculate elbow angles (flexion)
        def calculate_elbow_angle(shoulder, elbow, wrist):
            """Calculate the elbow flexion angle"""
            v1 = shoulder - elbow
            v2 = wrist - elbow
            cos_angle = np.dot(v1, v2) / (
                np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6
            )
            angle = np.arccos(np.clip(cos_angle, -1.0, 1.0)) * 180 / np.pi
            return angle

        left_elbow_angle = calculate_elbow_angle(left_shoulder, left_elbow, left_wrist)
        right_elbow_angle = calculate_elbow_angle(
            right_shoulder, right_elbow, right_wrist
        )

        # Detect gestures based on arm positions and angles

        # Double biceps (bodybuilder pose): both arms up with elbows bent showing biceps
        # Arms should be raised to approximately shoulder height with elbows bent ~90 degrees
        if (
            # Both arms raised (negative angles mean arms are above horizontal)
            left_angle < -30
            and left_angle > -150
            and right_angle < -30
            and right_angle > -150
            and
            # Both elbows bent significantly (smaller angle = more bent)
            left_elbow_angle < 100
            and left_elbow_angle > 40
            and right_elbow_angle < 100
            and right_elbow_angle > 40
            and
            # Wrists should be above elbows (flexing biceps)
            left_wrist[1] < left_elbow[1]
            and right_wrist[1] < right_elbow[1]
        ):
            return "double_biceps"

        # Cross arms: arms crossed in front
        if (
            abs(left_wrist[0] - right_shoulder[0]) < 50
            and abs(right_wrist[0] - left_shoulder[0]) < 50
        ):
            return "cross_arms"

        # Right arm up, left arm to side
        if right_angle < -60 and right_angle > -120 and abs(left_angle) < 45:
            return "right_arm_up_left_arm_side"

        # Left arm up, right arm to side
        if (
            left_angle < -60
            and left_angle > -120
            and (abs(right_angle - 180) < 45 or abs(right_angle) < 45)
        ):
            return "left_arm_up_right_arm_side"

        # Both arms up (extended)
        if (
            left_angle < -60
            and left_angle > -120
            and right_angle < -60
            and right_angle > -120
            and left_elbow_angle > 140
            and right_elbow_angle > 140
        ):
            return "both_arms_up"

        # Both arms down (extended)
        if (
            left_angle > 60
            and left_angle < 120
            and right_angle > 60
            and right_angle < 120
        ):
            return "both_arms_down"

        # Right arm biceps, left arm down (MOVE FORWARD)
        # Right arm showing biceps pose, left arm relaxed down
        if (
            # Right arm raised
            right_angle < -30
            and right_angle > -150
            and
            # Right elbow bent (biceps flex)
            right_elbow_angle < 100
            and right_elbow_angle > 40
            and
            # Right wrist above elbow
            right_wrist[1] < right_elbow[1]
            and
            # Left arm down
            left_angle > 45
            and left_angle < 135
        ):
            return "right_arm_biceps_left_arm_down"

        # Left arm biceps, right arm down (MOVE BACKWARD)
        # Left arm showing biceps pose, right arm relaxed down
        if (
            # Left arm raised
            left_angle < -30
            and left_angle > -150
            and
            # Left elbow bent (biceps flex)
            left_elbow_angle < 100
            and left_elbow_angle > 40
            and
            # Left wrist above elbow
            left_wrist[1] < left_elbow[1]
            and
            # Right arm down
            right_angle > 45
            and right_angle < 135
        ):
            return "left_arm_biceps_right_arm_down"

        # Left arm down, right arm to side (YAW RIGHT)
        # Right arm horizontal, left arm pointing down
        if (
            (
                abs(right_angle - 180) < 45 or abs(right_angle) < 45
            )  # Right arm horizontal to side
            and right_elbow_angle > 140  # Right arm extended
            and left_angle > 45
            and left_angle < 135  # Left arm down
        ):
            return "left_arm_down_right_arm_side"

        # Right arm down, left arm to side (YAW LEFT)
        # Left arm horizontal, right arm pointing down
        if (
            (
                abs(left_angle - 180) < 45 or abs(left_angle) < 45
            )  # Left arm horizontal to side
            and left_elbow_angle > 140  # Left arm extended
            and right_angle > 45
            and right_angle < 135  # Right arm down
        ):
            return "right_arm_down_left_arm_side"

        return "neutral"

    def execute(self, blackboard: Blackboard):
        """Execute the pose control state."""

        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in PoseControl state.")
            return ABORT

        self.mavdrone: MavDrone = blackboard["mavdrone"]
        self.node = YasminNode.get_instance()
        self.blackboard = blackboard

        try:
            # Initialize components
            if not self._initialize_components():
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

    def _initialize_components(self):
        """Initialize YOLO model and camera."""
        try:
            # Initialize YOLO model
            yasmin.YASMIN_LOG_INFO("Loading YOLO11n-pose model...")
            self.model = YOLO("yolo11n-pose.pt")
            yasmin.YASMIN_LOG_INFO("YOLO model loaded.")

            # Initialize camera
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
            yasmin.YASMIN_LOG_ERROR(f"Error initializing components: {e}")
            return False

    def process_image(self, img: np.array) -> None:
        """Process image to detect poses and execute commands."""
        try:
            # Resize for better performance
            resized_img = cv2.resize(
                img, (self.processing_width, self.processing_height)
            )

            # Run pose detection
            results = self.model(resized_img, verbose=False)

            gesture = None

            for r in results:
                if r.keypoints is not None and len(r.keypoints) > 0:
                    keypoints = r.keypoints.xy.cpu().numpy()

                    # Process first person detected
                    if len(keypoints) > 0:
                        gesture = self.detect_gesture(keypoints[0])
                        break

            # Process detected gesture
            self.process_gesture_action(gesture)

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Error processing image: {e}")

    def process_gesture_action(self, gesture: str) -> None:
        """
        Process the action for detected gesture with confirmation threshold.

        Implements a bouncing threshold mechanism:
        - Gesture must be detected consistently for GESTURE_CONFIRMATION_THRESHOLD frames
        - Only confirmed gestures trigger drone commands
        - Prevents false positives and transitional movements
        """
        # Update current gesture
        self.previous_gesture = self.current_gesture
        self.current_gesture = gesture

        # Check if gesture changed
        if self.previous_gesture != self.current_gesture:
            # Reset confirmation counter when gesture changes
            self.gesture_detection_count = 0
            self.confirmed_gesture = None
            self.command_sent = False
            self.total_gesture_frames = 0

            if gesture and gesture != "neutral":
                yasmin.YASMIN_LOG_INFO(
                    f"New gesture detected: {gesture}, confirming... (0/{self.GESTURE_CONFIRMATION_THRESHOLD})"
                )
            return

        # Increment detection counter for same gesture
        if self.current_gesture and self.current_gesture != "neutral":
            self.gesture_detection_count += 1

            # Log confirmation progress (throttled)
            if self.gesture_detection_count <= self.GESTURE_CONFIRMATION_THRESHOLD:
                yasmin.YASMIN_LOG_INFO(
                    f"Confirming gesture '{self.current_gesture}': "
                    f"{self.gesture_detection_count}/{self.GESTURE_CONFIRMATION_THRESHOLD}"
                )

        # Check if gesture is confirmed (reached threshold)
        if (
            self.gesture_detection_count >= self.GESTURE_CONFIRMATION_THRESHOLD
            and self.confirmed_gesture != self.current_gesture
        ):
            self.confirmed_gesture = self.current_gesture
            self.gesture_start_time = time.time()
            yasmin.YASMIN_LOG_INFO(f"✓ Gesture CONFIRMED: {self.confirmed_gesture}")

        # Handle neutral gesture or no gesture
        if not self.confirmed_gesture or self.confirmed_gesture == "neutral":
            # Send stop command if transitioning from confirmed gesture to neutral
            if self.previous_gesture and self.previous_gesture != "neutral":
                self.gesture_commands["neutral"]()
            return

        # Increment total frames for confirmed gesture
        self.total_gesture_frames += 1

        # Reset if gesture held too long (safety mechanism)
        if self.total_gesture_frames > self.MAX_GESTURE_FRAMES:
            yasmin.YASMIN_LOG_WARN(
                f"Gesture '{self.confirmed_gesture}' held too long, resetting..."
            )
            self.gesture_detection_count = 0
            self.confirmed_gesture = None
            self.total_gesture_frames = 0
            self.gesture_commands["neutral"]()
            return

        # Calculate time elapsed since gesture confirmation
        time_elapsed = time.time() - self.gesture_start_time

        # Wait for ACTION_TIMEOUT before executing commands
        if time_elapsed >= ACTION_TIMEOUT:
            # Define continuous actions (executed repeatedly)
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

            # Define single actions (executed once)
            single_gestures = ["double_biceps", "cross_arms"]

            # Execute continuous actions
            if self.confirmed_gesture in continuous_gestures:
                self.gesture_commands[self.confirmed_gesture]()

            # Execute single actions (only once)
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
