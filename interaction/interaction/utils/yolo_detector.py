import numpy as np
from ultralytics import YOLO
from interaction.constants import (
    YOLO_MODEL_PATH,
    YOLO_CONFIDENCE_THRESHOLD,
    YOLO_IMAGE_SIZE,
)


class YoloDetector:
    """Handles YOLO11n-pose model loading and gesture detection."""

    def __init__(self):
        """Initialize the YOLO detector."""
        self.model = None
        self.model_loaded = False

    def load_model(self):
        """Load the YOLO11n-pose model."""
        try:
            self.model = YOLO(YOLO_MODEL_PATH, task="pose")
            self.model_loaded = True
            return True
        except Exception as e:
            print(f"Error loading YOLO model: {e}")
            self.model_loaded = False
            return False

    def detect_gesture(self, img: np.array) -> str:
        """
        Detect pose and return gesture name.

        Parameters
        ----------
        img : np.array
            Input image for pose detection

        Returns
        -------
        str
            Detected gesture name or "neutral" if no gesture detected
        """
        if not self.model_loaded or self.model is None:
            return "neutral"

        try:
            results = self.model(
                img,
                imgsz=YOLO_IMAGE_SIZE,
                conf=YOLO_CONFIDENCE_THRESHOLD,
                verbose=False,
            )

            for r in results:
                if r.keypoints is not None and len(r.keypoints) > 0:
                    keypoints = r.keypoints.xy.cpu().numpy()

                    # Process first person detected
                    if len(keypoints) > 0:
                        return self._analyze_keypoints(keypoints[0])

            return "neutral"

        except Exception as e:
            print(f"Error during gesture detection: {e}")
            return "neutral"

    def _analyze_keypoints(self, keypoints: np.array) -> str:
        """
        Analyze keypoints to determine gesture.

        Parameters
        ----------
        keypoints : np.array
            Array of keypoints [17, 2] in COCO format

        Returns
        -------
        str
            Detected gesture name
        """
        if keypoints is None or len(keypoints) < 17:
            return "neutral"

        # Extract relevant keypoints (indices in COCO format)
        left_shoulder = keypoints[5]
        right_shoulder = keypoints[6]
        left_elbow = keypoints[7]
        right_elbow = keypoints[8]
        left_wrist = keypoints[9]
        right_wrist = keypoints[10]

        # Check if all required keypoints are valid
        required_points = [
            left_shoulder,
            right_shoulder,
            left_elbow,
            right_elbow,
            left_wrist,
            right_wrist,
        ]
        if any(point[0] == 0 and point[1] == 0 for point in required_points):
            return "neutral"

        # Calculate angles for arms
        def calculate_arm_angle(shoulder, wrist):
            """Calculate the angle of the arm relative to horizontal"""
            arm_vector = wrist - shoulder
            angle = np.arctan2(arm_vector[1], arm_vector[0]) * 180 / np.pi
            return angle

        left_angle = calculate_arm_angle(left_shoulder, left_wrist)
        right_angle = calculate_arm_angle(right_shoulder, right_wrist)

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

        # Double biceps (bodybuilder pose)
        if (
            left_angle < -30
            and left_angle > -150
            and right_angle < -30
            and right_angle > -150
            and left_elbow_angle < 100
            and left_elbow_angle > 40
            and right_elbow_angle < 100
            and right_elbow_angle > 40
            and left_wrist[1] < left_elbow[1]
            and right_wrist[1] < right_elbow[1]
        ):
            return "double_biceps"

        # Cross arms
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
        if (
            right_angle < -30
            and right_angle > -150
            and right_elbow_angle < 100
            and right_elbow_angle > 40
            and right_wrist[1] < right_elbow[1]
            and left_angle > 45
            and left_angle < 135
        ):
            return "right_arm_biceps_left_arm_down"

        # Left arm biceps, right arm down (MOVE BACKWARD)
        if (
            left_angle < -30
            and left_angle > -150
            and left_elbow_angle < 100
            and left_elbow_angle > 40
            and left_wrist[1] < left_elbow[1]
            and right_angle > 45
            and right_angle < 135
        ):
            return "left_arm_biceps_right_arm_down"

        # Left arm down, right arm to side (YAW RIGHT)
        if (
            (abs(right_angle - 180) < 45 or abs(right_angle) < 45)
            and right_elbow_angle > 140
            and left_angle > 45
            and left_angle < 135
        ):
            return "left_arm_down_right_arm_side"

        # Right arm down, left arm to side (YAW LEFT)
        if (
            (abs(left_angle - 180) < 45 or abs(left_angle) < 45)
            and left_elbow_angle > 140
            and right_angle > 45
            and right_angle < 135
        ):
            return "right_arm_down_left_arm_side"

        return "neutral"
