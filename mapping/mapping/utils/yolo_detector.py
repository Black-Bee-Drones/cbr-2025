import cv2
import numpy as np
import os
from typing import List, Dict, Tuple, Optional
import time
import random

try:
    import ultralytics
    from ultralytics import YOLO

    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

from mapping.constants import (
    YOLO_MODEL_PATH,
    YOLO_CONFIDENCE_THRESHOLD,
    YOLO_IMAGE_SIZE,
    DETECTION_SAVE_PATH,
    IMAGE_CENTER_X,
    IMAGE_CENTER_Y,
    IMAGE_OFFSET_Y,
)


class YOLODetector:
    """
    YOLO-based landing base detector for CB5 Phase 1.

    Handles loading YOLO model, running inference, and processing detections
    for landing base recognition.
    """

    def __init__(self, model_path: str = YOLO_MODEL_PATH):
        """
        Initialize YOLO detector.

        Args:
            model_path: Path to trained YOLO model file
        """
        self.model_path = model_path
        self.model = None
        self.confidence_threshold = YOLO_CONFIDENCE_THRESHOLD
        self.image_size = YOLO_IMAGE_SIZE
        self.detection_count = 0

        os.makedirs(DETECTION_SAVE_PATH, exist_ok=True)

        self._load_model()

    def _load_model(self):
        """Load YOLO model."""
        if not YOLO_AVAILABLE:
            print("  YOLO not available - using simulation")
            return

        if os.path.exists(self.model_path):
            try:
                self.model = YOLO(self.model_path, task="detect", verbose=True)
                print(f"- YOLO model loaded: {self.model_path}")
            except Exception as e:
                print(f"  Failed to load YOLO model: {e}")
                self.model = None
        else:
            print(f"  YOLO model not found: {self.model_path}")
            print("   Using simulation mode for detection")

    def detect(
        self,
        image: np.ndarray,
        save_image: bool = True,
        timestamp: Optional[int] = None,
        return_all: bool = False,
    ) -> List[Dict[str, any]] | Dict[str, any] | None:
        """
        Detect landing bases in image.

        Args:
            image: Input image (BGR format)
            save_image: Whether to save detection images
            timestamp: Optional timestamp for filenames
            return_all: If True, returns all detections; if False, returns best detection only

        Returns:
            If return_all=True: List of all detections with bounding boxes and confidence scores
            If return_all=False: Single best detection dict or None if no detections
        """
        detections = []

        current_timestamp = (
            timestamp if timestamp is not None else int(time.time() * 1000)
        )

        if self.model is None or not YOLO_AVAILABLE:
            sim_detection = self._simulate_detection(
                image, save_image, current_timestamp
            )
            if return_all:
                return [sim_detection] if sim_detection else []
            return sim_detection

        try:
            results = self.model(
                image, imgsz=self.image_size, conf=self.confidence_threshold
            )

            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        confidence = box.conf[0].cpu().numpy()
                        class_id = int(box.cls[0].cpu().numpy())

                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)

                        detection = {
                            "bbox": [int(x1), int(y1), int(x2), int(y2)],
                            "center": (center_x, center_y),
                            "confidence": float(confidence),
                            "class_id": class_id,
                            "area": (x2 - x1) * (y2 - y1),
                        }
                        detections.append(detection)

            if detections and save_image:
                self._save_detection_image(image, detections, current_timestamp)

        except Exception as e:
            print(f"x YOLO detection failed: {e}")

        if return_all:
            return detections
        else:
            return self.get_best_detection(detections)

    def get_best_detection(
        self, detections: List[Dict[str, any]]
    ) -> Optional[Dict[str, any]]:
        """
        Get the best detection based on confidence and size.

        Args:
            detections: List of detections

        Returns:
            Best detection or None if no detections
        """
        if not detections:
            return None

        best_detection = max(detections, key=lambda d: d["confidence"] * d["area"])
        return best_detection

    def calculate_centering_error(self, detection: Dict[str, any]) -> Tuple[int, int]:
        """
        Calculate pixel error for centering the drone on detection.

        Args:
            detection: Detection dictionary with center coordinates

        Returns:
            Tuple of (error_x, error_y) in pixels
        """
        center_x, center_y = detection["center"]

        error_x = center_x - IMAGE_CENTER_X
        error_y = center_y - (IMAGE_CENTER_Y + IMAGE_OFFSET_Y)

        return error_x, error_y

    def is_centered(self, detection: Dict[str, any], tolerance_px: int = 20) -> bool:
        """
        Check if detection is centered within tolerance.

        Args:
            detection: Detection dictionary
            tolerance_px: Pixel tolerance for centering

        Returns:
            True if detection is centered within tolerance
        """
        error_x, error_y = self.calculate_centering_error(detection)
        distance = np.sqrt(error_x**2 + error_y**2)

        return distance <= tolerance_px

    def _save_detection_image(
        self, image: np.ndarray, detections: List[Dict[str, any]], timestamp: int
    ):
        """Save image with detection bounding boxes."""
        annotated_image = image.copy()

        for detection in detections:
            bbox = detection["bbox"]
            confidence = detection["confidence"]
            center = detection["center"]

            cv2.rectangle(
                annotated_image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2
            )

            cv2.circle(annotated_image, center, 5, (0, 0, 255), -1)

            text = f"Landing Base: {confidence:.2f}"
            cv2.putText(
                annotated_image,
                text,
                (bbox[0], bbox[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )

        cv2.line(
            annotated_image,
            (IMAGE_CENTER_X - 10, IMAGE_CENTER_Y),
            (IMAGE_CENTER_X + 10, IMAGE_CENTER_Y),
            (255, 0, 0),
            2,
        )
        cv2.line(
            annotated_image,
            (IMAGE_CENTER_X, IMAGE_CENTER_Y - 10),
            (IMAGE_CENTER_X, IMAGE_CENTER_Y + 10),
            (255, 0, 0),
            2,
        )

        filename = f"{DETECTION_SAVE_PATH}/{timestamp}/detection_{self.detection_count:04d}.jpg"
        cv2.imwrite(filename, annotated_image)
        print(filename)
        self.detection_count += 1

    def _simulate_detection(
        self, image: np.ndarray, save_image: bool, timestamp: int
    ) -> List[Dict[str, any]]:
        # Simulate occasional detection (10% chance)

        if random.random() < 0.1:
            # Create mock detection near image center with some offset
            offset_x = random.randint(-50, 50)
            offset_y = random.randint(-50, 50)

            center_x = IMAGE_CENTER_X + offset_x
            center_y = IMAGE_CENTER_Y + offset_y

            # Create mock bounding box
            bbox_size = 80
            x1 = max(0, center_x - bbox_size // 2)
            y1 = max(0, center_y - bbox_size // 2)
            x2 = min(image.shape[1], center_x + bbox_size // 2)
            y2 = min(image.shape[0], center_y + bbox_size // 2)

            detection = {
                "bbox": [x1, y1, x2, y2],
                "center": (center_x, center_y),
                "confidence": 0.85,
                "class_id": 0,
                "area": (x2 - x1) * (y2 - y1),
            }

            if save_image:
                self._save_detection_image(image, [detection], timestamp)

            return detection

        return None
