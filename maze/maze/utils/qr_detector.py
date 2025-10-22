"""QR Code detection utility for maze navigation"""

import cv2
import numpy as np
from typing import List, Tuple, Dict
from dataclasses import dataclass
from rclpy.node import Node
import time
from qreader import QReader


@dataclass
class QRCodeResult:
    """Data class for QR code detection result."""

    data: str
    position: Tuple[int, int]  # Center position (x, y)
    corners: np.ndarray  # Corner points
    confidence: float
    timestamp: float


class QRCodeDetector:
    """
    QR Code detector for identifying codes on maze walls.
    Uses qreader library for robust detection in challenging conditions.
    """

    def __init__(self, node: Node):
        """
        Initialize QR code detector.

        Parameters
        ----------
        node : Node
            ROS2 node for logging
        """
        self.node = node
        # Initialize QReader with optimized settings for real-time detection
        self.detector = QReader(
            model_size="s",  # Use small model for faster detection
            min_confidence=0.5,  # Adjust confidence threshold
        )
        self.detected_codes = []  # History of detected codes
        self.unique_codes = set()  # Set of unique code values detected

    def detect(self, image: np.ndarray) -> List[QRCodeResult]:
        """
        Detect QR codes in image using qreader.

        Parameters
        ----------
        image : np.ndarray
            Input image (RGB format)

        Returns
        -------
        List[QRCodeResult]
            List of detected QR codes
        """
        results = []

        if image is None:
            self.node.get_logger().warn("No image provided for QR detection")
            return results

        try:
            print("caling detection")
            detection_result = self.detector.detect_and_decode(
                image, return_detections=True
            )

            if detection_result and len(detection_result) == 2:
                decoded_texts, detections = detection_result

                # Process each detected QR code
                for decoded_text, detection in zip(decoded_texts, detections):
                    if decoded_text:  # Only process if text was decoded
                        # Extract bounding box coordinates
                        bbox = detection["bbox_xyxy"]  # Format: [x1, y1, x2, y2]

                        # Convert bbox to corner points
                        x1, y1, x2, y2 = bbox
                        corners = np.array(
                            [[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32
                        )

                        # Calculate center position
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)

                        # Get confidence score if available
                        confidence = detection.get("confidence", 1.0)

                        result = QRCodeResult(
                            data=decoded_text,
                            position=(center_x, center_y),
                            corners=corners,
                            confidence=confidence,
                            timestamp=time.time(),
                        )

                        results.append(result)
                        self.detected_codes.append(result)
                        self.unique_codes.add(decoded_text)
            else:
                print("Try to enhance image CLAHE")
                enhanced = self._enhance_image(image)
                enhanced_result = self.detector.detect_and_decode(
                    enhanced, return_detections=True
                )

                if enhanced_result and len(enhanced_result) == 2:
                    decoded_texts, detections = enhanced_result

                    for decoded_text, detection in zip(decoded_texts, detections):
                        if decoded_text:
                            bbox = detection["bbox_xyxy"]
                            x1, y1, x2, y2 = bbox
                            corners = np.array(
                                [[x1, y1], [x2, y1], [x2, y2], [x1, y2]],
                                dtype=np.float32,
                            )
                            center_x = int((x1 + x2) / 2)
                            center_y = int((y1 + y2) / 2)
                            confidence = detection.get("confidence", 0.8)

                            result = QRCodeResult(
                                data=decoded_text,
                                position=(center_x, center_y),
                                corners=corners,
                                confidence=confidence
                                * 0.9,  # Slightly lower confidence for enhanced
                                timestamp=time.time(),
                            )

                            results.append(result)
                            self.detected_codes.append(result)
                            self.unique_codes.add(decoded_text)

        except Exception as e:
            self.node.get_logger().error(f"QR detection error: {e}")

        return results

    def _enhance_image(self, image: np.ndarray) -> np.ndarray:
        """
        Enhance image for better QR code detection.

        Parameters
        ----------
        image : np.ndarray
            Input image (RGB format)

        Returns
        -------
        np.ndarray
            Enhanced image (RGB format)
        """
        try:
            # Convert RGB to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

            # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            enhanced = clahe.apply(gray)

            # Convert grayscale back to RGB for qreader
            enhanced_rgb = cv2.cvtColor(enhanced, cv2.COLOR_GRAY2RGB)

            return enhanced_rgb
        except Exception:
            return image

    def detect_multiple(self, image: np.ndarray) -> List[QRCodeResult]:
        """
        Detect multiple QR codes in a single image.

        Parameters
        ----------
        image : np.ndarray
            Input image (RGB format)

        Returns
        -------
        List[QRCodeResult]
            All detected QR codes
        """
        all_results = []

        # First, try standard detection
        results = self.detect(image)
        all_results.extend(results)

        # If less than expected, try sliding window approach
        if len(results) < 2:  # Assume we might have multiple codes
            window_results = self._sliding_window_detection(image)
            # Filter out duplicates
            for wr in window_results:
                if not any(r.data == wr.data for r in all_results):
                    all_results.append(wr)

        return all_results

    def _sliding_window_detection(
        self,
        image: np.ndarray,
        window_size: Tuple[int, int] = (300, 300),
        step_size: int = 100,
    ) -> List[QRCodeResult]:
        """
        Detect QR codes using sliding window approach.

        Parameters
        ----------
        image : np.ndarray
            Input image (RGB format)
        window_size : Tuple[int, int]
            Size of sliding window
        step_size : int
            Step size for sliding window

        Returns
        -------
        List[QRCodeResult]
            Detected QR codes
        """
        results = []
        h, w = image.shape[:2]
        window_h, window_w = window_size

        for y in range(0, h - window_h, step_size):
            for x in range(0, w - window_w, step_size):
                window = image[y : y + window_h, x : x + window_w]
                window_results = self.detect(window)

                # Adjust positions to global coordinates
                for result in window_results:
                    result.position = (result.position[0] + x, result.position[1] + y)
                    results.append(result)

        return results

    def draw_detections(
        self, image: np.ndarray, results: List[QRCodeResult]
    ) -> np.ndarray:
        """
        Draw QR code detections on image.

        Parameters
        ----------
        image : np.ndarray
            Input image (RGB format)
        results : List[QRCodeResult]
            Detection results to draw

        Returns
        -------
        np.ndarray
            Image with drawn detections (RGB format)
        """
        output = image.copy()

        for result in results:
            # Draw corners in green (RGB format)
            if result.corners is not None:
                points = result.corners.astype(np.int32)
                cv2.polylines(output, [points], True, (0, 255, 0), 2)

            # Draw center in red (RGB format)
            cv2.circle(output, result.position, 5, (255, 0, 0), -1)

            # Draw text in yellow (RGB format)
            cv2.putText(
                output,
                f"QR: {result.data}",
                (result.position[0] - 30, result.position[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),  # Yellow in RGB format
                2,
            )

        return output

    def get_summary(self) -> Dict:
        """
        Get summary of all detected QR codes.

        Returns
        -------
        Dict
            Summary with total count and unique codes found
        """
        return {
            "total_detections": len(self.detected_codes),
            "unique_codes": list(self.unique_codes),
            "unique_count": len(self.unique_codes),
        }

    def reset(self):
        """Reset detection history."""
        self.detected_codes.clear()
        self.unique_codes.clear()
        self.node.get_logger().info("QR detector history reset")

    def save_detection(self, image: np.ndarray, result: QRCodeResult, filepath: str):
        """
        Save image with detection result.

        Parameters
        ----------
        image : np.ndarray
            Original image (RGB format)
        result : QRCodeResult
            Detection result
        filepath : str
            Path to save the image
        """
        try:
            # Draw annotations on RGB image
            annotated = self.draw_detections(image, [result])
            # Convert RGB to BGR for cv2.imwrite
            annotated_bgr = cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)
            cv2.imwrite(filepath, annotated_bgr)
            self.node.get_logger().info(f"Saved detection to {filepath}")
        except Exception as e:
            self.node.get_logger().error(f"Failed to save detection: {e}")
