#!/usr/bin/env python3
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from mirela_sdk.control.pid.pid_controller import PIDController
from mirela_sdk.image_processing.camera.image_handler import ImageHandler
from mirela_sdk.image_processing.camera.imx219_cam import IMX219Config

from delivery.utils import YoloDetector
from delivery.constants import (
    IMAGE_CENTER_X,
    IMAGE_CENTER_Y,
    IMAGE_SOURCE,
    POSITION_CONTROLLER_TOLERANCE_XY,
)


class DetectionTester(Node):
    """Node for testing package detection and alignment calculations."""

    def __init__(self):
        super().__init__("detection_tester")

        # Initialize components
        self.get_logger().info("Initializing Image Handler...")
        self.image_handler = ImageHandler(
            node=self,
            image_source=IMAGE_SOURCE,
            config=IMX219Config(sensor_id=0, width=1640, height=1232, flip=2),
        )
        self.image_handler.open()
        time.sleep(1)

        self.get_logger().info("Initializing YOLO Detector...")
        self.yolo_detector = YoloDetector()

        # Initialize PID controllers with same parameters as AlignAndDescend
        self.get_logger().info("Initializing PID Controllers...")
        self._init_pid_controllers()

        self.get_logger().info("Setup complete! Ready to test detection.")

    def _init_pid_controllers(self):
        """Initialize PID controllers matching AlignAndDescend state."""
        # X control (forward/backward)
        self.pid_x = PIDController(
            kp=0.0014,
            ki=0.0,
            kd=0.0,
            setpoint=IMAGE_CENTER_Y,
            output_limits=(-0.2, 0.2),
            integral_limits=(-0.1, 0.1),
        )

        # Y control (left/right)
        self.pid_y = PIDController(
            kp=0.0014,
            ki=0.0,
            kd=0.0,
            setpoint=IMAGE_CENTER_X,
            output_limits=(-0.2, 0.2),
            integral_limits=(-0.1, 0.1),
        )

        # Yaw control (rotation)
        self.pid_yaw = PIDController(
            kp=0.013,
            ki=0.0,
            kd=0.001,
            setpoint=0.0,
            output_limits=(-0.15, 0.15),
            integral_limits=(-0.05, 0.05),
        )

    def _calculate_package_angle(self, bbox):
        """
        Calculate the rotation angle of package from bounding box.

        Args:
            bbox: [x1, y1, x2, y2] bounding box coordinates

        Returns:
            angle: Rotation angle in degrees (-90 to 90)
        """
        x1, y1, x2, y2 = bbox
        points = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)

        rect = cv2.minAreaRect(points)
        (_, _), (width, height), angle = rect

        # Normalize angle to be between -90 and 90
        if width < height:
            angle = angle - 90 if angle > 0 else angle + 90

        if angle < -45:
            angle = 90 + angle
        elif angle > 45:
            angle = angle - 90

        return angle

    def test_detection_loop(self, show_image=True, delay=0.5):
        """
        Main test loop that captures frames and analyzes detections.

        Args:
            show_image: Whether to display annotated image
            delay: Delay between frames in seconds
        """
        frame_count = 0
        detection_count = 0

        self.get_logger().info("=" * 80)
        self.get_logger().info("Starting detection test loop...")
        self.get_logger().info("Press Ctrl+C to stop")
        self.get_logger().info("=" * 80)

        try:
            while rclpy.ok():
                frame_count += 1

                # Capture frame
                frame = self.image_handler.take_photo()

                cv2.imwrite(f"aou{frame_count}.png", frame)

                # Detect packages
                detection = self.yolo_detector.detect(
                    frame=frame, desired_class=["package"], save_image=False
                )

                print(detection)

                if "package" in detection.keys():
                    detection_count += 1
                    package = detection["package"]

                    # Extract detection data
                    center_x, center_y = package["center"]
                    bbox = package["bbox"]
                    confidence = package["confidence"]

                    # Calculate angle
                    angle = self._calculate_package_angle(bbox)

                    # Calculate errors
                    x_error = center_x - IMAGE_CENTER_X
                    y_error = center_y - IMAGE_CENTER_Y

                    # Calculate PID outputs
                    vx = self.pid_x.update(center_y)
                    vy = -self.pid_y.update(center_x)
                    vyaw = self.pid_yaw.update(angle)

                    # Check if aligned
                    x_aligned = abs(x_error) < POSITION_CONTROLLER_TOLERANCE_XY
                    y_aligned = abs(y_error) < POSITION_CONTROLLER_TOLERANCE_XY
                    angle_aligned = abs(angle) < 5.0
                    fully_aligned = x_aligned and y_aligned and angle_aligned

                    # Print results
                    self.get_logger().info("")
                    self.get_logger().info(f"{'=' * 80}")
                    self.get_logger().info(
                        f"Frame: {frame_count} | Detections: {detection_count}"
                    )
                    self.get_logger().info(f"{'=' * 80}")
                    self.get_logger().info(f"Detection Confidence: {confidence:.3f}")
                    self.get_logger().info("")

                    # Position info
                    self.get_logger().info("POSITION:")
                    self.get_logger().info(
                        f"  Center: ({center_x:.0f}, {center_y:.0f})"
                    )
                    self.get_logger().info(
                        f"  Target: ({IMAGE_CENTER_X:.0f}, {IMAGE_CENTER_Y:.0f})"
                    )
                    self.get_logger().info(
                        f"  Error X: {x_error:+.0f}px {'✓' if x_aligned else '✗'}"
                    )
                    self.get_logger().info(
                        f"  Error Y: {y_error:+.0f}px {'✓' if y_aligned else '✗'}"
                    )
                    self.get_logger().info("")

                    # Angle info
                    self.get_logger().info("ALIGNMENT:")
                    self.get_logger().info(
                        f"  Angle: {angle:+.1f}° {'✓' if angle_aligned else '✗'}"
                    )
                    self.get_logger().info(
                        f"  BBox size: {bbox[2]-bbox[0]:.0f}x{bbox[3]-bbox[1]:.0f}px"
                    )
                    self.get_logger().info("")

                    # PID outputs
                    self.get_logger().info("PID OUTPUTS (simulated velocities):")
                    self.get_logger().info(f"  Vx (forward/back): {vx:+.4f} m/s")
                    self.get_logger().info(f"  Vy (left/right):   {vy:+.4f} m/s")
                    self.get_logger().info(f"  Vyaw (rotation):   {vyaw:+.4f} rad/s")
                    self.get_logger().info("")

                    # Status
                    status = "✓ ALIGNED!" if fully_aligned else "✗ NOT ALIGNED"
                    self.get_logger().info(f"STATUS: {status}")
                    self.get_logger().info(f"{'=' * 80}")

                    # Annotate and show image
                    if show_image:
                        annotated_frame = self._annotate_frame(
                            frame.copy(),
                            package,
                            angle,
                            x_error,
                            y_error,
                            vx,
                            vy,
                            vyaw,
                            fully_aligned,
                        )
                        cv2.imwrite(f"dect{frame_count}.png", annotated_frame)
                        #cv2.imshow("Detection Test", annotated_frame)
                        #cv2.waitKey(1)

                else:
                    self.get_logger().info(f"Frame {frame_count}: No package detected")

                    if show_image:
                        # Show frame with crosshair
                        annotated_frame = frame.copy()
                        self._draw_crosshair(annotated_frame)
                        cv2.putText(
                            annotated_frame,
                            "NO DETECTION",
                            (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.5,
                            (0, 0, 255),
                            3,
                        )
                        cv2.imshow("Detection Test", annotated_frame)
                        cv2.waitKey(1)

                time.sleep(delay)

        except KeyboardInterrupt:
            self.get_logger().info("\nTest stopped by user")
        finally:
            if show_image:
                cv2.destroyAllWindows()
            self.get_logger().info("\nTest summary:")
            self.get_logger().info(f"  Total frames: {frame_count}")
            self.get_logger().info(f"  Detections: {detection_count}")
            if frame_count > 0:
                self.get_logger().info(
                    f"  Detection rate: {100*detection_count/frame_count:.1f}%"
                )

    def _annotate_frame(
        self, frame, package, angle, x_error, y_error, vx, vy, vyaw, aligned
    ):
        """Annotate frame with detection info."""
        center_x, center_y = package["center"]
        bbox = package["bbox"]
        confidence = package["confidence"]

        # Draw bounding box
        color = (0, 255, 0) if aligned else (0, 165, 255)
        cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 3)

        # Draw center point
        cv2.circle(frame, (int(center_x), int(center_y)), 8, (0, 0, 255), -1)

        # Draw crosshair at image center
        self._draw_crosshair(frame)

        # Draw line from center to target
        cv2.line(
            frame,
            (int(center_x), int(center_y)),
            (int(IMAGE_CENTER_X), int(IMAGE_CENTER_Y)),
            (255, 0, 255),
            2,
        )

        y_offset = 30
        texts = [
            f"Confidence: {confidence:.2f}",
            f"Center: ({center_x:.0f}, {center_y:.0f})",
            f"Error X: {x_error:+.0f}px, Y: {y_error:+.0f}px",
            f"Angle: {angle:+.1f}deg",
            f"Vx: {vx:+.3f}, Vy: {vy:+.3f}, Vyaw: {vyaw:+.3f}",
            "ALIGNED!" if aligned else "NOT ALIGNED",
        ]

        for i, text in enumerate(texts):
            text_color = (
                (0, 255, 0) if (i == len(texts) - 1 and aligned) else (255, 255, 255)
            )
            cv2.putText(
                frame,
                text,
                (10, y_offset + i * 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                text_color,
                2,
            )

        return frame

    def _draw_crosshair(self, frame, size=30):
        cx, cy = int(IMAGE_CENTER_X), int(IMAGE_CENTER_Y)
        color = (255, 0, 0)
        thickness = 2

        cv2.line(frame, (cx - size, cy), (cx + size, cy), color, thickness)
        cv2.line(frame, (cx, cy - size), (cx, cy + size), color, thickness)
        cv2.circle(frame, (cx, cy), 5, color, thickness)


def main():
    """Main function."""
    rclpy.init()

    try:
        tester = DetectionTester()
        tester.test_detection_loop(show_image=False, delay=0.01)

    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
