#!/usr/bin/env python3

"""
QR Code detection test script for Tello drone and webcam.
Tests camera stream and QR code detection without full state machine.
"""

import rclpy
from rclpy.node import Node
import cv2
import time
import os
import numpy as np
from typing import Optional

from maze.utils.tello_wrapper import TelloWrapper
from maze.utils.qr_detector import QRCodeDetector
from maze.constants import ENTRY_HEIGHT


class WebcamWrapper:
    """Simple webcam wrapper with Tello-like interface."""

    def __init__(self, node: Node, camera_index: int = 0):
        """
        Initialize webcam wrapper.

        Parameters
        ----------
        node : Node
            ROS2 node for logging
        camera_index : int
            Camera device index (0 for default webcam)
        """
        self.node = node
        self.camera_index = camera_index
        self.cap = None
        self.is_connected = False
        self.is_flying = False  # Always False for webcam

    def connect(self) -> bool:
        """Connect to webcam."""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            if self.cap.isOpened():
                # Set camera resolution for better QR detection
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                self.is_connected = True
                self.node.get_logger().info(f"Connected to webcam {self.camera_index}")
                return True
            else:
                self.node.get_logger().error(
                    f"Failed to open webcam {self.camera_index}"
                )
                return False
        except Exception as e:
            self.node.get_logger().error(f"Webcam connection error: {e}")
            return False

    def disconnect(self):
        """Disconnect from webcam."""
        if self.cap:
            self.cap.release()
            self.is_connected = False
            self.node.get_logger().info("Disconnected from webcam")

    def get_battery(self) -> int:
        """Mock battery level for compatibility."""
        return 100  # Always full battery for webcam

    def start_video_stream(self):
        """Compatibility method - webcam is always streaming."""
        self.node.get_logger().info("Webcam stream ready")

    def stop_video_stream(self):
        """Compatibility method."""
        return

    def take_photo(self) -> Optional[np.ndarray]:
        """Capture frame from webcam in RGB format."""
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                # Convert BGR to RGB immediately
                return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return None

    def get_frame(self) -> Optional[np.ndarray]:
        """Get current frame (alias for compatibility)."""
        return self.take_photo()

    def takeoff(self, height: float) -> bool:
        """Mock takeoff for compatibility."""
        self.node.get_logger().info("Webcam doesn't fly :)")
        return False

    def land(self) -> bool:
        """Mock land for compatibility."""
        return True

    def rotate_clockwise(self, degrees: int) -> bool:
        """Mock rotation for compatibility."""
        self.node.get_logger().info(f"Can't rotate webcam {degrees}°")
        return False


def test_qr_detection():
    """Test QR code detection with Tello or webcam."""

    rclpy.init()
    node = Node("qr_test")

    camera = None

    try:
        # Choose camera source
        print("=" * 60)
        print("QR CODE DETECTION TEST")
        print("=" * 60)
        print("\nSelect camera source:")
        print("1. Tello drone camera")
        print("2. Webcam")

        source_choice = input("Select option (1 or 2): ")

        if source_choice == "1":
            # Initialize Tello
            node.get_logger().info("Connecting to Tello...")
            camera = TelloWrapper(node)

            if not camera.connect():
                node.get_logger().error("Failed to connect to Tello")
                return

            battery = camera.get_battery()
            node.get_logger().info(f"Battery: {battery}%")

            if battery < 30:
                node.get_logger().warn("Low battery warning")
                if battery < 20:
                    node.get_logger().error("Battery too low")
                    return

        elif source_choice == "2":
            # Initialize Webcam
            node.get_logger().info("Connecting to webcam...")
            camera_index = input("Enter camera index (default 0): ").strip()
            camera_index = int(camera_index) if camera_index else 0

            camera = WebcamWrapper(node, camera_index)

            if not camera.connect():
                node.get_logger().error("Failed to connect to webcam")
                return

            node.get_logger().info("Webcam connected successfully")

        else:
            print("Invalid option")
            return

        # Initialize QR detector
        qr_detector = QRCodeDetector(node)

        # Start video stream
        node.get_logger().info("Starting video stream...")
        camera.start_video_stream()
        time.sleep(2)  # Wait for stream to stabilize

        # Test options for Tello only
        if source_choice == "1":  # Tello
            print("\nTest Options:")
            print("1. Test on ground (no takeoff)")
            print("2. Test in flight")
            flight_choice = input("Select option (1 or 2): ")

            if flight_choice == "2":
                input("Press Enter to takeoff...")
                if not camera.takeoff(ENTRY_HEIGHT):
                    node.get_logger().error("Takeoff failed")
                    return
                node.get_logger().info(f"Hovering at {ENTRY_HEIGHT}m")

        # Create output directory for saved detections
        output_dir = os.path.expanduser("~/maze_qr_detections")
        os.makedirs(output_dir, exist_ok=True)
        node.get_logger().info(f"Saving detections to: {output_dir}")

        # Detection loop
        node.get_logger().info("Starting QR detection...")
        node.get_logger().info(
            "Press 'q' to quit, 's' to save current frame, 'r' to rotate"
        )

        frame_count = 0
        detection_count = 0

        # Frame rate control for Tello (slower processing)
        frame_skip = 5 if source_choice == "1" else 1  # Skip frames for Tello
        consecutive_failures = 0  # Track consecutive frame failures

        while True:
            # Capture frame with error handling for Tello stream issues
            try:
                frame = camera.take_photo()
                if frame is None:
                    consecutive_failures += 1
                    if consecutive_failures > 10:
                        node.get_logger().warn(
                            "Too many consecutive frame failures, waiting..."
                        )
                        time.sleep(1)  # Longer pause to let stream recover
                        consecutive_failures = 0
                    else:
                        time.sleep(0.1)
                    continue

                # Reset failure counter on success
                consecutive_failures = 0

            except Exception as e:
                # Handle H264 stream errors gracefully
                consecutive_failures += 1
                if "h264" in str(e).lower() or "decode" in str(e).lower():
                    # Common H264 errors, just wait briefly
                    time.sleep(0.1)
                    continue
                else:
                    node.get_logger().error(f"Frame capture error: {e}")
                    time.sleep(0.5)
                    continue

            frame_count += 1

            # Skip frames for Tello to reduce processing load
            if frame_count % frame_skip != 0:
                # Frame is already in RGB, convert to BGR for display
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imshow("QR Detection Test", frame_bgr)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                continue

            # Detect QR codes (frame is in RGB)
            results = qr_detector.detect(frame)

            # Draw detections (returns RGB frame with annotations)
            display_frame_rgb = qr_detector.draw_detections(frame, results)

            # Log and save detections (no cooldown, save every detection)
            if results:
                for result in results:
                    node.get_logger().info(
                        f"QR Code detected: '{result.data}' at position {result.position} (confidence: {result.confidence:.2f})"
                    )
                    detection_count += 1

                # Save single image with all detections
                timestamp = time.strftime("%Y%m%d_%H%M%S_%f")[
                    :-3
                ]  # Include milliseconds
                # Create filename with all detected QR codes
                qr_codes = "_".join(sorted([result.data for result in results]))
                filename = os.path.join(
                    output_dir, f"qr_detection_{qr_codes}_{timestamp}.jpg"
                )

                # Convert RGB to BGR for cv2.imwrite
                display_frame_bgr = cv2.cvtColor(display_frame_rgb, cv2.COLOR_RGB2BGR)
                cv2.imwrite(filename, display_frame_bgr)
                node.get_logger().info(f"Saved detection image to {filename}")

            # Convert RGB to BGR for cv2.imshow display
            display_frame_bgr = cv2.cvtColor(display_frame_rgb, cv2.COLOR_RGB2BGR)
            cv2.imshow("QR Detection Test", display_frame_bgr)

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break
            elif key == ord("s"):
                # Save current frame (convert RGB to BGR for cv2.imwrite)
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(output_dir, f"frame_{timestamp}.jpg")
                frame_bgr_save = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imwrite(filename, frame_bgr_save)
                node.get_logger().info(f"Saved frame to {filename}")
            elif key == ord("r"):
                # Rotate drone (only for Tello)
                if camera.is_flying:
                    node.get_logger().info("Rotating 90° clockwise")
                    camera.rotate_clockwise(90)

            # Display stats every 100 frames
            if frame_count % 100 == 0:
                node.get_logger().info(
                    f"Frames: {frame_count}, Detections: {detection_count}"
                )

        # Print summary
        summary = qr_detector.get_summary()
        node.get_logger().info("Detection Summary:")
        node.get_logger().info(f"  Total frames: {frame_count}")
        node.get_logger().info(f"  Total detections: {summary['total_detections']}")
        node.get_logger().info(f"  Unique QR codes: {summary['unique_codes']}")

        # Land if flying (Tello only)
        if camera and camera.is_flying:
            input("Press Enter to land...")
            if not camera.land():
                node.get_logger().error("Landing failed")

        node.get_logger().info("QR detection test complete!")

    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")
        if camera and camera.is_flying:
            camera.land()

    except Exception as e:
        node.get_logger().error(f"Test failed: {e}")
        import traceback

        traceback.print_exc()
        if camera and hasattr(camera, "emergency_stop") and camera.is_flying:
            camera.emergency_stop()

    finally:
        cv2.destroyAllWindows()
        if camera:
            camera.stop_video_stream()
            camera.disconnect()
        node.destroy_node()
        rclpy.shutdown()


def main():
    test_qr_detection()


if __name__ == "__main__":
    main()
