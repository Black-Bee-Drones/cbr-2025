#!/usr/bin/env python3
"""
Test script for pose-based drone control using YOLO11n-pose.
Replicates exact behavior of PoseControl state.
Usage: ros2 run interaction test_detection [--no-viz]
"""

import cv2
import time
import argparse
from interaction.utils.yolo_detector import YoloDetector
from interaction.constants import (
    ACTION_TIMEOUT,
    YOLO_IMAGE_SIZE,
    TAKEOFF_POSE,
    VELOCITY_UP_DOWN,
    VELOCITY_SIDES,
    VELOCITY_IN_OUT,
    VELOCITY_YAW,
    GESTURE_CONFIRMATION_THRESHOLD,
    GESTURE_CONFIRMATION_THRESHOLD_SINGLE,
)

# Gesture to command mapping (matching pose_control_state.py)
GESTURE_COMMANDS = {
    "double_biceps": "TAKEOFF",
    "cross_arms": "LAND",
    "right_arm_up_left_arm_side": "MOVE LEFT",
    "left_arm_up_right_arm_side": "MOVE RIGHT",
    "both_arms_down": "MOVE DOWN",
    "both_arms_up": "MOVE UP",
    "right_arm_biceps_left_arm_down": "MOVE FORWARD",
    "left_arm_biceps_right_arm_down": "MOVE BACKWARD",
    "left_arm_down_right_arm_side": "YAW RIGHT",
    "right_arm_down_left_arm_side": "YAW LEFT",
    "neutral": "STOP/HOVER",
}

# Drone command details for each gesture
GESTURE_DRONE_COMMANDS = {
    "double_biceps": f"arm_takeoff({TAKEOFF_POSE})",
    "cross_arms": "land()",
    "right_arm_up_left_arm_side": f"offboard_velocity(linear_y={-VELOCITY_SIDES})",
    "left_arm_up_right_arm_side": f"offboard_velocity(linear_y={VELOCITY_SIDES})",
    "both_arms_down": f"offboard_velocity(linear_z={-VELOCITY_UP_DOWN})",
    "both_arms_up": f"offboard_velocity(linear_z={VELOCITY_UP_DOWN})",
    "right_arm_biceps_left_arm_down": f"offboard_velocity(linear_x={VELOCITY_IN_OUT})",
    "left_arm_biceps_right_arm_down": f"offboard_velocity(linear_x={-VELOCITY_IN_OUT})",
    "left_arm_down_right_arm_side": f"offboard_velocity(angular_z={-VELOCITY_YAW})",
    "right_arm_down_left_arm_side": f"offboard_velocity(angular_z={VELOCITY_YAW})",
    "neutral": "offboard_velocity(0.0, 0.0, 0.0, 0.0)",
}

# Command colors for visualization
COMMAND_COLORS = {
    "TAKEOFF": (0, 255, 0),  # Green
    "LAND": (0, 0, 255),  # Red
    "MOVE LEFT": (255, 255, 0),  # Cyan
    "MOVE RIGHT": (255, 0, 255),  # Magenta
    "MOVE DOWN": (0, 165, 255),  # Orange
    "MOVE UP": (255, 0, 0),  # Blue
    "MOVE FORWARD": (0, 255, 128),  # Spring Green
    "MOVE BACKWARD": (128, 255, 0),  # Chartreuse
    "YAW RIGHT": (128, 0, 128),  # Purple
    "YAW LEFT": (255, 192, 203),  # Pink
    "STOP/HOVER": (128, 128, 128),  # Gray
}


class PoseControlTester:
    """
    Replicates PoseControl state behavior for testing.
    Matches exactly the process_gesture_action logic.
    """

    def __init__(self, show_visualization: bool = True):
        """Initialize the tester."""
        self.show_visualization = show_visualization

        # YoloDetector instance
        self.yolo_detector = None

        # Gesture tracking (matching pose_control_state.py)
        self.current_gesture = None
        self.previous_gesture = None
        self.gesture_start_time = 0.0
        self.command_sent = False

        # Gesture confirmation variables
        self.gesture_detection_count = 0
        self.confirmed_gesture = None
        self.total_gesture_frames = 0

        # FPS tracking
        self.fps = 0.0
        self.frame_times = []

        # Command execution tracking
        self.continuous_command_count = 0
        self.last_command_time = 0

    def initialize_detector(self):
        """Initialize YOLO detector using the YoloDetector class."""
        print("Initializing YOLO detector...")
        self.yolo_detector = YoloDetector()
        if not self.yolo_detector.load_model():
            print("Error: Failed to load YOLO model")
            return False
        print("YOLO detector loaded successfully.")
        return True

    def process_gesture_action(self, gesture: str) -> None:
        """
        Process the action for detected gesture with confirmation threshold.
        EXACT COPY of pose_control_state.py logic.

        Single actions (takeoff, land) require GESTURE_CONFIRMATION_THRESHOLD_SINGLE frames.
        Continuous gestures require GESTURE_CONFIRMATION_THRESHOLD frames.
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
            self.continuous_command_count = 0

            if gesture and gesture != "neutral":
                print(
                    f"[NEW] Gesture: {gesture} - Confirming... (0/{required_threshold})"
                )
            return

        if self.current_gesture and self.current_gesture != "neutral":
            self.gesture_detection_count += 1

            if self.gesture_detection_count <= required_threshold:
                print(
                    f"[CONFIRMING] {self.current_gesture}: "
                    f"{self.gesture_detection_count}/{required_threshold}"
                )

        # Check if gesture is confirmed (reached threshold)
        if (
            self.gesture_detection_count >= required_threshold
            and self.confirmed_gesture != self.current_gesture
        ):
            self.confirmed_gesture = self.current_gesture
            self.gesture_start_time = time.time()
            self.last_command_time = time.time()
            command = GESTURE_COMMANDS.get(self.confirmed_gesture, "UNKNOWN")
            print(
                f"[✓ CONFIRMED] Gesture: {self.confirmed_gesture:30s} | Command: {command}"
            )

        if not self.confirmed_gesture or self.confirmed_gesture == "neutral":
            if self.previous_gesture and self.previous_gesture != "neutral":
                print(
                    f"[DRONE CMD] mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0) - STOP"
                )
            return

        self.total_gesture_frames += 1

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
                # For continuous gestures, command is sent repeatedly
                self.continuous_command_count += 1
                drone_cmd = GESTURE_DRONE_COMMANDS.get(
                    self.confirmed_gesture, "UNKNOWN"
                )
                current_time = time.time()
                time_since_last = current_time - self.last_command_time
                self.last_command_time = current_time

                print(
                    f"[DRONE CMD] mavdrone.{drone_cmd} "
                    f"[CONTINUOUS #{self.continuous_command_count}, Δt={time_since_last:.3f}s]"
                )

            elif self.confirmed_gesture in single_gestures and not self.command_sent:
                command = GESTURE_COMMANDS.get(self.confirmed_gesture, "UNKNOWN")
                drone_cmd = GESTURE_DRONE_COMMANDS.get(
                    self.confirmed_gesture, "UNKNOWN"
                )
                print(
                    f"[DRONE CMD] mavdrone.{drone_cmd} "
                    f"[SINGLE ACTION - Executed ONCE]"
                )
                self.command_sent = True

    def calculate_fps(self):
        """Calculate current FPS."""
        current_time = time.time()
        self.frame_times.append(current_time)

        # Keep only last 30 frames for FPS calculation
        if len(self.frame_times) > 30:
            self.frame_times.pop(0)

        if len(self.frame_times) > 1:
            elapsed = self.frame_times[-1] - self.frame_times[0]
            self.fps = (len(self.frame_times) - 1) / elapsed if elapsed > 0 else 0.0

    def draw_visualization(self, frame, pose_results):
        """Draw visualization on frame including skeleton."""
        # Draw skeleton/keypoints if available
        if pose_results is not None:
            for r in pose_results:
                if r.keypoints is not None and len(r.keypoints) > 0:
                    # Draw skeleton using YOLO's built-in plot
                    frame = r.plot()
                    break

        # Draw FPS
        cv2.putText(
            frame,
            f"FPS: {self.fps:.1f}",
            (frame.shape[1] - 120, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        # Get command for display
        detected_command = None
        if self.confirmed_gesture:
            detected_command = GESTURE_COMMANDS.get(self.confirmed_gesture, "UNKNOWN")

        # Draw confirmation status and gesture display
        if self.confirmed_gesture and detected_command:
            # Confirmed gesture - draw in color
            color = COMMAND_COLORS.get(detected_command, (255, 255, 255))

            # Draw background rectangles for better text visibility
            cv2.rectangle(frame, (10, 10), (630, 180), (0, 0, 0), -1)
            cv2.rectangle(frame, (10, 10), (630, 180), color, 3)

            # Draw status
            cv2.putText(
                frame,
                "✓ CONFIRMED",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            # Draw gesture name
            cv2.putText(
                frame,
                f"Gesture: {self.confirmed_gesture}",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            # Draw command
            cv2.putText(
                frame,
                f"Command: {detected_command}",
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                color,
                2,
            )

            # Draw drone command details
            drone_cmd = GESTURE_DRONE_COMMANDS.get(self.confirmed_gesture, "")
            # Truncate if too long
            if len(drone_cmd) > 50:
                drone_cmd = drone_cmd[:47] + "..."

            cv2.putText(
                frame,
                f"Drone: {drone_cmd}",
                (20, 140),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (200, 200, 200),
                1,
            )

            # Show execution count for continuous gestures
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

            if self.confirmed_gesture in continuous_gestures:
                cv2.putText(
                    frame,
                    f"CONTINUOUS - Sent {self.continuous_command_count}x",
                    (20, 165),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1,
                )
            else:
                status = "SENT" if self.command_sent else "WAITING"
                cv2.putText(
                    frame,
                    f"SINGLE ACTION - {status}",
                    (20, 165),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                    1,
                )

        elif self.current_gesture and self.current_gesture != "neutral":
            # Gesture detected but not confirmed - show confirmation progress
            single_gestures = ["double_biceps", "cross_arms"]
            required_threshold = (
                GESTURE_CONFIRMATION_THRESHOLD_SINGLE
                if self.current_gesture in single_gestures
                else GESTURE_CONFIRMATION_THRESHOLD
            )

            progress_color = (
                (255, 165, 0)
                if self.gesture_detection_count < required_threshold
                else (0, 255, 0)
            )

            cv2.rectangle(frame, (10, 10), (630, 130), (0, 0, 0), -1)
            cv2.rectangle(frame, (10, 10), (630, 130), progress_color, 2)

            # Draw confirmation status
            cv2.putText(
                frame,
                f"CONFIRMING... {self.gesture_detection_count}/{required_threshold}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                progress_color,
                2,
            )

            # Draw detected gesture
            cv2.putText(
                frame,
                f"Gesture: {self.current_gesture}",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (200, 200, 200),
                2,
            )

            # Draw progress bar
            bar_width = 600
            bar_height = 20
            bar_x = 20
            bar_y = 90
            progress = min(self.gesture_detection_count / required_threshold, 1.0)

            # Background bar
            cv2.rectangle(
                frame,
                (bar_x, bar_y),
                (bar_x + bar_width, bar_y + bar_height),
                (50, 50, 50),
                -1,
            )
            # Progress bar
            cv2.rectangle(
                frame,
                (bar_x, bar_y),
                (bar_x + int(bar_width * progress), bar_y + bar_height),
                progress_color,
                -1,
            )
            # Border
            cv2.rectangle(
                frame,
                (bar_x, bar_y),
                (bar_x + bar_width, bar_y + bar_height),
                (255, 255, 255),
                1,
            )

        else:
            # No gesture detected
            cv2.rectangle(frame, (10, 10), (300, 50), (0, 0, 0), -1)
            cv2.putText(
                frame,
                "No gesture detected",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (128, 128, 128),
                2,
            )

        return frame

    def run(self):
        """Run the pose detection test."""
        # Initialize detector
        if not self.initialize_detector():
            return

        # Initialize webcam
        def _build_gstreamer_pipeline() -> str:
            """Build the GStreamer pipeline string for IMX219 camera."""
            return (
                f"nvarguscamerasrc sensor-id={0} ! "
                f"video/x-raw(memory:NVMM), width=(int){YOLO_IMAGE_SIZE}, height=(int){YOLO_IMAGE_SIZE}, "
                f"framerate=(fraction){30}/1, format=(string)NV12 ! "
                f"nvvidconv flip-method={2} ! "
                f"video/x-raw, width=(int){YOLO_IMAGE_SIZE}, height=(int){YOLO_IMAGE_SIZE}, format=(string)BGRx ! "
                f"videoconvert ! "
                f"video/x-raw, format=(string)BGR ! "
                f"appsink max-buffer=1 drop=true sync=false"
            )
    
        gstreamer_pipeline = _build_gstreamer_pipeline()
        cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
        
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open IMX219 camera (sensor_id={self._config.sensor_id})")

        print("\n" + "=" * 80)
        print("POSE-BASED DRONE CONTROL TEST")
        print("=" * 80)
        print(f"Confirmation Thresholds:")
        print(f"  - Continuous gestures: {GESTURE_CONFIRMATION_THRESHOLD} frames")
        print(
            f"  - Single actions:      {GESTURE_CONFIRMATION_THRESHOLD_SINGLE} frames"
        )
        print(f"Action Timeout: {ACTION_TIMEOUT}s")
        print(f"Visualization: {'ENABLED' if self.show_visualization else 'DISABLED'}")
        print("\nGestures and Commands:")
        for gesture, command in GESTURE_COMMANDS.items():
            if gesture != "neutral":
                drone_cmd = GESTURE_DRONE_COMMANDS.get(gesture, "")
                is_single = gesture in ["double_biceps", "cross_arms"]
                threshold = (
                    GESTURE_CONFIRMATION_THRESHOLD_SINGLE
                    if is_single
                    else GESTURE_CONFIRMATION_THRESHOLD
                )
                action_type = (
                    f"[SINGLE-{threshold}]"
                    if is_single
                    else f"[CONTINUOUS-{threshold}]"
                )
                print(f"  {gesture:30s} → {command:15s} {action_type}")
                print(f"    Drone: mavdrone.{drone_cmd}")
        print("\nPress 'q' to quit")
        print("=" * 80 + "\n")

        try:
            while cap.isOpened():
                success, frame = cap.read()
                if not success:
                    print("Failed to read frame")
                    break

                # Calculate FPS
                self.calculate_fps()

                # Detect gesture using YoloDetector and get pose results for visualization
                detected_gesture = self.yolo_detector.detect_gesture(frame)

                # Get pose results for drawing skeleton
                pose_results = None
                if self.show_visualization and self.yolo_detector.model is not None:
                    try:
                        pose_results = self.yolo_detector.model(frame, verbose=False)
                    except:
                        pass

                # Process gesture action (exact copy of pose_control_state.py logic)
                self.process_gesture_action(detected_gesture)

                # Draw visualization if enabled
                if self.show_visualization:
                    display_frame = self.draw_visualization(frame.copy(), pose_results)
                    cv2.imshow("Pose Control Test", display_frame)

                    # Break on 'q' key
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                else:
                    # Small delay to prevent 100% CPU usage
                    time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n\nTest interrupted by user.")

        finally:
            # Cleanup
            cap.release()
            if self.show_visualization:
                cv2.destroyAllWindows()
            print("\nTest completed.")
            print(f"Final FPS: {self.fps:.1f}")


def main():
    """Main function with argument parsing."""
    parser = argparse.ArgumentParser(
        description="Test pose-based drone control with YOLO11n-pose"
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Disable visualization (no OpenCV window)",
    )

    args = parser.parse_args()

    # Run tester
    tester = PoseControlTester(show_visualization=not args.no_viz)
    tester.run()


if __name__ == "__main__":
    main()
