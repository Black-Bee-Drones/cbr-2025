#!/usr/bin/env python3
"""
Test script for pose-based drone control using YOLO11n-pose.
Displays detected poses and corresponding drone commands.
"""

import cv2
import numpy as np
from ultralytics import YOLO


def draw_angle(img, p1, p2, p3, angle, color=(0, 255, 0)):
    """
    Draw angle arc between three points
    p1 - p2 - p3 forms the angle with p2 as vertex
    """
    # Convert to integer tuples
    p1 = tuple(map(int, p1))
    p2 = tuple(map(int, p2))
    p3 = tuple(map(int, p3))

    # Draw lines
    cv2.line(img, p1, p2, color, 2)
    cv2.line(img, p2, p3, color, 2)

    # Draw angle arc
    radius = 30
    angle_start = np.arctan2(p1[1] - p2[1], p1[0] - p2[0]) * 180 / np.pi
    angle_end = np.arctan2(p3[1] - p2[1], p3[0] - p2[0]) * 180 / np.pi

    # Draw arc
    cv2.ellipse(
        img,
        p2,
        (radius, radius),
        0,
        min(angle_start, angle_end),
        max(angle_start, angle_end),
        color,
        2,
    )

    # Put angle text
    text_pos = (p2[0] + 35, p2[1])
    cv2.putText(
        img, f"{int(angle)}°", text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
    )

    return img


def draw_arm_angle(img, shoulder, wrist, angle, label, color=(255, 255, 0)):
    """
    Draw the arm angle relative to horizontal
    """
    # Convert to integer tuples
    shoulder = tuple(map(int, shoulder))
    wrist = tuple(map(int, wrist))

    # Draw arm line
    cv2.line(img, shoulder, wrist, color, 3)

    # Draw horizontal reference line (dashed effect by drawing small segments)
    horizon_end = (shoulder[0] + 60, shoulder[1])
    for i in range(0, 60, 10):
        cv2.line(
            img,
            (shoulder[0] + i, shoulder[1]),
            (shoulder[0] + i + 5, shoulder[1]),
            (128, 128, 128),
            1,
        )

    # Draw angle arc
    radius = 40
    cv2.ellipse(
        img, shoulder, (radius, radius), 0, 0, angle if angle > 0 else 0, color, 1
    )

    # Put angle text
    text_pos = (shoulder[0] - 30, shoulder[1] - 45)
    cv2.putText(
        img,
        f"{label}: {int(angle)}°",
        text_pos,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        color,
        1,
    )

    return img


def detect_gesture(keypoints, return_angles=False):
    """
    Analyze keypoints to determine the gesture.

    Parameters
    ----------
    keypoints : np.array
        Array of keypoints [17, 2] in COCO format
    return_angles : bool
        If True, also return calculated angles

    Returns
    -------
    str or None or tuple
        Detected gesture name or None if no gesture detected
        If return_angles=True, returns (gesture, angles_dict)
    """
    if keypoints is None or len(keypoints) < 17:
        if return_angles:
            return None, None
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
        if return_angles:
            return None, None
        return None

    # Calculate angles for arms
    def calculate_arm_angle(shoulder, elbow, wrist):
        """Calculate the angle of the arm relative to horizontal"""
        arm_vector = wrist - shoulder
        angle = np.arctan2(arm_vector[1], arm_vector[0]) * 180 / np.pi
        return angle

    left_angle = calculate_arm_angle(left_shoulder, left_elbow, left_wrist)
    right_angle = calculate_arm_angle(right_shoulder, right_elbow, right_wrist)

    # Calculate elbow angles (flexion)
    def calculate_elbow_angle(shoulder, elbow, wrist):
        """Calculate the elbow flexion angle"""
        v1 = shoulder - elbow
        v2 = wrist - elbow
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0)) * 180 / np.pi
        return angle

    left_elbow_angle = calculate_elbow_angle(left_shoulder, left_elbow, left_wrist)
    right_elbow_angle = calculate_elbow_angle(right_shoulder, right_elbow, right_wrist)

    # Store angles for visualization
    angles_data = {
        "left_shoulder": left_shoulder,
        "right_shoulder": right_shoulder,
        "left_elbow": left_elbow,
        "right_elbow": right_elbow,
        "left_wrist": left_wrist,
        "right_wrist": right_wrist,
        "left_arm_angle": left_angle,
        "right_arm_angle": right_angle,
        "left_elbow_angle": left_elbow_angle,
        "right_elbow_angle": right_elbow_angle,
    }

    # Detect gestures based on arm positions and angles
    gesture = None

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
        gesture = "double_biceps"

    # Cross arms: arms crossed in front
    elif (
        abs(left_wrist[0] - right_shoulder[0]) < 50
        and abs(right_wrist[0] - left_shoulder[0]) < 50
    ):
        gesture = "cross_arms"

    # Right arm up, left arm to side
    elif right_angle < -60 and right_angle > -120 and abs(left_angle) < 45:
        gesture = "right_arm_up_left_arm_side"

    # Left arm up, right arm to side
    elif (
        left_angle < -60
        and left_angle > -120
        and (abs(right_angle - 180) < 45 or abs(right_angle) < 45)
    ):
        gesture = "left_arm_up_right_arm_side"

    # Both arms up (extended)
    elif (
        left_angle < -60
        and left_angle > -120
        and right_angle < -60
        and right_angle > -120
        and left_elbow_angle > 140
        and right_elbow_angle > 140
    ):
        gesture = "both_arms_up"

    # Both arms down (extended)
    elif (
        left_angle > 60 and left_angle < 120 and right_angle > 60 and right_angle < 120
    ):
        gesture = "both_arms_down"

    # Right arm biceps, left arm down (MOVE FORWARD)
    # Right arm showing biceps pose, left arm relaxed down
    elif (
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
        gesture = "right_arm_biceps_left_arm_down"

    # Left arm biceps, right arm down (MOVE BACKWARD)
    # Left arm showing biceps pose, right arm relaxed down
    elif (
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
        gesture = "left_arm_biceps_right_arm_down"

    # Left arm down, right arm to side (YAW RIGHT)
    # Right arm horizontal, left arm pointing down
    elif (
        (
            abs(right_angle - 180) < 45 or abs(right_angle) < 45
        )  # Right arm horizontal to side
        and right_elbow_angle > 140  # Right arm extended
        and left_angle > 45
        and left_angle < 135  # Left arm down
    ):
        gesture = "left_arm_down_right_arm_side"

    # Right arm down, left arm to side (YAW LEFT)
    # Left arm horizontal, right arm pointing down
    elif (
        (
            abs(left_angle - 180) < 45 or abs(left_angle) < 45
        )  # Left arm horizontal to side
        and left_elbow_angle > 140  # Left arm extended
        and right_angle > 45
        and right_angle < 135  # Right arm down
    ):
        gesture = "right_arm_down_left_arm_side"

    else:
        gesture = "neutral"

    if return_angles:
        return gesture, angles_data
    return gesture


# Gesture to command mapping
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


def main():
    """Main function to run pose detection and command interpretation."""

    # Gesture confirmation threshold settings
    GESTURE_CONFIRMATION_THRESHOLD = 5
    MAX_GESTURE_FRAMES = 300

    # Gesture confirmation variables
    current_gesture = None
    previous_gesture = None
    gesture_detection_count = 0
    confirmed_gesture = None
    total_gesture_frames = 0

    print("Loading YOLO11n-pose model...")
    model = YOLO("yolo11n-pose.pt")

    # Initialize webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Cannot access webcam")
        return

    # Set webcam resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("\n" + "=" * 60)
    print("POSE-BASED DRONE CONTROL TEST")
    print("=" * 60)
    print(f"Gesture Confirmation Threshold: {GESTURE_CONFIRMATION_THRESHOLD} frames")
    print("\nGestures and Commands:")
    for gesture, command in GESTURE_COMMANDS.items():
        if gesture != "neutral":
            print(f"  {gesture:30s} → {command}")
    print("\nPress 'q' to quit")
    print("=" * 60 + "\n")

    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("Failed to read frame")
            break

        # Run pose detection
        results = model(frame, verbose=False)

        detected_gesture = None
        detected_command = None
        angles_data = None

        for r in results:
            if r.keypoints is not None and len(r.keypoints) > 0:
                keypoints = r.keypoints.xy.cpu().numpy()

                # Process first person detected
                if len(keypoints) > 0:
                    detected_gesture, angles_data = detect_gesture(
                        keypoints[0], return_angles=True
                    )

        # Gesture confirmation logic
        previous_gesture = current_gesture
        current_gesture = detected_gesture

        # Check if gesture changed
        if previous_gesture != current_gesture:
            gesture_detection_count = 0
            confirmed_gesture = None
            total_gesture_frames = 0

            if current_gesture and current_gesture != "neutral":
                print(
                    f"[NEW] Gesture: {current_gesture} - Confirming... (0/{GESTURE_CONFIRMATION_THRESHOLD})"
                )

        # Increment detection counter for same gesture
        elif current_gesture and current_gesture != "neutral":
            gesture_detection_count += 1

            if gesture_detection_count <= GESTURE_CONFIRMATION_THRESHOLD:
                print(
                    f"[CONFIRMING] {current_gesture}: {gesture_detection_count}/{GESTURE_CONFIRMATION_THRESHOLD}"
                )

        # Check if gesture is confirmed
        if (
            gesture_detection_count >= GESTURE_CONFIRMATION_THRESHOLD
            and confirmed_gesture != current_gesture
        ):
            confirmed_gesture = current_gesture
            detected_command = GESTURE_COMMANDS.get(confirmed_gesture, "UNKNOWN")
            print(
                f"[✓ CONFIRMED] Gesture: {confirmed_gesture:25s} | Command: {detected_command}"
            )

        # Update total frames for confirmed gesture
        if confirmed_gesture:
            total_gesture_frames += 1

            # Reset if held too long
            if total_gesture_frames > MAX_GESTURE_FRAMES:
                print(f"[RESET] Gesture '{confirmed_gesture}' held too long")
                gesture_detection_count = 0
                confirmed_gesture = None
                total_gesture_frames = 0

        # Set command for display
        if confirmed_gesture:
            detected_command = GESTURE_COMMANDS.get(confirmed_gesture, "UNKNOWN")

        # Draw skeleton and angles if pose was detected
        if detected_gesture:
            for r in results:
                if r.keypoints is not None and len(r.keypoints) > 0:
                    # Draw skeleton on frame
                    annotated_frame = r.plot()
                    frame = annotated_frame
                    break

            # Draw angles if available
            if angles_data:
                # Draw elbow angles
                draw_angle(
                    frame,
                    angles_data["left_shoulder"],
                    angles_data["left_elbow"],
                    angles_data["left_wrist"],
                    angles_data["left_elbow_angle"],
                    color=(0, 255, 255),
                )  # Yellow

                draw_angle(
                    frame,
                    angles_data["right_shoulder"],
                    angles_data["right_elbow"],
                    angles_data["right_wrist"],
                    angles_data["right_elbow_angle"],
                    color=(255, 0, 255),
                )  # Magenta

                # Draw arm angles relative to horizontal
                draw_arm_angle(
                    frame,
                    angles_data["left_shoulder"],
                    angles_data["left_wrist"],
                    angles_data["left_arm_angle"],
                    "L.Arm",
                    color=(0, 255, 0),
                )  # Green

                draw_arm_angle(
                    frame,
                    angles_data["right_shoulder"],
                    angles_data["right_wrist"],
                    angles_data["right_arm_angle"],
                    "R.Arm",
                    color=(0, 165, 255),
                )  # Orange

                # Display angle values in a panel
                cv2.rectangle(
                    frame,
                    (10, frame.shape[0] - 100),
                    (250, frame.shape[0] - 10),
                    (0, 0, 0),
                    -1,
                )
                cv2.rectangle(
                    frame,
                    (10, frame.shape[0] - 100),
                    (250, frame.shape[0] - 10),
                    (255, 255, 255),
                    1,
                )

                cv2.putText(
                    frame,
                    f"L.Arm: {int(angles_data['left_arm_angle'])}° | L.Elbow: {int(angles_data['left_elbow_angle'])}°",
                    (15, frame.shape[0] - 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1,
                )

                cv2.putText(
                    frame,
                    f"R.Arm: {int(angles_data['right_arm_angle'])}° | R.Elbow: {int(angles_data['right_elbow_angle'])}°",
                    (15, frame.shape[0] - 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 255),
                    1,
                )

                cv2.putText(
                    frame,
                    "Angles (from horizontal)",
                    (15, frame.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (128, 128, 128),
                    1,
                )

        # Draw confirmation status and gesture display
        if confirmed_gesture and detected_command:
            # Confirmed gesture - draw in color
            color = COMMAND_COLORS.get(detected_command, (255, 255, 255))

            # Draw background rectangles for better text visibility
            cv2.rectangle(frame, (10, 10), (630, 130), (0, 0, 0), -1)
            cv2.rectangle(frame, (10, 10), (630, 130), color, 3)

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
                f"Gesture: {confirmed_gesture}",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )

            # Draw command
            cv2.putText(
                frame,
                f"Command: {detected_command}",
                (20, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                color,
                3,
            )

        elif current_gesture and current_gesture != "neutral":
            # Gesture detected but not confirmed - show confirmation progress
            progress_color = (
                (255, 165, 0)
                if gesture_detection_count < GESTURE_CONFIRMATION_THRESHOLD
                else (0, 255, 0)
            )

            cv2.rectangle(frame, (10, 10), (630, 130), (0, 0, 0), -1)
            cv2.rectangle(frame, (10, 10), (630, 130), progress_color, 2)

            # Draw confirmation status
            cv2.putText(
                frame,
                f"CONFIRMING... {gesture_detection_count}/{GESTURE_CONFIRMATION_THRESHOLD}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                progress_color,
                2,
            )

            # Draw detected gesture
            cv2.putText(
                frame,
                f"Gesture: {current_gesture}",
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
            progress = min(
                gesture_detection_count / GESTURE_CONFIRMATION_THRESHOLD, 1.0
            )

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

        # Display frame
        cv2.imshow("Pose Control Test", frame)

        # Break on 'q' key
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    print("\nTest completed.")


if __name__ == "__main__":
    main()
