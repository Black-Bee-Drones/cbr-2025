#!/usr/bin/env python3
import sys
import math
from typing import List, Dict, Tuple

from mapping.states.navigation_states import CaptureAndDetect
from mapping.constants import (
    DUPLICATE_BASE_RADIUS,
    CAMERA_FOV_HORIZONTAL,
    CAMERA_FOV_VERTICAL,
    IMAGE_CENTER_X,
    IMAGE_CENTER_Y,
)


def create_test_detection(pixel_x: int, pixel_y: int, confidence: float = 0.8) -> Dict:
    """Create a test detection dictionary."""
    return {
        "center": (pixel_x, pixel_y),
        "bbox": [pixel_x - 40, pixel_y - 40, pixel_x + 40, pixel_y + 40],
        "confidence": confidence,
        "class_id": 0,
        "area": 6400.0,
    }


def test_meters_per_pixel():
    """Test meters per pixel calculation at different altitudes."""
    print("\n" + "=" * 60)
    print("TEST 1: Meters per Pixel Calculation")
    print("=" * 60)

    test_altitudes = [2.0, 3.0, 4.0]

    for altitude in test_altitudes:
        mpx_x, mpx_y = CaptureAndDetect.calculate_meters_per_pixel(altitude)

        # Calculate expected coverage
        coverage_width = (
            2 * altitude * math.tan(math.radians(CAMERA_FOV_HORIZONTAL / 2))
        )
        coverage_height = 2 * altitude * math.tan(math.radians(CAMERA_FOV_VERTICAL / 2))

        print(f"\nAltitude: {altitude}m")
        print(f"  Coverage: {coverage_width:.2f}m x {coverage_height:.2f}m")
        print(f"  Meters/pixel X: {mpx_x:.5f}")
        print(f"  Meters/pixel Y: {mpx_y:.5f}")
        print(f"  Pixels for 1m X: {1/mpx_x:.1f}")
        print(f"  Pixels for 1m Y: {1/mpx_y:.1f}")

    return True


def test_position_estimation():
    """Test position estimation from pixel coordinates."""
    print("\n" + "=" * 60)
    print("TEST 2: Position Estimation")
    print("=" * 60)

    drone_x, drone_y = 5.0, 5.0
    altitude = 3.0

    test_cases = [
        # (pixel_x, pixel_y, description)
        (IMAGE_CENTER_X, IMAGE_CENTER_Y, "Center - should be at drone position"),
        (IMAGE_CENTER_X + 100, IMAGE_CENTER_Y, "Right of center"),
        (IMAGE_CENTER_X - 100, IMAGE_CENTER_Y, "Left of center"),
        (IMAGE_CENTER_X, IMAGE_CENTER_Y + 100, "Below center (forward)"),
        (IMAGE_CENTER_X, IMAGE_CENTER_Y - 100, "Above center (backward)"),
    ]

    for pixel_x, pixel_y, description in test_cases:
        detection = create_test_detection(pixel_x, pixel_y)
        est_x, est_y = CaptureAndDetect.estimate_detection_position(
            detection, drone_x, drone_y, altitude
        )

        offset_from_drone = math.sqrt((est_x - drone_x) ** 2 + (est_y - drone_y) ** 2)

        print(f"\n{description}")
        print(f"  Pixel: ({pixel_x}, {pixel_y})")
        print(f"  Drone position: ({drone_x:.2f}, {drone_y:.2f})")
        print(f"  Estimated position: ({est_x:.2f}, {est_y:.2f})")
        print(f"  Distance from drone: {offset_from_drone:.2f}m")

    return True


def test_duplicate_detection():
    """Test duplicate detection logic."""
    print("\n" + "=" * 60)
    print("TEST 3: Duplicate Detection")
    print("=" * 60)

    visited_bases = [
        {"x": 5.0, "y": 5.0, "z": 0.0},
        {"x": 8.0, "y": 8.0, "z": 0.0},
    ]

    test_positions = [
        # (x, y, expected_duplicate, description)
        (5.5, 5.5, True, "Within radius of first base"),
        (5.0 + DUPLICATE_BASE_RADIUS - 0.1, 5.0, True, "Just inside X threshold"),
        (5.0, 5.0 + DUPLICATE_BASE_RADIUS - 0.1, True, "Just inside Y threshold"),
        (5.0 + DUPLICATE_BASE_RADIUS + 0.1, 5.0, False, "Just outside X threshold"),
        (5.0, 5.0 + DUPLICATE_BASE_RADIUS + 0.1, False, "Just outside Y threshold"),
        (7.0, 7.0, True, "Between bases (within radius of second)"),  # 1.0m from (8,8)
        (10.0, 10.0, False, "Far from all bases"),
    ]

    print(f"Visited bases: {visited_bases}")
    print(f"Duplicate radius: {DUPLICATE_BASE_RADIUS}m\n")

    all_passed = True
    for x, y, expected_duplicate, description in test_positions:
        is_duplicate = CaptureAndDetect.is_duplicate_detection((x, y), visited_bases)

        status = "✓" if is_duplicate == expected_duplicate else "✗"
        print(f"{status} {description}")
        print(f"  Position: ({x:.2f}, {y:.2f})")
        print(f"  Expected: {'Duplicate' if expected_duplicate else 'Valid'}")
        print(f"  Result: {'Duplicate' if is_duplicate else 'Valid'}")

        if is_duplicate != expected_duplicate:
            all_passed = False

    return all_passed


def test_detection_ordering():
    """Test detection ordering for different movement directions."""
    print("\n" + "=" * 60)
    print("TEST 4: Detection Ordering")
    print("=" * 60)

    # Create detections in random order
    detections = [
        create_test_detection(800, 600),  # Top-left
        create_test_detection(900, 600),  # Top-right
        create_test_detection(800, 700),  # Bottom-left
        create_test_detection(900, 700),  # Bottom-right
        create_test_detection(850, 650),  # Center
    ]

    print("Original detection positions:")
    for i, det in enumerate(detections):
        print(f"  {i}: pixel ({det['center'][0]}, {det['center'][1]})")

    # Test forward movement ordering
    print("\nForward movement ordering (bottom first, left to right):")
    ordered_forward = CaptureAndDetect.order_detections(detections, moving_forward=True)
    for i, det in enumerate(ordered_forward):
        print(f"  {i}: pixel ({det['center'][0]}, {det['center'][1]})")

    # Verify forward ordering
    if (
        ordered_forward[0]["center"] == (800, 700)  # Bottom-left first
        and ordered_forward[1]["center"] == (900, 700)  # Bottom-right second
        and ordered_forward[-1]["center"] == (900, 600)
    ):  # Top-right last
        print("  ✓ Forward ordering correct")
    else:
        print("  ✗ Forward ordering incorrect")
        return False

    # Test backward movement ordering
    print("\nBackward movement ordering (top first, left to right):")
    ordered_backward = CaptureAndDetect.order_detections(
        detections, moving_forward=False
    )
    for i, det in enumerate(ordered_backward):
        print(f"  {i}: pixel ({det['center'][0]}, {det['center'][1]})")

    # Verify backward ordering
    if (
        ordered_backward[0]["center"] == (800, 600)  # Top-left first
        and ordered_backward[1]["center"] == (900, 600)  # Top-right second
        and ordered_backward[-1]["center"] == (900, 700)
    ):  # Bottom-right last
        print("  ✓ Backward ordering correct")
    else:
        print("  ✗ Backward ordering incorrect")
        return False

    return True


def test_multiple_detection_scenario():
    """Test complete scenario with multiple detections."""
    print("\n" + "=" * 60)
    print("TEST 5: Multiple Detection Scenario")
    print("=" * 60)

    # Setup
    drone_x, drone_y = 5.0, 5.0
    altitude = 3.0
    visited_bases = [
        {"x": 4.0, "y": 4.0, "z": 0.0},  # Already visited base
    ]

    # Create multiple detections
    detections = [
        create_test_detection(
            IMAGE_CENTER_X - 200, IMAGE_CENTER_Y + 100, 0.85
        ),  # Left-forward
        create_test_detection(
            IMAGE_CENTER_X + 150, IMAGE_CENTER_Y + 150, 0.75
        ),  # Right-forward
        create_test_detection(
            IMAGE_CENTER_X - 100, IMAGE_CENTER_Y - 50, 0.90
        ),  # Left-back (duplicate)
    ]

    print(f"Drone position: ({drone_x:.2f}, {drone_y:.2f}), altitude: {altitude}m")
    print(f"Visited bases: {visited_bases}")
    print(f"Number of detections: {len(detections)}\n")

    # Order detections
    ordered = CaptureAndDetect.order_detections(detections, moving_forward=True)

    # Filter and process
    valid_detections = []
    for i, detection in enumerate(ordered):
        pixel_x, pixel_y = detection["center"]

        # Estimate position
        est_x, est_y = CaptureAndDetect.estimate_detection_position(
            detection, drone_x, drone_y, altitude
        )

        # Check duplicate
        is_duplicate = CaptureAndDetect.is_duplicate_detection(
            (est_x, est_y), visited_bases
        )

        print(f"Detection {i+1}:")
        print(f"  Pixel: ({pixel_x}, {pixel_y})")
        print(f"  Estimated position: ({est_x:.2f}, {est_y:.2f})")
        print(f"  Confidence: {detection['confidence']:.2f}")

        if is_duplicate:
            print(f"  Status: DUPLICATE (filtered)")
        else:
            detection["estimated_position"] = (est_x, est_y)
            valid_detections.append(detection)
            print(f"  Status: VALID")
        print()

    print(f"Summary:")
    print(f"  Total detections: {len(detections)}")
    print(f"  Valid detections: {len(valid_detections)}")
    print(f"  Filtered duplicates: {len(detections) - len(valid_detections)}")

    if len(valid_detections) > 1:
        print(f"\n  Would process {len(valid_detections)} detections in order")
        print(f"  Would return to waypoint after each landing")

    return len(valid_detections) == 1  # Should have 1 valid detection


def test_position_matching():
    """Test matching detections by position (for CenterOnDetection)."""
    print("\n" + "=" * 60)
    print("TEST 6: Position Matching for Centering")
    print("=" * 60)

    # Target position from CaptureAndDetect
    target_position = (6.5, 4.5)

    # Current drone position (moved slightly)
    drone_x, drone_y = 5.2, 5.1
    altitude = 3.0

    # Create detections at different positions
    detections = [
        create_test_detection(
            IMAGE_CENTER_X + 50, IMAGE_CENTER_Y - 100, 0.85
        ),  # Close to target
        create_test_detection(
            IMAGE_CENTER_X - 200, IMAGE_CENTER_Y + 200, 0.90
        ),  # Far from target
        create_test_detection(
            IMAGE_CENTER_X + 100, IMAGE_CENTER_Y, 0.75
        ),  # Medium distance
    ]

    print(f"Target position: ({target_position[0]:.2f}, {target_position[1]:.2f})")
    print(f"Current drone: ({drone_x:.2f}, {drone_y:.2f}), altitude: {altitude}m\n")

    best_match = None
    min_distance = float("inf")

    for i, detection in enumerate(detections):
        # Estimate position
        est_x, est_y = CaptureAndDetect.estimate_detection_position(
            detection, drone_x, drone_y, altitude
        )

        # Calculate distance to target
        distance = math.sqrt(
            (est_x - target_position[0]) ** 2 + (est_y - target_position[1]) ** 2
        )

        print(f"Detection {i+1}:")
        print(f"  Pixel: {detection['center']}")
        print(f"  Estimated position: ({est_x:.2f}, {est_y:.2f})")
        print(f"  Distance to target: {distance:.2f}m")

        if distance < min_distance:
            min_distance = distance
            best_match = i + 1

    print(f"\n✓ Best match: Detection {best_match} (distance: {min_distance:.2f}m)")

    if min_distance < 0.5:
        print("  Good match - likely correct detection")
    elif min_distance < 1.0:
        print("  Fair match - possibly correct detection")
    else:
        print("  Poor match - may be wrong detection!")

    return True


def main():
    """Run all tests."""
    print("\n" + "=" * 60)
    print("Simplified Detection Filtering Test Suite")
    print("=" * 60)

    results = []

    test_functions = [
        ("Meters per Pixel", test_meters_per_pixel),
        ("Position Estimation", test_position_estimation),
        ("Duplicate Detection", test_duplicate_detection),
        ("Detection Ordering", test_detection_ordering),
        ("Multiple Detection Scenario", test_multiple_detection_scenario),
        ("Position Matching", test_position_matching),
    ]

    for test_name, test_func in test_functions:
        try:
            passed = test_func()
            results.append((test_name, passed))
        except Exception as e:
            print(f"\n✗ Test failed with exception: {e}")
            import traceback

            traceback.print_exc()
            results.append((test_name, False))

    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)

    for test_name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{status}: {test_name}")

    total = len(results)
    passed = sum(1 for _, p in results if p)

    print(f"\nTotal: {passed}/{total} tests passed")

    if passed == total:
        print("\n✓ All tests passed!")
        return 0
    else:
        print(f"\n✗ {total - passed} test(s) failed")
        return 1


if __name__ == "__main__":
    exit(main())
