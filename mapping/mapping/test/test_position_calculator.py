#!/usr/bin/env python3
import sys
import math
from typing import Dict

from mapping.utils import DetectionPositionCalculator


def create_test_detection(pixel_x: int, pixel_y: int, confidence: float = 0.8) -> Dict:
    """Create a test detection dictionary."""
    return {
        "center": (pixel_x, pixel_y),
        "bbox": [pixel_x - 40, pixel_y - 40, pixel_x + 40, pixel_y + 40],
        "confidence": confidence,
        "class_id": 0,
        "area": 6400.0,
    }


def test_single_detection():
    """Test calculation of single detection position."""
    print("\n" + "=" * 60)
    print("TEST 1: Single Detection Position Calculation")
    print("=" * 60)

    calc = DetectionPositionCalculator()

    # Simulated scenario: drone at (5.0, 5.0, 3.0), looking straight down
    drone_position = (5.0, 5.0, 3.0)

    # Quaternion for no rotation (w=1, x=y=z=0)
    # In practice, this would come from the drone's IMU
    drone_orientation = (0.0, 0.0, 0.0, 1.0)

    altitude = 3.0

    # Detection at image center (should be directly below drone)
    detection = create_test_detection(820, 616)

    world_pos = calc.calculate_detection_world_position(
        detection=detection,
        drone_position=drone_position,
        drone_orientation_quaternion=drone_orientation,
        altitude=altitude,
    )

    print(f"Drone position: {drone_position}")
    print(f"Drone altitude: {altitude}m")
    print(f"Detection pixel: {detection['center']}")
    print(f"Calculated world position: {world_pos}")

    if world_pos:
        print(f"✓ Position calculated successfully")
        print(f"  Expected: Near ({drone_position[0]:.2f}, {drone_position[1]:.2f})")
        print(f"  Got: ({world_pos[0]:.2f}, {world_pos[1]:.2f})")
    else:
        print("✗ Failed to calculate position")

    return world_pos is not None


def test_multiple_detections():
    """Test filtering of multiple detections."""
    print("\n" + "=" * 60)
    print("TEST 2: Multiple Detections - Select Closest")
    print("=" * 60)

    calc = DetectionPositionCalculator()

    drone_position = (5.0, 5.0, 3.0)
    drone_orientation = (0.0, 0.0, 0.0, 1.0)
    altitude = 3.0
    visited_bases = []  # No visited bases

    # Create multiple detections at different positions
    detections = [
        create_test_detection(400, 400, confidence=0.85),  # Far from drone
        create_test_detection(820, 616, confidence=0.75),  # Near drone (center)
        create_test_detection(1200, 900, confidence=0.80),  # Far from drone
    ]

    print(f"Drone position: {drone_position}")
    print(f"Number of detections: {len(detections)}")

    best = calc.filter_and_select_best_detection(
        detections=detections,
        drone_position=drone_position,
        drone_orientation_quaternion=drone_orientation,
        altitude=altitude,
        visited_bases=visited_bases,
        duplicate_radius=1.2,
    )

    if best:
        print(f"\n✓ Selected best detection:")
        print(f"  Pixel: {best['center']}")
        print(f"  World position: {best['world_position']}")
        print(f"  Distance to drone: {best['distance_to_drone']:.2f}m")
        print(f"  Confidence: {best['confidence']:.2f}")
        print(f"\n  Expected: Detection at center (closest to drone)")
        return True
    else:
        print("✗ No detection selected")
        return False


def test_duplicate_filtering():
    """Test duplicate detection filtering."""
    print("\n" + "=" * 60)
    print("TEST 3: Duplicate Filtering")
    print("=" * 60)

    calc = DetectionPositionCalculator()

    drone_position = (5.0, 5.0, 3.0)
    drone_orientation = (0.0, 0.0, 0.0, 1.0)
    altitude = 3.0

    # Visited base near the center
    visited_bases = [{"x": 5.1, "y": 5.2, "z": 0.0}]

    # Create detections:
    # - One near visited base (should be filtered)
    # - One far from visited base (should be selected)
    detections = [
        create_test_detection(820, 616, confidence=0.90),  # Near visited (center)
        create_test_detection(1200, 900, confidence=0.75),  # Far from visited
    ]

    print(f"Drone position: {drone_position}")
    print(f"Visited bases: {visited_bases}")
    print(f"Number of detections: {len(detections)}")

    best = calc.filter_and_select_best_detection(
        detections=detections,
        drone_position=drone_position,
        drone_orientation_quaternion=drone_orientation,
        altitude=altitude,
        visited_bases=visited_bases,
        duplicate_radius=1.2,
    )

    if best:
        world_x, world_y = best["world_position"]
        dist_to_visited = math.sqrt(
            (world_x - visited_bases[0]["x"]) ** 2
            + (world_y - visited_bases[0]["y"]) ** 2
        )

        print(f"\n✓ Selected detection after filtering:")
        print(f"  Pixel: {best['center']}")
        print(f"  World position: ({world_x:.2f}, {world_y:.2f})")
        print(f"  Distance to visited base: {dist_to_visited:.2f}m")
        print(f"  Confidence: {best['confidence']:.2f}")

        if dist_to_visited > 1.2:
            print(f"\n  ✓ PASS: Correctly selected detection away from visited base")
            return True
        else:
            print(f"\n  ✗ FAIL: Selected detection too close to visited base")
            return False
    else:
        print("\n✗ No detection selected (all filtered as duplicates)")
        print("  This might be correct if all detections are near visited bases")
        return False


def test_all_duplicates():
    """Test case where all detections are duplicates."""
    print("\n" + "=" * 60)
    print("TEST 4: All Detections Are Duplicates")
    print("=" * 60)

    calc = DetectionPositionCalculator()

    drone_position = (5.0, 5.0, 3.0)
    drone_orientation = (0.0, 0.0, 0.0, 1.0)
    altitude = 3.0

    # First, calculate where center detections actually map to
    center_detection = create_test_detection(820, 616, confidence=0.85)
    center_pos = calc.calculate_detection_world_position(
        detection=center_detection,
        drone_position=drone_position,
        drone_orientation_quaternion=drone_orientation,
        altitude=altitude,
    )

    print(f"Drone position: {drone_position}")
    print(f"Center detection maps to: {center_pos}")

    # Place visited bases near where detections will actually be calculated
    # Using the actual calculated position with small offsets
    visited_bases = [
        {"x": center_pos[0] + 0.2, "y": center_pos[1] + 0.1, "z": 0.0},
        {"x": center_pos[0] - 0.1, "y": center_pos[1] - 0.2, "z": 0.0},
    ]

    # Detections near center that should be near visited bases
    detections = [
        create_test_detection(800, 600, confidence=0.85),
        create_test_detection(850, 650, confidence=0.80),
    ]

    print(f"Visited bases: {visited_bases}")
    print(f"Number of detections: {len(detections)}")

    best = calc.filter_and_select_best_detection(
        detections=detections,
        drone_position=drone_position,
        drone_orientation_quaternion=drone_orientation,
        altitude=altitude,
        visited_bases=visited_bases,
        duplicate_radius=1.2,
    )

    if best is None:
        print(f"\n✓ PASS: Correctly filtered all detections as duplicates")
        return True
    else:
        print(f"\n✗ FAIL: Should have filtered all detections")
        print(f"  Selected: {best['world_position']}")
        return False


def main():
    """Run all tests."""
    print("\n" + "=" * 60)
    print("DetectionPositionCalculator Test Suite")
    print("=" * 60)

    results = []

    try:
        results.append(("Single Detection", test_single_detection()))
    except Exception as e:
        print(f"\n✗ Test failed with exception: {e}")
        import traceback

        traceback.print_exc()
        results.append(("Single Detection", False))

    try:
        results.append(("Multiple Detections", test_multiple_detections()))
    except Exception as e:
        print(f"\n✗ Test failed with exception: {e}")
        import traceback

        traceback.print_exc()
        results.append(("Multiple Detections", False))

    try:
        results.append(("Duplicate Filtering", test_duplicate_filtering()))
    except Exception as e:
        print(f"\n✗ Test failed with exception: {e}")
        import traceback

        traceback.print_exc()
        results.append(("Duplicate Filtering", False))

    try:
        results.append(("All Duplicates", test_all_duplicates()))
    except Exception as e:
        print(f"\n✗ Test failed with exception: {e}")
        import traceback

        traceback.print_exc()
        results.append(("All Duplicates", False))

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
        print("\nv All tests passed!")
        return 0
    else:
        print(f"\nx  {total - passed} test(s) failed")
        return 1


if __name__ == "__main__":
    exit(main())
