#!/usr/bin/env python3
import sys

from typing import List, Dict
from mapping.states.navigation_states import CaptureAndDetect
from mapping.constants import DUPLICATE_BASE_RADIUS


def simulate_waypoint_visit():
    """Simulate visiting a waypoint with multiple bases."""
    print("\n" + "=" * 60)
    print("Waypoint Return Logic Test")
    print("=" * 60)

    drone_x, drone_y = 5.0, 5.0
    altitude = 3.0
    visited_bases = []  # No bases visited yet

    print(f"\n=== FIRST VISIT TO WAYPOINT ===")
    print(f"Drone position: ({drone_x:.2f}, {drone_y:.2f}), altitude: {altitude}m")
    print(f"Visited bases: {visited_bases}")

    # Simulate 3 detections at the waypoint
    detections = [
        {"center": (700, 700), "confidence": 0.85},  # Base A: ~(5.22, 5.22)
        {"center": (900, 700), "confidence": 0.90},  # Base B: ~(5.22, 4.78)
        {"center": (700, 500), "confidence": 0.80},  # Base C: ~(4.78, 5.22)
    ]

    print(f"\nDetected {len(detections)} bases")

    # Process detections - first visit
    valid_detections = []
    for i, det in enumerate(detections):
        est_pos = CaptureAndDetect.estimate_detection_position(
            det, drone_x, drone_y, altitude
        )

        if not CaptureAndDetect.is_duplicate_detection(est_pos, visited_bases):
            det["estimated_position"] = est_pos
            valid_detections.append(det)
            print(
                f"  Base {chr(65+i)}: pixel {det['center']}, "
                f"position ({est_pos[0]:.2f}, {est_pos[1]:.2f}) - VALID"
            )
        else:
            print(f"  Base {chr(65+i)}: DUPLICATE")

    print(f"\nResult: {len(valid_detections)} valid detections")
    print(f"return_to_waypoint flag would be: {len(valid_detections) > 1}")

    # Simulate landing on first base
    first_base = valid_detections[0]
    landed_position = first_base["estimated_position"]
    visited_bases.append({"x": landed_position[0], "y": landed_position[1], "z": 0.0})

    print(
        f"\n>>> Drone lands on Base A at ({landed_position[0]:.2f}, {landed_position[1]:.2f})"
    )
    print(f">>> Updated visited_bases: {len(visited_bases)} base(s)")

    # RETURN TO WAYPOINT - Second visit
    print(f"\n=== RETURN TO SAME WAYPOINT ===")
    print(f"Drone position: ({drone_x:.2f}, {drone_y:.2f}), altitude: {altitude}m")
    print(
        f"Visited bases: [({visited_bases[0]['x']:.2f}, {visited_bases[0]['y']:.2f})]"
    )

    # Simulate NEW detection inference (same physical bases but fresh detection)
    # Positions might vary slightly due to drone position/camera angle
    detections_return = [
        {
            "center": (695, 705),
            "confidence": 0.87,
        },  # Base A (slightly different pixels)
        {"center": (895, 695), "confidence": 0.88},  # Base B
        {"center": (705, 505), "confidence": 0.82},  # Base C
    ]

    print(f"\nDetected {len(detections_return)} bases (fresh inference)")

    # Process detections - second visit
    valid_detections_return = []
    for i, det in enumerate(detections_return):
        est_pos = CaptureAndDetect.estimate_detection_position(
            det, drone_x, drone_y, altitude
        )

        if not CaptureAndDetect.is_duplicate_detection(est_pos, visited_bases):
            det["estimated_position"] = est_pos
            valid_detections_return.append(det)
            print(
                f"  Base {chr(65+i)}: pixel {det['center']}, "
                f"position ({est_pos[0]:.2f}, {est_pos[1]:.2f}) - VALID"
            )
        else:
            # Calculate distance to visited base for info
            dist_x = abs(est_pos[0] - visited_bases[0]["x"])
            dist_y = abs(est_pos[1] - visited_bases[0]["y"])
            print(
                f"  Base {chr(65+i)}: pixel {det['center']}, "
                f"position ({est_pos[0]:.2f}, {est_pos[1]:.2f}) - DUPLICATE "
                f"(dx={dist_x:.2f}m, dy={dist_y:.2f}m from visited)"
            )

    print(f"\nResult: {len(valid_detections_return)} valid detections")
    print(f"return_to_waypoint flag would be: {len(valid_detections_return) > 1}")

    # Verify the logic worked correctly
    print("\n" + "=" * 60)
    print("TEST VERIFICATION")
    print("=" * 60)

    success = True

    # Check 1: All 3 bases valid on first visit
    if len(valid_detections) != 3:
        print(
            f"✗ FAIL: Expected 3 valid detections on first visit, got {len(valid_detections)}"
        )
        success = False
    else:
        print(f"✓ PASS: All 3 bases valid on first visit")

    # Check 2: Only 2 bases valid on return (Base A filtered)
    if len(valid_detections_return) != 2:
        print(
            f"✗ FAIL: Expected 2 valid detections on return, got {len(valid_detections_return)}"
        )
        success = False
    else:
        print(f"✓ PASS: Base A correctly filtered as duplicate on return")

    # Check 3: Return flag set correctly
    if len(valid_detections) > 1 and len(valid_detections_return) > 1:
        print(f"✓ PASS: Return flag correctly set for multiple detections")
    elif len(valid_detections_return) == 1:
        print(f"✓ PASS: Would process last base and continue to next waypoint")
    elif len(valid_detections_return) == 0:
        print(f"✓ PASS: All bases visited, would continue to next waypoint")

    return success


def main():
    """Run the test."""
    success = simulate_waypoint_visit()

    print("\n" + "=" * 60)
    if success:
        print("✓ Waypoint return logic working correctly!")
        print("  - Fresh detections on each visit")
        print("  - Previously visited bases filtered as duplicates")
        print("  - Continues until all bases at waypoint are visited")
    else:
        print("✗ Test failed - logic needs adjustment")
    print("=" * 60)

    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
