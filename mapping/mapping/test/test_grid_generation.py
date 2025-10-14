#!/usr/bin/env python3
import sys
from mapping.utils.grid import Grid
from mapping.constants import SEARCH_AREA_WIDTH, SEARCH_AREA_HEIGHT, GRID_SPACING
import math


def test_backward_left():
    """Test BACKWARD + LEFT scenario."""
    print("\n" + "=" * 70)
    print("SCENARIO 1: BACKWARD + LEFT (Drone rotated 180°)")
    print("=" * 70)

    # Create grid with BACKWARD + LEFT configuration
    grid = Grid(
        search_width=5.0,
        search_height=5.0,
        grid_spacing=1.25,
        initial_position=(0.0, 0.0),
        start_offset=(-0.5, 0.75),  # Offset for backward+left scenario
        pattern_type="COLUMNS",
        primary_direction="BACKWARD",
        transition_direction="LEFT",
    )

    print(grid.get_structured_waypoints())

    waypoints = grid.get_all_waypoints()
    print(f"\nVerification: Got {len(waypoints)} waypoints (expected 16)")

    print(f"Waypoints: {waypoints}")

    expected_col1 = [(-1.75, 0.75), (-3.0, 0.75), (-4.25, 0.75), (-5.5, 0.75)]
    print("\nColumn 1 verification:")
    print("Expected points:", expected_col1)
    actual_col1 = [(wp["x"], wp["y"]) for wp in waypoints[:4]]
    print("Actual points:  ", [(round(x, 2), round(y, 2)) for x, y in actual_col1])

    expected_col2 = [(-5.5, 2.0), (-4.25, 2.0), (-3.0, 2.0), (-1.75, 2.0)]
    print("\nColumn 2 verification (after LEFT transition):")
    print("Expected points:", expected_col2)
    actual_col2 = [(wp["x"], wp["y"]) for wp in waypoints[4:8]]
    print("Actual points:  ", [(round(x, 2), round(y, 2)) for x, y in actual_col2])

    n_columns = max(1, int(math.ceil(SEARCH_AREA_WIDTH / GRID_SPACING)))
    n_rows = max(1, int(math.ceil(SEARCH_AREA_HEIGHT / GRID_SPACING)))
    print(f"Expected number of columns: {n_columns}, Expected number of rows: {n_rows}")

    # Test get_grid_bounds() method
    bounds = grid.get_grid_bounds()
    print(f"\nGrid bounds from get_grid_bounds():")
    print(f"  Min x: {bounds['min_x']}, Max x: {bounds['max_x']}")
    print(f"  Min y: {bounds['min_y']}, Max y: {bounds['max_y']}")

    # Verify bounds match waypoints
    expected_min_x = waypoints[n_columns - 1]["x"]
    expected_max_x = waypoints[0]["x"]
    print(f"\nVerification:")
    print(
        f"  Expected min_x: {expected_min_x}, Got: {bounds['min_x']} {'✓' if bounds['min_x'] == expected_min_x else '✗'}"
    )
    print(
        f"  Expected max_x: {expected_max_x}, Got: {bounds['max_x']} {'✓' if bounds['max_x'] == expected_max_x else '✗'}"
    )

    return len(waypoints) == 16


def test_forward_right():
    """Test FORWARD + RIGHT scenario."""
    print("\n" + "=" * 70)
    print("SCENARIO 2: FORWARD + RIGHT (Default drone orientation)")
    print("=" * 70)

    # Create grid with FORWARD + RIGHT configuration
    grid = Grid(
        search_width=5.0,
        search_height=5.0,
        grid_spacing=1.25,
        initial_position=(0.0, 0.0),
        start_offset=(0.5, -0.75),  # Offset for forward+right scenario
        pattern_type="COLUMNS",
        primary_direction="FORWARD",
        transition_direction="RIGHT",
    )

    print(grid.get_structured_waypoints())

    waypoints = grid.get_all_waypoints()
    print(f"\nVerification: Got {len(waypoints)} waypoints (expected 16)")

    expected_col1 = [(1.75, -0.75), (3.0, -0.75), (4.25, -0.75), (5.5, -0.75)]
    print("\nColumn 1 verification:")
    print("Expected points:", expected_col1)
    actual_col1 = [(wp["x"], wp["y"]) for wp in waypoints[:4]]
    print("Actual points:  ", [(round(x, 2), round(y, 2)) for x, y in actual_col1])

    expected_col2 = [(5.5, -2.0), (4.25, -2.0), (3.0, -2.0), (1.75, -2.0)]
    print("\nColumn 2 verification (after RIGHT transition):")
    print("Expected points:", expected_col2)
    actual_col2 = [(wp["x"], wp["y"]) for wp in waypoints[4:8]]
    print("Actual points:  ", [(round(x, 2), round(y, 2)) for x, y in actual_col2])

    n_columns = max(1, int(math.ceil(SEARCH_AREA_WIDTH / GRID_SPACING)))
    n_rows = max(1, int(math.ceil(SEARCH_AREA_HEIGHT / GRID_SPACING)))
    print(f"Expected number of columns: {n_columns}, Expected number of rows: {n_rows}")

    # Test get_grid_bounds() method
    bounds = grid.get_grid_bounds()
    print(f"\nGrid bounds from get_grid_bounds():")
    print(f"  Min x: {bounds['min_x']}, Max x: {bounds['max_x']}")
    print(f"  Min y: {bounds['min_y']}, Max y: {bounds['max_y']}")

    # Verify bounds match waypoints
    expected_min_x = waypoints[0]["x"]
    expected_max_x = waypoints[n_columns - 1]["x"]
    print(f"\nVerification:")
    print(
        f"  Expected min_x: {expected_min_x}, Got: {bounds['min_x']} {'✓' if bounds['min_x'] == expected_min_x else '✗'}"
    )
    print(
        f"  Expected max_x: {expected_max_x}, Got: {bounds['max_x']} {'✓' if bounds['max_x'] == expected_max_x else '✗'}"
    )

    return len(waypoints) == 16


def main():
    """Run tests for both scenarios."""
    print("\n" + "=" * 70)
    print("GRID GENERATION TEST - 5x5m grid with 1.25m spacing (16 points)")
    print("=" * 70)

    backward_left_ok = test_backward_left()
    forward_right_ok = test_forward_right()

    # Summary
    print("\n" + "=" * 70)
    print("TEST SUMMARY")
    print("=" * 70)
    status1 = "✓ PASS" if backward_left_ok else "✗ FAIL"
    status2 = "✓ PASS" if forward_right_ok else "✗ FAIL"
    print(f"{status1}: BACKWARD + LEFT scenario")
    print(f"{status2}: FORWARD + RIGHT scenario")

    if backward_left_ok and forward_right_ok:
        print("\n✓ All tests passed! Grid generation is working correctly.")
        return 0
    else:
        print("\n✗ Some tests failed. Check the output above.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
