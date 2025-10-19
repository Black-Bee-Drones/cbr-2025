"""Scan position state for maze navigation"""

import time
from yasmin import State
from yasmin.blackboard import Blackboard

from maze.constants import (
    SUCCESS,
    FAILURE,
    HIGH_SCAN_HEIGHT,
    LOW_SCAN_HEIGHT,
    PHOTO_DELAY,
    PHOTOS_PER_POSITION,
)
from maze.utils.maze_data import Direction, ScanData


class ScanPosition(State):
    """
    Scan current position at both height levels.
    Takes photos and lidar readings in 4 directions.
    """

    def __init__(self):
        """Initialize scan position state."""
        super().__init__(outcomes=[SUCCESS, FAILURE])

    def execute(self, blackboard: Blackboard) -> str:
        """Execute scan position state."""

        node = blackboard["node"]
        tello = blackboard["tello"]
        lidar_client = blackboard["lidar_client"]
        qr_detector = blackboard["qr_detector"]
        maze_map = blackboard["maze_map"]

        current_pos = maze_map.current_position
        node.get_logger().info("========================================")
        node.get_logger().info(f"SCANNING POSITION {current_pos}")
        node.get_logger().info("========================================")

        try:
            # Scan at high level
            node.get_logger().info(f"Scanning at high level ({HIGH_SCAN_HEIGHT}m)")

            if not tello.set_height(HIGH_SCAN_HEIGHT):
                node.get_logger().error("Failed to set high scan height")
                return FAILURE

            # Notify mock lidar about height change
            if hasattr(lidar_client, "set_height_level"):
                lidar_client.set_height_level(HIGH_SCAN_HEIGHT)

            high_scan = self._perform_scan(
                blackboard,
                height_level="high",
            )

            if high_scan is None:
                return FAILURE

            # Add high scan data to map
            maze_map.add_scan_data(current_pos, "high", high_scan)

            # Scan at low level
            node.get_logger().info(f"Scanning at low level ({LOW_SCAN_HEIGHT}m)")

            if not tello.set_height(LOW_SCAN_HEIGHT):
                node.get_logger().error("Failed to set low scan height")
                return FAILURE

            # Notify mock lidar about height change
            if hasattr(lidar_client, "set_height_level"):
                lidar_client.set_height_level(LOW_SCAN_HEIGHT)

            low_scan = self._perform_scan(blackboard, height_level="low")

            if low_scan is None:
                return FAILURE

            # Add low scan data to map
            maze_map.add_scan_data(current_pos, "low", low_scan)

            # Return to navigation height
            if not tello.set_height(HIGH_SCAN_HEIGHT):
                node.get_logger().error("Failed to return to navigation height")
                return FAILURE

            # Update QR codes found
            all_qr_codes = high_scan.qr_codes + low_scan.qr_codes
            for qr_code in all_qr_codes:
                if qr_code not in blackboard["qr_codes_found"]:
                    blackboard["qr_codes_found"].append(qr_code)
                    node.get_logger().info(f"New QR code found: {qr_code}")

            # Log scan summary
            node.get_logger().info("Scan complete:")
            node.get_logger().info(f"  High passages: {high_scan.get_passages()}")
            node.get_logger().info(f"  Low passages: {low_scan.get_passages()}")
            node.get_logger().info(f"  QR codes: {all_qr_codes}")

            return SUCCESS

        except Exception as e:
            node.get_logger().error(f"Scan position error: {e}")
            return FAILURE

    def _perform_scan(self, blackboard: Blackboard, height_level: str) -> ScanData:
        """
        Perform 360-degree scan at current height.

        Parameters
        ----------
        blackboard : Blackboard
            State machine blackboard
        height_level : str
            "high" or "low" scan level
        skip_entry : bool
            Skip the entry direction scan

        Returns
        -------
        ScanData
            Scan data with photos and lidar readings
        """
        node = blackboard["node"]
        tello = blackboard["tello"]
        lidar_client = blackboard["lidar_client"]
        qr_detector = blackboard["qr_detector"]
        maze_map = blackboard["maze_map"]

        position_id = blackboard["points_visited"]
        scan_data = ScanData(
            position_id=position_id,
            grid_position=maze_map.current_position,
            height_level=height_level,
        )

        # Determine directions to scan
        directions = [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST]

        # Note: We don't skip any directions for this maze layout
        # All directions need to be scanned to find passages

        # Scan each direction
        for i, direction in enumerate(directions):
            # Calculate rotation needed
            current_dir = maze_map.current_direction
            rotation_needed = direction.to_relative(current_dir)

            if rotation_needed > 0:
                # Rotate to face direction
                if rotation_needed <= 180:
                    if not tello.rotate_clockwise(rotation_needed):
                        node.get_logger().error(f"Failed to rotate to {direction.name}")
                        return None
                else:
                    # Rotate counter-clockwise is shorter
                    if not tello.rotate_counter_clockwise(360 - rotation_needed):
                        node.get_logger().error(f"Failed to rotate to {direction.name}")
                        return None

                maze_map.update_direction(direction)

            # Wait for stabilization
            time.sleep(PHOTO_DELAY)

            # Take photo
            photo = tello.take_photo()
            if photo is None:
                node.get_logger().warn(f"Failed to take photo at {direction.name}")
                continue

            # Set direction for mock lidar
            if hasattr(lidar_client, "set_direction"):
                # Map Direction enum to index: North=0, East=1, South=2, West=3
                direction_map = {
                    Direction.NORTH: 0,
                    Direction.EAST: 1,
                    Direction.SOUTH: 2,
                    Direction.WEST: 3,
                }
                lidar_client.set_direction(direction_map[direction])

            # Get lidar reading
            lidar_reading = lidar_client.get_distance()
            if lidar_reading is None:
                node.get_logger().warn(f"Failed to get lidar at {direction.name}")
                lidar_reading = 0.0  # Default to wall

            # Detect QR codes
            qr_results = qr_detector.detect(photo)
            print(qr_results)
            qr_code = qr_results[0].data if qr_results else None

            # Add scan data
            scan_data.add_scan(direction, photo, lidar_reading, qr_code)

            node.get_logger().info(
                f"  {direction.name}: lidar={lidar_reading:.2f}m, "
                f"QR={'[' + qr_code + ']' if qr_code else 'none'}"
            )

        return scan_data
