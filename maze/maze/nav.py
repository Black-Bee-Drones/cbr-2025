#!/usr/bin/env python3

"""
Navigation test script for Tello drone.
Tests basic movement and height control without full state machine.
"""

import rclpy
from rclpy.node import Node
import time

from maze.utils.tello_wrapper import TelloWrapper
from maze.utils.lidar_client import MockLidarClient
from maze.constants import ENTRY_HEIGHT, HIGH_SCAN_HEIGHT, LOW_SCAN_HEIGHT


def test_navigation():
    """Test basic Tello navigation commands."""

    rclpy.init()
    node = Node("nav_test")

    try:
        # Initialize Tello
        print("=" * 60)
        print("TELLO NAVIGATION TEST")
        print("=" * 60)

        node.get_logger().info("Connecting to Tello...")
        tello = TelloWrapper(node)

        if not tello.connect():
            node.get_logger().error("Failed to connect to Tello")
            return

        battery = tello.get_battery()
        node.get_logger().info(f"Battery: {battery}%")

        if battery < 30:
            node.get_logger().warn("Low battery, aborting test")
            return

        # Initialize mock lidar for testing
        lidar = MockLidarClient(node)
        lidar.connect()

        # Test sequence
        node.get_logger().info("Starting navigation test sequence...")

        # Takeoff
        input("Press Enter to takeoff...")
        if not tello.takeoff(ENTRY_HEIGHT):
            node.get_logger().error("Takeoff failed")
            return

        # Test height adjustments
        input("Press Enter to test height adjustments...")
        node.get_logger().info(f"Moving to high scan height ({HIGH_SCAN_HEIGHT}m)")
        tello.set_height(HIGH_SCAN_HEIGHT)
        time.sleep(2)

        node.get_logger().info(f"Moving to low scan height ({LOW_SCAN_HEIGHT}m)")
        tello.set_height(LOW_SCAN_HEIGHT)
        time.sleep(2)

        node.get_logger().info(f"Returning to entry height ({ENTRY_HEIGHT}m)")
        tello.set_height(ENTRY_HEIGHT)
        time.sleep(2)

        # Test rotations
        input("Press Enter to test rotations...")
        node.get_logger().info("Testing 90° clockwise rotation")
        tello.rotate_clockwise(90)
        time.sleep(1)

        node.get_logger().info("Testing 90° counter-clockwise rotation")
        tello.rotate_counter_clockwise(90)
        time.sleep(1)

        # Test movements
        input("Press Enter to test movements...")
        node.get_logger().info("Moving forward 0.5m")
        tello.move_forward(0.5)
        time.sleep(1)

        node.get_logger().info("Moving back 0.5m")
        tello.move_back(0.5)
        time.sleep(1)

        node.get_logger().info("Moving left 0.3m")
        tello.move_left(0.3)
        time.sleep(1)

        node.get_logger().info("Moving right 0.3m")
        tello.move_right(0.3)
        time.sleep(1)

        # Test lidar readings
        input("Press Enter to test lidar readings...")
        for i in range(4):
            distance = lidar.get_distance()
            node.get_logger().info(f"Lidar reading {i+1}: {distance:.2f}m")
            tello.rotate_clockwise(90)
            time.sleep(1)

        # Land
        input("Press Enter to land...")
        if not tello.land():
            node.get_logger().error("Landing failed")

        node.get_logger().info("Navigation test complete!")

    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")
        if tello and tello.is_flying:
            tello.land()

    except Exception as e:
        node.get_logger().error(f"Test failed: {e}")
        if tello and tello.is_flying:
            tello.emergency_stop()

    finally:
        if tello:
            tello.disconnect()
        if lidar:
            lidar.disconnect()
        node.destroy_node()
        rclpy.shutdown()


def main():
    test_navigation()


if __name__ == "__main__":
    main()
