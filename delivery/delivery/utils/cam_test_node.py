import rclpy
from rclpy.node import Node
import cv2
from delivery.utils import YoloDetector
from mirela_sdk.image_processing.camera.image_handler import ImageHandler
from std_msgs.msg import Float32MultiArray
import os

from delivery.constants import (
    IMAGE_SOURCE
)

# colcon build --symlink-install --packages-select delivery
# source install/setup.bash
# ros2 run delivery cam_test_node

class CamTest(Node):
    """
    Node for testing Yolo detection on camera feed.
    """
    def __init__(self):
        super().__init__('cam_test_node')
        self.get_logger().info("Innitializing CamTest Node")

        self.detector: YoloDetector = YoloDetector()

        handler = ImageHandler(node=self, image_source=IMAGE_SOURCE, image_processing_callback=self.processo_frame)
        handler.run()

    def processo_frame(self, frame):
        if frame is None or frame.size == 0:
            self.get_logger().warn("Empty frame received")
            return

        detections = self.detector.detect(
            frame = frame,
            save_image = False,
        )

        print(detections)
        for k, v in detections.values():
            self.get_logger().info(f"{k}: {v}")

            if "center" in v:
                cv2.circle(frame, center=v["center"], radius=5, color=(0, 255, 0), thickness=-1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = CamTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()