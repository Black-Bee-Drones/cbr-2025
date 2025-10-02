import rclpy
from rclpy.node import Node
import cv2
from delivery.utils import YOLODetector
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

        self.detector = YOLODetector()

        handler = ImageHandler(node=self, image_source=IMAGE_SOURCE, image_processing_callback=self.processo_frame)
        handler.run()

        self.last_detections = []

    def processo_frame(self, frame):
        """
        Process the incoming frame for object detection.
        """
        if frame is None or frame.size == 0:
            self.get_logger().warn("Empty frame received")
            return

        results = self.detector.detect(desired_class="package",image=frame, save_image=False)

        dets = []
        for r in results:
            cls_id = r["class_id"]
            x1, y1, x2, y2 = r["bbox"]
            dets.append((cls_id, x1, y1, x2 - x1, y2 - y1))
            # desenhar na imagem
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            if(cls_id == 0):
                label = f"base {r['confidence']:.2f}"
            if(cls_id == 1):
                label = f"package {r['confidence']:.2f}"
            cv2.putText(frame, label, (x1, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        self.last_detections = dets

        cv2.imshow("Camera Test YOLO", frame)
        cv2.waitKey(1)

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