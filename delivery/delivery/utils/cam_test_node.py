import rclpy
from rclpy.node import Node
import cv2
from delivery.utils import YoloDetector
from std_msgs.msg import Float32MultiArray
from delivery.constants import IMAGE_SOURCE  # pode ser um número da webcam ou caminho do vídeo


class CamTest(Node):
    """
    Node for testing YOLO detection on camera feed without ImageHandler.
    """
    def __init__(self):
        super().__init__('cam_test_node')
        self.get_logger().info("Initializing CamTest Node")

        # Inicializa o detector YOLO
        self.detector = YoloDetector()

        # Abre a câmera
        # IMAGE_SOURCE pode ser um índice (0,1..) ou path para vídeo/arquivo
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera or video: {IMAGE_SOURCE}")
            raise RuntimeError("Camera not available")

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None or frame.size == 0:
            self.get_logger().warn("Empty frame received")
            return

        # Detecta objetos com YOLO
        detections = self.detector.detect(frame, save_image=False)

        # Debug: imprime as detecções
        print(detections)
        for det_name, det in detections.items():
            print(det_name)
            print(det)
            self.get_logger().info(f"{det_name}: {det}")
            if "center" in det:
                cx, cy = map(int, det["center"])
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            if "bbox" in det:
                x1, y1, x2, y2 = map(int, det["bbox"])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

    def destroy_node(self):
        self.get_logger().info("Shutting down CamTest Node")
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
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
