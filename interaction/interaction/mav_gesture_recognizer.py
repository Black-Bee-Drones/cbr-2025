import rclpy
import rclpy.clock
from rclpy.node import Node
from std_msgs.msg import Int16
import numpy as np
from cvzone.HandTrackingModule import HandDetector
from mirela_sdk.image_processing.camera.image_handler import ImageHandler
import time
import math
from interaction.constants import RTL_COUNT_TOPIC, LAND_GESTURE_ID

class GestureRecognizer(Node):
    """
    Esta classe reconhece gestos com as duas mãos usando a câmera OAK-D e publica
    um ID de ação correspondente em um tópico ROS2.
    """
    # Dicionário de gestos.
    gestures: dict[tuple[int, ...], int] = {
        # Fingers (R), Fingers (L)      #ID  # Ação
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0): -1,  # NADA (Nenhuma ação)
        (1, 0, 0, 0, 0, 1, 0, 0, 0, 0): 1,   # POUSAR
        (0, 1, 0, 0, 0, 0, 1, 0, 0, 0): 2,   # CIMA
        (1, 1, 0, 0, 0, 1, 1, 0, 0, 0): 3,   # BAIXO
        (0, 1, 0, 0, 0, 0, 0, 0, 0, 0): 4,   # ESQUERDA
        (0, 0, 0, 0, 0, 0, 1, 0, 0, 0): 5,   # DIREITA
        (0, 1, 1, 1, 1, 0, 1, 1, 1, 1): 11,  # FRENTE
        (0, 1, 1, 1, 0, 0, 1, 1, 1, 0): 12,  # TRÁS
        (0, 0, 0, 0, 1, 0, 0, 0, 0, 0): 13,  # YAW-HORÁRIO
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 1): 14,  # YAW-ANTI-HORÁRIO
        (0, 1, 0, 0, 1, 0, 1, 0, 0, 1): 15,  # TAKEOFF
    }

    def __init__(
        self,
        model_complexity: int = 1,
        min_detection_confidence: float = 0.6,
        min_tracking_confidence: float = 0.6,
    ) -> None:
        """
        Inicializa o nó GestureRecognizer.
        """
        super().__init__("gesture_recognizer")

        self.detector = HandDetector(
            modelComplexity=model_complexity,
            detectionCon=min_detection_confidence,
            minTrackCon=min_tracking_confidence,
        )

        self.left_indice: int = 0
        self.right_indice: int = 0
        self.msg = Int16()

        self.acao_pub = self.create_publisher(Int16, "/drone/hands_action", 10)

        self.last_published_action_id = -1 

        self.declare_parameter("image_source", "oakd")
        self.image_source = self.get_parameter("image_source").get_parameter_value().string_value
        self.get_logger().info(f"Usando fonte de imagem: {self.image_source}")

        self.image_handler = ImageHandler(
            self,
            image_source=self.image_source,
            image_processing_callback=self.process,
            show_result="Gesture Recognizer",
            oakd_num=1,
        )

        self.frame_time = None

        self.image_handler.run()


    def process(self, img: np.array) -> None:
        """
        Processa o frame da imagem para detectar mãos e reconhecer gestos.
        """

        if self.frame_time is not None:
            duration_obj = self.get_clock().now() - self.frame_time
            duration_sec = duration_obj.nanoseconds / 1e9
            fps = 1.0 / duration_sec
            # Log do FPS. Use throttle para não poluir o console.
            self.get_logger().info(f"Processing FPS: {fps:.2f}", throttle_duration_sec=1.0)
        self.frame_time = self.get_clock().now()

        hands, img = self.detector.findHands(img)

        if not hands or len(hands) != 2:
            return

        self.right_indice, self.left_indice = (0, 1) if hands[0]["type"] == "Right" else (1, 0)
        fingers_right = self.detector.fingersUp(hands[self.right_indice])
        fingers_left = self.detector.fingersUp(hands[self.left_indice])
        self.msg.data = self.recognize_gesture(fingers_right, fingers_left)

        if self.msg.data != -1:
            self.acao_pub.publish(self.msg)
            self.get_logger().info(f"Gesto Publicado: ID {self.msg.data}")

        
    def recognize_gesture(self, fingers_right: list, fingers_left: list) -> int:
        """
        Reconhece o gesto com base na combinação da posição dos dedos.
        """
        return self.gestures.get(tuple(fingers_right + fingers_left), -1)

def main(args=None):
    rclpy.init(args=args)
    node = GestureRecognizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando o nó de reconhecimento de gestos.")
        node.image_handler.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
