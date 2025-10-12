import rclpy
import cv2
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import Int16
import numpy as np
from cvzone.HandTrackingModule import HandDetector
from mirela_sdk.image_processing.camera.image_handler import ImageHandler
from mirela_sdk.image_processing.camera import IMX219Config
from mirela_sdk.control.mavros.mavros_api import MavDrone
import time
from time import sleep

from interaction.constants import (
    TAKEOFF_HEIGHT,
    VELOCITY_UP_DOWN,
    VELOCITY_SIDES,
    VELOCITY_IN_OUT,
    VELOCITY_YAW,
    ACTION_TIMEOUT,
    SLEEP_AFTER_TAKEOFF,
    SLEEP_AFTER_LAND,
    RTL_COUNT_TOPIC
)


class GestureRecognizer(Node):
    """
    Esta classe reconhece gestos com as duas mãos usando a câmera OAK-D e 
    controla diretamente um MavDrone com base nos gestos detectados.
    
    IMPORTANTE: Para evitar deadlocks causados por rclpy.spin_once() dentro dos 
    métodos do MavDrone, esta classe utiliza callback groups do ROS2 para separar
    a execução de comandos do drone em um grupo de callback diferente, permitindo
    execução paralela segura com MultiThreadedExecutor.
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
        self.processing_width = 640
        self.processing_height = 480
        
        # Callback groups para separar processamento de imagem e comandos do drone
        self.image_callback_group = MutuallyExclusiveCallbackGroup()
        self.drone_callback_group = ReentrantCallbackGroup()
        
        # Inicializa o controle do drone
        self.mavdrone = MavDrone(node=self, mavros=False, indoor=True)
        
        # Variáveis de controle de ações
        self.current_action: int = -1
        self.previous_action: int = -1
        self.action_start_time: float = time.time()
        self.command_sent: bool = False
        self.land_pub = self.create_publisher(Int16, RTL_COUNT_TOPIC, 10, callback_group=self.drone_callback_group)
        
        # Timer para processar comandos de drone em callback group separado
        self.drone_command_timer = self.create_timer(
            0.01,  # 100Hz para comandos de drone
            self.execute_drone_commands,
            callback_group=self.drone_callback_group
        )
        
        # Define as ações contínuas (executadas continuamente enquanto o gesto está ativo)
        self.continuous_actions: dict[int, tuple[str, callable]] = {
            -1: ("Parar", lambda: self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)),
            2: ("Subir", lambda: self.mavdrone.offboard_velocity(linear_z=VELOCITY_UP_DOWN)),
            3: ("Descer", lambda: self.mavdrone.offboard_velocity(linear_z=-VELOCITY_UP_DOWN)),
            4: ("Ir para Esquerda", lambda: self.mavdrone.offboard_velocity(linear_y=VELOCITY_SIDES)),
            5: ("Ir para Direita", lambda: self.mavdrone.offboard_velocity(linear_y=-VELOCITY_SIDES)),
            11: ("Ir para Frente", lambda: self.mavdrone.offboard_velocity(linear_x=VELOCITY_IN_OUT)),
            12: ("Ir para Trás", lambda: self.mavdrone.offboard_velocity(linear_x=-VELOCITY_IN_OUT)),
            13: ("Girar Horário", lambda: self.mavdrone.offboard_velocity(angular_z=VELOCITY_YAW)),
            14: ("Girar Anti-Horário", lambda: self.mavdrone.offboard_velocity(angular_z=-VELOCITY_YAW)),
        }

        # Define as ações únicas (executadas uma vez quando o gesto é detectado)
        self.single_actions: dict[int, tuple[str, callable]] = {
            1: ("Pousar", lambda: self.land_action()),
            15: ("Decolar", lambda: self.mavdrone.arm_takeoff(TAKEOFF_HEIGHT)),
        }
        
        # Comando atual a ser executado
        self.pending_action_id: int = -1 

        self.declare_parameter("image_source", "imx219")
        self.image_source = self.get_parameter("image_source").get_parameter_value().string_value
        self.get_logger().info(f"Usando fonte de imagem: {self.image_source}")

        self.image_handler = ImageHandler(
            node=self,
            image_source="imx219",
            image_processing_callback=self.process,
            config=IMX219Config(sensor_id=0, width=1640, height=1232, flip=2)
        )
        time.sleep(1.0)  # Aguarda a inicialização da câmera.
        self.get_logger().info("Câmera inicializada.")

        self.frame_time = None

        self.image_handler.open()
        time.sleep(1.0)  # Aguarda a câmera abrir.
        self.get_logger().info("Câmera aberta e pronta.")

        self.image_handler.run()
        self.get_logger().info("Processamento de imagem iniciado.")


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

        # Resize para melhor performance
        resized_img = cv2.resize(img, (self.processing_width, self.processing_height))

        hands, img = self.detector.findHands(resized_img)

        if not hands or len(hands) != 2:
            return

        self.right_indice, self.left_indice = (0, 1) if hands[0]["type"] == "Right" else (1, 0)
        fingers_right = self.detector.fingersUp(hands[self.right_indice])
        fingers_left = self.detector.fingersUp(hands[self.left_indice])
        gesture_id = self.recognize_gesture(fingers_right, fingers_left)

        # Processa o gesto detectado diretamente
        self.process_gesture_action(gesture_id)
        
        # Limpa ação pendente se não há gesto válido
        if gesture_id == -1:
            self.pending_action_id = -1

        
    def recognize_gesture(self, fingers_right: list, fingers_left: list) -> int:
        """
        Reconhece o gesto com base na combinação da posição dos dedos.
        """
        return self.gestures.get(tuple(fingers_right + fingers_left), -1)
    
    def execute_drone_commands(self):
        """
        Timer callback executado em callback group separado para comandos do drone.
        Isso permite execução paralela segura com MultiThreadedExecutor, evitando 
        deadlocks com rclpy.spin_once() que alguns métodos do MavDrone podem usar.
        """
        if self.pending_action_id == -1:
            return
            
        try:
            if self.pending_action_id in self.continuous_actions:
                action_name, action_func = self.continuous_actions[self.pending_action_id]
                action_func()
                
            elif self.pending_action_id in self.single_actions and not self.command_sent:
                action_name, action_func = self.single_actions[self.pending_action_id]
                self.get_logger().info(f"Executando Ação Única: {action_name}")
                action_func()
                self.command_sent = True
                self.get_logger().info(f"Ação {action_name} executada com sucesso!")
                
        except Exception as e:
            self.get_logger().error(f"Erro ao executar comando do drone: {e}")
    
    def land_action(self):
        """
        Executa a ação de pouso do drone.
        """
        self.land_pub.publish(Int16(data=1))
        self.get_logger().info(f"RTL Land Trigger published.")
        self.mavdrone.land()
    
    def process_gesture_action(self, gesture_id: int) -> None:
        """
        Processa a ação correspondente ao gesto detectado.
        """
        self.previous_action = self.current_action
        self.current_action = gesture_id
        
        # Debug: mostrar todas as ações detectadas
        self.get_logger().info(f"Gesto detectado ID: {self.current_action}")

        if self.previous_action != self.current_action:
            self.action_start_time = time.time()
            self.command_sent = False
            self.get_logger().info(f"Nova ação detectada: {self.current_action}, aguardando timeout...")
            return

        time_elapsed = time.time() - self.action_start_time
        self.get_logger().info(f"Tempo decorrido: {time_elapsed:.2f}s, timeout: {ACTION_TIMEOUT}s")
        
        if time_elapsed >= ACTION_TIMEOUT:
            if self.current_action in self.continuous_actions:
                action_name, _ = self.continuous_actions[self.current_action]
                self.get_logger().info(f"Executando Ação Contínua: {action_name}")
                # Define a ação pendente para execução no timer callback separado
                self.pending_action_id = self.current_action
            
            elif self.current_action in self.single_actions and not self.command_sent:
                action_name, _ = self.single_actions[self.current_action]
                self.get_logger().info(f"Preparando Ação Única: {action_name}")
                # Define a ação pendente para execução no timer callback separado
                self.pending_action_id = self.current_action

def main(args=None):
    rclpy.init(args=args)
    node = GestureRecognizer()
    
    # Usa MultiThreadedExecutor para permitir execução paralela dos callback groups
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupção de teclado recebida. Pousando o drone...")
        # Pousa o drone de forma segura
        node.mavdrone.land()
        sleep(SLEEP_AFTER_LAND)
        node.image_handler.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
