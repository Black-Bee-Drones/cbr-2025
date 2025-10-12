import rclpy
import cv2
import time
import numpy as np
from cvzone.HandTrackingModule import HandDetector
from mirela_sdk.image_processing.camera.image_handler import ImageHandler
from mirela_sdk.image_processing.camera import IMX219Config
from mirela_sdk.control.mavros.mavros_api import MavDrone

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import Int16

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


class GestureControl(State):
    """
    Estado que reconhece gestos com as duas mãos usando a câmera OAK-D e 
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
    ):
        super().__init__(outcomes=[SUCCEED, ABORT])
        
        self.model_complexity = model_complexity
        self.min_detection_confidence = min_detection_confidence
        self.min_tracking_confidence = min_tracking_confidence
        
        # Variáveis de controle
        self.detector = None
        self.image_handler = None
        self.current_action: int = -1
        self.previous_action: int = -1
        self.action_start_time: float = 0.0
        self.command_sent: bool = False
        self.processing_width = 640
        self.processing_height = 480
        self.frame_time = None
        
        # Variáveis de controle do estado
        self.mavdrone = None
        self.land_count = 0
        self.drone_command_timer = None
        self.pending_action_id: int = -1
        self.node = None
        self.blackboard = None

    def execute(self, blackboard: Blackboard):
        """Executa o estado de controle por gestos."""
        
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in GestureControl state.")
            return ABORT
            
        self.mavdrone: MavDrone = blackboard["mavdrone"]
        self.node = YasminNode.get_instance()
        self.blackboard = blackboard
        
        try:
            # Inicializa os componentes
            if not self._initialize_components():
                return ABORT
                
            yasmin.YASMIN_LOG_INFO("Iniciando controle por gestos...")
            
            # Loop principal do estado
            start_time = time.time()
            max_duration = 120.0  # 2 minutos máximo no estado
            
            while rclpy.ok() and (time.time() - start_time) < max_duration:
                # Verifica se deve sair do estado (exemplo: contador RTL)
                if self._should_exit_state(blackboard):
                    yasmin.YASMIN_LOG_INFO("Condição de saída do estado detectada.")
                    break
                    
                # Processa um ciclo do ROS
                rclpy.spin_once(self.node, timeout_sec=0.01)
                
            yasmin.YASMIN_LOG_INFO("Encerrando controle por gestos...")
            return SUCCEED
            
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Erro no estado GestureControl: {e}")
            return ABORT
            
        finally:
            self._cleanup()

    def _initialize_components(self):
        """Inicializa todos os componentes necessários."""
        try:
            # Inicializa detector de mãos
            self.detector = HandDetector(
                modelComplexity=self.model_complexity,
                detectionCon=self.min_detection_confidence,
                minTrackCon=self.min_tracking_confidence,
            )
            
            # Inicializa câmera
            self.image_handler = ImageHandler(
                node=self.node,
                image_source="webcam",
                image_processing_callback=self.process_image,
                # config=IMX219Config(sensor_id=0, width=1640, height=1232, flip=2)
            )
            
            time.sleep(1.0)  # Aguarda inicialização da câmera
            yasmin.YASMIN_LOG_INFO("Câmera inicializada.")
            
            self.image_handler.open()
            time.sleep(1.0)  # Aguarda câmera abrir
            yasmin.YASMIN_LOG_INFO("Câmera aberta e pronta.")
            
            self.image_handler.run()
            yasmin.YASMIN_LOG_INFO("Processamento de imagem iniciado.")
            
            return True
            
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Erro na inicialização dos componentes: {e}")
            return False

    def process_image(self, img: np.array) -> None:
        """Processa o frame da imagem para detectar mãos e reconhecer gestos."""
        
        try:
            if self.frame_time is not None:
                duration_obj = self.node.get_clock().now() - self.frame_time
                duration_sec = duration_obj.nanoseconds / 1e9
                fps = 1.0 / duration_sec
                # Log throttled para não poluir
                yasmin.YASMIN_LOG_INFO(f"Processing FPS: {fps:.2f}")
            self.frame_time = self.node.get_clock().now()

            # Resize para melhor performance
            resized_img = cv2.resize(img, (self.processing_width, self.processing_height))

            hands, img = self.detector.findHands(resized_img)

            if not hands or len(hands) != 2:
                self.pending_action_id = -1
                return

            right_indice, left_indice = (0, 1) if hands[0]["type"] == "Right" else (1, 0)
            fingers_right = self.detector.fingersUp(hands[right_indice])
            fingers_left = self.detector.fingersUp(hands[left_indice])
            gesture_id = self.recognize_gesture(fingers_right, fingers_left)

            # Processa o gesto detectado
            self.process_gesture_action(gesture_id)
            
            # Limpa ação pendente se não há gesto válido
            if gesture_id == -1:
                self.pending_action_id = -1
                
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Erro no processamento de imagem: {e}")

    def recognize_gesture(self, fingers_right: list, fingers_left: list) -> int:
        """Reconhece o gesto com base na combinação da posição dos dedos."""
        return self.gestures.get(tuple(fingers_right + fingers_left), -1)

    def process_gesture_action(self, gesture_id: int) -> None:
        """Processa a ação correspondente ao gesto detectado."""
        self.previous_action = self.current_action
        self.current_action = gesture_id
        
        yasmin.YASMIN_LOG_INFO(f"Gesto detectado ID: {self.current_action}")

        if self.previous_action != self.current_action:
            self.action_start_time = time.time()
            self.command_sent = False
            yasmin.YASMIN_LOG_INFO(f"Nova ação detectada: {self.current_action}, aguardando timeout...")
            return

        time_elapsed = time.time() - self.action_start_time
        
        if time_elapsed >= ACTION_TIMEOUT:
            # Define as ações disponíveis
            continuous_actions = {
                -1: "Parar",
                2: "Subir",
                3: "Descer", 
                4: "Ir para Esquerda",
                5: "Ir para Direita",
                11: "Ir para Frente",
                12: "Ir para Trás",
                13: "Girar Horário",
                14: "Girar Anti-Horário",
            }
            
            single_actions = {
                1: "Pousar",
                15: "Decolar",
            }
            
            if self.current_action in continuous_actions:
                action_name = continuous_actions[self.current_action]
                yasmin.YASMIN_LOG_INFO(f"Executando Ação Contínua: {action_name}")
                self.pending_action_id = self.current_action
            
            elif self.current_action in single_actions and not self.command_sent:
                action_name = single_actions[self.current_action]
                yasmin.YASMIN_LOG_INFO(f"Preparando Ação Única: {action_name}")
                self.pending_action_id = self.current_action

    def execute_drone_commands(self):
        """Timer callback executado em callback group separado para comandos do drone."""
        if self.pending_action_id == -1 or not self.mavdrone:
            return
            
        try:
            # Ações contínuas
            if self.pending_action_id == -1:
                self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
            elif self.pending_action_id == 2:
                self.mavdrone.offboard_velocity(linear_z=VELOCITY_UP_DOWN)
            elif self.pending_action_id == 3:
                self.mavdrone.offboard_velocity(linear_z=-VELOCITY_UP_DOWN)
            elif self.pending_action_id == 4:
                self.mavdrone.offboard_velocity(linear_y=VELOCITY_SIDES)
            elif self.pending_action_id == 5:
                self.mavdrone.offboard_velocity(linear_y=-VELOCITY_SIDES)
            elif self.pending_action_id == 11:
                self.mavdrone.offboard_velocity(linear_x=VELOCITY_IN_OUT)
            elif self.pending_action_id == 12:
                self.mavdrone.offboard_velocity(linear_x=-VELOCITY_IN_OUT)
            elif self.pending_action_id == 13:
                self.mavdrone.offboard_velocity(angular_z=VELOCITY_YAW)
            elif self.pending_action_id == 14:
                self.mavdrone.offboard_velocity(angular_z=-VELOCITY_YAW)
            # Ações únicas
            elif self.pending_action_id == 1 and not self.command_sent:
                yasmin.YASMIN_LOG_INFO("Executando comando de pouso")
                self.land_pub.publish(Int16(data=1))
                self.mavdrone.land()
                self.command_sent = True
            elif self.pending_action_id == 15 and not self.command_sent:
                yasmin.YASMIN_LOG_INFO("Executando comando de decolagem")
                self.mavdrone.arm_takeoff(TAKEOFF_HEIGHT)
                self.command_sent = True
                
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Erro ao executar comando do drone: {e}")

    def _count_callback(self, msg: Int16):
        """Callback que atualiza o contador RTL no blackboard."""
        if self.blackboard:
            self.land_count += 1
            yasmin.YASMIN_LOG_INFO(f"RTL Count: {self.land_count}/6")

    def _should_exit_state(self, blackboard: Blackboard) -> bool:
        """Verifica se deve sair do estado baseado em condições específicas."""
        # Verifica o contador RTL para transição para próximo estado
        rtl_counter = blackboard.get("rtl_land_counter", 0)
        if rtl_counter >= 6:  # RTL_REQUIRED_COUNT das constantes
            yasmin.YASMIN_LOG_INFO(f"RTL counter reached {rtl_counter}/6, transitioning to next state.")
            return True
            
        return False

    def _cleanup(self):
        """Limpa recursos utilizados pelo estado."""
        try:
            if self.image_handler:
                self.image_handler.cleanup()
                
            if self.drone_command_timer:
                self.node.destroy_timer(self.drone_command_timer)
                
            if self.land_pub:
                self.node.destroy_publisher(self.land_pub)
                
            if self.land_count_sub:
                self.node.destroy_subscription(self.land_count_sub)
                
            # Para o drone
            if self.mavdrone:
                self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                
            yasmin.YASMIN_LOG_INFO("Limpeza do estado GestureControl concluída.")
            
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Erro durante limpeza: {e}")