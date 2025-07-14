import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from time import time, sleep
from mirela_sdk.control.mavros.mavros_api import MavDrone

class GestureController(Node):
    """
    Este nó controla um MavDrone com base nos IDs de gestos recebidos.
    """
    def __init__(self) -> None:
        super().__init__("gesture_controller")

        self.mavdrone = MavDrone(node=self, mavros=False)
        self.create_subscription(Int16, "/drone/hands_action", self._moviment_callback, 10)

        self.current_action: int = -1
        self.previous_action: int = -1
        self.action_start_time: float = time()
        self.command_sent: bool = False
        
        self.continuous_actions: dict[int, tuple[str, callable]] = {
            -1: ("Parar", lambda: self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)),
            2: ("Subir", lambda: self.mavdrone.offboard_velocity(linear_z=0.5)),
            3: ("Descer", lambda: self.mavdrone.offboard_velocity(linear_z=-0.5)),
            4: ("Ir para Esquerda", lambda: self.mavdrone.offboard_velocity(linear_y=0.5)),
            5: ("Ir para Direita", lambda: self.mavdrone.offboard_velocity(linear_y=-0.5)),
            11: ("Ir para Frente", lambda: self.mavdrone.offboard_velocity(linear_x=0.5)),
            12: ("Ir para Trás", lambda: self.mavdrone.offboard_velocity(linear_x=-0.5)),
            13: ("Girar Horário", lambda: self.mavdrone.offboard_velocity(angular_z=-0.5)),
            14: ("Girar Anti-Horário", lambda: self.mavdrone.offboard_velocity(angular_z=0.5)),
        }

        self.single_actions: dict[int, tuple[str, callable]] = {
            1: ("Pousar", lambda: self.mavdrone.land()),
        }
        
        # self.arm_and_takeoff(2.0)

    def arm_and_takeoff(self):
        """
        Prepara o drone para o voo, armando e decolando para uma altitude segura.
        """
        self.get_logger().info("Aguardando conexão com o MAVROS...")
        while not self.mavdrone.get_state.connected and rclpy.ok():
            self.get_logger().info("Esperando conexão com o FCU via MAVROS...")
            sleep(1)
        
        self.get_logger().info("MAVROS conectado. Armado e decolando para 3 metros...")
        self.mavdrone.arm_takeoff(3.0)
        sleep(5)
        self.get_logger().info("Drone pronto para receber comandos de gestos.")

    def _moviment_callback(self, msg: Int16) -> None:
        """
        Callback para processar os comandos de gestos recebidos.
        """
        self.previous_action = self.current_action
        self.current_action = msg.data

        if self.previous_action != self.current_action:
            self.action_start_time = time()
            self.command_sent = False
            return

        if time() - self.action_start_time >= 0.5:
            if self.current_action in self.continuous_actions:
                action_name, action_func = self.continuous_actions[self.current_action]
                self.get_logger().info(f"Ação Contínua: {action_name}")
                action_func()
            
            elif self.current_action in self.single_actions and not self.command_sent:
                action_name, action_func = self.single_actions[self.current_action]
                self.get_logger().info(f"Ação Única: {action_name}")
                action_func()
                self.command_sent = True

def main(args=None):
    rclpy.init(args=args)
    controller = GestureController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Interrupção de teclado recebida. Pousando o drone...")
        controller.mavdrone.land()
        sleep(5)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
