import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from time import time, sleep
from mirela_sdk.control.mavros.mavros_api import MavDrone

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


class GestureController(Node):
    """
    Este nó controla um MavDrone com base nos IDs de gestos recebidos.
    """
    def __init__(self) -> None:
        super().__init__("gesture_controller")

        self.mavdrone = MavDrone(node=self, mavros=False, indoor=True)
        self.create_subscription(Int16, "/drone/hands_action", self._moviment_callback, 10)

        self.current_action: int = -1
        self.previous_action: int = -1
        self.action_start_time: float = time()
        self.command_sent: bool = False
        self.land_pub = self.create_publisher(Int16, RTL_COUNT_TOPIC, 10)
        
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

        self.single_actions: dict[int, tuple[str, callable]] = {
            1: ("Pousar", lambda: self.land_action()),
            15: ("Decolar", lambda: self.arm_takeoff_action()),
        }
        
    def arm_takeoff_action(self) -> None:
        """
        Arma o drone e decola para a altura especificada nas constantes.
        Após decolar, aguarda um tempo também definido nas constantes.
        """
        self.mavdrone.arm_takeoff(TAKEOFF_HEIGHT)
        sleep(SLEEP_AFTER_TAKEOFF)

    def land_action(self):

        self.land_pub.publish(Int16(data=1))
        self.get_logger().info(f"RTL Land Trigger published.")

        self.mavdrone.land()
        self.mavdrone.delay(SLEEP_AFTER_LAND)


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

        if time() - self.action_start_time >= ACTION_TIMEOUT:
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
        sleep(SLEEP_AFTER_LAND)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
