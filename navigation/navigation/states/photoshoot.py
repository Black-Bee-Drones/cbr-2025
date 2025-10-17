import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

import time

from navigation.constants import (
    COMMAND_SLEEP
)

class Photoshoot(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        try:
            tello = blackboard["tello"]

            photo_iteration = 0

            for i in range (0,7):
                frame = tello.get_frame_read()
                time.sleep(COMMAND_SLEEP)
                obstacle_distance = 2 # Pegar distancia real... # Distância com LiDar

                # Rodar QR CODE Detector no OpenCV

                photo_iteration += 1
                tello.rotate_counter_clockwise(90)
                if i == 4:
                    current_height = tello.get_height()
                    if current_height > 70:
                        delta_h = abs(40-current_height)
                        if delta_h != 0:
                            tello.move("down", delta_h)
                    else:
                        tello.move("up", 120 - current_height)                        

                # Salva próximo waypoint [Altura, Yaw]
                if obstacle_distance >= 1:
                    if i == 2:
                        continue
                    next_waypoint = {"height": tello.get_height(), "yaw": tello.get_yaw()}


            error_h = next_waypoint["height"] - tello.get_height()
            error_yaw = next_waypoint["yaw"] - tello.get_yaw()

            if error_yaw != 0 and error_yaw != 360:
                if error_yaw < 0:
                    tello.rotate_clockwise((360+error_yaw) % 360)
                else:
                    tello.rotate_counter_clockwise(360 - error_yaw)

            # z-axis movement
            if error_h > 0:
                tello.move("up", error_h)
            elif error_h < 0:
                tello.move("down", abs(error_h))
        
            return SUCCEED
        except Exception as e:
            return ABORT

        
