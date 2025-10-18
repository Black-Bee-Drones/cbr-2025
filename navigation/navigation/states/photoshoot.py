import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

import cv2
import os
import time

from navigation.constants import (
    COMMAND_SLEEP, RED, RESET, GREEN, RESET
)

class Photoshoot(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        try:
            tello = blackboard["tello"]

            for i in range (0,7):
                read = tello.get_frame_read()
                current_waypoint = blackboard["current_waypoint"]
                cv2.imwrite(f"Waypoint_{current_waypoint}_{i}.png",read.frame)
                time.sleep(COMMAND_SLEEP)
                obstacle_distance = 2 # Pegar distancia real... # Distância com LiDar

                qcd = cv2.QRCodeDetector()
                retval, decoded_info_list, points_list, straight_qrcode_list = qcd.detectAndDecodeMulti(read.frame)
                
                if retval and len(decoded_info_list) > 0:
                    annotated_frame = read.frame.copy()
                    
                    for j, (decoded_info, points) in enumerate(zip(decoded_info_list, points_list)):
                        if points is not None and len(points) == 4:
                            points = points.astype(int)
                            cv2.polylines(annotated_frame, [points], True, (0, 255, 0), 3)
                            
                            text = f"QR: {decoded_info[:1]}" 
                            text_pos = (points[0][0], points[0][1] - 10)
                            cv2.putText(annotated_frame, text, text_pos, 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    os.makedirs("detections", exist_ok=True)
                    filename = f"detections/qr_{text}.png"
                    cv2.imwrite(filename, annotated_frame)
                    for info in decoded_info_list:
                        yasmin.YASMIN_LOG_INFO(f"{GREEN}QR Code: {info}{RESET}")

                tello.rotate_counter_clockwise(90)
                if i == 4:
                    current_height = tello.get_height()
                    if current_height >= 60:
                        delta_h = abs(40-current_height) 
                        tello.move("down", delta_h)
                    else:
                        tello.move("up", 120 - current_height) 
                    time.sleep(COMMAND_SLEEP)                       

                # Salva próximo waypoint [Altura, Yaw]
                if obstacle_distance >= 1:
                    if i == 2:
                        continue
                    time.sleep(COMMAND_SLEEP)
                    next_waypoint = {"height": tello.get_height(), "yaw": tello.get_yaw()}


            error_h = next_waypoint["height"] - tello.get_height()
            error_yaw = next_waypoint["yaw"] - tello.get_yaw()

            if error_yaw != 0 and error_yaw != 360:
                if error_yaw < 0:
                    tello.rotate_clockwise((360+error_yaw) % 360)
                else:
                    tello.rotate_counter_clockwise(360 - error_yaw)

            # z-axis movement
            if abs(error_h) >= 20:
                if error_h > 0:
                    tello.move("up", error_h)
                elif error_h < 0:
                    tello.move("down", abs(error_h))
                time.sleep(COMMAND_SLEEP)
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"{RED}Tello could not land. {e}{RESET}")
            return ABORT

        
