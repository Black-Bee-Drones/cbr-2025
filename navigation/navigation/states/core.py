import rclpy

import time
import threading

import yasmin
from yasmin import Blackboard
from yasmin import State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

import djitellopy

from navigation.constants import (
    # Log formatting constants
    RED, GREEN, YELLOW, RESET,
    # Timeout settings
    CONNECTION_TIMEOUT,
    TAKEOFF_HEIGHT,
    COMMAND_SLEEP
)

def start_qr_detector_node(tello_instance, blackboard):
    """Function to run QR detector node in a separate thread"""
    import cv2
    import os
    import time
    
    class QRDetector:
        def __init__(self, tello, blackboard):
            self.tello = tello
            self.blackboard = blackboard
            self.detection_folder = 'qr_detections'
            os.makedirs(self.detection_folder, exist_ok=True)
            self.running = True
            self.seen_qr_codes = set()  # Track already seen QR codes
            
            yasmin.YASMIN_LOG_INFO(f'{GREEN}QR Detector started!{RESET}')

        def detect_qr_loop(self):
            while self.running:
                try:
                    # Get frame from Tello
                    read = self.tello.get_frame_read()
                    if read.frame is None:
                        time.sleep(0.5)
                        continue
                    
                    frame = read.frame
                    timestamp = int(time.time())
                    
                    # Detect QR codes
                    qcd = cv2.QRCodeDetector()
                    retval, decoded_info_list, points_list, straight_qrcode_list = qcd.detectAndDecodeMulti(frame)
                    
                    if retval and len(decoded_info_list) > 0:
                        annotated_frame = frame.copy()
                        new_qr_found = False
                        
                        for j, (decoded_info, points) in enumerate(zip(decoded_info_list, points_list)):
                            if points is not None and len(points) == 4:
                                points = points.astype(int)
                                cv2.polylines(annotated_frame, [points], True, (0, 255, 0), 3)
                                
                                text = f"QR: {decoded_info[:50]}"
                                text_pos = (points[0][0], points[0][1] - 10)
                                cv2.putText(annotated_frame, text, text_pos, 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                                
                                # Check if this is a new QR code
                                if decoded_info not in self.seen_qr_codes:
                                    self.seen_qr_codes.add(decoded_info)
                                    new_qr_found = True
                                    
                                    # Update blackboard to enable fast forwarding when QR is detected
                                    self.blackboard["is_fast_forwarding"] = True
                                    
                                    yasmin.YASMIN_LOG_INFO(f"{GREEN}QR Code detected! Content: {decoded_info} - Fast forwarding enabled!{RESET}")
                        
                        if new_qr_found:
                            detection_filename = f"{self.detection_folder}/qr_detection_{timestamp}.png"
                            cv2.imwrite(detection_filename, annotated_frame)
                            
                        time.sleep(1/20)
                except Exception as e:
                    yasmin.YASMIN_LOG_ERROR(f"{RED}Error during QR detection: {e}{RESET}")

        
        def stop(self):
            self.running = False

    qr_detector = QRDetector(tello_instance, blackboard)
    qr_detector.detect_qr_loop()
    
    return qr_detector

class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.qr_detector_thread = None
        self.qr_detector = None

    def execute(self, blackboard: Blackboard):
        try:
            tello = djitellopy.Tello()
            blackboard["tello"] = tello

            blackboard["current_waypoint"] = 0
            blackboard["is_fast_forwarding"] = False
            
            yasmin.YASMIN_LOG_INFO(f"{YELLOW}Waiting for Tello to be fully ready...{RESET}")

            tello.connect(wait_for_state=True) 

            max_attempts = CONNECTION_TIMEOUT
            for attempt in range(max_attempts):
                try:
                    battery = tello.get_battery()
                    if battery > 0: 
                        yasmin.YASMIN_LOG_INFO(f"{GREEN}Tello is ready! Battery: {battery}%{RESET}")
                        break
                except:
                    time.sleep(1)
                    yasmin.YASMIN_LOG_INFO(f"{YELLOW}Waiting... ({attempt+1}/{max_attempts}){RESET}")
            else:
                raise Exception("Tello did not respond within timeout period")
            
            tello.streamon()
            time.sleep(2)  # Give time for stream to start
            
            # Start QR detector in a separate thread
            self.qr_detector_thread = threading.Thread(
                target=start_qr_detector_node, 
                args=(tello, blackboard),
                daemon=True
            )
            self.qr_detector_thread.start()
            blackboard["qr_detector_thread"] = self.qr_detector_thread
            
            yasmin.YASMIN_LOG_INFO(f"{GREEN}QR detector thread started successfully!{RESET}")
            
            input(f"{YELLOW}Tello connected. Press Enter to continue when ready...{RESET}")

            
            yasmin.YASMIN_LOG_INFO(f"{GREEN}Phase 4 initialized.{RESET}")
            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"{RED}Phase 4 could not be initialized. {e}{RESET}")
            return ABORT


class Takeoff(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        try:
            tello = blackboard["tello"]
            tello.takeoff() 
            time.sleep(3) # Wait to stabilize
            yasmin.YASMIN_LOG_INFO(f"{GREEN}Tello TAKE OFF was successful.{RESET}")
            takeoff_error = 60 - tello.get_height()
            while abs(takeoff_error) >= 20:
                tello.send_rc_control(
                            left_right_velocity = 0, 
                            forward_backward_velocity = 0, 
                            up_down_velocity = 15 if takeoff_error > 0 else -15,
                            yaw_velocity = 0                                            
                        )
                time.sleep(tello.TIME_BTW_RC_CONTROL_COMMANDS)
                takeoff_error = 60 - tello.get_height()
                print(takeoff_error)

            yasmin.YASMIN_LOG_INFO(f"{YELLOW}Adjusted to desired take off attitude.{RESET}")
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"{RED}Tello could not takeoff. {e}{RESET}")
            return ABORT


class Land(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        try:
            tello = blackboard["tello"]
            
            if "qr_detector_thread" in blackboard and blackboard["qr_detector_thread"]:
                yasmin.YASMIN_LOG_INFO(f"{YELLOW}Stopping QR detector thread...{RESET}")
                yasmin.YASMIN_LOG_INFO(f"{GREEN}QR detector thread will stop automatically.{RESET}")
            
            tello.streamoff()
            
            yasmin.YASMIN_LOG_INFO(f"{GREEN}Tello LAND was successful.{RESET}")            
            ...
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"{RED}Tello could not land. {e}{RESET}")
            yasmin.YASMIN_LOG_WARN(f"{YELLOW}Human intervention is needed.{RESET}")
            return ABORT

