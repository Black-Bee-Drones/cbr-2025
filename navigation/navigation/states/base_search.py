import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

import cv2
import numpy as np
import time
from ultralytics import YOLO

from navigation.constants import (
    COMMAND_SLEEP, GREEN, RED, YELLOW, RESET, YOLO_CONFIDENCE, SEARCH_POSITIONS
)

class BaseSearch(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        try:
            self.model = YOLO("models/best.pt") 
            yasmin.YASMIN_LOG_INFO(f"{GREEN}YOLO model loaded successfully{RESET}")
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"{RED}Failed to load YOLO model: {e}{RESET}")
            self.model = None
    
    def get_frame_center(self, frame):
        height, width = frame.shape[:2]
        return width // 2, height // 2
    
    def get_bbox_center(self, bbox):
        x1, y1, x2, y2 = bbox
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        return center_x, center_y
    
    def calculate_movement(self, frame_center, bbox_center, threshold=50):
        frame_cx, frame_cy = frame_center
        bbox_cx, bbox_cy = bbox_center
        
        error_x = bbox_cx - frame_cx
        error_y = bbox_cy - frame_cy
        
        movement = {"x": 0, "y": 0, "z": 0, "yaw": 0}
        
        if abs(error_x) > threshold:
            movement["y"] = int(error_x * 0.1)  
        
        if abs(error_y) > threshold:
            movement["z"] = int(-error_y * 0.1)
        
        return movement, abs(error_x) < threshold and abs(error_y) < threshold
    
    def detect_at_position(self, tello):
        if not self.model:
            yasmin.YASMIN_LOG_ERROR(f"{RED}YOLO model not available{RESET}")
            return False
        
        read = tello.get_frame_read()
        frame = read.frame
        
        if frame is None:
            yasmin.YASMIN_LOG_WARN(f"{YELLOW}No frame received{RESET}")
            return False
        
        results = self.model(frame, conf=YOLO_CONFIDENCE)
        
        best_detection = None
        best_confidence = 0
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    confidence = float(box.conf[0])
                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_detection = box.xyxy[0].cpu().numpy()
        
        if best_detection is not None:
            yasmin.YASMIN_LOG_INFO(f"{GREEN}Detection found! Confidence: {best_confidence:.2f}{RESET}")
            
            x1, y1, x2, y2 = best_detection.astype(int)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"Base: {best_confidence:.2f}", 
                       (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imwrite(f"base_detected_{int(time.time())}.png", frame)
            
            return True
        else:
            yasmin.YASMIN_LOG_INFO(f"{YELLOW}No detection at this position{RESET}")
            return False
    
    def center_on_target(self, tello, max_attempts=20):
        """Centraliza o drone na detecção encontrada"""
        if not self.model:
            return False
        
        attempts = 0
        while attempts < max_attempts:
            read = tello.get_frame_read()
            frame = read.frame
            
            if frame is None:
                time.sleep(0.5)
                attempts += 1
                continue
            
            results = self.model(frame, conf=YOLO_CONFIDENCE)
            
            best_detection = None
            best_confidence = 0
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        confidence = float(box.conf[0])
                        if confidence > best_confidence:
                            best_confidence = confidence
                            best_detection = box.xyxy[0].cpu().numpy()
            
            if best_detection is not None:
                frame_center = self.get_frame_center(frame)
                bbox_center = self.get_bbox_center(best_detection)
                
                movement, centered = self.calculate_movement(frame_center, bbox_center, threshold=30)
                
                if centered:
                    yasmin.YASMIN_LOG_INFO(f"{GREEN}Target perfectly centered!{RESET}")
                    return True
                
                if abs(movement["y"]) >= 20:
                    direction = "right" if movement["y"] > 0 else "left"
                    distance = min(abs(movement["y"]), 50)
                    yasmin.YASMIN_LOG_INFO(f"{YELLOW}Centering: moving {direction} {distance}cm{RESET}")
                    tello.move(direction, distance)
                    time.sleep(COMMAND_SLEEP)
                
                if abs(movement["z"]) >= 20:
                    direction = "up" if movement["z"] > 0 else "down"
                    distance = min(abs(movement["z"]), 50)
                    yasmin.YASMIN_LOG_INFO(f"{YELLOW}Centering: moving {direction} {distance}cm{RESET}")
                    tello.move(direction, distance)
                    time.sleep(COMMAND_SLEEP)
            else:
                yasmin.YASMIN_LOG_WARN(f"{YELLOW}Lost target during centering{RESET}")
                return False
            
            attempts += 1
        
        return False

    def execute(self, blackboard: Blackboard):
        try:
            tello = blackboard["tello"]
            
            yasmin.YASMIN_LOG_INFO(f"{GREEN}Starting base search with predefined positions{RESET}")
            
            search_positions = SEARCH_POSITIONS
            
            for i, position in enumerate(search_positions):
                yasmin.YASMIN_LOG_INFO(f"{YELLOW}Moving to {position['name']}: x={position['x']}, y={position['y']}{RESET}")
                
                tello.go_xyz_speed(
                    x=position["x"], 
                    y=position["y"], 
                    z=position["z"], 
                    speed=50
                )
                time.sleep(3)  
                
                detection_found = self.detect_at_position(tello)
                
                if detection_found:
                    yasmin.YASMIN_LOG_INFO(f"{GREEN}Base found at {position['name']}! Starting centering...{RESET}")
                    
                    centered = self.center_on_target(tello)
                    
                    if centered:
                        yasmin.YASMIN_LOG_INFO(f"{GREEN}Successfully centered on base! Landing...{RESET}")
                        return SUCCEED
                    else:
                        yasmin.YASMIN_LOG_WARN(f"{YELLOW}Could not center perfectly, but landing anyway{RESET}")
                        return SUCCEED
                else:
                    yasmin.YASMIN_LOG_INFO(f"{YELLOW}No base detected at {position['name']}, moving to next position{RESET}")
            
            yasmin.YASMIN_LOG_WARN(f"{YELLOW}Base not found in any search position{RESET}")
            return SUCCEED
                
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"{RED}Base search failed: {e}{RESET}")
            return ABORT        
