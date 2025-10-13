#!/usr/bin/env python3
"""
Test YOLO detection and centering error calculation.
Captures images and runs detection without drone movement.
"""

import cv2
import time
import rclpy
from yasmin_ros.yasmin_node import YasminNode
from mirela_sdk.image_processing.camera.image_handler import ImageHandler
from mirela_sdk.image_processing.camera import IMX219Config

from mapping.utils import YOLODetector
from mapping.constants import (
    CAMERA_SOURCE,
    IMAGE_CENTER_X,
    IMAGE_CENTER_Y,
    CENTERING_TOLERANCE_PX,
    DETECTION_SAVE_PATH,
    YOLO_MODEL_PATH
)


def test_detection():
    print(f"- Camera source: {CAMERA_SOURCE}")
    print(f"- Image center: ({IMAGE_CENTER_X}, {IMAGE_CENTER_Y})")
    print(f"- Centering tolerance: {CENTERING_TOLERANCE_PX}px")
    print(f"- Saving detections to: {DETECTION_SAVE_PATH}")
    
    rclpy.init()
    node = YasminNode.get_instance()

    image_handler = ImageHandler(
        node=node, 
        image_source=CAMERA_SOURCE,
        config=IMX219Config(sensor_id=0, width=1640, height=1232, flip=2),
    )
    time.sleep(2) 
    image_handler.open() 
    yolo_detector = YOLODetector(YOLO_MODEL_PATH)
    
    frame_count = 0
    detection_count = 0
    
    try:
        while True:
            try:
                frame = image_handler.take_photo()
            except RuntimeError as e:
                print(f"Camera error: {e}")
                break
            if frame is None:
                print("Failed to capture frame")
                continue
            frame_count += 1
            detection = yolo_detector.detect(frame, save_image=True)
            display_frame = frame
            cv2.line(display_frame, (IMAGE_CENTER_X - 20, IMAGE_CENTER_Y),
                    (IMAGE_CENTER_X + 20, IMAGE_CENTER_Y), (0, 0, 255), 2)
            cv2.line(display_frame, (IMAGE_CENTER_X, IMAGE_CENTER_Y - 20),
                    (IMAGE_CENTER_X, IMAGE_CENTER_Y + 20), (0, 0, 255), 2)
            cv2.circle(display_frame, (IMAGE_CENTER_X, IMAGE_CENTER_Y),
                      CENTERING_TOLERANCE_PX, (0, 255, 255), 1)
            if detection:
                detection_count += 1
                bbox = detection['bbox']
                center = detection['center']
                confidence = detection['confidence']
        
                cv2.rectangle(display_frame, (bbox[0], bbox[1]),
                            (bbox[2], bbox[3]), (0, 255, 0), 2)
         
                cv2.circle(display_frame, center, 5, (255, 0, 0), -1)
                
                error_x, error_y = yolo_detector.calculate_centering_error(detection)
                is_centered = yolo_detector.is_centered(detection, CENTERING_TOLERANCE_PX)
                
                cv2.arrowedLine(display_frame, (IMAGE_CENTER_X, IMAGE_CENTER_Y),
                              center, (255, 0, 255), 2)
                
                info_text = [
                    f"Confidence: {confidence:.2f}",
                    f"Error: ({error_x:+.0f}, {error_y:+.0f})px",
                    f"Centered: {'YES' if is_centered else 'NO'}"
                ]
                
                y_offset = 30
                for text in info_text:
                    cv2.putText(display_frame, text, (10, y_offset),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                              (0, 255, 0) if is_centered else (0, 165, 255), 2)
                    y_offset += 25
                
                if frame_count % 10 == 0: 
                    print(f"Detection #{detection_count}: Error=({error_x:+3.0f}, {error_y:+3.0f})px, "
                          f"Centered={is_centered}, Conf={confidence:.2f}")

            cv2.putText(display_frame, f"Frame: {frame_count}", (10, display_frame.shape[0] - 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_frame, f"Detections: {detection_count}", (10, display_frame.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                       
            #cv2.imshow("YOLO Detection Test", display_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            else: #elif key == ord('s'):
                timestamp = int(time.time() * 1000)
                filename = f"{DETECTION_SAVE_PATH}/test_frame_{timestamp}.jpg"
                cv2.imwrite(filename, display_frame)
                print(f"Saved frame to {filename}")
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    finally:
        image_handler.close()  # Explicitly close camera
        cv2.destroyAllWindows()
        print(f"\nTest Summary:")
        print(f"- Total frames: {frame_count}")
        print(f"- Total detections: {detection_count}")
        print(f"- Detection rate: {100*detection_count/max(frame_count,1):.1f}%")
        rclpy.shutdown()


def main():
    test_detection()


if __name__ == "__main__":
    main()
