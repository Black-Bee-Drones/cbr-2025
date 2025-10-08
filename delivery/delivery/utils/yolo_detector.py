import cv2
import numpy as np
import os
from typing import List, Dict, Tuple, Optional
import time

from ultralytics import YOLO

from delivery.constants import (
    YOLO_MODEL_PATH_CROSS,
    YOLO_MODEL_PATH_PKG,
    YOLO_CONFIDENCE_THRESHOLD,
    DETECTION_SAVE_PATH,
)

IMAGE_CENTER_X = 1232 / 2
IMAGE_CENTER_Y = 1640 / 2


class YoloDetector:
    def __init__(self,
            model_cross_path: str = YOLO_MODEL_PATH_CROSS,
            model_package_path: str = YOLO_MODEL_PATH_PKG,
            conf_cross: float = YOLO_CONFIDENCE_THRESHOLD,
            conf_package: float = YOLO_CONFIDENCE_THRESHOLD,
            save_image_path: str = DETECTION_SAVE_PATH,
        ):
        self.model_cross = YOLO(model_cross_path)
        self.model_package = YOLO(model_package_path)

        self._conf_cross = conf_cross
        self._conf_package = conf_package

        self._save_image_path = save_image_path
        self.detection_count = 0

    def detect(self,
            frame: np.ndarray,
            desired_class: Tuple[str] = [],
            save_image: bool = True,
        ):
        """
        desired_class: ["cross", "base", "package"]; if []: all
        """
        detections = {}
        if (not desired_class) or ("cross" in desired_class):
            results = self.model_cross(
                frame,
                conf=self._conf_cross,
            )
            all_detections = self.results_to_list_of_dict(results)
            cross = self.get_best_detection(all_detections)
            if cross:
                detections["cross"] = cross

        if (not desired_class) or ("base" in desired_class) or ("package" in desired_class):
            results = self.model_package(
                frame,
                conf=self._conf_package,
            )
            all_detections = self.results_to_list_of_dict(results)
            base_detections, package_detections = self.organize_detections(all_detections)

            base = self.get_best_detection(base_detections)
            if base:
                detections["base"] = base

            package = self.get_best_detection(package_detections, inside_bbox=base["bbox"] if base else None)
            if package:
                detections["package"] = package
        
        if save_image:
            self._save_detection_image(
                frame.copy(), detections, time.time_ns()
            )

        return detections

    def results_to_list_of_dict(self, results):
        detections = []
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = box.conf[0].cpu().numpy()
                    class_id = int(box.cls[0].cpu().numpy())

                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    area = (x2 - x1) * (y2 - y1)

                    detection = {
                        "bbox": [int(x1), int(y1), int(x2), int(y2)],
                        "center": (center_x, center_y),
                        "confidence": float(confidence),
                        "class_id": class_id,
                        "score": float(area * confidence),
                    }
                    detections.append(detection)
        return detections

    def get_best_detection(self, detections, inside_bbox=None):
        """Retorna o dict com o maior valor de 'score'."""
        if not detections:
            return None

        if inside_bbox:
            x_min, y_min, x_max, y_max = inside_bbox

            # filtra os detections que estão dentro da bbox
            filtered = [
                d for d in detections 
                if (x_min <= d["center"][0] <= x_max) and (y_min <= d["center"][1] <= y_max)
            ]

            if filtered:
                d = max(filtered, key=lambda d: d["score"])
                d["inside"] = True
                return d

        d = max(detections, key=lambda d: d["score"])
        if inside_bbox:
            d["inside"] = False
        return d

    def organize_detections(self, detections):
        base_detections = []
        package_detections = []

        for det in detections:
            class_id = det["class_id"]
            if class_id == 0:
                base_detections.append(det)
            elif class_id == 1:
                package_detections.append(det)

        return base_detections, package_detections


    def save_image(self, frame):
        """Salva a imagem atual com timestamp no nome."""
        os.makedirs(self._save_image_path, exist_ok=True)
        cv2.imwrite(f"{self._save_image_path}/frame_{time.time_ns()}.jpg", frame)

    def _save_detection_image(
        self, image: np.ndarray, detections: Dict[str, Dict], timestamp: int
    ):
        """Save image with detection bounding boxes."""
        annotated_image = image.copy()
        
        # detections é um dict com keys como "cross", "base", "package"
        for detection_type, detection in detections.items():
            if detection and isinstance(detection, dict):
                bbox = detection.get("bbox", [])
                confidence = detection.get("confidence", 0.0)
                center = detection.get("center", (0, 0))

                if bbox and len(bbox) == 4:
                    cv2.rectangle(
                        annotated_image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2
                    )

                    cv2.circle(annotated_image, center, 5, (0, 0, 255), -1)

                    text = f"{detection_type}: {confidence:.2f}"
                    cv2.putText(
                        annotated_image,
                        text,
                        (bbox[0], bbox[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                    )

        # Desenhar crosshair no centro da imagem
        center_x = int(IMAGE_CENTER_X)
        center_y = int(IMAGE_CENTER_Y)
        cv2.line(
            annotated_image,
            (center_x - 10, center_y),
            (center_x + 10, center_y),
            (255, 0, 0),
            2,
        )
        cv2.line(
            annotated_image,
            (center_x, center_y - 10),
            (center_x, center_y + 10),
            (255, 0, 0),
            2,
        )

        # Criar diretório se não existir
        os.makedirs(self._save_image_path, exist_ok=True)
        
        # Salvar imagem com timestamp
        filename = f"{self._save_image_path}/frame_{int(timestamp)}.jpg"
        cv2.imwrite(filename, annotated_image)
        print(f"Detection image saved: {filename}")

