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

    def detect(self,
            frame: np.ndarray,
            desired_class: Tuple[str] = [],
            save_image: bool = False,
        ):
        """
        desired_class: ["cross", "base", "package"]; if []: all
        """
        detections = {}
        if (not desired_class) or ("cross" in desired_class):
            results = self.model_cross(
                frame.copy(),
                conf=self._conf_cross,
            )
            all_detections = self.results_to_list_of_dict(results)
            cross = self.get_best_detection(all_detections)
            if cross:
                detections["cross"] = cross

        if (not desired_class) or ("base" in desired_class) or ("package" in desired_class):
            results = self.model_package(
                frame.copy(),
                conf=self._conf_package,
            )
            all_detections = self.results_to_list_of_dict(results)
            base_detections, package_detections = self.organize_detections(all_detections)

            base = self.get_best_detection(base_detections)
            if base:
                detections["base"] = base

            package = self.get_best_detection(package_detections, inside_bbox=base["bbox"])
            if base:
                detections["package"] = package
        
        if save_image:
            self.save_image(frame.copy())

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
        group = {}
        for detection in detections:
            _id = detection["id"]
            if _id not in group:
                group[_id] = []
            group[_id].append(detection)
        return list(group.values())

    def save_image(self, frame):
        """Salva a imagem atual com timestamp no nome."""
        os.makedirs(self._save_image_path, exist_ok=True)
        cv2.imwrite(f"{self._save_image_path}/frame_{time.time_ns()}.jpg", frame)
