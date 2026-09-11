#!/usr/bin/env python3
"""
Human & Dummy Detector using YOLOv8 / YOLOv11
Fine-tune later on real rescue dummies for better accuracy.
"""

from ultralytics import YOLO
import numpy as np
from typing import List, Dict, Tuple
import cv2


class HumanDummyDetector:
    def __init__(self, model_path: str = "yolov8n.pt", conf_thres: float = 0.45, device: str = "cpu"):
        """
        model_path: path to .pt weights. Use yolov8n.pt or a fine-tuned model.
        Classes of interest: person (COCO 0). For dummies, fine-tune or treat as person.
        """
        self.model = YOLO(model_path)
        self.conf_thres = conf_thres
        self.device = device
        # COCO person class = 0. You can expand after fine-tuning.
        self.target_classes = {0}  # person

    def detect(self, frame: np.ndarray) -> List[Dict]:
        """
        Returns list of detections:
        [{"bbox": [x1,y1,x2,y2], "conf": float, "cls": int, "center": (cx, cy)}]
        """
        results = self.model.predict(
            frame,
            conf=self.conf_thres,
            classes=list(self.target_classes),
            verbose=False,
            device=self.device,
        )

        detections = []
        if not results:
            return detections

        r = results[0]
        if r.boxes is None:
            return detections

        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            detections.append({
                "bbox": [float(x1), float(y1), float(x2), float(y2)],
                "conf": conf,
                "cls": cls,
                "center": (cx, cy),
            })
        return detections

    def draw(self, frame: np.ndarray, detections: List[Dict]) -> np.ndarray:
        out = frame.copy()
        for d in detections:
            x1, y1, x2, y2 = map(int, d["bbox"])
            conf = d["conf"]
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"Person {conf:.2f}"
            cv2.putText(out, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return out
