from __future__ import annotations

from typing import Optional, Tuple

import cv2
from ultralytics import YOLO


class Detector:
    """
    Wraps a YOLO model for detection + simple visualization helpers.
    This class does not manage timing or inference rate.
    """

    def __init__(self, model_path, imgsz: int = 640, conf: float = 0.25):
        self.model = YOLO(str(model_path))
        self.imgsz = int(imgsz)
        self.conf = float(conf)

    def detect(self, frame):
        """
        Run detection on a single frame.

        Returns:
            r0: Ultralytics Results object (results[0])
            annotated: image with YOLO boxes and (optionally) a crosshair on best box
            best: (cx, cy, conf) for best detection or None
        """
        results = self.model(
            frame,
            imgsz=self.imgsz,
            conf=self.conf,
            verbose=False,
        )

        r0 = results[0]
        annotated = r0.plot()

        best = self.get_best_detection_center(r0)
        if best is not None:
            cx, cy, _ = best
            self.draw_crosshair(annotated, cx, cy)

        return r0, annotated, best

    def has_valid_detection(self, r0) -> bool:
        """
        True if any box exists (YOLO already applied self.conf threshold).
        """
        try:
            return (r0 is not None) and (r0.boxes is not None) and (len(r0.boxes) > 0)
        except Exception:
            return False

    def get_best_detection_center(self, r0) -> Optional[Tuple[int, int, float]]:
        """
        Returns (cx, cy, conf) of the highest-confidence detection in r0, or None.
        """
        try:
            if r0 is None or r0.boxes is None or len(r0.boxes) == 0:
                return None

            confs = r0.boxes.conf  # tensor [N]
            boxes = r0.boxes.xyxy  # tensor [N,4]

            if confs is None or boxes is None or len(confs) == 0:
                return None

            best_i = int(confs.argmax().item())
            best_conf = float(confs[best_i].item())

            x1, y1, x2, y2 = boxes[best_i]
            x1 = float(x1.item()); y1 = float(y1.item())
            x2 = float(x2.item()); y2 = float(y2.item())

            cx = int(round((x1 + x2) / 2.0))
            cy = int(round((y1 + y2) / 2.0))
            return cx, cy, best_conf
        except Exception:
            return None

    @staticmethod
    def draw_crosshair(img, cx: int, cy: int, size: int = 12, thickness: int = 2):
        """
        Draws a crosshair centered at (cx, cy) on the image.
        """
        h, w = img.shape[:2]
        cx = max(0, min(w - 1, int(cx)))
        cy = max(0, min(h - 1, int(cy)))

        cv2.line(img, (cx - size, cy), (cx + size, cy), (255, 255, 255), thickness, cv2.LINE_AA)
        cv2.line(img, (cx, cy - size), (cx, cy + size), (255, 255, 255), thickness, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 3, (0, 0, 0), -1)
