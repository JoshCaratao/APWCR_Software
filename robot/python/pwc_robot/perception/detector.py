import cv2
from ultralytics import YOLO


class Detector:
    """
    Wraps a YOLO model for detection + simple visualization helpers.
    This class does not manage timing or inference rate.

    Responsibilities:
      - Run YOLO on a frame
      - Return candidates (centers, conf, area, boxes)
      - Return num_detections
      - Provide drawing helpers (crosshair)

    """

    def __init__(self, model_path, imgsz: int = 640, conf_thresh: float = 0.25):
        self.model = YOLO(str(model_path))
        self.imgsz = int(imgsz)
        self.conf_thresh = float(conf_thresh)

    def detect(self, frame):
        """
        Run detection on a single frame.

        Returns:
            r0: Ultralytics Results object (results[0])
            annotated: numpy image with YOLO boxes/labels drawn (via r0.plot())
            candidates: list of dicts, each like:
                {
                  "cx": int,
                  "cy": int,
                  "conf": float,
                  "area": float,
                  "xyxy": (x1, y1, x2, y2)
                }
            num_detections: int
        """
        results = self.model(
            frame,
            imgsz=self.imgsz,
            conf=self.conf_thresh,
            verbose=False,
        )

        # Get ultralytics result object and annotate frame based on results
        r0 = results[0]
        annotated_frame = r0.plot()

        candidates = self.get_candidates(r0)
        num_detections = len(candidates)

        return r0, annotated_frame, candidates, num_detections

    def has_valid_detection(self, r0):
        """
        True if any box exists (YOLO already applied self.conf_thresh).
        """
        try:
            return (r0 is not None) and (r0.boxes is not None) and (len(r0.boxes) > 0)
        except Exception:
            return False

    def get_candidates(self, r0):
        """
        Extract candidate detections from a YOLO Results object.

        Returns:
            list[dict] with keys: cx, cy, conf, area, xyxy
        """
        try:
            if r0 is None or r0.boxes is None or len(r0.boxes) == 0:
                return []

            confs = r0.boxes.conf  # tensor [N]
            boxes = r0.boxes.xyxy  # tensor [N,4]

            if confs is None or boxes is None or len(confs) == 0:
                return []
            
            candidates = []
            for i in range(len(confs)):
                conf = float(confs[i].item())

                x1, y1, x2, y2 = boxes[i]
                x1 = float(x1.item())
                y1 = float(y1.item())
                x2 = float(x2.item())
                y2 = float(y2.item())

                # Calculate Centers based on box coordinates
                cx = int(round((x1 + x2) / 2.0))
                cy = int(round((y1 + y2) / 2.0))
                
                # Calculate width and height of box for area calc
                w = max(0.0, x2 - x1)
                h = max(0.0, y2 - y1)
                area = w*h
                
                # Append to candidates list
                candidates.append(
                    {
                        "cx": cx,
                        "cy": cy,
                        "conf": conf,
                        "area": area,
                        "xyxy": (x1, y1, x2, y2),
                    }
                )
            
            return candidates
 
        except Exception:
            return []

           
    # Draw crosshair on best detection for better center visualization
    @staticmethod
    def draw_crosshair(img, cx: int, cy: int, size: int = 12, thickness: int = 2):
        h, w = img.shape[:2]
        cx = max(0, min(w - 1, int(cx)))
        cy = max(0, min(h - 1, int(cy)))

        # outline then main line for contrast
        outline = thickness + 2

        cv2.line(img, (cx - size, cy), (cx + size, cy), (0, 0, 0), outline, cv2.LINE_AA)
        cv2.line(img, (cx, cy - size), (cx, cy + size), (0, 0, 0), outline, cv2.LINE_AA)

        cv2.line(img, (cx - size, cy), (cx + size, cy), (255, 255, 255), thickness, cv2.LINE_AA)
        cv2.line(img, (cx, cy - size), (cx, cy + size), (255, 255, 255), thickness, cv2.LINE_AA)

        cv2.circle(img, (cx, cy), 3, (0, 0, 0), -1, cv2.LINE_AA)

