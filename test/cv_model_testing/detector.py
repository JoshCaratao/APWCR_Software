from ultralytics import YOLO

class Detector:
    def __init__(self, model_path: str, imgsz: int = 512, conf: float = 0.25):
        """
        Wraps a YOLO model for pet-waste detection.
        """
        self.model = YOLO(model_path)
        self.imgsz = imgsz
        self.conf = conf

    def detect(self, frame):
        """
        Run detection on a single frame.
        Returns (results, annotated_frame).
        """
        results = self.model(
            frame,
            imgsz=self.imgsz,
            conf=self.conf,
            verbose=False,
        )
        annotated = results[0].plot()
        return results[0], annotated