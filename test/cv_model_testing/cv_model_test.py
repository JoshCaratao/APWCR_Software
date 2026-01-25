
from pathlib import Path
import yaml
import time


print('Importing yaml file....')
PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = PROJECT_ROOT / "robot" / "config" / "robot_test.yaml"

with open(CONFIG_PATH, "r") as f:
    CONFIG = yaml.safe_load(f)


print('Importing ultralytics YOLO...')
from ultralytics import YOLO

print('Importing Open-CV...')
import cv2

print('Importing Camera Class...')
from camera import Camera

print('Importing Detector Class...')
from detector import Detector


print('Entering Main Function...')
def main():
    # --- Instantiate Variables based on config file ---
    detector_cfg = CONFIG["detector"]
    camera_cfg = CONFIG["camera"]

    MODEL_PATH = PROJECT_ROOT / detector_cfg["model_path"]
    IMG_SIZE = detector_cfg["img_size"]
    CONF_THRES = detector_cfg["confidence_threshold"]
    INFER_HZ = float(detector_cfg.get("inference_hz", 10))  # default 10 Hz if missing

    CAM_INDEX = camera_cfg["index"]
    CAM_WIDTH = camera_cfg["width"]
    CAM_HEIGHT = camera_cfg["height"]

    assert MODEL_PATH.exists(), f"Model not found: {MODEL_PATH}"
    # -------------------------------

    print('Initializing Camera Object...')
    # Initialize Camera Object
    camera = Camera(index=CAM_INDEX, width=CAM_WIDTH, height=CAM_HEIGHT)
    if not camera.open():
        return

    print('Initializing Detector Object...')
    # Initialize Detector Object
    detector = Detector(
        model_path=MODEL_PATH,
        imgsz=IMG_SIZE,
        conf=CONF_THRES,
    )

    window_name = "Pet Waste Detection - Live"

    infer_period_s = 1.0 / max(INFER_HZ, 0.1)  # avoid divide by zero
    last_infer_t = 0.0
    last_annotated = None

    print('Beginning Main Loop...')
    try:
        while True:
            ret, frame = camera.read()
            if not ret:
                print("Failed to grab frame.")
                break
            
            now = time.perf_counter()

            # Only run detection at the desired Hz
            if (now - last_infer_t) >= infer_period_s:
                results, annotated_frame = detector.detect(frame)
                last_annotated = annotated_frame
                last_infer_t = now
            
            # Display last inference result, or raw frame until first inference runs
            display_frame = last_annotated if last_annotated is not None else frame
            cv2.imshow(window_name, display_frame)


            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()