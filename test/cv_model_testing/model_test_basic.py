from pathlib import Path
import yaml

print("Importing yaml file...")
PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = PROJECT_ROOT / "robot" / "config" / "robot_test.yaml"

with open(CONFIG_PATH, "r") as f:
    CONFIG = yaml.safe_load(f)

print("Importing ultralytics YOLO...")
from ultralytics import YOLO  # noqa: F401

print("Importing Open-CV...")
import cv2

print("Importing Camera Class...")
from camera import Camera

print("Importing Basic Detector Class...")
from detectorBasic import DetectorBasic


print("Entering Main Function...")


def main():
    # --- Instantiate Variables based on config file ---
    detector_cfg = CONFIG["detector"]
    camera_cfg = CONFIG["camera"]

    MODEL_PATH = PROJECT_ROOT / detector_cfg["model_path"]
    IMG_SIZE = detector_cfg["img_size"]
    CONF_THRES = detector_cfg["confidence_threshold"]

    CAM_INDEX = camera_cfg["index"]
    CAM_WIDTH = camera_cfg["width"]
    CAM_HEIGHT = camera_cfg["height"]

    assert MODEL_PATH.exists(), f"Model not found: {MODEL_PATH}"
    # -----------------------------------------------

    print("Initializing Camera Object...")
    camera = Camera(index=CAM_INDEX, width=CAM_WIDTH, height=CAM_HEIGHT)
    if not camera.open():
        return

    print("Initializing Detector Object...")
    detector = DetectorBasic(
        model_path=MODEL_PATH,
        imgsz=IMG_SIZE,
        conf=CONF_THRES,
    )

    window_name = "Pet Waste Detection - Live"

    print("Beginning Main Loop...")
    try:
        while True:
            ret, frame = camera.read()
            if not ret:
                print("Failed to grab frame.")
                break

            results, annotated_frame = detector.detect(frame)

            cv2.imshow(window_name, annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
