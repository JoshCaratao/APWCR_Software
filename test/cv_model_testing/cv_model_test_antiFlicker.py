from pathlib import Path
import time
import yaml

print("Importing ultralytics YOLO...")
from ultralytics import YOLO  # noqa: F401

print("Importing OpenCV...")
import cv2

print("Importing Camera Class...")
from camera import Camera

print("Importing Detector Class...")
from detector import Detector


print("Importing yaml file...")
PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = PROJECT_ROOT / "robot" / "config" / "robot_test.yaml"

with open(CONFIG_PATH, "r") as f:
    CONFIG = yaml.safe_load(f)


def main():
    detector_cfg = CONFIG["detector"]
    camera_cfg = CONFIG["camera"]

    MODEL_PATH = PROJECT_ROOT / detector_cfg["model_path"]
    IMG_SIZE = int(detector_cfg.get("img_size", 640))
    CONF_THRES = float(detector_cfg.get("confidence_threshold", 0.45))

    INFER_HZ = float(detector_cfg.get("inference_hz", 10))
    MIN_STREAK = int(detector_cfg.get("min_consecutive_detections", 2))
    HOLD_S = float(detector_cfg.get("hold_seconds", 0.4))

    CAM_INDEX = camera_cfg["index"]
    CAM_WIDTH = camera_cfg["width"]
    CAM_HEIGHT = camera_cfg["height"]

    assert MODEL_PATH.exists(), f"Model not found: {MODEL_PATH}"

    print("Initializing Camera Object...")
    camera = Camera(index=CAM_INDEX, width=CAM_WIDTH, height=CAM_HEIGHT)
    if not camera.open():
        return

    print("Initializing Detector Object...")
    detector = Detector(
        model_path=MODEL_PATH,
        imgsz=IMG_SIZE,
        conf=CONF_THRES,
    )

    window_name = "Pet Waste Detection - Live"

    infer_period_s = 1.0 / max(INFER_HZ, 0.1)
    last_infer_t = 0.0

    last_display = None

    # Anti-flicker state
    streak = 0
    stable_detected = False
    last_stable_t = 0.0

    # Optional: keep last stable center for downstream control/debug
    stable_center = None  # (cx, cy, conf)

    print("Beginning Main Loop...")
    try:
        while True:
            ret, frame = camera.read()
            if not ret:
                print("Failed to grab frame.")
                break

            now = time.perf_counter()

            # Only run inference at INFER_HZ
            if (now - last_infer_t) >= infer_period_s:
                r0, annotated, best = detector.detect(frame)
                last_display = annotated
                last_infer_t = now

                detected_now = detector.has_valid_detection(r0)

                if detected_now:
                    streak += 1
                else:
                    streak = 0

                if streak >= MIN_STREAK:
                    stable_detected = True
                    last_stable_t = now
                    stable_center = best
                else:
                    if stable_detected and HOLD_S > 0.0:
                        if (now - last_stable_t) > HOLD_S:
                            stable_detected = False
                            stable_center = None
                    else:
                        stable_detected = False
                        stable_center = None

            display_frame = last_display if last_display is not None else frame

            status = "STABLE DETECTION" if stable_detected else "searching"
            cv2.putText(
                display_frame,
                f"{status} | streak={streak}/{MIN_STREAK} | imgsz={IMG_SIZE} | hz={INFER_HZ}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0) if stable_detected else (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

            # Show center coords when stable (handy for control debugging)
            if stable_detected and stable_center is not None:
                cx, cy, conf = stable_center
                cv2.putText(
                    display_frame,
                    f"center=({cx},{cy})",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

            cv2.imshow(window_name, display_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
