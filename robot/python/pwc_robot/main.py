# pwc_robot/main.py
import time

# ------------------------------------------------------
# Import Configuration Loader, Utils, and Robot Packages
# ------------------------------------------------------

from pwc_robot.config_loader import load_config, resolve_paths, require_keys
from pwc_robot.utils.rate import Rate
from pwc_robot.perception.camera import Camera
from pwc_robot.perception.detector import Detector
from pwc_robot.perception.computer_vision import ComputerVision


def main(config_name: str = "robot_default.yaml") -> None:

    # --------------------------------
    # Load and Validate Configurations
    # --------------------------------
  
    cfg = resolve_paths(load_config(config_name))

    require_keys(cfg, {
        "camera": ["index", "width", "height","capture_hz", "copy_on_get"],
        "detector": [
            "model_path",
            "img_size",
            "confidence_threshold",
            "inference_hz",
            "min_consecutive_detections",
            "hold_seconds",
        ],
    })

    # --- Camera config (width/height can be None) ---
    cam_cfg = cfg["camera"]
    cam_index = int(cam_cfg["index"])
    cam_width = cam_cfg["width"]
    cam_height = cam_cfg["height"]
    if cam_width is not None:
        cam_width = int(cam_width)
    if cam_height is not None:
        cam_height = int(cam_height)

    # Optional camera-thread settings
    cam_capture_hz = cam_cfg.get("capture_hz", None)
    if cam_capture_hz is not None:
        cam_capture_hz = float(cam_capture_hz)

    cam_copy_on_get = bool(cam_cfg.get("copy_on_get", True))

    # Create Camera Object using configs
    camera = Camera(
        index=cam_index, 
        width=cam_width, 
        height=cam_height, 
        capture_hz = cam_capture_hz, 
        copy_on_get = cam_copy_on_get
    )

    # --- Detector config ---
    det_cfg = cfg["detector"]

    model_path = det_cfg["model_path"]  # resolve_paths converts to absolute string/path
    img_size = int(det_cfg["img_size"])
    conf_thres = float(det_cfg["confidence_threshold"])
    infer_hz = float(det_cfg["inference_hz"])
    min_streak = int(det_cfg["min_consecutive_detections"])
    hold_s = float(det_cfg["hold_seconds"])
    show_window = det_cfg["show_window"]

    # Create Detector Object using Configs
    detector = Detector(model_path=model_path, imgsz=img_size, conf=conf_thres)

    # --- Instantiate ComputerVision Object ---
    cv = ComputerVision(
        camera=camera,
        detector=detector,
        min_consecutive_detections=min_streak,
        hold_seconds=hold_s,
        window_name="Pet Waste Detection - Live",
        show_window=show_window,
    )

    if not cv.start():
        raise SystemExit("Camera failed to open.")

    # --- Establish Scheduling Rates ---
    vision_rate = Rate(hz=infer_hz) # Computer-Vision Model Detection Rate

    print(
        f"[main] vision_hz={infer_hz} | imgsz={img_size} | conf={conf_thres} | "
        f"min_streak={min_streak} | hold={hold_s}s | cam=({cam_index}, {cam_width}x{cam_height})"
    )

    # -------------
    # RUN MAIN LOOP
    # -------------
    try:
        while True:
            now = time.perf_counter()

            if vision_rate.ready(now):
                obs = cv.tick()
                if obs is None:
                    continue

                # For later: robot logic can use these
                # stable = obs["stable_detected"]
                # center = obs["stable_center"]

            if cv.should_quit():
                break

    finally:
        cv.stop()


if __name__ == "__main__":
    main()
