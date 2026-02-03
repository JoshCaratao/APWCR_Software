import time
import threading

# ------------------------------------------------------
# Import Configuration Loader, Utils, and Robot Packages
# ------------------------------------------------------

from pwc_robot.config_loader import load_config, resolve_paths, require_keys
from pwc_robot.utils.rate import Rate
from pwc_robot.perception.camera import Camera
from pwc_robot.perception.detector import Detector
from pwc_robot.perception.computer_vision import ComputerVision
from pwc_robot.gui.gui_server import run_flask



def main(config_name: str = "robot_default.yaml") -> None:

    # --------------------------------
    # Load and Validate Configurations
    # --------------------------------
  
    cfg = resolve_paths(load_config(config_name))

    # Required Configs that must be in config file (YAML)
    require_keys(cfg, {
        "camera": [
            "index", 
            "width", 
            "height",
            "capture_hz", 
            "copy_on_get"
        ],
        "detector": [
            "model_path",
            "img_size",
            "confidence_threshold",
        ],
        "comp_vision":[
            "target_infer_hz",
            "show_window",
            "targeting_mode",
            "targeting_conf_weight",
            "targeting_area_weight",
            "stable_window"
        ],
        "gui": [
            "enabled",
            "host",
            "port",
            "stream_hz"
        ]
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

    cam_capture_hz = float(cam_cfg["capture_hz"]) if cam_cfg["capture_hz"] is not None else None
    cam_copy_on_get = bool(cam_cfg["copy_on_get"])


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
    
    # Create Detector Object using Configs
    detector = Detector(model_path=model_path, 
        imgsz=img_size, 
        conf_thresh=conf_thres)


    # --- computer_vision config ---
    comp_vision_config = cfg["comp_vision"]

    target_infer_hz = float(comp_vision_config["target_infer_hz"])
    show_window = bool(comp_vision_config["show_window"])
    targeting_mode = str(comp_vision_config["targeting_mode"])
    targeting_conf_w = float(comp_vision_config["targeting_conf_weight"])
    targeting_area_w = float(comp_vision_config["targeting_area_weight"])
    stable_window = int(comp_vision_config["stable_window"])

    # --- Instantiate ComputerVision Object ---
    cv = ComputerVision(
        camera=camera,
        detector=detector,
        window_name="Pet Waste Detection - Live",
        show_window=show_window,
        target_infer_hz = target_infer_hz,
        targeting_mode = targeting_mode,
        targeting_conf_w = targeting_conf_w,
        targeting_area_w = targeting_area_w,
        stable_window = stable_window
    )

    # Stop if camera fails to open
    if not cv.start():
        raise SystemExit("Camera failed to open.")
    

    # --- GUI Thread (Flask Streaming Server) ---

    # Establish GUI configs
    gui_cfg = cfg["gui"]

    gui_enabled = bool(gui_cfg["enabled"])
    gui_host = str(gui_cfg["host"])
    gui_port = int(gui_cfg["port"])
    gui_stream_hz = float(gui_cfg["stream_hz"])

    # If gui_enabled config is true, create flask thread 
    if gui_enabled:
        gui_thread = threading.Thread(
            target=run_flask,
            kwargs={
                "cv": cv,  
                "host": gui_host,
                "port": gui_port,
                "stream_hz": gui_stream_hz,
            },
            daemon=True,  # dies when main exits
            name="flask-gui",
        )
        # Start flask thread
        gui_thread.start()
        pretty_host = "localhost" if gui_host == "0.0.0.0" else gui_host
        print(f"[GUI] running on http://{pretty_host}:{gui_port} (stream_hz={gui_stream_hz})")

    else:
        print("[GUI] disabled in config")


    # --- Establish Scheduling Rates ---
    vision_rate = Rate(hz=target_infer_hz) # Computer-Vision Model Detection Rate

    print(
        f"[main] vision_hz={target_infer_hz} | imgsz={img_size} | conf={conf_thres} | "
        f"stable_window={stable_window} | cam=({cam_index}, {cam_width}x{cam_height})"
    )

    # -------------
    # RUN MAIN LOOP
    # -------------
    try:
        while True:
            # Instantiate timer for rate use
            now = time.perf_counter()

            #Instantiate vision_obs object for control use
            vision_obs = None


            if vision_rate.ready(now):
                vision_obs = cv.tick()
                

            if cv.should_quit():
                break
            
            # Sleep for 1ms
            time.sleep(0.001)

    finally:
        cv.stop()


if __name__ == "__main__":
    main()
