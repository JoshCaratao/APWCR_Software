import time
import threading
import math

# ------------------------------------------------------
# Import Configuration Loader, Utils, and Robot Packages
# ------------------------------------------------------

print("importing Robot Packages ... ")
from pwc_robot.config_loader import load_config, resolve_paths, require_keys
from pwc_robot.utils.rate import Rate
from pwc_robot.perception.camera import Camera
from pwc_robot.perception.detector import Detector
from pwc_robot.perception.computer_vision import ComputerVision
from pwc_robot.gui.gui_server import run_flask
from pwc_robot.controller.controller import Controller
from pwc_robot.comms.serial_link import SerialLink
from pwc_robot.perception.ground_plane import GroundPlaneCalib


def main(config_name: str = "robot_default.yaml") -> None:

    # --------------------------------
    # Load and Validate Configurations
    # --------------------------------
    print("Loading Configurations ... ")
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
        "ground_plane": [
            "enabled",
            "fx",
            "fy",
            "cx",
            "cy",
            "cam_height_ft",
            "pitch_deg",
            "min_v_px",
            "max_range_ft",
        ],
        "gui": [
            "enabled",
            "host",
            "port",
            "stream_hz",
            "quiet",
            "manual_speed_linear",
            "manual_speed_angular"
        ],
        "controller": {
            "deadman_s": [],
            "default_speed_linear": [],
            "default_speed_angular": [],
            "max_speed_linear": [],
            "max_speed_angular": [],
            "min_speed_linear": [],
            "min_speed_angular": [],
            "target_hold_s": [],
            "control_hz": [],
            "approach": [
                "kp_ang",
                "deadzone_x",
                "x_shift",

                "use_ground_plane_range",
                "desired_range_ft",
                "kp_lin_ft",
                "deadzone_range_ft",

                "kp_lin_pixel",
                "deadzone_y",
                "y_shift",
            ],
        },
        "comms": [
            "comms_enabled",
            "comms_hz",
            "port",
            "auto_detect",
            "baud",
            "timeout_s",
            "write_timeout_s",
            "rx_stale_s",
            "reconnect_s"
        ]
    })
    
    print("Loading Camera ... ")
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

    print("Loading Detector ... ")
    # --- Detector config ---
    det_cfg = cfg["detector"]

    model_path = det_cfg["model_path"]  # resolve_paths converts to absolute string/path
    img_size = int(det_cfg["img_size"])
    conf_thres = float(det_cfg["confidence_threshold"])
    
    # Create Detector Object using Configs
    detector = Detector(model_path=model_path, 
        imgsz=img_size, 
        conf_thresh=conf_thres)
    
    # --- Ground-plane config ---
    gp_cfg = cfg.get("ground_plane", {})
    gp_enabled = bool(gp_cfg.get("enabled", False))

    gp_calib = None
    gp_min_v_px = 0
    gp_max_range_ft = 0.0

    if gp_enabled:
        gp_calib = GroundPlaneCalib(
            fx=float(gp_cfg["fx"]),
            fy=float(gp_cfg["fy"]),
            cx=float(gp_cfg["cx"]),
            cy=float(gp_cfg["cy"]),
            cam_height_ft=float(gp_cfg["cam_height_ft"]),
            pitch_rad=math.radians(float(gp_cfg["pitch_deg"])),
        )
        gp_min_v_px = int(gp_cfg.get("min_v_px", 0))

        # max_range_ft can be null in YAML -> becomes None in Python
        gp_max_range_ft_val = gp_cfg.get("max_range_ft", None)
        gp_max_range_ft = None if gp_max_range_ft_val is None else float(gp_max_range_ft_val)



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
        target_infer_hz=target_infer_hz,
        targeting_mode=targeting_mode,
        targeting_conf_w=targeting_conf_w,
        targeting_area_w=targeting_area_w,
        stable_window=stable_window,

        # Ground Plane params
        ground_plane_enabled=gp_enabled,
        calib=gp_calib,
        ground_plane_min_v_px=gp_min_v_px,
        ground_plane_max_range_ft=gp_max_range_ft,
    )

    # Stop if camera fails to open
    if not cv.start():
        raise SystemExit("Camera failed to open.")
    
    # ---- Controller Config ----
    print("Loading Controller ... ")
    ctrl_cfg = cfg["controller"]
    approach_cfg = ctrl_cfg["approach"]
    control_hz = float(ctrl_cfg["control_hz"])
    # ---- Instantiate Controller Object ----
    controller = Controller(
        deadman_s=ctrl_cfg["deadman_s"],
        default_speed_linear=ctrl_cfg["default_speed_linear"],
        default_speed_angular=ctrl_cfg["default_speed_angular"],
        max_speed_linear=ctrl_cfg["max_speed_linear"],
        max_speed_angular=ctrl_cfg["max_speed_angular"],
        min_speed_linear=ctrl_cfg["min_speed_linear"],
        min_speed_angular=ctrl_cfg["min_speed_angular"],
        target_hold_s=ctrl_cfg["target_hold_s"],

        # centering (pixel-x)
        kp_ang=approach_cfg["kp_ang"],
        deadzone_x=approach_cfg["deadzone_x"],
        x_shift=approach_cfg["x_shift"],

        # range (ground-plane)
        use_ground_plane_range=approach_cfg["use_ground_plane_range"],
        desired_range_ft=approach_cfg["desired_range_ft"],
        kp_lin_ft=approach_cfg["kp_lin_ft"],
        deadzone_range_ft=approach_cfg["deadzone_range_ft"],

        # fallback (pixel-y)
        kp_lin_pixel=approach_cfg["kp_lin_pixel"],
        deadzone_y=approach_cfg["deadzone_y"],
        y_shift=approach_cfg["y_shift"],
    )


    # --- Comms Config ---
    comms = None
    comms_cfg = cfg["comms"]
    comms_enabled = bool(comms_cfg["comms_enabled"])
    if comms_enabled:
        print("Establishing Arduino Comms ...")
        comms = SerialLink(comms_cfg)
        comms_hz = float(comms_cfg["comms_hz"])
    else:
        print("Comms Link Bypassed ...")
    
    
    # --- GUI Thread (Flask Streaming Server) ---
    print("Loading User Interface ... ")
    # Establish GUI configs
    gui_cfg = cfg["gui"]

    gui_enabled = bool(gui_cfg["enabled"])
    gui_host = str(gui_cfg["host"])
    gui_port = int(gui_cfg["port"])
    gui_stream_hz = float(gui_cfg["stream_hz"])
    quiet = bool(gui_cfg["quiet"])
    manual_speed_linear = float(gui_cfg["manual_speed_linear"])
    manual_speed_angular = float(gui_cfg["manual_speed_angular"])

    # If gui_enabled config is true, create flask thread 
    if gui_enabled:
        gui_thread = threading.Thread(
            target=run_flask,
            kwargs={
                "cv": cv, 
                "controller": controller, 
                "serial_link": comms,
                "host": gui_host,
                "port": gui_port,
                "stream_hz": gui_stream_hz,
                "quiet": quiet,
                "manual_speed_linear": manual_speed_linear,
                "manual_speed_angular": manual_speed_angular,
            },
            daemon=True,  # dies when main exits
            name="flask-gui",
        )
        # Start flask thread
        gui_thread.start()
        # pretty_host = "localhost" if gui_host == "0.0.0.0" else gui_host
        # print(f"[GUI] running on http://{pretty_host}:{gui_port} (stream_hz={gui_stream_hz})")

    else:
        print("[GUI] disabled in config")
        

    # --- Establish Scheduling Rates ---
    vision_rate = Rate(hz=target_infer_hz)  # Computer-Vision Model Detection Rate
    controller_rate = Rate(hz = control_hz) # Controller Rate
    debug_comment_rate = Rate(hz = 1.0)
    if comms_enabled:
        comms_rate = Rate(hz=comms_hz)          # Arduino Comms Rate


    print(
        f"[main] vision_hz={target_infer_hz} | imgsz={img_size} | conf={conf_thres} | "
        f"stable_window={stable_window} | cam=({cam_index}, {cam_width}x{cam_height})"
    )

    # -------------
    # RUN MAIN LOOP
    # -------------
    lock = threading.Lock()
    try:
        #Instantiate vision_obs object for control use
        last_vision_obs = {}
        drive_cmd = None
        mech_cmd = None

        print("ENTERING MAIN LOOP ...")

        while True:
            # Instantiate timer for rate use
            t0 = time.perf_counter()

            # Computer Vision Tick
            if vision_rate.ready(t0):
                vision_obs = cv.tick()
                if vision_obs is not None:
                    last_vision_obs = vision_obs

            # Controller Tick
            t1 = time.perf_counter()
            if controller_rate.ready(t1):
                drive_cmd, mech_cmd = controller.tick(last_vision_obs)

            # Serial Comms Tick
            t2 = time.perf_counter()
            if comms_enabled and comms_rate.ready(t2):
                #t_before = time.perf_counter()
                comms.tick(drive_cmd, mech_cmd)
                #dt = time.perf_counter() - t_before
                #if dt > 0.01:
                    #print(f"[comms] tick blocked {dt:.3f}s")

                

            if cv.should_quit():
                break
            

            # if debug_comment_rate.ready(t1) and drive_cmd is not None and mech_cmd is not None:
            #     with lock:
            #         comm_status = comms.get_status()
            #         print("Port:", comm_status["port"], "State:", comm_status["state"])
            #         print("Bytes Rx:", comm_status.get("bytes_rx"), "Bytes Tx:", comm_status.get("bytes_tx"))
            #         print("Last Comm Error:", comm_status.get("last_error"))

            #         print(controller.state)
            #         print(drive_cmd)
            #         print(mech_cmd)
            
            # Sleep for 1ms
            time.sleep(0.001)

    finally:
        if comms_enabled:
            comms.close()
        cv.stop()


if __name__ == "__main__":
    main()
