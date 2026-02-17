from __future__ import annotations

import time
from typing import Any, Dict, Optional

import cv2
from flask import Flask, Response, jsonify, render_template, stream_with_context, request
import logging
import socket


def create_app(
    cv,
    controller,
    serial_link,  # <-- NEW: pass SerialLink into the GUI
    manual_speed_linear,
    manual_speed_angular,
    lid_deg_closed: float,
    lid_deg_opened: float,
    sweeper_deg_extend: float,
    sweeper_deg_closed: float,
    stream_hz: float,
) -> Flask:
    """
    Create the Flask app for the robot GUI and pass in:
      - cv: ComputerVision
      - controller: Controller
      - serial_link: SerialLink (telemetry + link stats)

    stream_hz: target MJPEG stream rate (frames/sec)
    """

    app = Flask(
        __name__,
        template_folder="templates",
        static_folder="static",
        static_url_path="/static",
    )

    # --- General HTML Browser Service ---
    @app.get("/")
    def gui():
        return render_template(
            "gui.html",
            manual_speed_linear=manual_speed_linear,
            manual_speed_angular=manual_speed_angular,
            lid_deg_closed=lid_deg_closed,
            lid_deg_opened=lid_deg_opened,
            sweeper_deg_extend=sweeper_deg_extend,
            sweeper_deg_closed=sweeper_deg_closed,
        )

    # --- Annotated Stream Service ---
    @app.get("/stream/comp_vision")
    def stream_comp_vision():
        resp = Response(
            stream_with_context(mjpeg_generator()),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )
        resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
        resp.headers["Pragma"] = "no-cache"
        resp.headers["Expires"] = "0"
        resp.headers["X-Accel-Buffering"] = "no"
        return resp

    # --- Perception Status Data Service ---
    @app.get("/perception/status")
    def perception_status():
        obs = cv.get_latest_obs()
        if obs is None:
            return jsonify(
                {
                    "ok": False,
                    "reason": "no_obs_yet",
                    # Keep UI stable with defaults
                    "target_infer_hz": None,
                    "measured_infer_hz": None,
                    "num_detections": 0,
                    "target_policy": None,
                    "target": "N/A",
                    "target_status": "SEARCHING ...",
                    "target_data": None,

                    # Ground-plane (defaults)
                    "target_gp_fw_dist": None,
                    "target_gp_lt_dist": None,
                    "target_gp_valid": False,
                }
            )

        out: Dict[str, Any] = {
            "ok": True,
            # Speeds
            "target_infer_hz": obs.get("target_infer_hz", None),
            "measured_infer_hz": obs.get("measured_infer_hz", None),
            # High-level detection info
            "num_detections": obs.get("num_detections", 0),
            "target_policy": obs.get("target_policy", None),
            "target": obs.get("target", "N/A"),
            "target_status": obs.get("target_status", "SEARCHING ..."),
            # Target details
            "target_data": obs.get("target_data", None),

            # Optional stability progress
            "stable_count": obs.get("stable_count", None),
            "stable_window": obs.get("stable_window", None),
            "timestamp": obs.get("timestamp", None),

            # Ground-plane projection (feet)
            "target_gp_fw_dist": obs.get("target_gp_fw_dist", None),
            "target_gp_lt_dist": obs.get("target_gp_lt_dist", None),
            "target_gp_valid": bool(obs.get("target_gp_valid", False)),
        }

        # Make sure target_data is JSON-safe if it includes numpy types
        td = out["target_data"]
        if td is not None:
            out["target_data"] = {
                "conf": float(td.get("conf", 0.0)),
                "area": float(td.get("area", 0.0)),
                "cx": int(td.get("cx", 0)),
                "cy": int(td.get("cy", 0)),
                "xyxy": [int(v) for v in td.get("xyxy", [])],
            }

        return jsonify(out)

    # --- Controller Status Data Service ---
    @app.get("/controller/status")
    def controller_status():
        try:
            last = controller.get_last_cmd()

            # Support either shape:
            # 1) last already has {"drive": {...}, "mech": {...}}
            # 2) last is legacy {"linear": ..., "angular": ...}
            drive = last.get("drive", None)
            mech = last.get("mech", None)

            if drive is None:
                drive = {
                    "linear": float(last.get("linear", 0.0)),
                    "angular": float(last.get("angular", 0.0)),
                }

            return jsonify(
                {
                    "ok": True,
                    "status": controller.get_status(),
                    "cmd": {
                        "linear": float(drive.get("linear", 0.0)),
                        "angular": float(drive.get("angular", 0.0)),
                        "mech": mech,
                    },
                }
            )

        except Exception as e:
            return jsonify({"ok": False, "reason": str(e)}), 200

    # --- Telemetry Status Data Service (NEW) ---
    @app.get("/telemetry/status")
    def telemetry_status():
        """
        What you said you want to display:
        - Connection state
        - Wheel state
        - Mech state
        - Telemetry tick Hz (plus rx/tx Hz since you already compute them)
        """
        try:
            if serial_link is None:
                return jsonify(
                    {
                        "ok": False,
                        "reason": "no_serial_link",
                        "connection": {"state": "DISABLED"},
                        "wheel": None,
                        "mech": None,
                        "ultrasonic": None,

                    }
                )

            status = serial_link.get_status()
            tel = serial_link.get_latest_telemetry()

            # Unpack wheel/mech/ultrasonic into plain JSON dictionaries
            wheel = None
            mech = None
            ultrasonic = None

            if tel is not None:
                if tel.wheel is not None:
                    wheel = {
                        "left_rpm": float(tel.wheel.left_rpm),
                        "right_rpm": float(tel.wheel.right_rpm),
                    }
                if tel.mech is not None:
                    mech = {
                        "servo_LID_deg": (None if tel.mech.servo_LID_deg is None else float(tel.mech.servo_LID_deg)),
                        "servo_SWEEP_deg": (None if tel.mech.servo_SWEEP_deg is None else float(tel.mech.servo_SWEEP_deg)),
                        "motor_RHS_deg": (None if tel.mech.motor_RHS_deg is None else float(tel.mech.motor_RHS_deg)),
                        "motor_LHS_deg": (None if tel.mech.motor_LHS_deg is None else float(tel.mech.motor_LHS_deg)),
                    }
                u = getattr(tel, "ultrasonic", None)
                if u is not None:  # (safe even if older Telemetry)
                    ultrasonic = {
                        "distance_in": (None if tel.ultrasonic.distance_in is None else float(tel.ultrasonic.distance_in)),
                        "valid": bool(tel.ultrasonic.valid),
                    }

            return jsonify(
                {
                    "ok": True,
                    "connection": {
                        "state": status.get("state", "UNKNOWN"),
                        "port": status.get("port", None),
                        "baud": status.get("baud", None),
                        "last_rx_age_s": status.get("last_rx_age_s", None),
                        "rx_stale_s": status.get("rx_stale_s", None),
                        "tick_hz": status.get("tick_hz", None),
                        "rx_hz": status.get("rx_hz", None),
                        "tx_hz": status.get("tx_hz", None),
                        "last_error": status.get("last_error", None),
                    },
                    "wheel": wheel,
                    "mech": mech,
                    "ultrasonic": ultrasonic
                }
            )

        except Exception as e:
            return jsonify({"ok": False, "reason": str(e)}), 200

    # --- Controller Commands ---
    @app.post("/controller/mode")
    def controller_mode():
        data = request.get_json(silent=True) or {}
        mode = data.get("mode", "")
        try:
            controller.set_mode(mode)
            return jsonify({"ok": True})
        except Exception as e:
            return jsonify({"ok": False, "reason": str(e)}), 400

    @app.post("/controller/manual_cmd")
    def controller_manual_cmd():
        data = request.get_json(silent=True) or {}

        # Drive fields (keep backward compatible defaults)
        linear = float(data.get("linear", 0.0))
        angular = float(data.get("angular", 0.0))

        # Optional mechanism fields
        mech_in = data.get("mech", None)
        mech: Optional[Dict[str, Any]] = None

        if isinstance(mech_in, dict):
            mech = {}

            # Only include keys if present; None means "no change"
            if "servo_LID_deg" in mech_in:
                v = mech_in.get("servo_LID_deg", None)
                mech["servo_LID_deg"] = (None if v is None else float(v))

            if "servo_SWEEP_deg" in mech_in:
                v = mech_in.get("servo_SWEEP_deg", None)
                mech["servo_SWEEP_deg"] = (None if v is None else float(v))

            # (optional future extension)
            if "motor_RHS" in mech_in:
                mech["motor_RHS"] = mech_in.get("motor_RHS", None)
            if "motor_LHS" in mech_in:
                mech["motor_LHS"] = mech_in.get("motor_LHS", None)

            if len(mech) == 0:
                mech = None

        # Preferred: controller can accept a single structured cmd dict
        # If your controller only supports linear/angular today, add a small overload there.
        controller.update_user_cmd(
            linear=linear,
            angular=angular,
            mech=mech,
        )
        return jsonify({"ok": True})


    def mjpeg_generator():
        """
        Stream latest annotated frames as an MJPEG multipart response.
        Notes:
          - This function runs per-client connection (each browser tab gets its own generator).
          - It must never call cv.tick() or block robot control.
          - We throttle using stream_hz so a browser doesn't consume all CPU.
        """
        frame_period_s = 1.0 / max(float(stream_hz), 1e-6)

        STREAM_W = 1280  # 640 or 854 works great for dashboards
        JPEG_QUALITY = 90

        try:
            while True:
                t0 = time.perf_counter()

                frame = cv.get_latest_annotated_frame()
                if frame is None:
                    time.sleep(0.02)
                    continue

                h, w = frame.shape[:2]

                # Downscale for streaming only (keep aspect)
                if w > STREAM_W:
                    new_h = int(h * (STREAM_W / w))
                    frame = cv2.resize(frame, (STREAM_W, new_h), interpolation=cv2.INTER_AREA)

                ok, buf = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
                )
                if not ok:
                    time.sleep(0.01)
                    continue

                jpg_bytes = buf.tobytes()

                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: "
                    + str(len(jpg_bytes)).encode("ascii")
                    + b"\r\n\r\n"
                    + jpg_bytes
                    + b"\r\n"
                )

                dt = time.perf_counter() - t0
                sleep_s = frame_period_s - dt
                if sleep_s > 0:
                    time.sleep(sleep_s)

        except (GeneratorExit, BrokenPipeError, ConnectionResetError):
            return


    return app


def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "unknown"
    finally:
        s.close()
    return ip


def run_flask(
    cv,
    controller,
    serial_link,  
    *,
    host: str = "0.0.0.0",
    port: int = 5000,
    stream_hz: float = 15.0,
    quiet: bool = True,
    manual_speed_linear: float = 1.0,
    manual_speed_angular: float = 10.0,
    lid_deg_closed: float = 0.0,
    lid_deg_opened: float = 80.0,
    sweeper_deg_extend: float = 0.0,
    sweeper_deg_closed: float = 30.0,
):
    """
    Run the Flask app. Intended to be launched in a daemon thread from pwc_robot/main.py.

    host/port/stream_hz should come from robot-default.yaml config.
    """
    lan_ip = get_local_ip()
    print(f"[GUI] running on:")
    print(f"  http://localhost:{port}")
    print(f"  http://{lan_ip}:{port}")

    if quiet:
        logging.getLogger("werkzeug").setLevel(logging.ERROR)

    app = create_app(
        cv,
        controller,
        serial_link,
        manual_speed_linear,
        manual_speed_angular,
        lid_deg_closed,
        lid_deg_opened,
        sweeper_deg_extend,
        sweeper_deg_closed,
        stream_hz=stream_hz,
    )
    app.run(
        host=host,
        port=port,
        debug=False,
        threaded=True,
        use_reloader=False,
    )
