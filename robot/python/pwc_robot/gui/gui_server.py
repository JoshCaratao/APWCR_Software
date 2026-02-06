from __future__ import annotations

import time
from typing import Any, Dict

import cv2
from flask import Flask, Response, jsonify, render_template, stream_with_context, request
import logging
import socket


def create_app(cv, controller, manual_speed_linear, manual_speed_angular, stream_hz: float) -> Flask:
    """
    Create the Flask app for the robot GUI and pass in computer_vision object from main.

    stream_hz: target MJPEG stream rate (frames/sec)

    Note:
      - host/port are not strictly needed inside the Flask app object,
        but we keep them as parameters so it's obvious the app is created
        using the same config values that run_flask() will use.
    """

    # Tell Flask where templates live, and where static assets live (CSS/JS)
    app = Flask(
        __name__,
        template_folder="templates",
        static_folder="static",
        static_url_path="/static",
    )

    # --- General HTML Browser Service ---
    @app.get("/")
    def gui():
        return render_template("gui.html")

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
           return jsonify({
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
        })

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

        # Optional, but nice if you want to show stability progress
        "stable_count": obs.get("stable_count", None),
        "stable_window": obs.get("stable_window", None),
        "timestamp": obs.get("timestamp", None),
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
    
    @app.get("/controller/status")
    def controller_status():
        return jsonify({
            "ok": True,
            "status": controller.get_status(),
            "cmd": controller.get_last_cmd(),
        })

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
        linear = float(data.get("linear", manual_speed_linear))
        angular = float(data.get("angular", manual_speed_angular))
        controller.update_user_cmd(linear=linear, angular=angular)
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

        try:
            while True:
                t0 = time.perf_counter()

                frame = cv.get_latest_annotated_frame()
                if frame is None:
                    time.sleep(0.02)
                    continue

                ok, buf = cv2.imencode(".jpg", frame)
                if not ok:
                    time.sleep(0.01)
                    continue

                jpg_bytes = buf.tobytes()

                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(jpg_bytes)).encode("ascii") + b"\r\n\r\n"
                    + jpg_bytes
                    + b"\r\n"
                )

                dt = time.perf_counter() - t0
                sleep_s = frame_period_s - dt
                if sleep_s > 0:
                    time.sleep(sleep_s)

        except GeneratorExit:
            return
        except (BrokenPipeError, ConnectionResetError):
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
        *, 
        host: str = "0.0.0.0", 
        port: int = 5000, 
        stream_hz: float = 15.0, 
        quiet: bool = True, 
        manual_speed_linear: float = 1.0, 
        manual_speed_angular: float = 10.0):
    """
    Run the Flask app. Intended to be launched in a daemon thread from pwc_robot/main.py.

    host/port/stream_hz should come from robot-default.yaml config.
    """
    # Declare how to connect to stream (MUST BE THROUGH LAN)
    lan_ip = get_local_ip()
    print(f"[GUI] running on:")
    print(f"  http://localhost:{port}")
    print(f"  http://{lan_ip}:{port}")

    if quiet:
        logging.getLogger("werkzeug").setLevel(logging.ERROR)

    app = create_app(cv, controller, manual_speed_linear, manual_speed_angular, stream_hz=stream_hz)
    app.run(
        host=host,
        port=port,
        debug=False,
        threaded=True,
        use_reloader=False,
    )
