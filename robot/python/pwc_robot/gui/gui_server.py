from __future__ import annotations

import time
from typing import Any, Dict

import cv2
from flask import Flask, Response, jsonify, render_template, stream_with_context


def create_app(cv, stream_hz: float) -> Flask:
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
            return jsonify({"ok": False, "reason": "no_obs_yet"})

        out: Dict[str, Any] = dict(obs)

        for k in ("best", "stable_center"):
            if k in out and out[k] is not None:
                out[k] = list(out[k])

        out["ok"] = True
        return jsonify(out)

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


def run_flask(cv, *, host: str = "0.0.0.0", port: int = 5000, stream_hz: float = 15.0):
    """
    Run the Flask app. Intended to be launched in a daemon thread from pwc_robot/main.py.

    host/port/stream_hz should come from robot-default.yaml config.
    """
    app = create_app(cv, stream_hz=stream_hz)
    app.run(
        host=host,
        port=port,
        debug=False,
        threaded=True,
        use_reloader=False,
    )
