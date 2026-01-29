from __future__ import annotations

import time
from typing import Any, Dict

import cv2
from flask import Flask, Response, jsonify, render_template, stream_with_context


def create_app(cv, host: str, port: int, stream_hz: float, target_infer_hz: float) -> Flask:
    """
    Create the Flask app for the robot GUI and pass in computer_vision object from main.

    stream_hz: target MJPEG stream rate (frames/sec)

    Note:
      - host/port are not strictly needed inside the Flask app object,
        but we keep them as parameters so it's obvious the app is created
        using the same config values that run_flask() will use.
    """

    # Creates web server and tells Flask where HTML files live
    app = Flask(__name__, template_folder="templates")

    # When someone opens /, send them to gui.html
    @app.get("/")
    def gui():
        return render_template("gui.html")

    # Generator function to repeatedly yield image bytes
    def mjpeg_generator():
        """
        Stream latest annotated frames as an MJPEG multipart response.

        Notes:
          - This function runs per-client connection (each browser tab gets its own generator).
          - It must never call cv.tick() or block robot control.
          - We throttle using stream_hz so a browser doesn't consume all CPU.
        """

        # Seconds per frame (example: 15 Hz -> ~0.0667 s between frames)
        frame_period_s = 1.0 / max(float(stream_hz), 1e-6)

        try:
            while True:
                # Measure loop time so we can sleep the remainder to hit stream_hz
                t0 = time.perf_counter()

                # Grab most recent annotated frame from cv object
                frame = cv.get_latest_annotated_frame()
                if frame is None:
                    # If CV hasn't produced a frame yet, back off a bit
                    time.sleep(0.02)
                    continue

                # Encode CV frames to JPEG bytes for the browser
                ok, buf = cv2.imencode(".jpg", frame)
                if not ok:
                    time.sleep(0.01)
                    continue

                jpg_bytes = buf.tobytes()

                # Send JPEG Byte Chunks to Browser
                # MJPEG is basically: boundary + headers + JPEG + boundary + headers + JPEG ...
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(jpg_bytes)).encode("ascii") + b"\r\n\r\n"
                    + jpg_bytes
                    + b"\r\n"
                )

                # Throttle to stream_hz
                dt = time.perf_counter() - t0
                sleep_s = frame_period_s - dt
                if sleep_s > 0:
                    time.sleep(sleep_s)

        except GeneratorExit:
            # Browser tab closed / client disconnected cleanly
            return
        except (BrokenPipeError, ConnectionResetError):
            # Client disconnected abruptly (Wi-Fi drop, refresh, etc.)
            return

    @app.get("/stream/annotated")
    def stream_annotated():
        # stream_with_context ensures Flask keeps the request context during streaming
        resp = Response(
            stream_with_context(mjpeg_generator()),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

        # These headers reduce caching/buffering and can improve perceived latency
        resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
        resp.headers["Pragma"] = "no-cache"
        resp.headers["Expires"] = "0"

        # Helpful if you ever run behind a proxy like nginx (prevents buffering)
        resp.headers["X-Accel-Buffering"] = "no"

        return resp

    @app.get("/api/obs")
    def api_obs():
        obs = cv.get_latest_obs()
        if obs is None:
            return jsonify({"ok": False, "reason": "no_obs_yet"})

        # Make sure JSON is friendly (tuples -> lists)
        out: Dict[str, Any] = dict(obs)

        for k in ("best", "stable_center"):
            if k in out and out[k] is not None:
                out[k] = list(out[k])

        out["ok"] = True
        return jsonify(out)

    return app


def run_flask(
    cv,
    *,
    host: str = "0.0.0.0",
    port: int = 5000,
    stream_hz: float = 15.0,
    target_infer_hz: float
) -> None:
    """
    Run the Flask app. Intended to be launched in a daemon thread from pwc_robot/main.py.

    host/port/stream_hz should come from robot-default.yaml config.
    """

    # Create the Flask app using the configured host/port/stream_hz
    app = create_app(cv, host=host, port=port, stream_hz=stream_hz, target_infer_hz = target_infer_hz)

    # Start Flask's built-in server.
    # This is fine for dev/testing and for your robot LAN GUI.
    app.run(
        host=host,
        port=port,
        debug=False,
        threaded=True,       # allow multiple connections (each stream gets its own thread)
        use_reloader=False,  # critical when running from a thread (prevents double-start)
    )
