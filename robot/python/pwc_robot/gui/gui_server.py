from __future__ import annotations

import csv
import time
import threading
from typing import Any, Dict, Optional
from pathlib import Path

import cv2
from flask import Flask, Response, jsonify, render_template, stream_with_context, request, send_file
import logging
import socket


def create_app(
    cv,
    controller,
    serial_link,  # <-- NEW: pass SerialLink into the GUI
    manual_speed_linear,
    manual_speed_angular,
    control_test_command_hz: float,
    control_test_sample_hz: float,
    control_test_pre_zero_s: float,
    control_test_post_zero_s: float,
    rhs_arm_jog_duty,
    rhs_arm_stow_deg,
    lhs_arm_jog_duty,
    lhs_arm_stow_deg,
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

    control_test_dir = Path(__file__).resolve().parents[3] / "data" / "control_tests"
    control_test_dir.mkdir(parents=True, exist_ok=True)

    control_test_lock = threading.Lock()
    control_test_state: Dict[str, Any] = {
        "active": False,
        "thread": None,
        "stop_event": None,
        "samples": 0,
        "latest_sample": None,
        "latest_path": None,
        "latest_filename": None,
        "status": "IDLE",
        "spec": None,
    }

    def _safe_num(value: Any, default: float = 0.0) -> float:
        try:
            return float(value)
        except Exception:
            return default

    def _slug_num(value: float) -> str:
        s = f"{value:.2f}".rstrip("0").rstrip(".")
        return s.replace("-", "neg").replace(".", "p")

    def _build_control_test_filename(spec: Dict[str, Any]) -> str:
        test_type = str(spec["test_type"]).upper()
        duration = _slug_num(float(spec["duration_s"]))
        cmd_value = _slug_num(float(spec["command_value"]))
        units = str(spec["command_units"]).replace("/", "ps")
        ts = time.strftime("%Y%m%d_%H%M%S")
        return f"MotorResponse-{test_type}-{duration}s-{cmd_value}{units}-{ts}.csv"

    def _control_test_command_for_elapsed(spec: Dict[str, Any], elapsed_s: float) -> Dict[str, Any]:
        pre_zero_s = float(spec["pre_zero_s"])
        active_s = float(spec["duration_s"])
        post_zero_s = float(spec["post_zero_s"])

        if elapsed_s < pre_zero_s:
            return {
                "phase": "pre_zero",
                "linear": 0.0,
                "angular": 0.0,
            }

        if elapsed_s < (pre_zero_s + active_s):
            return {
                "phase": "active_step",
                "linear": float(spec["linear"]),
                "angular": float(spec["angular"]),
            }

        if elapsed_s < (pre_zero_s + active_s + post_zero_s):
            return {
                "phase": "post_zero",
                "linear": 0.0,
                "angular": 0.0,
            }

        return {
            "phase": "complete",
            "linear": 0.0,
            "angular": 0.0,
        }

    def _capture_control_test_sample(spec: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        telemetry = serial_link.get_latest_telemetry() if serial_link is not None else None
        controller_status = controller.get_status()
        last_cmd = controller.get_last_cmd()

        wheel = getattr(telemetry, "wheel", None)
        drive = last_cmd.get("drive", {})
        elapsed_s = time.time() - float(spec["start_time_s"])
        command_state = _control_test_command_for_elapsed(spec, elapsed_s)

        if telemetry is None or wheel is None:
            return {
                "elapsed_s": elapsed_s,
                "test_type": spec["test_type"],
                "test_phase": command_state["phase"],
                "command_units": spec["command_units"],
                "command_value": float(spec["command_value"]),
                "command_linear_ftps": _safe_num(drive.get("linear", command_state["linear"])),
                "command_angular_dps": _safe_num(drive.get("angular", command_state["angular"])),
                "left_target_rpm": 0.0,
                "right_target_rpm": 0.0,
                "left_measured_rpm": 0.0,
                "right_measured_rpm": 0.0,
                "left_motor_duty": 0.0,
                "right_motor_duty": 0.0,
                "controller_state": str(controller_status.get("state", "N/A")),
                "arduino_time_ms": 0,
                "ack_seq": 0,
            }

        return {
            "elapsed_s": elapsed_s,
            "test_type": spec["test_type"],
            "test_phase": command_state["phase"],
            "command_units": spec["command_units"],
            "command_value": float(spec["command_value"]),
            "command_linear_ftps": _safe_num(drive.get("linear", command_state["linear"])),
            "command_angular_dps": _safe_num(drive.get("angular", command_state["angular"])),
            "left_target_rpm": _safe_num(getattr(wheel, "left_target_rpm", 0.0)),
            "right_target_rpm": _safe_num(getattr(wheel, "right_target_rpm", 0.0)),
            "left_measured_rpm": _safe_num(getattr(wheel, "left_rpm", 0.0)),
            "right_measured_rpm": _safe_num(getattr(wheel, "right_rpm", 0.0)),
            "left_motor_duty": _safe_num(getattr(wheel, "left_duty", 0.0)),
            "right_motor_duty": _safe_num(getattr(wheel, "right_duty", 0.0)),
            "controller_state": str(controller_status.get("state", "N/A")),
            "arduino_time_ms": int(getattr(telemetry, "arduino_time_ms", 0) or 0),
            "ack_seq": int(getattr(telemetry, "ack_seq", 0) or 0),
        }

    def _run_control_test(spec: Dict[str, Any], csv_path: Path, stop_event: threading.Event) -> None:
        headers = [
            "elapsed_s",
            "test_type",
            "test_phase",
            "command_units",
            "command_value",
            "command_linear_ftps",
            "command_angular_dps",
            "left_target_rpm",
            "right_target_rpm",
            "left_measured_rpm",
            "right_measured_rpm",
            "left_motor_duty",
            "right_motor_duty",
            "controller_state",
            "arduino_time_ms",
            "ack_seq",
        ]

        command_period_s = 1.0 / max(float(control_test_command_hz), 1e-6)
        sample_period_s = 1.0 / max(float(control_test_sample_hz), 1e-6)
        total_duration_s = (
            float(spec["pre_zero_s"]) +
            float(spec["duration_s"]) +
            float(spec["post_zero_s"])
        )
        next_command_s = time.time()
        next_sample_s = time.time()

        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()

            while not stop_event.is_set():
                now_s = time.time()
                elapsed_s = now_s - float(spec["start_time_s"])
                if elapsed_s >= total_duration_s:
                    break

                if now_s >= next_command_s:
                    command_state = _control_test_command_for_elapsed(spec, elapsed_s)
                    controller.update_user_cmd(
                        linear=float(command_state["linear"]),
                        angular=float(command_state["angular"]),
                        mech=None,
                    )
                    next_command_s += command_period_s

                if now_s >= next_sample_s:
                    sample = _capture_control_test_sample(spec)
                    if sample is not None:
                        writer.writerow(sample)
                        f.flush()
                        with control_test_lock:
                            control_test_state["samples"] = int(control_test_state["samples"]) + 1
                            control_test_state["latest_sample"] = sample
                    next_sample_s += sample_period_s

                time.sleep(0.01)

        controller.update_user_cmd(linear=0.0, angular=0.0, mech=None)
        with control_test_lock:
            control_test_state["active"] = False
            control_test_state["thread"] = None
            control_test_state["stop_event"] = None
            if control_test_state["status"] != "STOPPED":
                control_test_state["status"] = "COMPLETE"

    # --- General HTML Browser Service ---
    @app.get("/")
    def gui():
        return render_template(
            "gui.html",
            manual_speed_linear=manual_speed_linear,
            manual_speed_angular=manual_speed_angular,
            rhs_arm_jog_duty=rhs_arm_jog_duty,
            rhs_arm_stow_deg=rhs_arm_stow_deg,
            lhs_arm_jog_duty=lhs_arm_jog_duty,
            lhs_arm_stow_deg=lhs_arm_stow_deg,
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
            telemetry = serial_link.get_latest_telemetry() if serial_link is not None else None

            if drive is None:
                drive = {
                    "linear": float(last.get("linear", 0.0)),
                    "angular": float(last.get("angular", 0.0)),
                }

            wheel = getattr(telemetry, "wheel", None)
            left_target_rpm = None
            right_target_rpm = None
            if wheel is not None:
                left_target_rpm = wheel.left_target_rpm
                right_target_rpm = wheel.right_target_rpm

            return jsonify(
                {
                    "ok": True,
                    "status": controller.get_status(),
                    "cmd": {
                        "linear": float(drive.get("linear", 0.0)),
                        "angular": float(drive.get("angular", 0.0)),
                        "left_target_rpm": None if left_target_rpm is None else float(left_target_rpm),
                        "right_target_rpm": None if right_target_rpm is None else float(right_target_rpm),
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
                def _f_or_none(v):
                    return None if v is None else float(v)

                if tel.wheel is not None:
                    wheel = {
                        "left_rpm": _f_or_none(tel.wheel.left_rpm),
                        "right_rpm": _f_or_none(tel.wheel.right_rpm),
                        "left_duty": _f_or_none(tel.wheel.left_duty),
                        "right_duty": _f_or_none(tel.wheel.right_duty),
                        "left_target_rpm": _f_or_none(tel.wheel.left_target_rpm),
                        "right_target_rpm": _f_or_none(tel.wheel.right_target_rpm),
                    }
                if tel.mech is not None:
                    mech = {
                        "servo_LID_deg": _f_or_none(tel.mech.servo_LID_deg),
                        "servo_SWEEP_deg": _f_or_none(tel.mech.servo_SWEEP_deg),
                        "motor_RHS_deg": _f_or_none(tel.mech.motor_RHS_deg),
                        "motor_LHS_deg": _f_or_none(tel.mech.motor_LHS_deg),
                        "motor_RHS_rpm": _f_or_none(tel.mech.motor_RHS_rpm),
                        "motor_LHS_rpm": _f_or_none(tel.mech.motor_LHS_rpm),
                        "motor_RHS_duty": _f_or_none(tel.mech.motor_RHS_duty),
                        "motor_LHS_duty": _f_or_none(tel.mech.motor_LHS_duty),
                    }
                u = getattr(tel, "ultrasonic", None)
                if u is not None:  # (safe even if older Telemetry)
                    ultrasonic = {
                        "distance_in": _f_or_none(tel.ultrasonic.distance_in),
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
                    "ultrasonic": ultrasonic,
                    "note": (tel.note if tel is not None else None),
                    "ack_seq": (None if tel is None else int(tel.ack_seq)),
                    "arduino_time_ms": (None if tel is None else int(tel.arduino_time_ms)),
  
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
            if "reset_RHS_zero" in mech_in:
                mech["reset_RHS_zero"] = bool(mech_in.get("reset_RHS_zero", False))
            if "reset_LHS_zero" in mech_in:
                mech["reset_LHS_zero"] = bool(mech_in.get("reset_LHS_zero", False))

            if len(mech) == 0:
                mech = None
        
        #print("[GUI_SERVER] manual_cmd raw:", data)
        #print("[GUI_SERVER] parsed mech:", mech)


        # Preferred: controller can accept a single structured cmd dict
        # If your controller only supports linear/angular today, add a small overload there.
        controller.update_user_cmd(
            linear=linear,
            angular=angular,
            mech=mech,
        )
        return jsonify({"ok": True})

    @app.post("/control_test/start")
    def control_test_start():
        data = request.get_json(silent=True) or {}
        test_type = str(data.get("test_type", "forward")).strip().lower()
        command_value = abs(float(data.get("command_value", 0.0)))
        duration_s = max(1.0, float(data.get("duration_s", 6.0)))

        if controller.get_status().get("state") != "MANUAL":
            return jsonify({"ok": False, "reason": "controller_not_manual"}), 400

        if test_type == "forward":
            linear = command_value
            angular = 0.0
            units = "ftps"
        elif test_type == "reverse":
            linear = -command_value
            angular = 0.0
            units = "ftps"
        elif test_type == "turn_left":
            linear = 0.0
            angular = command_value
            units = "degps"
        elif test_type == "turn_right":
            linear = 0.0
            angular = -command_value
            units = "degps"
        else:
            return jsonify({"ok": False, "reason": "invalid_test_type"}), 400

        with control_test_lock:
            if control_test_state["active"]:
                return jsonify({"ok": False, "reason": "test_already_active"}), 409

            spec = {
                "test_type": test_type,
                "command_value": command_value,
                "duration_s": duration_s,
                "pre_zero_s": float(control_test_pre_zero_s),
                "post_zero_s": float(control_test_post_zero_s),
                "command_units": units,
                "linear": linear,
                "angular": angular,
                "start_time_s": time.time(),
            }
            filename = _build_control_test_filename(spec)
            csv_path = control_test_dir / filename
            stop_event = threading.Event()
            thread = threading.Thread(
                target=_run_control_test,
                args=(spec, csv_path, stop_event),
                daemon=True,
            )

            control_test_state["active"] = True
            control_test_state["thread"] = thread
            control_test_state["stop_event"] = stop_event
            control_test_state["samples"] = 0
            control_test_state["latest_sample"] = None
            control_test_state["latest_path"] = str(csv_path)
            control_test_state["latest_filename"] = filename
            control_test_state["status"] = "RUNNING"
            control_test_state["spec"] = spec

            thread.start()

        return jsonify({"ok": True, "filename": filename, "path": str(csv_path)})

    @app.post("/control_test/stop")
    def control_test_stop():
        with control_test_lock:
            stop_event = control_test_state.get("stop_event")
            was_active = bool(control_test_state.get("active"))
            if stop_event is not None:
                stop_event.set()
            if was_active:
                control_test_state["status"] = "STOPPED"

        controller.update_user_cmd(linear=0.0, angular=0.0, mech=None)
        return jsonify({"ok": True, "was_active": was_active})

    @app.get("/control_test/status")
    def control_test_status():
        with control_test_lock:
            latest_sample = control_test_state.get("latest_sample")
            latest_filename = control_test_state.get("latest_filename")
            latest_path = control_test_state.get("latest_path")
            active = bool(control_test_state.get("active"))
            status = str(control_test_state.get("status", "IDLE"))
            samples = int(control_test_state.get("samples", 0))
            spec = control_test_state.get("spec")

        return jsonify(
            {
                "ok": True,
                "active": active,
                "status": status,
                "samples": samples,
                "latest_sample": latest_sample,
                "latest_filename": latest_filename,
                "latest_path": latest_path,
                "data_dir": str(control_test_dir),
                "spec": spec,
            }
        )

    @app.get("/control_test/download_latest")
    def control_test_download_latest():
        with control_test_lock:
            latest_path = control_test_state.get("latest_path")
            latest_filename = control_test_state.get("latest_filename")

        if not latest_path:
            return jsonify({"ok": False, "reason": "no_control_test_file"}), 404

        path = Path(latest_path)
        if not path.exists():
            return jsonify({"ok": False, "reason": "control_test_file_missing"}), 404

        return send_file(path, as_attachment=True, download_name=latest_filename)


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
    control_test_command_hz: float = 15.0,
    control_test_sample_hz: float = 10.0,
    control_test_pre_zero_s: float = 1.0,
    control_test_post_zero_s: float = 1.0,
    rhs_arm_jog_duty: float = 0.35,
    rhs_arm_stow_deg: float = 100.0,
    lhs_arm_jog_duty: float = 0.35,
    lhs_arm_stow_deg: float = 0.0,
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
        control_test_command_hz,
        control_test_sample_hz,
        control_test_pre_zero_s,
        control_test_post_zero_s,
        rhs_arm_jog_duty,
        rhs_arm_stow_deg,
        lhs_arm_jog_duty,
        lhs_arm_stow_deg,
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
