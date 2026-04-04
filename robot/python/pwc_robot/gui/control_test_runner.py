from __future__ import annotations

import csv
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional


class ControlTestRunner:
    """
    Server-side control test recorder for drive and mechanism step tests.

    The GUI starts/stops tests through Flask routes, but the test orchestration
    and CSV recording live here so gui_server.py stays small and readable.
    """

    def __init__(
        self,
        *,
        controller,
        serial_link,
        data_dir: Path,
        command_hz: float,
        sample_hz: float,
        pre_zero_s: float,
        post_zero_s: float,
    ) -> None:
        self._controller = controller
        self._serial_link = serial_link
        self._data_dir = Path(data_dir)
        self._data_dir.mkdir(parents=True, exist_ok=True)

        self._command_hz = float(command_hz)
        self._sample_hz = float(sample_hz)
        self._pre_zero_s = float(pre_zero_s)
        self._post_zero_s = float(post_zero_s)

        self._lock = threading.Lock()
        self._state: Dict[str, Any] = {
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

    def start(self, *, test_type: str, command_value: float, duration_s: float) -> Dict[str, Any]:
        spec = self._make_spec(
            test_type=test_type,
            command_value=command_value,
            duration_s=duration_s,
        )

        filename = self._build_filename(spec)
        csv_path = self._data_dir / filename
        stop_event = threading.Event()
        thread = threading.Thread(
            target=self._run_test,
            args=(spec, csv_path, stop_event),
            daemon=True,
            name="control-test-runner",
        )

        with self._lock:
            if self._state["active"]:
                return {"ok": False, "reason": "test_already_active"}

            self._state["active"] = True
            self._state["thread"] = thread
            self._state["stop_event"] = stop_event
            self._state["samples"] = 0
            self._state["latest_sample"] = None
            self._state["latest_path"] = str(csv_path)
            self._state["latest_filename"] = filename
            self._state["status"] = "RUNNING"
            self._state["spec"] = spec

        thread.start()
        return {
            "ok": True,
            "filename": filename,
            "path": str(csv_path),
            "spec": spec,
        }

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            stop_event = self._state.get("stop_event")
            was_active = bool(self._state.get("active"))
            if stop_event is not None:
                stop_event.set()
            if was_active:
                self._state["status"] = "STOPPED"

        self._controller.update_user_cmd(linear=0.0, angular=0.0, mech=self._neutral_mech_cmd())
        return {"ok": True, "was_active": was_active}

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "ok": True,
                "active": bool(self._state.get("active")),
                "status": str(self._state.get("status", "IDLE")),
                "samples": int(self._state.get("samples", 0)),
                "latest_sample": self._state.get("latest_sample"),
                "latest_filename": self._state.get("latest_filename"),
                "latest_path": self._state.get("latest_path"),
                "data_dir": str(self._data_dir),
                "spec": self._state.get("spec"),
            }

    def get_latest_download(self) -> Dict[str, Optional[str]]:
        with self._lock:
            return {
                "latest_path": self._state.get("latest_path"),
                "latest_filename": self._state.get("latest_filename"),
            }

    def _safe_num(self, value: Any, default: float = 0.0) -> float:
        try:
            return float(value)
        except Exception:
            return default

    def _slug_num(self, value: float) -> str:
        s = f"{value:.2f}".rstrip("0").rstrip(".")
        return s.replace("-", "neg").replace(".", "p")

    def _build_filename(self, spec: Dict[str, Any]) -> str:
        test_type = str(spec["test_type"]).upper()
        duration = self._slug_num(float(spec["duration_s"]))
        cmd_value = self._slug_num(float(spec["command_value"]))
        units = str(spec["command_units"]).replace("/", "ps")
        ts = time.strftime("%Y%m%d_%H%M%S")
        return f"MotorResponse-{test_type}-{duration}s-{cmd_value}{units}-{ts}.csv"

    def _neutral_mech_cmd(self) -> Dict[str, Any]:
        return {
            "motor_RHS": None,
            "motor_LHS": None,
        }

    def _make_spec(self, *, test_type: str, command_value: float, duration_s: float) -> Dict[str, Any]:
        tt = str(test_type).strip().lower()
        raw_value = float(command_value)
        mag = abs(raw_value)

        spec: Dict[str, Any] = {
            "test_type": tt,
            "command_value": raw_value,
            "duration_s": max(1.0, float(duration_s)),
            "pre_zero_s": self._pre_zero_s,
            "post_zero_s": self._post_zero_s,
            "start_time_s": time.time(),
        }

        if tt == "forward":
            spec.update({
                "domain": "drive",
                "command_units": "ftps",
                "linear": mag,
                "angular": 0.0,
            })
            return spec

        if tt == "reverse":
            spec.update({
                "domain": "drive",
                "command_units": "ftps",
                "linear": -mag,
                "angular": 0.0,
            })
            return spec

        if tt == "turn_left":
            spec.update({
                "domain": "drive",
                "command_units": "degps",
                "linear": 0.0,
                "angular": mag,
            })
            return spec

        if tt == "turn_right":
            spec.update({
                "domain": "drive",
                "command_units": "degps",
                "linear": 0.0,
                "angular": -mag,
            })
            return spec

        mech_map = {
            "rhs_mech_speed": ("RHS", "RPM", "rpm"),
            "lhs_mech_speed": ("LHS", "RPM", "rpm"),
            "rhs_mech_pos": ("RHS", "POS_DEG", "deg"),
            "lhs_mech_pos": ("LHS", "POS_DEG", "deg"),
        }
        if tt in mech_map:
            side, mode, units = mech_map[tt]
            spec.update({
                "domain": "mechanism",
                "mech_side": side,
                "mech_mode": mode,
                "command_units": units,
            })
            return spec

        raise ValueError("invalid_test_type")

    def _command_for_elapsed(self, spec: Dict[str, Any], elapsed_s: float) -> Dict[str, Any]:
        pre_zero_s = float(spec["pre_zero_s"])
        active_s = float(spec["duration_s"])
        post_zero_s = float(spec["post_zero_s"])

        if elapsed_s < pre_zero_s:
            phase = "pre_zero"
            active_value = 0.0
        elif elapsed_s < (pre_zero_s + active_s):
            phase = "active_step"
            active_value = float(spec["command_value"])
        elif elapsed_s < (pre_zero_s + active_s + post_zero_s):
            phase = "post_zero"
            active_value = 0.0
        else:
            phase = "complete"
            active_value = 0.0

        if spec["domain"] == "drive":
            return {
                "phase": phase,
                "linear": active_value if spec["test_type"] in ("forward", "reverse") else 0.0,
                "angular": active_value if spec["test_type"] in ("turn_left", "turn_right") else 0.0,
                "mech": self._neutral_mech_cmd(),
            }

        mech_cmd = self._neutral_mech_cmd()
        mech_key = "motor_RHS" if spec["mech_side"] == "RHS" else "motor_LHS"
        mech_cmd[mech_key] = {"mode": spec["mech_mode"], "value": active_value}
        return {
            "phase": phase,
            "linear": 0.0,
            "angular": 0.0,
            "mech": mech_cmd,
        }

    def _capture_sample(self, spec: Dict[str, Any]) -> Dict[str, Any]:
        telemetry = self._serial_link.get_latest_telemetry() if self._serial_link is not None else None
        controller_status = self._controller.get_status()
        elapsed_s = time.time() - float(spec["start_time_s"])
        command_state = self._command_for_elapsed(spec, elapsed_s)

        wheel = getattr(telemetry, "wheel", None)
        mech = getattr(telemetry, "mech", None)

        sample = {
            "elapsed_s": elapsed_s,
            "test_type": spec["test_type"],
            "test_phase": command_state["phase"],
            "command_units": spec["command_units"],
            "command_value": float(spec["command_value"]),
            "command_linear_ftps": self._safe_num(command_state["linear"]),
            "command_angular_dps": self._safe_num(command_state["angular"]),
            "command_mech_side": spec.get("mech_side"),
            "command_mech_mode": spec.get("mech_mode"),
            "command_mech_value": 0.0 if command_state["phase"] != "active_step" else float(spec["command_value"]),
            "drive_left_target_rpm": 0.0,
            "drive_right_target_rpm": 0.0,
            "drive_left_measured_rpm": 0.0,
            "drive_right_measured_rpm": 0.0,
            "drive_left_motor_duty": 0.0,
            "drive_right_motor_duty": 0.0,
            "mech_rhs_target_rpm": 0.0,
            "mech_lhs_target_rpm": 0.0,
            "mech_rhs_measured_rpm": 0.0,
            "mech_lhs_measured_rpm": 0.0,
            "mech_rhs_measured_deg": 0.0,
            "mech_lhs_measured_deg": 0.0,
            "mech_rhs_motor_duty": 0.0,
            "mech_lhs_motor_duty": 0.0,
            "test_target_rpm": 0.0,
            "test_measured_rpm": 0.0,
            "test_measured_deg": 0.0,
            "test_motor_duty": 0.0,
            "controller_state": str(controller_status.get("state", "N/A")),
            "arduino_time_ms": int(getattr(telemetry, "arduino_time_ms", 0) or 0),
            "ack_seq": int(getattr(telemetry, "ack_seq", 0) or 0),
        }

        if wheel is not None:
            sample.update({
                "drive_left_target_rpm": self._safe_num(getattr(wheel, "left_target_rpm", 0.0)),
                "drive_right_target_rpm": self._safe_num(getattr(wheel, "right_target_rpm", 0.0)),
                "drive_left_measured_rpm": self._safe_num(getattr(wheel, "left_rpm", 0.0)),
                "drive_right_measured_rpm": self._safe_num(getattr(wheel, "right_rpm", 0.0)),
                "drive_left_motor_duty": self._safe_num(getattr(wheel, "left_duty", 0.0)),
                "drive_right_motor_duty": self._safe_num(getattr(wheel, "right_duty", 0.0)),
            })

        if mech is not None:
            sample.update({
                "mech_rhs_target_rpm": self._safe_num(getattr(mech, "motor_RHS_target_rpm", 0.0)),
                "mech_lhs_target_rpm": self._safe_num(getattr(mech, "motor_LHS_target_rpm", 0.0)),
                "mech_rhs_measured_rpm": self._safe_num(getattr(mech, "motor_RHS_rpm", 0.0)),
                "mech_lhs_measured_rpm": self._safe_num(getattr(mech, "motor_LHS_rpm", 0.0)),
                "mech_rhs_measured_deg": self._safe_num(getattr(mech, "motor_RHS_deg", 0.0)),
                "mech_lhs_measured_deg": self._safe_num(getattr(mech, "motor_LHS_deg", 0.0)),
                "mech_rhs_motor_duty": self._safe_num(getattr(mech, "motor_RHS_duty", 0.0)),
                "mech_lhs_motor_duty": self._safe_num(getattr(mech, "motor_LHS_duty", 0.0)),
            })

        if spec["domain"] == "mechanism":
            if spec["mech_side"] == "RHS":
                sample["test_target_rpm"] = sample["mech_rhs_target_rpm"]
                sample["test_measured_rpm"] = sample["mech_rhs_measured_rpm"]
                sample["test_measured_deg"] = sample["mech_rhs_measured_deg"]
                sample["test_motor_duty"] = sample["mech_rhs_motor_duty"]
            else:
                sample["test_target_rpm"] = sample["mech_lhs_target_rpm"]
                sample["test_measured_rpm"] = sample["mech_lhs_measured_rpm"]
                sample["test_measured_deg"] = sample["mech_lhs_measured_deg"]
                sample["test_motor_duty"] = sample["mech_lhs_motor_duty"]

        return sample

    def _headers_for_spec(self, spec: Dict[str, Any]) -> list[str]:
        common_headers = [
            "elapsed_s",
            "test_type",
            "test_phase",
            "command_units",
            "command_value",
            "controller_state",
            "arduino_time_ms",
            "ack_seq",
        ]

        if spec["domain"] == "drive":
            return common_headers + [
                "command_linear_ftps",
                "command_angular_dps",
                "drive_left_target_rpm",
                "drive_right_target_rpm",
                "drive_left_measured_rpm",
                "drive_right_measured_rpm",
                "drive_left_motor_duty",
                "drive_right_motor_duty",
            ]

        return common_headers + [
            "command_mech_side",
            "command_mech_mode",
            "command_mech_value",
            "test_target_rpm",
            "test_measured_rpm",
            "test_measured_deg",
            "test_motor_duty",
        ]

    def _run_test(self, spec: Dict[str, Any], csv_path: Path, stop_event: threading.Event) -> None:
        headers = self._headers_for_spec(spec)

        command_period_s = 1.0 / max(self._command_hz, 1e-6)
        sample_period_s = 1.0 / max(self._sample_hz, 1e-6)
        total_duration_s = float(spec["pre_zero_s"]) + float(spec["duration_s"]) + float(spec["post_zero_s"])
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
                    command_state = self._command_for_elapsed(spec, elapsed_s)
                    self._controller.update_user_cmd(
                        linear=float(command_state["linear"]),
                        angular=float(command_state["angular"]),
                        mech=command_state["mech"],
                    )
                    next_command_s += command_period_s

                if now_s >= next_sample_s:
                    sample = self._capture_sample(spec)
                    writer.writerow({key: sample.get(key) for key in headers})
                    f.flush()
                    with self._lock:
                        self._state["samples"] = int(self._state["samples"]) + 1
                        self._state["latest_sample"] = sample
                    next_sample_s += sample_period_s

                time.sleep(0.01)

        self._controller.update_user_cmd(linear=0.0, angular=0.0, mech=self._neutral_mech_cmd())
        with self._lock:
            self._state["active"] = False
            self._state["thread"] = None
            self._state["stop_event"] = None
            if self._state["status"] != "STOPPED":
                self._state["status"] = "COMPLETE"
