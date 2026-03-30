from __future__ import annotations

import json
from typing import Any


CMD_TYPE = "cmd"
TEL_TYPE = "telemetry"

MOTOR_FIELDS = (
    "drive_lhs",
    "drive_rhs",
    "mech_rhs",
    "mech_lhs",
)


def encode_command_frame(*, seq: int, motor_commands: dict[str, float]) -> bytes:
    """
    Encode one feedforward-model-test command frame as newline-delimited JSON.

    Example:
      {
        "type": "cmd",
        "seq": 12,
        "drive_lhs_cmd": 0.20,
        "drive_rhs_cmd": 0.0,
        "mech_rhs_cmd": 0.0,
        "mech_lhs_cmd": 0.0
      }
    """
    frame: dict[str, Any] = {
        "type": CMD_TYPE,
        "seq": int(seq),
    }

    for motor in MOTOR_FIELDS:
        frame[f"{motor}_cmd"] = float(motor_commands.get(motor, 0.0))

    payload = json.dumps(frame, separators=(",", ":"), ensure_ascii=False)
    return (payload + "\n").encode("utf-8")


def decode_telemetry_line(line: str) -> dict[str, Any] | None:
    """
    Decode one telemetry JSON line from the test firmware.

    Expected shape:
      {
        "type": "telemetry",
        "arduino_time_ms": 1234,
        "ack_seq": 12,
        "drive_lhs_cmd": 0.20,
        "drive_rhs_cmd": 0.0,
        "mech_rhs_cmd": 0.0,
        "mech_lhs_cmd": 0.0,
        "drive_lhs_rpm": 34.2,
        "drive_rhs_rpm": 0.0,
        "mech_rhs_rpm": 0.0,
        "mech_lhs_rpm": 0.0
      }
    """
    line = line.strip()
    if not line:
        return None

    i = line.find("{")
    if i < 0:
        return None
    line = line[i:]

    try:
        obj = json.loads(line)
    except json.JSONDecodeError:
        return None

    if not isinstance(obj, dict):
        return None

    if obj.get("type") != TEL_TYPE:
        return None

    try:
        out: dict[str, Any] = {
            "arduino_time_ms": int(obj["arduino_time_ms"]),
            "ack_seq": int(obj["ack_seq"]),
        }
    except (KeyError, TypeError, ValueError):
        return None

    for motor in MOTOR_FIELDS:
        out[f"{motor}_cmd"] = _to_float(obj.get(f"{motor}_cmd"), default=0.0)
        out[f"{motor}_rpm"] = _to_float(obj.get(f"{motor}_rpm"), default=None)

    return out


def safe_decode_line(line_bytes: bytes) -> str:
    """Convert raw serial bytes into a safe UTF-8 string."""
    return line_bytes.decode("utf-8", errors="replace")


def _to_float(value: Any, default: float | None) -> float | None:
    """Best-effort float conversion with a fallback default."""
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default
