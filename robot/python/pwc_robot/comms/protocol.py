"""
pwc_robot/comms/protocol.py

Wire protocol helpers for Arduino <-> Laptop communication.

Design:
- Newline-delimited JSON frames (one JSON object per line)
- Laptop -> Arduino sends a FULL command frame each time
- Arduino -> Laptop sends a FULL telemetry frame each time
- Command frames include seq (monotonic int)
- Telemetry frames include ack_seq (last seq applied) as implicit ACK

This module does not do serial I/O. serial_link.py owns the port.
"""

from __future__ import annotations

import json
from typing import Any, Dict, Optional

from pwc_robot.controller.commands import (
    DriveCommand,
    MechanismCommand,
    MechMotorCommand,
)

from pwc_robot.comms.types import (
    Telemetry,
    WheelState,
    MechanismState,
    UltrasonicState,
)

# -----------------------------
# Message type strings
# -----------------------------
CMD_TYPE = "cmd"
TEL_TYPE = "telemetry"


# -----------------------------
# Encoding (Laptop -> Arduino)
# -----------------------------

def encode_command_frame(
    *,
    seq: int,
    host_time_ms: int,
    drive: DriveCommand,
    mech: MechanismCommand,
) -> bytes:
    """
    Encode a full command frame for Arduino (one JSON line).

    Schema:
      {
        "type": "cmd",
        "seq": <int>,
        "host_time_ms": <int>,
        "drive": {"linear": <float>, "angular": <float>},
        "mech": {
          "motor_RHS": {"mode": "POS_DEG", "value": 12.3} | null,
          "motor_LHS": {"mode": "DUTY", "value": 0.2} | null,
          "servo_LID_deg": <float> | null,
          "servo_SWEEP_deg": <float> | null
        }
      }
    """
    frame: Dict[str, Any] = {
        "type": CMD_TYPE,
        "seq": int(seq),
        "host_time_ms": int(host_time_ms),
        "drive": {
            "linear": float(drive.linear),
            "angular": float(drive.angular),
        },
        "mech": _encode_mech(mech),
    }

    s = json.dumps(frame, separators=(",", ":"), ensure_ascii=False)
    return (s + "\n").encode("utf-8")


def _encode_mech(mech: MechanismCommand) -> Dict[str, Any]:
    def encode_motor(m: Optional[MechMotorCommand]) -> Optional[Dict[str, Any]]:
        if m is None:
            return None
        return {"mode": str(m.mode), "value": float(m.value)}

    return {
        "motor_RHS": encode_motor(mech.motor_RHS),
        "motor_LHS": encode_motor(mech.motor_LHS),
        "servo_LID_deg": None if mech.servo_LID_deg is None else float(mech.servo_LID_deg),
        "servo_SWEEP_deg": None if mech.servo_SWEEP_deg is None else float(mech.servo_SWEEP_deg),
    }


# -----------------------------
# Decoding (Arduino -> Laptop)
# -----------------------------

def decode_telemetry_line(line: str) -> Optional[Telemetry]:
    """
    Decode one telemetry JSON line from Arduino.

    Schema:
      {
        "type": "telemetry",
        "arduino_time_ms": <int>,
        "ack_seq": <int>,
        "wheel": {"left_rpm": <float>, "right_rpm": <float>} | null,
        "mech": {
          "servo_LID_deg": <float> | null,
          "servo_SWEEP_deg": <float> | null,
          "motor_RHS_deg": <float> | null,
          "motor_LHS_deg": <float> | null
        } | null,
        "ultrasonic": {"distance_in": <float>, "valid": <bool>} | null,
        "note": <str> | null
      }
    """
    line = line.strip()
    if not line:
        return None

    try:
        obj = json.loads(line)
    except json.JSONDecodeError:
        return None

    if not isinstance(obj, dict):
        return None

    if obj.get("type") != TEL_TYPE:
        return None

    try:
        arduino_time_ms = int(obj["arduino_time_ms"])
        ack_seq = int(obj["ack_seq"])
    except (KeyError, TypeError, ValueError):
        return None

    wheel = _decode_wheel(obj.get("wheel"))
    mech = _decode_mech(obj.get("mech"))
    ultrasonic = _decode_ultrasonic(obj.get("ultrasonic"))  # NEW

    note_val = obj.get("note")
    note = str(note_val) if note_val is not None else None

    return Telemetry(
        arduino_time_ms=arduino_time_ms,
        ack_seq=ack_seq,
        wheel=wheel,
        mech=mech,
        ultrasonic=ultrasonic,  
        note=note,
    )


def _decode_wheel(w: Any) -> Optional[WheelState]:
    if w is None:
        return None
    if not isinstance(w, dict):
        return None
    try:
        left = float(w["left_rpm"])
        right = float(w["right_rpm"])
    except (KeyError, TypeError, ValueError):
        return None
    return WheelState(left_rpm=left, right_rpm=right)


def _decode_mech(m: Any) -> Optional[MechanismState]:
    if m is None:
        return None
    if not isinstance(m, dict):
        return None

    def f(key: str) -> Optional[float]:
        val = m.get(key)
        if val is None:
            return None
        try:
            return float(val)
        except (TypeError, ValueError):
            return None

    return MechanismState(
        servo_LID_deg=f("servo_LID_deg"),
        servo_SWEEP_deg=f("servo_SWEEP_deg"),
        motor_RHS_deg=f("motor_RHS_deg"),
        motor_LHS_deg=f("motor_LHS_deg"),
    )


def _decode_ultrasonic(u: Any) -> Optional[UltrasonicState]:
    if u is None:
        return None
    if not isinstance(u, dict):
        return None

    valid_raw = u.get("valid")
    valid = isinstance(valid_raw, bool) and valid_raw

    dist_val = u.get("distance_in")
    distance_in: Optional[float] = None
    if dist_val is not None:
        try:
            distance_in = float(dist_val)
        except (TypeError, ValueError):
            distance_in = None
            valid = False

    if not valid:
        distance_in = None

    return UltrasonicState(distance_in=distance_in, valid=valid)



# -----------------------------
# Utility
# -----------------------------

def safe_decode_line(line_bytes: bytes) -> str:
    """Convert raw bytes from serial into a safe UTF-8 string."""
    return line_bytes.decode("utf-8", errors="replace")
