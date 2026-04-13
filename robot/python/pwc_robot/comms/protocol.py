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
          "reset_RHS_zero": true | false,
          "reset_LHS_zero": true | false,
          "servo_LID_deg": <float> | null,
          "servo_SWEEP_deg": <float> | null
        }
      }
    """
    def enc2(value: float) -> float:
        return round(float(value), 2)

    def enc1(value: float) -> float:
        return round(float(value), 1)

    def encode_motor(cmd: Optional[MechMotorCommand]) -> Optional[Dict[str, Any]]:
        if cmd is None:
            return None
        value = enc1(cmd.value) if cmd.mode.value == "POS_DEG" else enc2(cmd.value)
        return {"mode": cmd.mode.value, "value": value}

    frame: Dict[str, Any] = {
        "type": CMD_TYPE,
        "seq": int(seq),
        "host_time_ms": int(host_time_ms),
        "drive": {
            "linear": enc2(drive.linear),
            "angular": enc2(drive.angular),
        },
        "mech": _encode_mech(mech, encode_motor, enc1),
    }

    s = json.dumps(frame, separators=(",", ":"), ensure_ascii=False)
    return (s + "\n").encode("utf-8")


def _encode_mech(
    mech: MechanismCommand,
    encode_motor,
    enc_angle,
) -> Dict[str, Any]:
    out: Dict[str, Any] = {}

    motor_rhs = encode_motor(mech.motor_RHS)
    motor_lhs = encode_motor(mech.motor_LHS)
    if motor_rhs is not None:
        out["motor_RHS"] = motor_rhs
    if motor_lhs is not None:
        out["motor_LHS"] = motor_lhs

    if bool(mech.reset_RHS_zero):
        out["reset_RHS_zero"] = True
    if bool(mech.reset_LHS_zero):
        out["reset_LHS_zero"] = True

    if mech.servo_LID_deg is not None:
        out["servo_LID_deg"] = enc_angle(mech.servo_LID_deg)
    if mech.servo_SWEEP_deg is not None:
        out["servo_SWEEP_deg"] = enc_angle(mech.servo_SWEEP_deg)

    return out


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
        "wheel": {
          "left_rpm": <float>,
          "right_rpm": <float>,
          "left_duty": <float>,
          "right_duty": <float>,
          "left_target_rpm": <float>,
          "right_target_rpm": <float>
        } | null,
        "mech": {
          "servo_LID_deg": <float> | null,
          "servo_SWEEP_deg": <float> | null,
          "motor_RHS_deg": <float> | null,
          "motor_LHS_deg": <float> | null,
          "bucket_ground_deg": <float> | null,
          "motor_RHS_target_rpm": <float> | null,
          "motor_LHS_target_rpm": <float> | null,
          "motor_RHS_rpm": <float> | null,
          "motor_LHS_rpm": <float> | null,
          "motor_RHS_duty": <float> | null,
          "motor_LHS_duty": <float> | null
        } | null,
        "ultrasonic": {"distance_in": <float>, "valid": <bool>} | null,
        "note": <str> | null
      }
    """
    line = line.strip()
    i = line.find("{")
    if i < 0:
        return None
    line = line[i:]

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

    def f(key: str) -> Optional[float]:
        val = w.get(key)
        if val is None:
            return None
        try:
            return float(val)
        except (TypeError, ValueError):
            return None

    return WheelState(
        left_rpm=f("left_rpm"),
        right_rpm=f("right_rpm"),
        left_duty=f("left_duty"),
        right_duty=f("right_duty"),
        left_target_rpm=f("left_target_rpm"),
        right_target_rpm=f("right_target_rpm"),
        left_stall_fault=bool(w.get("left_stall_fault", False)),
        right_stall_fault=bool(w.get("right_stall_fault", False)),
    )


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
        bucket_ground_deg=f("bucket_ground_deg"),
        motor_RHS_target_rpm=f("motor_RHS_target_rpm"),
        motor_LHS_target_rpm=f("motor_LHS_target_rpm"),
        motor_RHS_rpm=f("motor_RHS_rpm"),
        motor_LHS_rpm=f("motor_LHS_rpm"),
        motor_RHS_duty=f("motor_RHS_duty"),
        motor_LHS_duty=f("motor_LHS_duty"),
        motor_RHS_stall_fault=bool(m.get("motor_RHS_stall_fault", False)),
        motor_LHS_stall_fault=bool(m.get("motor_LHS_stall_fault", False)),
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
