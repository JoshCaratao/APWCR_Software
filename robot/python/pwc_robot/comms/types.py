"""
pwc_robot/comms/types.py

Comms-owned data models for the serial link.

Important:
  DriveCommand and MechanismCommand already live in:
    pwc_robot/controller/commands.py

This file defines the Python-side structured representation of what the
Arduino sends back (telemetry frames), plus link state metadata.

Wire format note:
- These types are NOT sent over serial directly.
- Serial sends newline-delimited JSON. protocol.py decodes JSON into these types.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class LinkState(str, Enum):
    """High-level health state for the serial connection."""
    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    CONNECTED = "CONNECTED"
    STALE = "STALE"   # connected but telemetry is too old
    ERROR = "ERROR"


@dataclass
class WheelState:
    """
    Wheel speed feedback from Arduino.

    Units:
    - Choose one and stick to it in protocol.py and on the Arduino.
    - Recommended for bring-up: RPM.
    """
    left_rpm: float
    right_rpm: float


@dataclass
class MechanismState:
    """
    Mechanism state feedback from Arduino (plain floats).

    Conventions:
    - Servo angles in degrees (0-180 typical).
    - Motor angles in degrees (encoder-derived, can be any real number).

    Field meanings should match your robot hardware naming.
    """
    servo_LID_deg: Optional[float] = None
    servo_SWEEP_deg: Optional[float] = None

    motor_RHS_deg: Optional[float] = None
    motor_LHS_deg: Optional[float] = None

@dataclass
class UltrasonicState:
    """
    Ultrasonic distance feedback from Arduino.

    Conventions:
    - distance_cm is the computed range in centimeters.
    - valid indicates whether the reading is trustworthy (no timeout/out-of-range).
    - distance_cm may be None if valid is False.

    Notes:
    - Keeping this as its own dataclass makes it easy to add more fields later
      (e.g., raw_pulse_us, filtered_cm, sensor_id).
    """
    distance_in: Optional[float] = None
    valid: bool = False

@dataclass
class Telemetry:
    """
    Full telemetry frame (Arduino -> Laptop).

    Required fields:
    - arduino_time_ms: Arduino millis() timestamp
    - ack_seq: last command sequence number Arduino has applied (acts as ACK)

    Optional fields:
    - wheel: wheel speed feedback
    - mech: mechanism angles
    - note: optional debug string if you ever want it

    Host-side fields (filled by serial_link.py, not sent by Arduino):
    - host_rx_time_s: time.time() or perf_counter() when received
    - rx_age_s: how old the latest telemetry is (computed continuously)
    """
    # From Arduino
    arduino_time_ms: int
    ack_seq: int

    wheel: Optional[WheelState] = None
    mech: Optional[MechanismState] = None
    ultrasonic: Optional[UltrasonicState] = None
    note: Optional[str] = None

    # Filled in by Python on receipt
    host_rx_time_s: float = 0.0
    rx_age_s: Optional[float] = None


@dataclass
class LinkStats:
    """
    Useful link statistics for debugging and GUI display.
    This is purely host-side state.
    """
    state: LinkState = LinkState.DISCONNECTED
    port: Optional[str] = None
    baud: Optional[int] = None

    tx_seq: int = 0
    last_ack_seq: Optional[int] = None

    bytes_tx: int = 0
    bytes_rx: int = 0

    # timestamps (host time, seconds)
    last_tx_time_s: Optional[float] = None
    last_rx_time_s: Optional[float] = None
    last_ok_time_s: Optional[float] = None  # last time link was considered healthy

    # error info
    last_error: Optional[str] = None
