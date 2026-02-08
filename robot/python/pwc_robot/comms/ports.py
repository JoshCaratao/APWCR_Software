"""
pwc_robot/comms/ports.py

Serial port discovery helpers.

Responsibilities:
- List available serial ports
- Provide a heuristic to find an Arduino-like port

This module intentionally does not define defaults like baud, timeouts, or port.
Those live in the YAML config.
"""

from __future__ import annotations

from typing import List, Optional

import serial.tools.list_ports


def list_serial_ports() -> List[str]:
    """Return device names for all detected serial ports."""
    return [p.device for p in serial.tools.list_ports.comports()]


def find_arduino_port() -> Optional[str]:
    """
    Attempt to find an Arduino-like serial port using common USB-serial hints.
    Returns a port string, or None if nothing found.
    """
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None

    hints = (
        "Arduino",
        "CH340",
        "CP210",
        "FTDI",
        "USB Serial",
        "ttyACM",
        "ttyUSB",
        "usbmodem",
        "usbserial",
    )

    for p in ports:
        haystack = f"{p.device} {p.description or ''} {p.manufacturer or ''}"
        if any(h in haystack for h in hints):
            return p.device

    return ports[0].device
