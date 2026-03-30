from __future__ import annotations

import time
from typing import Optional

import serial

from feedforward_model_test.protocol import (
    MOTOR_FIELDS,
    decode_telemetry_line,
    encode_command_frame,
    safe_decode_line,
)


class SerialLink:
    """
    Minimal serial transport for the feedforward model test.

    Responsibilities:
    - Open and close the configured serial port
    - Send one command frame at a time
    - Read telemetry lines and keep the latest decoded sample
    """

    def __init__(self, comms_cfg: dict) -> None:
        """Store serial settings and initialize link state."""
        self.port = str(comms_cfg["port"])
        self.baud = int(comms_cfg["baud"])
        self.timeout_s = float(comms_cfg["timeout_s"])
        self.write_timeout_s = float(comms_cfg["write_timeout_s"])

        self._ser: Optional[serial.Serial] = None
        self._tx_seq = 0
        self._latest_telemetry: dict | None = None

    def open(self) -> None:
        """Open the configured serial port and clear startup noise."""
        if self._ser is not None and self._ser.is_open:
            return

        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=self.timeout_s,
            write_timeout=self.write_timeout_s,
        )

        # Clear reset noise / partial lines after connect.
        time.sleep(1.5)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

    def close(self) -> None:
        """Close the serial port if it is currently open."""
        if self._ser is None:
            return
        try:
            self._ser.close()
        finally:
            self._ser = None

    def send_commands(self, motor_commands: dict[str, float]) -> int:
        """Send one command frame and return the transmitted sequence number."""
        if self._ser is None or not self._ser.is_open:
            raise RuntimeError("Serial port is not open.")

        self._tx_seq += 1
        seq = self._tx_seq

        payload = encode_command_frame(seq=seq, motor_commands=motor_commands)
        self._ser.write(payload)
        return seq

    def read_telemetry(self) -> dict | None:
        """Read and decode one telemetry line if available."""
        if self._ser is None or not self._ser.is_open:
            raise RuntimeError("Serial port is not open.")

        raw = self._ser.readline()
        if not raw:
            return None

        line = safe_decode_line(raw)
        tel = decode_telemetry_line(line)
        if tel is None:
            return None

        tel["host_time_s"] = time.time()
        self._latest_telemetry = tel
        return tel

    def get_latest_telemetry(self) -> dict | None:
        """Return a copy of the most recently decoded telemetry sample."""
        return None if self._latest_telemetry is None else dict(self._latest_telemetry)

    @staticmethod
    def zero_commands() -> dict[str, float]:
        """Return a zero-command dictionary for all supported motor channels."""
        return {motor: 0.0 for motor in MOTOR_FIELDS}
