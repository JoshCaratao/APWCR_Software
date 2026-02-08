"""
pwc_robot/comms/serial_link.py

Serial transport layer for Arduino <-> Laptop communication.

Responsibilities:
- Open/close/reconnect the serial port
- Drain telemetry reads every tick (Arduino -> Laptop)
- Send one full command frame per tick (Laptop -> Arduino), controlled by main loop Rate
- Maintain latest_telemetry and link_stats/state

This module does NOT define the wire format. protocol.py does.
"""

from __future__ import annotations

import time
from typing import Optional

import serial  # pyserial

from pwc_robot.controller.commands import DriveCommand, MechanismCommand
from pwc_robot.comms.ports import find_arduino_port
from pwc_robot.comms.protocol import (
    encode_command_frame,
    decode_telemetry_line,
    safe_decode_line,
)
from pwc_robot.comms.types import LinkState, LinkStats, Telemetry


class SerialLink:
    def __init__(self, comms_cfg: dict) -> None:
        self.enabled: bool = bool(comms_cfg.get("comms_enabled", True))
        self.port: Optional[str] = comms_cfg.get("port", None)
        self.baud: int = int(comms_cfg.get("baud", 115200))
        self.timeout_s: float = float(comms_cfg.get("timeout_s", 0.05))
        self.write_timeout_s: float = float(comms_cfg.get("write_timeout_s", 0.05))
        self.auto_detect: bool = bool(comms_cfg.get("auto_detect", True))

        self.rx_stale_s: float = float(comms_cfg.get("rx_stale_s", 0.5))
        self.reconnect_s: float = float(comms_cfg.get("reconnect_s", 1.0))

        self._ser: Optional[serial.Serial] = None

        self.latest_telemetry: Optional[Telemetry] = None
        self.link_stats: LinkStats = LinkStats(
            state=LinkState.DISCONNECTED,
            port=self.port,
            baud=self.baud,
        )

        # -----------------------------
        # Measured rates (EMA)
        # -----------------------------
        self._last_tick_time_s: Optional[float] = None
        self._last_rx_event_time_s: Optional[float] = None
        self._last_tx_event_time_s: Optional[float] = None
        self._tick_hz_ema: Optional[float] = None
        self._rx_hz_ema: Optional[float] = None
        self._tx_hz_ema: Optional[float] = None
        self._hz_alpha: float = float(comms_cfg.get("hz_alpha", 0.2))

        self._last_reconnect_attempt_s: float = 0.0
        self._tx_seq: int = 0

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self.link_stats.state = LinkState.DISCONNECTED

        # Reset measured rates so they don't show stale values after disconnect
        self._last_tick_time_s = None
        self._last_rx_event_time_s = None
        self._last_tx_event_time_s = None
        self._tick_hz_ema = None
        self._rx_hz_ema = None
        self._tx_hz_ema = None

    def tick(
        self,
        drive_cmd: Optional[DriveCommand],
        mech_cmd: Optional[MechanismCommand],
    ) -> None:
        """
        Called at a fixed rate by main (comms_hz from YAML).

        Behavior:
        - Ensure port open (reconnect if needed)
        - Drain reads every tick
        - Write one command frame per tick if commands provided
        - Update link state every tick
        """
        now_s = time.perf_counter()

        # Measured comms tick rate
        inst = self._event_hz(self._last_tick_time_s, now_s)
        if inst is not None:
            self._tick_hz_ema = self._ema_update(self._tick_hz_ema, inst)
        self._last_tick_time_s = now_s

        if not self.enabled:
            self.close()
            return

        if self._ser is None or not self._ser.is_open:
            self._maybe_reconnect(now_s)

        self._drain_reads(now_s)

        if drive_cmd is not None and mech_cmd is not None:
            self._write_command(now_s, drive_cmd, mech_cmd)

        self._update_link_state(now_s)

    # -----------------------------
    # Connection management
    # -----------------------------

    def _maybe_reconnect(self, now_s: float) -> None:
        if (now_s - self._last_reconnect_attempt_s) < self.reconnect_s:
            return

        self._last_reconnect_attempt_s = now_s
        self.link_stats.state = LinkState.CONNECTING

        port = self.port
        if port is None and self.auto_detect:
            port = find_arduino_port()

        self.link_stats.port = port
        self.link_stats.baud = self.baud

        if port is None:
            self.link_stats.state = LinkState.DISCONNECTED
            return

        try:
            self._ser = serial.Serial(
                port=port,
                baudrate=self.baud,
                timeout=self.timeout_s,
                write_timeout=self.write_timeout_s,
            )
            self.link_stats.last_error = None
            self.link_stats.state = LinkState.CONNECTING
        except Exception as e:
            self._ser = None
            self.link_stats.last_error = f"{type(e).__name__}: {e}"
            self.link_stats.state = LinkState.ERROR

    # -----------------------------
    # TX / RX
    # -----------------------------

    def _write_command(self, now_s: float, drive: DriveCommand, mech: MechanismCommand) -> None:
        if self._ser is None or not self._ser.is_open:
            return

        self._tx_seq += 1
        seq = self._tx_seq

        # IMPORTANT:
        # - now_s is perf_counter() (monotonic, not wall clock)
        # - host_time_ms should be a real wall-clock timestamp if you ever log it or compare across devices
        host_time_ms = int(time.time() * 1000.0)

        payload = encode_command_frame(
            seq=seq,
            host_time_ms=host_time_ms,
            drive=drive,
            mech=mech,
        )

        try:
            n = self._ser.write(payload)
            self.link_stats.bytes_tx += int(n)
            self.link_stats.last_tx_time_s = now_s

            # Measured TX rate (successful writes)
            inst = self._event_hz(self._last_tx_event_time_s, now_s)
            if inst is not None:
                self._tx_hz_ema = self._ema_update(self._tx_hz_ema, inst)
            self._last_tx_event_time_s = now_s

            self.link_stats.tx_seq = seq
        except Exception as e:
            self.link_stats.last_error = f"{type(e).__name__}: {e}"
            self._handle_serial_error()

    def _drain_reads(self, now_s: float) -> None:
        if self._ser is None or not self._ser.is_open:
            return

        try:
            while True:
                waiting = getattr(self._ser, "in_waiting", 0)
                if not waiting or waiting <= 0:
                    break

                raw = self._ser.readline()
                if not raw:
                    break

                line = safe_decode_line(raw)
                tel = decode_telemetry_line(line)
                if tel is None:
                    # ignore junk lines, do not update last_rx_time_s
                    continue

                # Valid telemetry frame
                self.link_stats.bytes_rx += len(raw)
                self.link_stats.last_rx_time_s = now_s

                # Measured RX rate (valid telemetry frames)
                inst = self._event_hz(self._last_rx_event_time_s, now_s)
                if inst is not None:
                    self._rx_hz_ema = self._ema_update(self._rx_hz_ema, inst)
                self._last_rx_event_time_s = now_s

                tel.host_rx_time_s = now_s
                self.latest_telemetry = tel
                self.link_stats.last_ack_seq = tel.ack_seq

        except Exception as e:
            self.link_stats.last_error = f"{type(e).__name__}: {e}"
            self._handle_serial_error()

    # -----------------------------
    # State / health
    # -----------------------------

    def _update_link_state(self, now_s: float) -> None:
        if self._ser is None or not self._ser.is_open:
            self.link_stats.state = LinkState.ERROR if self.link_stats.last_error else LinkState.DISCONNECTED
            return

        if self.link_stats.last_rx_time_s is None:
            self.link_stats.state = LinkState.CONNECTING
            return

        age = now_s - self.link_stats.last_rx_time_s
        if self.latest_telemetry is not None:
            self.latest_telemetry.rx_age_s = age

        self.link_stats.state = LinkState.STALE if age > self.rx_stale_s else LinkState.CONNECTED
        if self.link_stats.state == LinkState.CONNECTED:
            self.link_stats.last_ok_time_s = now_s

    def _handle_serial_error(self) -> None:
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self.link_stats.state = LinkState.ERROR

    def is_connected(self) -> bool:
        return self.link_stats.state == LinkState.CONNECTED

    def get_status(self) -> dict:
        # Keep it JSON friendly
        last_rx_age_s = None
        if self.link_stats.last_rx_time_s is not None:
            last_rx_age_s = time.perf_counter() - self.link_stats.last_rx_time_s

        return {
            # LinkState is str+Enum, so .value is a clean stable string ("CONNECTED", etc.)
            "state": self.link_stats.state.value,
            "port": self.link_stats.port,
            "baud": self.link_stats.baud,
            "last_rx_age_s": last_rx_age_s,
            "tick_hz": self._tick_hz_ema,
            "rx_hz": self._rx_hz_ema,
            "tx_hz": self._tx_hz_ema,
            "last_error": self.link_stats.last_error,
            "rx_stale_s": self.rx_stale_s,
        }

    def get_latest_telemetry(self) -> Optional[Telemetry]:
        return self.latest_telemetry

    # -------------------
    # Rate (Hz) helpers
    # -------------------
    def _ema_update(self, prev: Optional[float], new: float) -> float:
        if prev is None:
            return new
        a = self._hz_alpha
        return (1.0 - a) * prev + a * new

    def _event_hz(self, last_time_s: Optional[float], now_s: float) -> Optional[float]:
        if last_time_s is None:
            return None
        dt = now_s - last_time_s
        if dt <= 1e-6:
            return None
        return 1.0 / dt
