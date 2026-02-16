"""
pwc_robot/comms/serial_link.py

Serial transport layer for Arduino <-> Laptop communication.

Responsibilities:
- Open/close/reconnect the serial port
- Drain telemetry reads (Arduino -> Laptop)
- Send command frames (Laptop -> Arduino)
- Maintain latest_telemetry and link_stats/state

This module does NOT define the wire format. protocol.py does.

Recommended usage pattern in main:
1) comms.rx_tick() early in the loop
2) controller uses comms.get_latest_telemetry()
3) comms.tx_tick(...) late in the loop
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
        self._last_rx_tick_time_s: Optional[float] = None
        self._last_tx_tick_time_s: Optional[float] = None

        self._last_rx_event_time_s: Optional[float] = None
        self._last_tx_event_time_s: Optional[float] = None

        self._rx_tick_hz_ema: Optional[float] = None
        self._tx_tick_hz_ema: Optional[float] = None
        self._rx_hz_ema: Optional[float] = None
        self._tx_hz_ema: Optional[float] = None

        self._hz_alpha: float = float(comms_cfg.get("hz_alpha", 0.2))

        self._last_reconnect_attempt_s: float = 0.0
        self._tx_seq: int = 0

    # -----------------------------
    # Public API
    # -----------------------------

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self.link_stats.state = LinkState.DISCONNECTED

        # Reset measured rates so UI does not show stale values
        self._last_rx_tick_time_s = None
        self._last_tx_tick_time_s = None
        self._last_rx_event_time_s = None
        self._last_tx_event_time_s = None
        self._rx_tick_hz_ema = None
        self._tx_tick_hz_ema = None
        self._rx_hz_ema = None
        self._tx_hz_ema = None

    def rx_tick(self) -> None:
        """
        Read side tick.
        Call this near the beginning of your loop.

        Behavior:
        - Ensure port open (reconnect if needed)
        - Drain reads (non-blocking feel)
        - Update link state and rx_age
        """
        now_s = time.perf_counter()

        inst = self._event_hz(self._last_rx_tick_time_s, now_s)
        if inst is not None:
            self._rx_tick_hz_ema = self._ema_update(self._rx_tick_hz_ema, inst)
        self._last_rx_tick_time_s = now_s

        if not self.enabled:
            self.close()
            return

        if self._ser is None or not self._ser.is_open:
            self._maybe_reconnect(now_s)

        self._drain_reads(now_s)
        self._update_link_state(now_s)

    def tx_tick(
        self,
        drive_cmd: Optional[DriveCommand],
        mech_cmd: Optional[MechanismCommand],
    ) -> None:
        """
        Write side tick.
        Call this near the end of your loop.

        Behavior:
        - If connected enough to have received at least one telemetry frame,
          send one command frame per call.
        - Update link state after write attempt.
        """
        now_s = time.perf_counter()

        inst = self._event_hz(self._last_tx_tick_time_s, now_s)
        if inst is not None:
            self._tx_tick_hz_ema = self._ema_update(self._tx_tick_hz_ema, inst)
        self._last_tx_tick_time_s = now_s

        if not self.enabled:
            self.close()
            return

        if self._ser is None or not self._ser.is_open:
            self._maybe_reconnect(now_s)

        # Gate TX until we know RX works at least once
        if self.link_stats.last_rx_time_s is None:
            self._update_link_state(now_s)
            return

        if drive_cmd is None or mech_cmd is None:
            self._update_link_state(now_s)
            return

        self._write_command(now_s, drive_cmd, mech_cmd)
        self._update_link_state(now_s)

    def tick(
        self,
        drive_cmd: Optional[DriveCommand],
        mech_cmd: Optional[MechanismCommand],
    ) -> None:
        """
        Backward compatible wrapper.
        Equivalent to: rx_tick(); tx_tick(drive_cmd, mech_cmd)
        """
        self.rx_tick()
        self.tx_tick(drive_cmd, mech_cmd)

    def is_connected(self) -> bool:
        return self.link_stats.state == LinkState.CONNECTED

    def get_latest_telemetry(self) -> Optional[Telemetry]:
        return self.latest_telemetry

    def get_status(self) -> dict:
        last_rx_age_s = None
        if self.link_stats.last_rx_time_s is not None:
            last_rx_age_s = time.perf_counter() - self.link_stats.last_rx_time_s

        return {
            "state": self.link_stats.state.value,
            "port": self.link_stats.port,
            "baud": self.link_stats.baud,
            "last_rx_age_s": last_rx_age_s,
            "rx_tick_hz": self._rx_tick_hz_ema,
            "tx_tick_hz": self._tx_tick_hz_ema,
            "rx_hz": self._rx_hz_ema,
            "tx_hz": self._tx_hz_ema,
            "last_error": self.link_stats.last_error,
            "rx_stale_s": self.rx_stale_s,
            "bytes_rx": self.link_stats.bytes_rx,
            "bytes_tx": self.link_stats.bytes_tx,
        }

    # -----------------------------
    # Connection management
    # -----------------------------

    def _maybe_reconnect(self, now_s: float) -> None:
        """
        Attempt to open the serial port if it is currently closed.

        Behavior:
        - Rate-limited reconnect attempts (reconnect_s)
        - Auto-detect port if enabled and no explicit port given
        - Flush input/output buffers on successful open
        - Discard first partial line (Arduino may reset on connect)
        """
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

            # Arduino commonly resets on connect.
            # Clear boot noise and partial fragments.
            try:
                time.sleep(1.5)
                self._ser.reset_input_buffer()
                self._ser.reset_output_buffer()
                _ = self._ser.readline()
            except Exception:
                pass

            self.link_stats.last_error = None
            self.link_stats.state = LinkState.CONNECTING

        except Exception as e:
            self._ser = None
            self.link_stats.last_error = f"{type(e).__name__}: {e}"
            self.link_stats.state = LinkState.ERROR

    # -----------------------------
    # TX / RX internals
    # -----------------------------

    def _write_command(self, now_s: float, drive: DriveCommand, mech: MechanismCommand) -> None:
        if self._ser is None or not self._ser.is_open:
            return

        self._tx_seq += 1
        seq = self._tx_seq

        # host_time_ms should be wall-clock time for logs
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
            self.link_stats.tx_seq = seq

            inst = self._event_hz(self._last_tx_event_time_s, now_s)
            if inst is not None:
                self._tx_hz_ema = self._ema_update(self._tx_hz_ema, inst)
            self._last_tx_event_time_s = now_s

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

                self.link_stats.bytes_rx += len(raw)

                line = safe_decode_line(raw)
                tel = decode_telemetry_line(line)
                if tel is None:
                    continue

                self.link_stats.last_rx_time_s = now_s

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
