"""
High-level robot controller.

This module implements the primary control logic for the robot, acting as
the single authority that decides what motion commands should be sent to
the hardware at any given time.

The Controller class:
- Maintains the current control state (manual or autonomous sub-states)
- Accepts manual drive commands from the GUI when in manual mode
- Generates autonomous drive commands based on perception input when in auto
- Implements state transitions for the autonomous behavior
- Enforces safety behavior such as deadman timeouts for manual control

This controller is designed to be called periodically from the main control
loop. It produces high-level motion commands that are then passed to the
actuator layer (serial/Arduino interface), keeping decision-making logic
separate from hardware-specific implementation details.
"""

from __future__ import annotations

import time
import threading
from typing import Any, Dict, Tuple

from pwc_robot.controller.states import ControllerState
from pwc_robot.controller.commands import (
    DriveCommand,
    DRIVE_STOP,
    MechanismCommand,
    MECH_NOOP,  # In your project, MECH_NOOP should hold LHS motor at 0 deg
)
from pwc_robot.comms.types import Telemetry



class Controller:
    def __init__(
        self,
        state: ControllerState = ControllerState.MANUAL,
        deadman_s: float = 0.25,
        default_speed_linear: float = 0.5,
        default_speed_angular: float = 10,
        max_speed_linear: float = 1,
        max_speed_angular: float = 15,
        min_speed_linear: float = -1,
        min_speed_angular: float = -15,
        target_hold_s: float = 0.5,

        # --- Approach config (matches YAML) ---
        kp_ang: float = 20,
        deadzone_x: float = 0.075,
        x_shift: float = 0.5,

        use_ground_plane_range: bool = True,
        desired_range_ft: float = 0.5,
        kp_lin_ft: float = 1.0,
        deadzone_range_ft: float = 0.10,

        kp_lin_pixel: float = 1.0,      # fallback/debug
        deadzone_y: float = 0.03,
        y_shift: float = 0.85,

        # --- Ultrasonic safety gate ---
        ultrasonic_enabled: bool = True,
        ultrasonic_stop_in: float = 12.0,
        ultrasonic_release_in: float = 3.0,
        ultrasonic_stale_s: float = 0.40,

    ) -> None:

        

        self._lock = threading.Lock()

        # Current controller state (Manual or Autophase)
        self.state = state

        # Last GUI-Requested drive command + timestamp for deadman stop
        self._user_cmd: DriveCommand = DriveCommand()
        self._user_ts: float = time.time()

        # Safety
        self.deadman_s = float(deadman_s)

        # Simple timers for auto placeholders
        self._pickup_start_ts: float | None = None
        self._deposit_start_ts: float | None = None

        # Speeds
        self.default_speed_linear = float(default_speed_linear)
        self.default_speed_angular = float(default_speed_angular)
        self.max_speed_linear = float(max_speed_linear)
        self.max_speed_angular = float(max_speed_angular)
        self.min_speed_linear = float(min_speed_linear)
        self.min_speed_angular = float(min_speed_angular)

        # Target hold behavior
        self.target_hold_s = float(target_hold_s)
        self._last_target_seen_ts: float | None = None
        self._last_err_x: float = 0.0
        self._last_err_y: float = 0.0
        self._last_range_ft: float | None = None
        self._last_range_valid: bool = False


        # Control behavior (angular always pixel-x)
        self.kp_ang = float(kp_ang)
        self.deadzone_x = float(deadzone_x)
        self.x_shift = float(x_shift)

        # Linear control: ground-plane primary + pixel-y fallback
        self.use_ground_plane_range = bool(use_ground_plane_range)
        self.desired_range_ft = float(desired_range_ft)
        self.kp_lin_ft = float(kp_lin_ft)
        self.deadzone_range_ft = float(deadzone_range_ft)

        self.kp_lin_pixel = float(kp_lin_pixel)
        self.deadzone_y = float(deadzone_y)
        self.y_shift = float(y_shift)

        # For Flask UI
        self._last_drive_cmd: DriveCommand = DRIVE_STOP
        self._last_mech_cmd: MechanismCommand = MECH_NOOP

        # Ultrasonic safety gate
        self.ultrasonic_enabled = bool(ultrasonic_enabled)
        self.ultrasonic_stop_in = float(ultrasonic_stop_in)
        self.ultrasonic_release_in = float(ultrasonic_release_in)
        self.ultrasonic_stale_s = float(ultrasonic_stale_s)

        self._ultra_blocked: bool = False
        self._last_ultra_in: float | None = None
        self._last_ultra_valid: bool = False


    # --------------------
    # GUI-facing methods
    # --------------------
    def set_manual(self) -> None:
        """Switch to MANUAL and stop immediately."""
        with self._lock:
            self.state = ControllerState.MANUAL
            self._user_cmd = DriveCommand()
            self._user_ts = time.time()
            self._pickup_start_ts = None
            self._deposit_start_ts = None
            self._ultra_blocked = False


    def set_auto(self) -> None:
        """Switch to AUTO starting in searching."""
        with self._lock:
            self.state = ControllerState.AUTO_SEARCHING
            self._pickup_start_ts = None
            self._deposit_start_ts = None
            self._ultra_blocked = False


    def update_user_cmd(self, linear: float, angular: float) -> None:
        """Update GUI teleop intent (only used when in MANUAL)."""
        with self._lock:
            if self.state != ControllerState.MANUAL:
                return
            self._user_cmd.linear = float(linear)
            self._user_cmd.angular = float(angular)
            self._user_ts = time.time()

    def get_state(self) -> ControllerState:
        with self._lock:
            return self.state

    # --------------------
    # Auto phase handlers
    # --------------------
    def _auto_searching(self, vision_obs: Dict[str, Any]) -> Tuple[DriveCommand, MechanismCommand]:
        stable = bool(vision_obs.get("stable_detected"))

        if stable:
            with self._lock:
                self.state = ControllerState.AUTO_APPROACHING
            return DRIVE_STOP, MECH_NOOP

        # Rotate slowly to scan
        return DriveCommand(linear=0.0, angular=self.default_speed_angular), MECH_NOOP

    def _auto_approaching(self, vision_obs: Dict[str, Any]) -> Tuple[DriveCommand, MechanismCommand]:
        now = float(vision_obs.get("timestamp", time.perf_counter()))

        target = vision_obs.get("stable_target")
        frame = vision_obs.get("frame")

        # Ground-plane data from ComputerVision
        gp_valid = bool(vision_obs.get("target_gp_valid", False))
        gp_fw = vision_obs.get("target_gp_fw_dist", None)  # forward distance in ft

        # If we have a fresh target and frame, update last seen and last errors
        if target is not None and frame is not None:
            frame_w = int(frame.shape[1])
            frame_h = int(frame.shape[0])

            cx = float(target.get("cx", frame_w * 0.5))
            cy = float(target.get("cy", frame_h * 0.5))

            # --- Angular error (pixel-x), unchanged ---
            err_x = self._norm_shift(cx, frame_w, shift=self.x_shift)
            if abs(err_x) < self.deadzone_x:
                err_x = 0.0

            # --- Linear error: prefer ground-plane range if enabled + valid ---
            err_y = None  # compute and store a float

            if self.use_ground_plane_range and gp_valid and (gp_fw is not None):
                # Positive error => target farther than desired => drive forward
                range_err_ft = float(gp_fw) - self.desired_range_ft
                if abs(range_err_ft) < self.deadzone_range_ft:
                    range_err_ft = 0.0

                err_y = range_err_ft  # err_y is in FEET, not normalized pixels

                with self._lock:
                    self._last_range_ft = float(gp_fw)
                    self._last_range_valid = True

            else:
                # Fallback to your existing pixel-y approach error
                # (kept exactly the same sign convention you had)
                pixel_err_y = -self._norm_shift(cy, frame_h, shift=self.y_shift)
                if abs(pixel_err_y) < self.deadzone_y:
                    pixel_err_y = 0.0

                err_y = pixel_err_y

                with self._lock:
                    self._last_range_valid = False  # last range not updated this tick

            with self._lock:
                self._last_target_seen_ts = now
                self._last_err_x = err_x
                self._last_err_y = float(err_y)

        # Read held values for dropout tolerance
        with self._lock:
            last_seen = self._last_target_seen_ts
            held_err_x = self._last_err_x
            held_err_y = self._last_err_y
            held_range_valid = self._last_range_valid

        target_recent = (last_seen is not None) and ((now - last_seen) <= self.target_hold_s)

        # If target lost too long, go back to searching
        if not target_recent:
            with self._lock:
                self.state = ControllerState.AUTO_SEARCHING
            return DRIVE_STOP, MECH_NOOP

        # Approach complete:
        # - Always require angular centered
        # - For linear, if GP is active+valid, require range error zero (feet)
        # - Otherwise use pixel-y zero
        lin_done = (held_err_y == 0.0)
        ang_done = (held_err_x == 0.0)

        if ang_done and lin_done:
            with self._lock:
                self.state = ControllerState.AUTO_PICKUP
            return DRIVE_STOP, MECH_NOOP

        # Angular command (pixel-based)
        angular = self._p_controller(
            err=held_err_x,
            kp=self.kp_ang,
            lo=self.min_speed_angular,
            hi=self.max_speed_angular,
        )

        # Linear command depends on what held_err_y represents
        if self.use_ground_plane_range and held_range_valid:
            # held_err_y is in feet
            linear = self._p_controller(
                err=held_err_y,
                kp=self.kp_lin_ft,          # ft/s per ft
                lo=self.min_speed_linear,
                hi=self.max_speed_linear,
            )
        else:
            # held_err_y is normalized pixel error
            linear = self._p_controller(
                err=held_err_y,
                kp=self.kp_lin_pixel,       # ft/s per normalized error
                lo=self.min_speed_linear,
                hi=self.max_speed_linear,
            )

        return DriveCommand(linear=linear, angular=angular), MECH_NOOP


    def _auto_pickup(self) -> Tuple[DriveCommand, MechanismCommand]:
        return DRIVE_STOP, MECH_NOOP

    def _auto_deposit(self) -> Tuple[DriveCommand, MechanismCommand]:
        return DRIVE_STOP, MECH_NOOP

    # -------
    # Helpers
    # -------
    def _norm_shift(self, pixel: float, res: int, shift: float) -> float:
        if res <= 0:
            return 0.0
        norm = pixel / res
        return norm - shift

    def _clamp(self, x: float, lo: float, hi: float) -> float:
        if x < lo:
            return lo
        if x > hi:
            return hi
        return x

    def _p_controller(self, err: float, kp: float, lo: float, hi: float) -> float:
        return self._clamp(kp * err, lo, hi)
    
    
    def _apply_ultrasonic_gate(self, drive: DriveCommand, telemetry: Telemetry | None) -> DriveCommand:

        if not self.ultrasonic_enabled:
            return drive

        if telemetry is None:
            return drive

        if telemetry.rx_age_s is not None and telemetry.rx_age_s > self.ultrasonic_stale_s:
            with self._lock:
                self._last_ultra_valid = False
                self._last_ultra_in = None
                self._ultra_blocked = False
            return drive


        u = telemetry.ultrasonic
        if u is None or (not u.valid) or (u.distance_in is None):
            with self._lock:
                self._last_ultra_valid = False
                self._last_ultra_in = None
                self._ultra_blocked = False
            return drive

        d = float(u.distance_in)
        with self._lock:
            self._last_ultra_valid = True
            self._last_ultra_in = d

        # Hysteresis latch
        if self._ultra_blocked:
            if d >= (self.ultrasonic_stop_in + self.ultrasonic_release_in):
                self._ultra_blocked = False
        else:
            if d <= self.ultrasonic_stop_in:
                self._ultra_blocked = True

        if self._ultra_blocked:
            drive = DriveCommand(linear=drive.linear, angular=drive.angular)
            # Block forward only, allow reverse and turning
            drive.linear = min(drive.linear, 0.0)

        return drive


    # -----------------------------
    # Flask/Communication Functions
    # -----------------------------
    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "state": self.state.name,
                "deadman_s": self.deadman_s,
                "target_hold_s": self.target_hold_s,
                "ultrasonic": {
                    "enabled": self.ultrasonic_enabled,
                    "blocked": self._ultra_blocked,
                    "distance_in": self._last_ultra_in,
                    "valid": self._last_ultra_valid,
                    "stop_in": self.ultrasonic_stop_in,
                    "release_in": self.ultrasonic_release_in,
                    "stale_s": self.ultrasonic_stale_s,
                },
            }

    def get_last_cmd(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "drive": {
                    "linear": float(self._last_drive_cmd.linear),
                    "angular": float(self._last_drive_cmd.angular),
                },
                "mech": {
                    "motor_RHS": None if self._last_mech_cmd.motor_RHS is None else {
                        "mode": self._last_mech_cmd.motor_RHS.mode.value,
                        "value": float(self._last_mech_cmd.motor_RHS.value),
                    },
                    "motor_LHS": None if self._last_mech_cmd.motor_LHS is None else {
                        "mode": self._last_mech_cmd.motor_LHS.mode.value,
                        "value": float(self._last_mech_cmd.motor_LHS.value),
                    },
                    "servo_LID_deg": self._last_mech_cmd.servo_LID_deg,
                    "servo_SWEEP_deg": self._last_mech_cmd.servo_SWEEP_deg,
                },
            }

    def set_mode(self, mode: str) -> None:
        mode = mode.strip().lower()
        if mode == "manual":
            self.set_manual()
        elif mode == "auto":
            self.set_auto()
        else:
            raise ValueError("mode must be 'manual' or 'auto'")

    # --------------------
    # Internal tick
    # --------------------
    def tick(self, vision_obs: dict, telemetry: Telemetry | None = None) -> Tuple[DriveCommand, MechanismCommand]:
        with self._lock:
            st = self.state

        if st == ControllerState.MANUAL:
            with self._lock:
                cmd_age = time.time() - self._user_ts
                cmd = DriveCommand(self._user_cmd.linear, self._user_cmd.angular)

            drive_out = DRIVE_STOP if cmd_age > self.deadman_s else cmd
            drive_out = self._apply_ultrasonic_gate(drive_out, telemetry)
            mech_out = MECH_NOOP


            with self._lock:
                self._last_drive_cmd = drive_out
                self._last_mech_cmd = mech_out
            return drive_out, mech_out

        if st == ControllerState.AUTO_SEARCHING:
            drive_out, mech_out = self._auto_searching(vision_obs)
            drive_out = self._apply_ultrasonic_gate(drive_out, telemetry)

            with self._lock:
                self._last_drive_cmd = drive_out
                self._last_mech_cmd = mech_out
            return drive_out, mech_out

        if st == ControllerState.AUTO_APPROACHING:
            drive_out, mech_out = self._auto_approaching(vision_obs)
            drive_out = self._apply_ultrasonic_gate(drive_out, telemetry)

            with self._lock:
                self._last_drive_cmd = drive_out
                self._last_mech_cmd = mech_out
            return drive_out, mech_out

        if st == ControllerState.AUTO_PICKUP:
            drive_out, mech_out = self._auto_pickup()
            drive_out = self._apply_ultrasonic_gate(drive_out, telemetry)

            with self._lock:
                self._last_drive_cmd = drive_out
                self._last_mech_cmd = mech_out
            return drive_out, mech_out

        if st == ControllerState.AUTO_DEPOSIT:
            drive_out, mech_out = self._auto_deposit()
            drive_out = self._apply_ultrasonic_gate(drive_out, telemetry)

            with self._lock:
                self._last_drive_cmd = drive_out
                self._last_mech_cmd = mech_out
            return drive_out, mech_out

        # Fallback safety
        with self._lock:
            self.state = ControllerState.AUTO_SEARCHING
            self._last_drive_cmd = DRIVE_STOP
            self._last_mech_cmd = MECH_NOOP
        return DRIVE_STOP, MECH_NOOP
