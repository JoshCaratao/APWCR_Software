"""
High-level robot controller.

(Updated) Adds manual mechanism quick controls from the GUI:
- GUI can send mech: {"servo_LID_deg": <deg>} and/or {"servo_SWEEP_deg": <deg>}
- Manual mode outputs those as the current mechanism command

"""

from __future__ import annotations

import time
import threading
from typing import Any, Dict, Tuple, Optional

from pwc_robot.controller.states import ControllerState
from pwc_robot.controller.commands import (
    DriveCommand,
    DRIVE_STOP,
    MechanismCommand,
    MECH_NOOP,  # your default auto/idle mech intent
    MechMotorCommand,
    MechMotorMode,
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

        # NEW: last GUI-requested mechanism intent (manual quick controls)
        self._user_mech: MechanismCommand = MechanismCommand()
        self._user_mech_ts: float = time.time()

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

            # Clear manual mech intent (no change)
            self._user_mech = MechanismCommand()
            self._user_mech_ts = time.time()

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

            # Clear manual mech intent when entering auto
            self._user_mech = MechanismCommand()
            self._user_mech_ts = time.time()

    def update_user_cmd(self, *, linear: float, angular: float, mech: Optional[Dict[str, Any]] = None) -> None:
        """
        Update GUI teleop intent (only used when in MANUAL).

        mech (optional) supports:
          {"servo_LID_deg": <float|None>, "servo_SWEEP_deg": <float|None>}
        If a mech key is omitted, we leave it unchanged.
        If a mech key is present with None, we clear it to "no change".
        """
        with self._lock:
            if self.state != ControllerState.MANUAL:
                return

            self._user_cmd.linear = float(linear)
            self._user_cmd.angular = float(angular)
            self._user_ts = time.time()

            if isinstance(mech, dict):
                # Only touch keys that were provided
                if "servo_LID_deg" in mech:
                    v = mech.get("servo_LID_deg", None)
                    self._user_mech.servo_LID_deg = (None if v is None else float(v))

                if "servo_SWEEP_deg" in mech:
                    v = mech.get("servo_SWEEP_deg", None)
                    self._user_mech.servo_SWEEP_deg = (None if v is None else float(v))

                # (future) motors, if you add them
                if "motor_RHS" in mech:
                    self._user_mech.motor_RHS = self._parse_motor_cmd(mech.get("motor_RHS"))
                if "motor_LHS" in mech:
                    self._user_mech.motor_LHS = self._parse_motor_cmd(mech.get("motor_LHS"))

                self._user_mech_ts = time.time()
            
            #print("[CONTROLLER] update_user_cmd got mech:", mech)
            #print("[CONTROLLER] state:", self.state)


    def _parse_motor_cmd(self, obj: Any) -> Optional[MechMotorCommand]:
        if obj is None:
            return None
        if not isinstance(obj, dict):
            return None
        mode_s = obj.get("mode", None)
        val = obj.get("value", None)
        if mode_s is None or val is None:
            return None
        try:
            mode = MechMotorMode(str(mode_s))
        except Exception:
            return None
        try:
            fval = float(val)
        except Exception:
            return None
        return MechMotorCommand(mode=mode, value=fval)

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

        return DriveCommand(linear=0.0, angular=self.default_speed_angular), MECH_NOOP

    def _auto_approaching(self, vision_obs: Dict[str, Any]) -> Tuple[DriveCommand, MechanismCommand]:
        now = float(vision_obs.get("timestamp", time.perf_counter()))

        target = vision_obs.get("stable_target")
        frame = vision_obs.get("frame")

        gp_valid = bool(vision_obs.get("target_gp_valid", False))
        gp_fw = vision_obs.get("target_gp_fw_dist", None)

        if target is not None and frame is not None:
            frame_w = int(frame.shape[1])
            frame_h = int(frame.shape[0])

            cx = float(target.get("cx", frame_w * 0.5))
            cy = float(target.get("cy", frame_h * 0.5))

            err_x = self._norm_shift(cx, frame_w, shift=self.x_shift)
            if abs(err_x) < self.deadzone_x:
                err_x = 0.0

            if self.use_ground_plane_range and gp_valid and (gp_fw is not None):
                range_err_ft = float(gp_fw) - self.desired_range_ft
                if abs(range_err_ft) < self.deadzone_range_ft:
                    range_err_ft = 0.0

                err_y = range_err_ft

                with self._lock:
                    self._last_range_ft = float(gp_fw)
                    self._last_range_valid = True
            else:
                pixel_err_y = -self._norm_shift(cy, frame_h, shift=self.y_shift)
                if abs(pixel_err_y) < self.deadzone_y:
                    pixel_err_y = 0.0
                err_y = pixel_err_y
                with self._lock:
                    self._last_range_valid = False

            with self._lock:
                self._last_target_seen_ts = now
                self._last_err_x = err_x
                self._last_err_y = float(err_y)

        with self._lock:
            last_seen = self._last_target_seen_ts
            held_err_x = self._last_err_x
            held_err_y = self._last_err_y
            held_range_valid = self._last_range_valid

        target_recent = (last_seen is not None) and ((now - last_seen) <= self.target_hold_s)

        if not target_recent:
            with self._lock:
                self.state = ControllerState.AUTO_SEARCHING
            return DRIVE_STOP, MECH_NOOP

        lin_done = (held_err_y == 0.0)
        ang_done = (held_err_x == 0.0)

        if ang_done and lin_done:
            with self._lock:
                self.state = ControllerState.AUTO_PICKUP
            return DRIVE_STOP, MECH_NOOP

        angular = self._p_controller(
            err=held_err_x,
            kp=self.kp_ang,
            lo=self.min_speed_angular,
            hi=self.max_speed_angular,
        )

        if self.use_ground_plane_range and held_range_valid:
            linear = self._p_controller(
                err=held_err_y,
                kp=self.kp_lin_ft,
                lo=self.min_speed_linear,
                hi=self.max_speed_linear,
            )
        else:
            linear = self._p_controller(
                err=held_err_y,
                kp=self.kp_lin_pixel,
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

        if self._ultra_blocked:
            if d >= (self.ultrasonic_stop_in + self.ultrasonic_release_in):
                self._ultra_blocked = False
        else:
            if d <= self.ultrasonic_stop_in:
                self._ultra_blocked = True

        if self._ultra_blocked:
            drive = DriveCommand(linear=drive.linear, angular=drive.angular)
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

                # Keep last manual mechanism intent latched
                manual_mech = self._user_mech

            # Deadman applies to drive only
            drive_out = DRIVE_STOP if cmd_age > self.deadman_s else cmd
            drive_out = self._apply_ultrasonic_gate(drive_out, telemetry)

            # Mech stays latched until GUI sends a new mech command
            mech_out = manual_mech

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

        with self._lock:
            self.state = ControllerState.AUTO_SEARCHING
            self._last_drive_cmd = DRIVE_STOP
            self._last_mech_cmd = MECH_NOOP
        return DRIVE_STOP, MECH_NOOP
