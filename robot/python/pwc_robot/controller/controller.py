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
drive controller or Arduino interface, keeping decision-making logic
separate from hardware-specific implementation details.
"""

from __future__ import annotations

import time
import threading
from typing import Any, Dict

from pwc_robot.controller.states import ControllerState
from pwc_robot.controller.commands import DriveCommand, DRIVE_STOP


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
            kp_ang: float = 20,
            kp_lin: float = 0.5,
            deadzone_x: float = 0.05,
            deadzone_y: float = 0.05,
            x_shift: float = 0.5,
            y_shift: float = 0.7
        ) -> None:
        
        self._lock = threading.Lock()
        
        # Current controller state (Manual or Autophase). Defaults to Manual if none given
        self.state = state

        # Last GUI-Requested Command (user input) + timestamp for deadman stop
        self._user_cmd: DriveCommand = DriveCommand()
        self._user_ts: float = time.time()

        # Safety (set in config file)
        self.deadman_s = float(deadman_s)

        # Simple timers for auto placeholders
        self._pickup_start_ts: float | None = None
        self._deposit_start_ts: float | None = None

        # Speeds (set in config file)
        self.default_speed_linear = float(default_speed_linear)
        self.default_speed_angular = float(default_speed_angular)
        self.max_speed_linear = float(max_speed_linear)
        self.max_speed_angular = float(max_speed_angular)
        self.min_speed_linear = float(min_speed_linear)
        self.min_speed_angular = float(min_speed_angular)

        # Target Hold Behavior
        self.target_hold_s = float(target_hold_s)  # how long we tolerate target dropouts
        self._last_target_seen_ts: float | None = None
        self._last_err_x: float = 0.0
        self._last_err_y: float = 0.0

        # Control Behavior
        self.kp_ang = kp_ang
        self.kp_lin = kp_lin
        self.deadzone_x = deadzone_x
        self.deadzone_y = deadzone_y
        self.x_shift = x_shift
        self.y_shift = y_shift

        # For Flask UI
        self._last_cmd = DriveCommand()

    

    
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

    def set_auto(self) -> None:
        """Switch to AUTO starting in searching."""
        with self._lock:
            self.state = ControllerState.AUTO_SEARCHING
            self._pickup_start_ts = None
            self._deposit_start_ts = None

    def update_user_cmd(self, linear: float, angular: float) -> None:
        """Update GUI teleop intent (only used when in MANUAL)."""
        with self._lock:
            if self.state == ControllerState.MANUAL:
                self._user_cmd.linear = float(linear)
                self._user_cmd.angular = float(angular)
                self._user_ts = time.time()
            else:
                return

    def get_state(self) -> ControllerState:
        with self._lock:
            return self.state
    

    # --------------------
    # Auto phase handlers
    # --------------------
    def _auto_searching(self, vision_obs: Dict[str, Any]) -> DriveCommand:
        stable = bool(vision_obs.get("stable_detected"))

        #If stable target found, switch to approach
        if stable:
            with self._lock:
                self.state = ControllerState.AUTO_APPROACHING
            return DRIVE_STOP

        # If no stable target, Rotate slowly to scan
        return DriveCommand(linear=0.0, angular=self.default_speed_angular)
    
    def _auto_approaching(self, vision_obs: Dict[str, Any]) -> DriveCommand:
        now = float(vision_obs.get("timestamp", time.perf_counter()))

        # Get Target data from computer_vision output
        target = vision_obs.get("stable_target")
        frame = vision_obs.get("frame")
        frame_w = None
        frame_h = None


        # If we have a fresh target and frame, update last seen and last error
        if target is not None and frame is not None:
            frame_w = int(frame.shape[1])
            frame_h = int(frame.shape[0])

            cx = float(target.get("cx", frame_w * 0.5))
            cy = float(target.get("cy", frame_h * 0.5))

            # X error: normalized and shifted so 0.0 means "at x_shift"
            # Positive -> target is to the right of aimpoint
            err_x = self._norm_shift(cx, frame_w, shift=self.x_shift)

            # Y error: we want to drive forward until target reaches y_shift (below center)
            # Define err_y = y_shift - (cy/h). Positive -> target is above desired y, so move forward.
            err_y = -self._norm_shift(cy, frame_h, shift=self.y_shift)


            # apply deadzone to reduce jitter near center
            if abs(err_x) < self.deadzone_x:
                err_x = 0.0
            if abs(err_y) < self.deadzone_y:
                err_y = 0.0

            # Update Last seen target time
            with self._lock:
                self._last_target_seen_ts = now
                self._last_err_x = err_x
                self._last_err_y = err_y
            
        # Decide if target is "recent enough" to keep approaching by reading held values for dropout tolerance
        with self._lock:
            last_seen = self._last_target_seen_ts
            held_err_x = self._last_err_x
            held_err_y = self._last_err_y
        
        # boolean to check if target is recent
        target_recent = (last_seen is not None) and ((now - last_seen) <= self.target_hold_s)

        # If Target lost for longer than hold window, go back to searching
        if not target_recent:
            with self._lock:
                self.state = ControllerState.AUTO_SEARCHING
            return DRIVE_STOP
        
        # Approach complete when both axes are inside deadzones, switch to AUTO_PICKUP
        if held_err_x == 0.0 and held_err_y == 0.0:
            with self._lock:
                self.state = ControllerState.AUTO_PICKUP
            return DRIVE_STOP
        
        # If Target IS recent, use Proportional control for commands. Calculate Proportional Control command and clamp at same time
        angular = self._p_controller(err = held_err_x, kp = self.kp_ang, lo = self.min_speed_angular, hi = self.max_speed_angular)
        linear = self._p_controller(err = held_err_y, kp = self.kp_lin, lo = self.min_speed_linear, hi = self.max_speed_linear)

        return DriveCommand(linear = linear, angular = angular)
    
    def _auto_pickup(self):
        return DRIVE_STOP
    
    def _auto_deposit(self):
        return  DRIVE_STOP
    

    # -------
    # Helpers
    # -------

    def _norm_shift(self, pixel:float, res: int, shift: float) -> float:
        """
        Normalize a pixel coordinate to a centered, unitless error.

        Args:
            pixel: pixel coordinate (e.g., cx or cy)
            res: full size of that axis in pixels (width or height)

        Returns:
            Centered normalized error in range [-0.5, +0.5]
            where 0.0 means centered.
        """
        if res <= 0:
            return 0.0
        
        norm = pixel / res
        norm_shifted = norm - shift
        return norm_shifted
    
    def _clamp(self, x: float, lo: float, hi: float) -> float:
        """
        Clamp Speed Commands so they dont exceed specified minimums or maximums

        Args:
            x: proposed value
            lo: minimum acceptable value
            hi: maximum acceptable value

        Returns:
            returns new clamped value
        """
        if x < lo:
            return lo
        if x > hi:
            return hi
        return x
    
    def _p_controller(self, err: float, kp: float, lo: float, hi: float) -> float:
        return self._clamp(kp*err, lo, hi)
    

    # -----------------------------
    # Flask/Communication Functions
    # -----------------------------
    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "state": self.state.name,
                "deadman_s": self.deadman_s,
                "target_hold_s": self.target_hold_s,
            }

    def get_last_cmd(self) -> Dict[str, float]:
        # store this each tick as self._last_cmd
        with self._lock:
            return {"linear": float(self._last_cmd.linear), "angular": float(self._last_cmd.angular)}

    def set_mode(self, mode: str) -> None:
        mode = mode.strip().lower()
        if mode == "manual":
            self.set_manual()
        elif mode == "auto":
            self.set_auto()
        else:
            raise ValueError("mode must be 'manual' or 'auto'")

    # --------------------
    # Internal ticks
    # --------------------
    def tick(self, vision_obs: dict) -> DriveCommand:
        with self._lock:
            st = self.state

        if st == ControllerState.MANUAL:
            with self._lock:
                cmd_age = time.time() - self._user_ts
                cmd = DriveCommand(self._user_cmd.linear, self._user_cmd.angular)
            out = DRIVE_STOP if cmd_age > self.deadman_s else cmd

            with self._lock:
                self._last_cmd = out
            return out

        if st == ControllerState.AUTO_SEARCHING:
            out = self._auto_searching(vision_obs)
            with self._lock:
                self._last_cmd = out
            return out

        if st == ControllerState.AUTO_APPROACHING:
            out = self._auto_approaching(vision_obs)
            with self._lock:
                self._last_cmd = out
            return out

        if st == ControllerState.AUTO_PICKUP:
            out = self._auto_pickup()
            with self._lock:
                self._last_cmd = out
            return out

        if st == ControllerState.AUTO_DEPOSIT:
            out = self._auto_deposit()
            with self._lock:
                self._last_cmd = out
            return out

        # Fallback safety
        with self._lock:
            self.state = ControllerState.AUTO_SEARCHING
            self._last_cmd = DRIVE_STOP
        return DRIVE_STOP


            






    


        