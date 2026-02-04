"""
Controller state definitions.

This module defines the enumerated controller states used to represent the
robot's overall control behavior. These states combine manual operation and
autonomous phases into a single finite set.

States defined here are treated as the single source of truth and are shared
across the controller logic, GUI, and debugging tools.

This file should contain definitions only (enums and small helpers).
"""

from enum import Enum
from typing import Optional, Tuple


class ControllerState(str, Enum):
    """
    Single state variable representing both manual control and autonomous phases.

    MANUAL:
        GUI teleop commands drive the robot directly (with a deadman timeout).

    AUTO_*:
        Autonomous control is active. The controller generates commands based on
        perception and internal timers, transitioning through phases.
    """
    # Manual state
    MANUAL = "manual"

    # Autonomous phases
    AUTO_SEARCHING = "auto_searching"
    AUTO_APPROACHING = "auto_approaching"
    AUTO_PICKUP = "auto_pickup"
    AUTO_DEPOSIT = "auto_deposit"


def ui_labels(st: ControllerState) -> Tuple[str, Optional[str]]:
    """
    Convert an internal ControllerState into human-readable labels for the GUI.

    Returns:
        mode_label: "Manual" or "Auto"
        phase_label: Auto phase name, or None if in manual
    """
    if st == ControllerState.MANUAL:
        return "Manual", None
    if st == ControllerState.AUTO_SEARCHING:
        return "Auto", "Searching"
    if st == ControllerState.AUTO_APPROACHING:
        return "Auto", "Approaching"
    if st == ControllerState.AUTO_PICKUP:
        return "Auto", "Pickup"
    if st == ControllerState.AUTO_DEPOSIT:
        return "Auto", "Deposit"
    return "Unknown", None
