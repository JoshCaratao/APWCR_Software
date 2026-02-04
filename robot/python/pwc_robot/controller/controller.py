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

from pwc_robot.controller.states import ControllerState
from pwc_robot.controller.states import DriveCommand


class Controller:
    def __init__(self, state: ControllerState = ControllerState.MANUAL) -> None:
        
        # Current controller state (Manual or Autophase). Defaults to Manual if none given
        self.state = state

        