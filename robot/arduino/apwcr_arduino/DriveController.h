/*
  DriveController.h

  Purpose:
  Declares the drive controller for the differential drive base. This controller converts high-level
  drive commands from Python into low-level motor PWM outputs.

  Responsibilities:
  - Accept drive commands from Python (v,w or wheel targets)
  - Compute left and right wheel targets using robot geometry
  - Run speed PID loops using encoder feedback
  - Apply limits, ramps, and watchdog safety rules
  - Command the two drive DCMotor instances
*/
