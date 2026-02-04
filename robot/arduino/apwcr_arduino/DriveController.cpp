/*
  DriveController.cpp

  Purpose:
  Implements the differential drive control logic, including kinematics conversion and speed PID.
  This file is where drive behavior is enforced consistently for both manual and autonomous modes.

  Notes:
  - Always stop motors on watchdog timeout or e-stop.
  - Keep kinematics and units consistent with Params.h.
*/
