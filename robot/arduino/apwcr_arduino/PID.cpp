/*
  PID.cpp

  Purpose:
  Implements the PID controller used by APWCR. This file contains the PID update math and any
  anti-windup or clamping behavior.

  Notes:
  - Keep derivative handling stable (often derivative on measurement is less noisy).
  - reset() should be called when switching modes or disabling motors.
*/
