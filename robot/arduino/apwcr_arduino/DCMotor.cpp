/*
  DCMotor.cpp

  Purpose:
  Implements the DCMotor hardware driver. This file is the only place that should write the motor
  driver pins for the motor channel it owns.

  Notes:
  - Keep this code simple and predictable.
  - Controllers call setDuty() at a fixed rate.
*/
