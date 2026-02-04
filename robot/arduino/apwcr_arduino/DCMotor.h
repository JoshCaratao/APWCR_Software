/*
  DCMotor.h

  Purpose:
  Hardware driver abstraction for a single DC motor controlled by a DRV8871 motor driver.
  This class provides a simple, safe interface for commanding motor speed and direction
  using a signed duty value.

  DRV8871 control model:
  - One PWM pin controls motor speed (0–255)
  - One DIR pin controls motor direction
  - Positive duty rotates the motor in the configured forward direction
  - Negative duty rotates the motor in the reverse direction
  - Duty = 0 stops the motor (coast/brake handled by driver)

  Responsibilities:
  - Convert signed duty commands (-255 to 255) into PWM and DIR pin outputs
  - Apply clamping, optional inversion, and safety stop behavior
  - Be the only code that directly writes motor control pins

  This class does NOT:
  - Run PID control
  - Interpret robot-level commands
  - Know about encoders or kinematics
*/
