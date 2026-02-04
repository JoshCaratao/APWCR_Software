/*
  PID.h

  Purpose:
  Declares a small reusable PID controller class for APWCR. This PID is used for:
  - drive wheel speed control (rad/sec or ticks/sec)
  - mechanism position control (angle or ticks)

  Design goals:
  - Multiple independent PID instances without global variables
  - Explicit dt input so it works with task-based loop timing
  - Output clamping and anti-windup support for safety and stability
*/
