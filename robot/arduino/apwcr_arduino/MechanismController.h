/*
  MechanismController.h

  Purpose:
  Declares the controller for the pickup mechanism subsystem. The pickup mechanism includes two DC
  motors and one servo. Python sends setpoints and this controller executes them safely.

  Responsibilities:
  - Accept mechanism motor position targets and servo angle setpoints from Python
  - Run position control for the two mechanism motors if encoders are available
  - Ramp servo angles for smooth motion and reduced mechanical shock
  - Enforce subsystem limits and interlocks to protect hardware
  - Provide status indicating when targets are reached
*/
