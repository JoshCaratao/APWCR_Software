/*
  LidController.h

  Purpose:
  Declares a small controller for the lid mechanism. The lid is actuated by a servo and should be
  controlled independently from the pickup mechanism for clarity and safety.

  Responsibilities:
  - Accept lid servo setpoints from Python
  - Ramp angles smoothly and clamp to safe limits
  - Provide a reached-target status for sequencing
*/
