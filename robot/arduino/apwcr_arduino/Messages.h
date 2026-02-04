/*
  Messages.h

  Purpose:
  Defines the command and telemetry data structures used across the Arduino codebase. This provides a
  single source of truth for what the Arduino expects from Python and what Arduino reports back.

  Typical contents:
  - Command structs for drive (v,w or wheel speeds), pickup mechanism setpoints, lid setpoints
  - Enable and e-stop flags
  - Telemetry struct with encoder counts/speeds, ultrasonic distance, controller status, watchdog state
*/
