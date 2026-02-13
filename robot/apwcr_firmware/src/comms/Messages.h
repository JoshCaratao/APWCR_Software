#pragma once
#include <Arduino.h>

/*
===============================================================================
  Messages.h
===============================================================================

  PURPOSE
  -------
  Defines command and telemetry data structures exchanged between
  Arduino and laptop over newline-delimited JSON.

  Must mirror:
    pwc_robot/comms/types.py
    pwc_robot/comms/protocol.py

  Notes:
  - Optional numeric fields use NAN and are encoded as JSON null.
  - Field names must match Python exactly.
===============================================================================
*/


/*=============================================================================
  COMMAND STRUCTURES (Laptop -> Arduino)
=============================================================================*/

// "drive": {"linear": <float>, "angular": <float>}
struct DriveCommand {
  float linear_ftps = 0.0f;   // ft/s
  float angular_dps = 0.0f;   // deg/s
};

// Supported mechanism motor modes (matches Python string values)
enum class MechMotorMode : uint8_t {
  UNKNOWN = 0,
  POS_DEG,
  DUTY,
};

// {"mode": "...", "value": <float>} | null
struct MechMotorCommand {
  MechMotorMode mode = MechMotorMode::UNKNOWN;
  float value = 0.0f;
  bool present = false;   // true if object existed and parsed
};

// "mech": {...}
struct MechanismCommand {
  MechMotorCommand motor_RHS;
  MechMotorCommand motor_LHS;

  float servo_LID_deg = 0.0f;
  bool servo_LID_present = false;

  float servo_SWEEP_deg = 0.0f;
  bool servo_SWEEP_present = false;
};

// Full command frame
struct CommandFrame {
  uint32_t seq = 0;
  uint32_t host_time_ms = 0;

  DriveCommand drive;
  MechanismCommand mech;

  bool valid = false;  // set true after successful decode
};


/*=============================================================================
  TELEMETRY STRUCTURES (Arduino -> Laptop)
=============================================================================*/

// {"left_rpm": <float>|null, "right_rpm": <float>|null}
struct WheelState {
  float left_rpm  = NAN;
  float right_rpm = NAN;
};

// {"servo_LID_deg": <float>|null, ...}
struct MechanismState {
  float servo_LID_deg   = NAN;
  float servo_SWEEP_deg = NAN;

  float motor_RHS_deg = NAN;
  float motor_LHS_deg = NAN;
};

// {"distance_in": <float>|null, "valid": <bool>}
struct UltrasonicState {
  float distance_in = NAN;
  bool  valid = false;
};

// Full telemetry frame
struct TelemetryFrame {
  uint32_t arduino_time_ms = 0;
  uint32_t ack_seq = 0;

  WheelState wheel;
  MechanismState mech;
  UltrasonicState ultrasonic;

  const char* note = nullptr;  // optional debug string
};
