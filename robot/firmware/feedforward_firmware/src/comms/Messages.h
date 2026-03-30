#pragma once
#include <Arduino.h>

/*
===============================================================================
  Messages.h
===============================================================================

  Purpose:
  Defines the minimal command and telemetry structures used by the feedforward
  motor identification firmware.

  Must mirror:
    robot/python/feedforward_model_test/protocol.py

  Notes:
  - Commands are direct normalized motor commands in [-1, 1].
  - Telemetry reports the last applied commands and output-side RPM values.
  - Optional numeric telemetry fields use NAN and are encoded as JSON null.
===============================================================================
*/

/*=============================================================================
  MOTOR CHANNEL DATA
=============================================================================*/

struct MotorCommandSet {
  float drive_lhs_cmd = 0.0f;
  float drive_rhs_cmd = 0.0f;
  float mech_rhs_cmd = 0.0f;
  float mech_lhs_cmd = 0.0f;
};

struct MotorTelemetrySet {
  float drive_lhs_cmd = 0.0f;
  float drive_rhs_cmd = 0.0f;
  float mech_rhs_cmd = 0.0f;
  float mech_lhs_cmd = 0.0f;

  float drive_lhs_rpm = NAN;
  float drive_rhs_rpm = NAN;
  float mech_rhs_rpm = NAN;
  float mech_lhs_rpm = NAN;
};

/*=============================================================================
  FULL FRAMES
=============================================================================*/

struct CommandFrame {
  uint32_t seq = 0;
  MotorCommandSet motors;
  bool valid = false;
};

struct TelemetryFrame {
  uint32_t arduino_time_ms = 0;
  uint32_t ack_seq = 0;
  MotorTelemetrySet motors;
};
