#pragma once
#include <Arduino.h>

/*
  Params.h

  Purpose:
  Minimal constants for the feedforward-model-test firmware.
  These parameters are limited to motor output, encoder scaling, serial comms,
  and simple scheduler timing for identification sweeps.

  Board:
  Arduino Mega 2560
*/

/* ============================================================================
   ENCODER SCALING
   Output RPM should match the actual controlled output, not motor-shaft RPM.
============================================================================ */

// The encoder is treated as 48 counts per motor-shaft revolution as decoded.
constexpr int ENCODER_CPR = 48;

// Motor shaft revs per gearbox output rev.
constexpr float MOTOR_GEAR_RATIO = 98.78f;

// Downstream output ratios used by the deployed firmware.
constexpr float DRIVE_GEAR_RATIO = 2.0f;
constexpr float MECH_RHS_EXTERNAL_RATIO = 40.0f;
constexpr float MECH_LHS_EXTERNAL_RATIO = 40.0f / 18.0f;

// Counts per controlled output revolution.
constexpr float COUNTS_PER_DRIVE_OUTPUT_REV =
    ENCODER_CPR * MOTOR_GEAR_RATIO * DRIVE_GEAR_RATIO;

constexpr float COUNTS_PER_MECH_RHS_OUTPUT_REV =
    ENCODER_CPR * MOTOR_GEAR_RATIO * MECH_RHS_EXTERNAL_RATIO;

constexpr float COUNTS_PER_MECH_LHS_OUTPUT_REV =
    ENCODER_CPR * MOTOR_GEAR_RATIO * MECH_LHS_EXTERNAL_RATIO;

/* ============================================================================
   DIRECTION CONVENTIONS
   These keep telemetry sign aligned with the deployed firmware conventions.
============================================================================ */

constexpr bool DRIVE_INVERT_LHS_MOTOR = false;
constexpr bool DRIVE_INVERT_RHS_MOTOR = false;
constexpr bool DRIVE_INVERT_LHS_ENCODER = false;
constexpr bool DRIVE_INVERT_RHS_ENCODER = true;

constexpr bool MECH_INVERT_LHS_MOTOR = false;
constexpr bool MECH_INVERT_RHS_MOTOR = false;
constexpr bool MECH_INVERT_LHS_ENCODER = false;
constexpr bool MECH_INVERT_RHS_ENCODER = false;

/* ============================================================================
   MOTOR OUTPUT LIMITS
   Use a direct 0..255 PWM range so deadzone behavior is captured by the data.
============================================================================ */

constexpr uint8_t PWM_MIN = 0;
constexpr uint8_t PWM_MAX = 255;
constexpr float MAX_ABS_DUTY = 1.0f;

/* ============================================================================
   SERIAL / PROTOCOL
============================================================================ */

constexpr uint32_t SERIAL_BAUD = 230400;
constexpr uint16_t SERIAL_LINE_BUFFER_BYTES = 1024;
constexpr size_t SERIAL_JSON_DOC_BYTES = 768;

// Stop motors if commands stop arriving during a live test.
constexpr unsigned long COMMAND_TIMEOUT_MS = 2000UL;

/* ============================================================================
   TASK RATES
============================================================================ */

constexpr uint16_t RX_COMM_UPDATE_HZ = 400;
constexpr uint16_t ENCODER_SAMPLE_HZ = 100;
constexpr uint16_t TELEMETRY_UPDATE_HZ = 30;
