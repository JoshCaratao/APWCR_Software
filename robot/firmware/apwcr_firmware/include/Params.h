#pragma once
#include <Arduino.h>

/*
  Params.h

  Purpose:
  Central location for robot constants and tunable parameters.
  Uses US customary units (feet, inches, seconds, degrees).

  Board:
  Arduino Mega 2560

  Convention:
  - Distances: feet (ft) unless explicitly noted
  - Small distances: inches (in)
  - Speeds: ft/s
  - Angles: degrees
*/

/* ============================================================================
   ROBOT GEOMETRY
============================================================================ */

// Drive wheels
constexpr float WHEEL_RADIUS_FT = 2.35f / 12.0f;

// Distance between drive wheels
constexpr float TRACK_WIDTH_FT = 11.5f / 12.0f;   // measured from physical robot

// Derived
constexpr float WHEEL_CIRCUMFERENCE_FT =
    2.0f * PI * WHEEL_RADIUS_FT;

/* ============================================================================
   ENCODER PARAMETERS
============================================================================ */

// Encoder hardware
// The encoder is treated as 48 counts per motor-shaft revolution as decoded.
constexpr int ENCODER_CPR = 48;
constexpr float MOTOR_GEAR_RATIO = 98.78f;   // motor shaft revs per gearbox output rev
constexpr float DRIVE_GEAR_RATIO = 2.0f;     // gearbox output revs per wheel rev

// Derived counts (drive wheels)
constexpr float COUNTS_PER_WHEEL_REV =
    ENCODER_CPR * MOTOR_GEAR_RATIO * DRIVE_GEAR_RATIO;

// Linear distance per encoder count
constexpr float FEET_PER_COUNT =
    WHEEL_CIRCUMFERENCE_FT / COUNTS_PER_WHEEL_REV;

/* ============================================================================
   MOTOR LIMITS
============================================================================ */

// PWM limits
constexpr int PWM_MIN = 50;
constexpr int PWM_MAX = 255;

// Soft limits (customary)
constexpr float MAX_LINEAR_SPEED_FTPS = 3.0f;    // ft/s
constexpr float MAX_ANGULAR_SPEED_DPS = 180.0f;  // deg/s

/* ============================================================================
   DRIVE CONTROL (PID - wheel speed)
============================================================================ */

constexpr float DRIVE_KP = 0.95f;
constexpr float DRIVE_KI = 0.10f;
constexpr float DRIVE_KD = 0.00f;

constexpr float DRIVE_INTEGRAL_LIMIT = 5.0f;

// Drive direction inversions
// Set true if a side spins/measures opposite of expected forward direction.
constexpr bool DRIVE_INVERT_LHS_MOTOR = false;
constexpr bool DRIVE_INVERT_RHS_MOTOR = false;
constexpr bool DRIVE_INVERT_LHS_ENCODER = false;
constexpr bool DRIVE_INVERT_RHS_ENCODER = true;

// Drive output compensation
// These compensate real wheel/driver asymmetry after PID computes duty.
// Start all at neutral values, then tune empirically.
constexpr float DRIVE_LHS_FWD_SCALE = 1.00f;
constexpr float DRIVE_LHS_REV_SCALE = 1.00f;
constexpr float DRIVE_RHS_FWD_SCALE = 1.00f;
constexpr float DRIVE_RHS_REV_SCALE = 1.00f;

constexpr float DRIVE_LHS_FWD_FF = 0.00f;
constexpr float DRIVE_LHS_REV_FF = 0.00f;
constexpr float DRIVE_RHS_FWD_FF = 0.00f;
constexpr float DRIVE_RHS_REV_FF = 0.00f;

/* ============================================================================
   MECHANISM CONTROLLER PARAMETERS
============================================================================ */

// -----------------------------------------------------------------------------
// Mechanism encoder scaling
// -----------------------------------------------------------------------------

// External mechanism transmission ratios (output side)
// RHS: worm gear 40:1
// LHS: belt reduction from 18T (driver) to 40T (driven)
constexpr float MECH_RHS_EXTERNAL_RATIO = 40.0f;
constexpr float MECH_LHS_EXTERNAL_RATIO = 40.0f / 18.0f;

// Encoder counts per mechanism output revolution
// (encoder on motor shaft -> multiply by full downstream reduction)
constexpr float MECH_COUNTS_PER_REV_RHS =
    ENCODER_CPR * MOTOR_GEAR_RATIO * MECH_RHS_EXTERNAL_RATIO;

constexpr float MECH_COUNTS_PER_REV_LHS =
    ENCODER_CPR * MOTOR_GEAR_RATIO * MECH_LHS_EXTERNAL_RATIO;

// -----------------------------------------------------------------------------
// Mechanism direction inversions
// -----------------------------------------------------------------------------
constexpr bool MECH_INVERT_RHS_MOTOR   = false;
constexpr bool MECH_INVERT_LHS_MOTOR   = false;
constexpr bool MECH_INVERT_RHS_ENCODER = false;
constexpr bool MECH_INVERT_LHS_ENCODER = false;

// -----------------------------------------------------------------------------
// Mechanism motor output limits
// -----------------------------------------------------------------------------
constexpr int   MECH_PWM_MIN = PWM_MIN;
constexpr int   MECH_PWM_MAX = PWM_MAX;
constexpr float MECH_MAX_ABS_DUTY = 1.0f;

// Manual jog duty (open-loop)
// Use these for button-based forward/back jog in manual mode.
constexpr float MECH_JOG_DUTY_RHS = 0.35f;
constexpr float MECH_JOG_DUTY_LHS = 0.35f;

// -----------------------------------------------------------------------------
// Mechanism POSITION control PID (POS_DEG mode)
// -----------------------------------------------------------------------------
constexpr float MECH_RHS_POS_KP = 0.55f;
constexpr float MECH_RHS_POS_KI = 0.0f;
constexpr float MECH_RHS_POS_KD = 0.05f;

constexpr float MECH_LHS_POS_KP = 0.55f;
constexpr float MECH_LHS_POS_KI = 0.0f;
constexpr float MECH_LHS_POS_KD = 0.05f;

// Integral clamp + deadband for position loop
constexpr float MECH_POS_INTEGRAL_LIMIT_RHS = 5.0f;
constexpr float MECH_POS_INTEGRAL_LIMIT_LHS = 5.0f;

constexpr float MECH_POS_DEADBAND_DEG_RHS = 1.5f;
constexpr float MECH_POS_DEADBAND_DEG_LHS = 1.5f;

// Position mode software limits (output angle, deg)
constexpr float MECH_POS_MIN_DEG_RHS = -180.0f;
constexpr float MECH_POS_MAX_DEG_RHS =  180.0f;

constexpr float MECH_POS_MIN_DEG_LHS = -180.0f;
constexpr float MECH_POS_MAX_DEG_LHS =  180.0f;

// -----------------------------------------------------------------------------
// Mechanism update + safe/home
// -----------------------------------------------------------------------------
constexpr uint16_t MECH_UPDATE_HZ = 60;

// Startup convention:
// - Ground is our calibrated zero
// - The robot is expected to power up with the RHS arm already stowed
constexpr float MECH_RHS_STOW_DEG = 100.0f;
constexpr float MECH_RHS_HOME_DEG = MECH_RHS_STOW_DEG;
constexpr float MECH_LHS_HOME_DEG = 0.0f;

/* ============================================================================
   SERVO PARAMETERS
============================================================================ */

constexpr int SERVO_MIN_DEG = 0;
constexpr int SERVO_MAX_DEG = 180;

// Mechanical positions (tuned to CAD)
constexpr int LID_OPEN_DEG   = 75;
constexpr int LID_CLOSED_DEG = 5;

constexpr int SWEEP_DEPLOY_DEG = 150;
constexpr int SWEEP_STOW_DEG   = 5;

constexpr float SWEEP_SERVO_MIRROR_CENTER_DEG = 75.0f; // tune on hardware

/* ============================================================================
   ULTRASONIC SENSOR (HC-SR04)
============================================================================ */

// Unit helpers
constexpr float INCHES_PER_FOOT = 12.0f;
constexpr float CM_PER_INCH = 2.54f;

// What range do we actually care about for the robot?
constexpr float ULTRASONIC_MIN_IN = 1.0f;
constexpr float ULTRASONIC_MAX_RANGE_IN = 42.0f;

// Martinsos library uses max distance in centimeters
constexpr uint16_t ULTRASONIC_MAX_DISTANCE_CM =
    (uint16_t)(ULTRASONIC_MAX_RANGE_IN * CM_PER_INCH);

// Speed of sound (for computing a reasonable timeout from desired range)
constexpr float SPEED_OF_SOUND_CMPS = 34300.0f;    // ~20 C

// Timeout derived from range: round-trip time to ULTRASONIC_MAX_DISTANCE_CM
// with a 25% margin for messy reflections.
constexpr uint32_t ULTRASONIC_TIMEOUT_US_FROM_RANGE =
    (uint32_t)(1.25f * (2.0f * ULTRASONIC_MAX_DISTANCE_CM / SPEED_OF_SOUND_CMPS) * 1000000.0f);

// Hard cap on how long we're willing to block in pulseIn() worst case.
constexpr uint32_t ULTRASONIC_TIMEOUT_US_HARD = 10000UL;  // 10 ms

// Final timeout to pass to the Martinsos library
constexpr uint32_t ULTRASONIC_TIMEOUT_US =
    (ULTRASONIC_TIMEOUT_US_FROM_RANGE < ULTRASONIC_TIMEOUT_US_HARD)
      ? ULTRASONIC_TIMEOUT_US_FROM_RANGE
      : ULTRASONIC_TIMEOUT_US_HARD;

// Valid measurement max for wrapper sanity checks
constexpr float ULTRASONIC_MAX_VALID_IN = ULTRASONIC_MAX_RANGE_IN;

/* ============================================================================
   TASK RATES / TIMING
============================================================================ */

constexpr uint16_t DRIVE_UPDATE_HZ      = 80;
constexpr uint16_t RxCOMM_UPDATE_HZ     = 450;
constexpr uint16_t TELEMETRY_UPDATE_HZ  = 20;
constexpr uint16_t ULTRASONIC_UPDATE_HZ = 15;

// Safety
constexpr unsigned long COMMAND_TIMEOUT_MS = 6000;

/* ============================================================================
   TELEMETRY / COMMS
============================================================================ */

constexpr uint32_t SERIAL_BAUD = 230400;
constexpr uint16_t SERIAL_LINE_BUFFER_BYTES = 2048;
constexpr size_t SERIAL_JSON_DOC_BYTES = 1536;

/* ============================================================================
   DEBUG / SAFETY FLAGS
============================================================================ */

constexpr bool ENABLE_WATCHDOG = true;
constexpr bool ENABLE_SERIAL_DEBUG = false;

/* ============================================================================
   SERVO RAMP / DETACH BEHAVIOR
============================================================================ */

// How often we update ramp motion (tick rate)
constexpr uint16_t SERVO_UPDATE_HZ = 60;

// Ramp rates (deg/sec)
constexpr float LID_SERVO_RAMP_DPS   = 25.0f;
constexpr float SWEEP_SERVO_RAMP_DPS = 35.0f;

// How close is "at target"
constexpr float SERVO_DEADBAND_DEG = 2.0f;

// How long to sit at target before detaching (ms)
constexpr uint32_t LID_SERVO_SETTLE_MS = 1000;

// Lid: gravity holds closed, so detach after closing
constexpr bool LID_SERVO_AUTO_DETACH_ON_CLOSED = true;

constexpr uint32_t SWEEP_SERVO_SETTLE_MS = 1000;
constexpr bool SWEEP_SERVO_AUTO_DETACH_ON_CLOSED = true; // usually false
