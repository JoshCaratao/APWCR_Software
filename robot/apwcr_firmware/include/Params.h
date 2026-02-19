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
constexpr float WHEEL_RADIUS_FT = 2.0f / 12.0f;   // 2 inch radius → feet
constexpr float WHEEL_DIAMETER_FT = 4.0f / 12.0f;

// Distance between drive wheels
constexpr float TRACK_WIDTH_FT = 13.0f / 12.0f;   // adjust to CAD

// Derived
constexpr float WHEEL_CIRCUMFERENCE_FT =
    2.0f * PI * WHEEL_RADIUS_FT;

/* ============================================================================
   ENCODER PARAMETERS
============================================================================ */

// Encoder hardware
constexpr int ENCODER_CPR = 48;            // counts per motor shaft rev
constexpr int QUADRATURE_FACTOR = 4;       // x4 decoding
constexpr float DRIVE_GEAR_RATIO = 2.0f;   // motor : wheel

// Derived counts
constexpr int COUNTS_PER_WHEEL_REV =
    ENCODER_CPR * QUADRATURE_FACTOR * DRIVE_GEAR_RATIO;

// Linear distance per encoder count
constexpr float FEET_PER_COUNT =
    WHEEL_CIRCUMFERENCE_FT / COUNTS_PER_WHEEL_REV;

/* ============================================================================
   MOTOR LIMITS
============================================================================ */

// PWM limits
constexpr int PWM_MIN = 0;
constexpr int PWM_MAX = 255;

// Soft limits (customary)
constexpr float MAX_LINEAR_SPEED_FTPS = 3.0f;    // ft/s
constexpr float MAX_ANGULAR_SPEED_DPS = 180.0f;  // deg/s

/* ============================================================================
   DRIVE CONTROL (PID – linear speed)
============================================================================ */

constexpr float DRIVE_KP = 0.9f;
constexpr float DRIVE_KI = 0.0f;
constexpr float DRIVE_KD = 0.06f;

constexpr float DRIVE_INTEGRAL_LIMIT = 50.0f;

/* ============================================================================
   ARM / MECHANISM CONTROL
============================================================================ */

constexpr float ARM_KP = 1.2f;
constexpr float ARM_KI = 0.0f;
constexpr float ARM_KD = 0.1f;

constexpr int ARM_MAX_PWM = 200;

/* ============================================================================
   SERVO PARAMETERS
============================================================================ */

constexpr int SERVO_MIN_DEG = 0;
constexpr int SERVO_MAX_DEG = 100;

// Mechanical positions (tuned to CAD)
constexpr int LID_OPEN_DEG   = 80;
constexpr int LID_CLOSED_DEG = 0;

constexpr int SWEEP_DEPLOY_DEG = 65;
constexpr int SWEEP_STOW_DEG   = 15;

/* ============================================================================
   ULTRASONIC SENSOR (HC-SR04)
============================================================================ */

// Unit helpers
constexpr float INCHES_PER_FOOT = 12.0f;
constexpr float CM_PER_INCH = 2.54f;

// What range do we actually care about for the robot?
// Keeping this smaller makes ultrasonic reads faster and reduces blocking.
constexpr float ULTRASONIC_MIN_IN = 6.0f;          // 
constexpr float ULTRASONIC_MAX_RANGE_IN = 60.0f;   // 

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
// Keep this at or below your control period if you want tight timing.
constexpr uint32_t ULTRASONIC_TIMEOUT_US_HARD = 20000UL;  // 20 ms

// Final timeout to pass to the Martinsos library
constexpr uint32_t ULTRASONIC_TIMEOUT_US =
    (ULTRASONIC_TIMEOUT_US_FROM_RANGE < ULTRASONIC_TIMEOUT_US_HARD)
      ? ULTRASONIC_TIMEOUT_US_FROM_RANGE
      : ULTRASONIC_TIMEOUT_US_HARD;

// Valid measurement max for your wrapper sanity checks (keep aligned with range)
constexpr float ULTRASONIC_MAX_VALID_IN = ULTRASONIC_MAX_RANGE_IN;

/* ============================================================================
   TASK RATES / TIMING
============================================================================ */

constexpr uint16_t DRIVE_UPDATE_HZ      = 100;
constexpr uint16_t RxCOMM_UPDATE_HZ  = 500;
constexpr uint16_t TELEMETRY_UPDATE_HZ  = 25;
constexpr uint16_t ULTRASONIC_UPDATE_HZ = 15;

// Safety
constexpr unsigned long COMMAND_TIMEOUT_MS = 6000;

/* ============================================================================
   TELEMETRY / COMMS
============================================================================ */

constexpr uint32_t SERIAL_BAUD = 230400;
constexpr uint16_t SERIAL_LINE_BUFFER_BYTES = 2048;
constexpr size_t SERIAL_JSON_DOC_BYTES = 1536;  // start here; bump to 1536 if needed


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
constexpr float SWEEP_SERVO_RAMP_DPS = 10.0f;

// How close is "at target"
constexpr float SERVO_DEADBAND_DEG = 2.0f;

// How long to sit at target before detaching (ms)
constexpr uint32_t LID_SERVO_SETTLE_MS = 1000;

// Lid: gravity holds closed, so detach after closing
constexpr bool LID_SERVO_AUTO_DETACH_ON_CLOSED = true;

constexpr uint32_t SWEEP_SERVO_SETTLE_MS = 1000;
constexpr bool SWEEP_SERVO_AUTO_DETACH_ON_CLOSED = true; // usually false
