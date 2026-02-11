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
constexpr int ENCODER_CPR = 12;            // counts per motor shaft rev
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
constexpr int SERVO_MAX_DEG = 180;

// Mechanical positions (tuned to CAD)
constexpr int LID_OPEN_DEG   = 95;
constexpr int LID_CLOSED_DEG = 10;

constexpr int SWEEP_DEPLOY_DEG = 120;
constexpr int SWEEP_STOW_DEG   = 20;

/* ============================================================================
   ULTRASONIC SENSOR (HC-SR04)
============================================================================ */

// Speed of sound in air (at ~20C)
constexpr float SPEED_OF_SOUND_FTPS = 1125.0f;  // ft/s

// Valid measurement range
constexpr float ULTRASONIC_MIN_IN = 0.8f;   // ~2 cm
constexpr float ULTRASONIC_MAX_IN = 157.0f; // ~4 m

// Timeout for pulseIn (microseconds)
constexpr unsigned long ULTRASONIC_TIMEOUT_US = 30000UL;

// Conversion helper
constexpr float INCHES_PER_FOOT = 12.0f;

/* ============================================================================
   TASK RATES / TIMING
============================================================================ */

constexpr uint16_t DRIVE_UPDATE_HZ      = 100;
constexpr uint16_t TELEMETRY_UPDATE_HZ  = 20;
constexpr uint16_t ULTRASONIC_UPDATE_HZ = 10;

// Safety
constexpr unsigned long COMMAND_TIMEOUT_MS = 250;

/* ============================================================================
   TELEMETRY / COMMS
============================================================================ */

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint16_t TELEMETRY_BUFFER_BYTES = 256;

/* ============================================================================
   DEBUG / SAFETY FLAGS
============================================================================ */

constexpr bool ENABLE_WATCHDOG = true;
constexpr bool ENABLE_SERIAL_DEBUG = false;


/* ============================================================================
   ULTRASONIC GEOMETRY (CONE / FRONT-OF-ROBOT VALIDATION)
============================================================================ */

// Robot width at the plane you care about (outer-to-outer), inches
constexpr float ROBOT_WIDTH_IN = 16.0f;  // set from CAD / tape measure

// Sensor mounting: lateral offset from robot centerline (inches)
// 0.0 means sensor is centered
constexpr float ULTRASONIC_OFFSET_Y_IN = 0.0f;

// HC-SR04 approximate beam cone HALF-angle (degrees)
// Common rules of thumb are ~15° half-angle (≈30° total), but measure if possible.
constexpr float ULTRASONIC_CONE_HALF_ANGLE_DEG = 15.0f;

// Derived: half-width of robot
constexpr float ROBOT_HALF_WIDTH_IN = ROBOT_WIDTH_IN * 0.5f;

// A “coverage” threshold distance where the cone half-width equals robot half-width.
// Beyond this distance, the cone covers the full width (assuming centered sensor).
constexpr float ULTRASONIC_FULL_COVERAGE_DIST_IN =
    (ROBOT_HALF_WIDTH_IN + fabs(ULTRASONIC_OFFSET_Y_IN)) /
    tan(ULTRASONIC_CONE_HALF_ANGLE_DEG * (PI / 180.0f));

// Optional: if you only need the cone to cover SOME fraction of the width (relaxed gate)
// e.g., 0.6 means 60% of half-width required
constexpr float ULTRASONIC_COVERAGE_FRACTION = 1.0f;

constexpr float ULTRASONIC_COVERAGE_DIST_IN =
    (ULTRASONIC_COVERAGE_FRACTION * (ROBOT_HALF_WIDTH_IN + fabs(ULTRASONIC_OFFSET_Y_IN))) /
    tan(ULTRASONIC_CONE_HALF_ANGLE_DEG * (PI / 180.0f));
