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
constexpr float TRACK_WIDTH_FT = 12.0776f / 12.0f;   // measured from physical robot

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
constexpr int PWM_MIN = 0;
constexpr int PWM_MAX = 255;

// Soft limits (customary)
constexpr float MAX_LINEAR_SPEED_FTPS = 3.0f;    // ft/s
constexpr float MAX_ANGULAR_SPEED_DPS = 180.0f;  // deg/s

/* ============================================================================
   DRIVE CONTROL (wheel-speed feedforward + PID)
============================================================================ */

constexpr float DRIVE_LHS_KP = 0.04f;
constexpr float DRIVE_LHS_KI = 0.002f;
constexpr float DRIVE_LHS_KD = 0.00f;

constexpr float DRIVE_RHS_KP = 0.04f;
constexpr float DRIVE_RHS_KI = 0.002f;
constexpr float DRIVE_RHS_KD = 0.00f;

constexpr float DRIVE_INTEGRAL_LIMIT = 5.0f;

// Drive direction inversions
// Set true if a side spins/measures opposite of expected forward direction.
constexpr bool DRIVE_INVERT_LHS_MOTOR = false;
constexpr bool DRIVE_INVERT_RHS_MOTOR = false;
constexpr bool DRIVE_INVERT_LHS_ENCODER = false;
constexpr bool DRIVE_INVERT_RHS_ENCODER = true;

// Feedforward wheel-state thresholds.
constexpr float DRIVE_RPM_ZERO_DEADBAND = 1.0f;
constexpr float DRIVE_RPM_STOPPED_THRESH = 3.0f;

// ---------------------------------------------------------------------------
// Running-region linear feedforward fits from no-stop tests
//
// Model form inside DriveController:
//   duty_mag = run_intercept_duty + run_slope_duty_per_rpm * |target_rpm|
//
// Notes:
// - These fits are only trusted in the measured running region.
// - The intercept can be negative because the low-speed nonlinear region is
//   handled explicitly by u_break / u_move_min logic instead of forcing the
//   linear fit to cover startup behavior.
// ---------------------------------------------------------------------------

// RHS wheel
constexpr float DRIVE_RHS_POS_RUN_INTERCEPT_DUTY = -0.9804f;
constexpr float DRIVE_RHS_POS_RUN_SLOPE_DUTY_PER_RPM = 0.0366f;
constexpr float DRIVE_RHS_NEG_RUN_INTERCEPT_DUTY = -1.0147f;
constexpr float DRIVE_RHS_NEG_RUN_SLOPE_DUTY_PER_RPM = 0.0346f;

// LHS wheel
constexpr float DRIVE_LHS_POS_RUN_INTERCEPT_DUTY = -0.7449f;
constexpr float DRIVE_LHS_POS_RUN_SLOPE_DUTY_PER_RPM = 0.0303f;
constexpr float DRIVE_LHS_NEG_RUN_INTERCEPT_DUTY = -1.0648f;
constexpr float DRIVE_LHS_NEG_RUN_SLOPE_DUTY_PER_RPM = 0.0364f;

// ---------------------------------------------------------------------------
// Startup breakaway floors from with-stop tests
// ---------------------------------------------------------------------------

constexpr float DRIVE_RHS_POS_U_BREAK = 0.3400f;
constexpr float DRIVE_RHS_NEG_U_BREAK = 0.3300f;
constexpr float DRIVE_LHS_POS_U_BREAK = 0.3500f;
constexpr float DRIVE_LHS_NEG_U_BREAK = 0.3050f;

// ---------------------------------------------------------------------------
// Minimum sustaining floors for already-moving low-speed operation
//
// These are used only when:
// - target RPM is below the reliable linear-fit region, and
// - the wheel is already moving
// ---------------------------------------------------------------------------

constexpr float DRIVE_RHS_POS_U_MOVE_MIN = 0.3550f;
constexpr float DRIVE_RHS_NEG_U_MOVE_MIN = 0.3050f;
constexpr float DRIVE_LHS_POS_U_MOVE_MIN = 0.3550f;
constexpr float DRIVE_LHS_NEG_U_MOVE_MIN = 0.2850f;

// ---------------------------------------------------------------------------
// Lowest RPM where the running linear fit is treated as reliable
// ---------------------------------------------------------------------------

constexpr float DRIVE_RHS_POS_RPM_MIN_FIT = 35.0f;
constexpr float DRIVE_RHS_NEG_RPM_MIN_FIT = 36.0f;
constexpr float DRIVE_LHS_POS_RPM_MIN_FIT = 35.0f;
constexpr float DRIVE_LHS_NEG_RPM_MIN_FIT = 33.0f;

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


// -----------------------------------------------------------------------------
// Mechanism POSITION control PID (POS_DEG mode)
// -----------------------------------------------------------------------------
constexpr float MECH_RHS_POS_KP = 0.03f;
constexpr float MECH_RHS_POS_KI = 0.035f;
constexpr float MECH_RHS_POS_KD = 0.01f;

constexpr float MECH_LHS_POS_KP = 0.015f;
constexpr float MECH_LHS_POS_KI = 0.1f;
constexpr float MECH_LHS_POS_KD = 0.003f;

// Integral clamp + deadband for position loop
constexpr float MECH_POS_INTEGRAL_LIMIT_RHS = 5.0f;
constexpr float MECH_POS_INTEGRAL_LIMIT_LHS = 5.0f;

// Only enable integral near the target. Outside this zone, the mechanism
// position loop uses P + D only.
constexpr float MECH_POS_INTEGRAL_ZONE_DEG_RHS = 12.0f;
constexpr float MECH_POS_INTEGRAL_ZONE_DEG_LHS = 12.0f;

constexpr float MECH_POS_DEADBAND_DEG_RHS = 2.5f;
constexpr float MECH_POS_DEADBAND_DEG_LHS = 2.5f;

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
