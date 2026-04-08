/*
  APWCR Arduino Controller (Low Level Hardware Layer)

  Purpose:
  This firmware runs on the Arduino Mega as the deterministic hardware layer.
  It receives high-level command frames from the host (Python), updates
  drivetrain + mechanism controllers, samples sensors, and publishes telemetry.

  Architecture:
  - Comms:
      * SerialLink handles newline-delimited JSON RX/TX.
      * RX parses command frames and tracks sequence ACK + timeout.
  - Drive subsystem:
      * DriveController owns drive motors + drive encoders + PID wheel control.
      * DriveController also applies side/direction compensation to correct
        real drivetrain asymmetry.
  - Mechanism subsystem:
      * MechanismController owns mechanism motors + mechanism encoders + PID control.
      * MechanismController also owns lid servo + dual sweeper servos.
      * Sweeper servos are mirrored from one logical command.
  - Sensor subsystem:
      * DistanceSensor provides ultrasonic distance telemetry.
  - Scheduler:
      * Rate objects provide non-blocking periodic task updates.

  Safety behavior:
  - If command timeout occurs, firmware:
      * stops drivetrain
      * sends mechanism to safe/stow state
  - Timeout handling is edge-triggered to avoid repeatedly resetting commands.
*/

#include <Arduino.h>

#include "Pins.h"
#include "Params.h"

#include "utils/Rate.h"
#include "comms/SerialLink.h"
#include "sensors/DistanceSensor.h"
#include "control/DriveController.h"
#include "control/MechanismController.h"

/*=============================================================================
  GLOBALS
=============================================================================*/

// USB serial protocol link (Host <-> Arduino)
SerialLink g_link(SERIAL_USB);

// Ultrasonic distance sensor wrapper
DistanceSensor g_distance_sensor(
  PIN_ULTRASONIC_TRIG,
  PIN_ULTRASONIC_ECHO,
  ULTRASONIC_MAX_DISTANCE_CM,
  ULTRASONIC_TIMEOUT_US,
  ULTRASONIC_MIN_IN,
  ULTRASONIC_MAX_VALID_IN
);

/*=============================================================================
  DRIVE CONTROLLER CONFIG + INSTANCE
=============================================================================*/

static DriveController::Config makeDriveConfig() {
  DriveController::Config c;

  // Motor pins
  c.pin_lhs_dir = PIN_LHS_DRIVE_DIR;
  c.pin_lhs_pwm = PIN_LHS_DRIVE_PWM;
  c.pin_rhs_dir = PIN_RHS_DRIVE_DIR;
  c.pin_rhs_pwm = PIN_RHS_DRIVE_PWM;

  // Encoder pins
  c.pin_enc_lhs_a = PIN_ENC_LHS_DRIVE_A;
  c.pin_enc_lhs_b = PIN_ENC_LHS_DRIVE_B;
  c.pin_enc_rhs_a = PIN_ENC_RHS_DRIVE_A;
  c.pin_enc_rhs_b = PIN_ENC_RHS_DRIVE_B;

  // Inversions
  c.invert_lhs_motor = DRIVE_INVERT_LHS_MOTOR;
  c.invert_rhs_motor = DRIVE_INVERT_RHS_MOTOR;
  c.invert_lhs_encoder = DRIVE_INVERT_LHS_ENCODER;
  c.invert_rhs_encoder = DRIVE_INVERT_RHS_ENCODER;
  c.lhs_encoder_rpm_filter_alpha = DRIVE_LHS_ENCODER_RPM_FILTER_ALPHA;
  c.rhs_encoder_rpm_filter_alpha = DRIVE_RHS_ENCODER_RPM_FILTER_ALPHA;

  // Geometry / conversion
  c.track_width_ft = TRACK_WIDTH_FT;
  c.wheel_circumference_ft = WHEEL_CIRCUMFERENCE_FT;
  c.counts_per_wheel_rev = COUNTS_PER_WHEEL_REV;

  // Limits
  c.max_linear_ftps = MAX_LINEAR_SPEED_FTPS;
  c.max_angular_dps = MAX_ANGULAR_SPEED_DPS;

  // PID
  c.lhs_kp = DRIVE_LHS_KP;
  c.lhs_ki = DRIVE_LHS_KI;
  c.lhs_kd = DRIVE_LHS_KD;
  c.rhs_kp = DRIVE_RHS_KP;
  c.rhs_ki = DRIVE_RHS_KI;
  c.rhs_kd = DRIVE_RHS_KD;
  c.integral_limit = DRIVE_INTEGRAL_LIMIT;

  // Per-wheel feedforward characterization
  c.lhs_ff.rpm_zero_deadband = DRIVE_RPM_ZERO_DEADBAND;
  c.lhs_ff.rpm_stopped_thresh = DRIVE_RPM_STOPPED_THRESH;
  c.lhs_ff.pos.run_intercept_duty = DRIVE_LHS_POS_RUN_INTERCEPT_DUTY;
  c.lhs_ff.pos.run_slope_duty_per_rpm = DRIVE_LHS_POS_RUN_SLOPE_DUTY_PER_RPM;
  c.lhs_ff.pos.u_break = DRIVE_LHS_POS_U_BREAK;
  c.lhs_ff.pos.u_move_min = DRIVE_LHS_POS_U_MOVE_MIN;
  c.lhs_ff.pos.rpm_min_fit = DRIVE_LHS_POS_RPM_MIN_FIT;
  c.lhs_ff.neg.run_intercept_duty = DRIVE_LHS_NEG_RUN_INTERCEPT_DUTY;
  c.lhs_ff.neg.run_slope_duty_per_rpm = DRIVE_LHS_NEG_RUN_SLOPE_DUTY_PER_RPM;
  c.lhs_ff.neg.u_break = DRIVE_LHS_NEG_U_BREAK;
  c.lhs_ff.neg.u_move_min = DRIVE_LHS_NEG_U_MOVE_MIN;
  c.lhs_ff.neg.rpm_min_fit = DRIVE_LHS_NEG_RPM_MIN_FIT;

  c.rhs_ff.rpm_zero_deadband = DRIVE_RPM_ZERO_DEADBAND;
  c.rhs_ff.rpm_stopped_thresh = DRIVE_RPM_STOPPED_THRESH;
  c.rhs_ff.pos.run_intercept_duty = DRIVE_RHS_POS_RUN_INTERCEPT_DUTY;
  c.rhs_ff.pos.run_slope_duty_per_rpm = DRIVE_RHS_POS_RUN_SLOPE_DUTY_PER_RPM;
  c.rhs_ff.pos.u_break = DRIVE_RHS_POS_U_BREAK;
  c.rhs_ff.pos.u_move_min = DRIVE_RHS_POS_U_MOVE_MIN;
  c.rhs_ff.pos.rpm_min_fit = DRIVE_RHS_POS_RPM_MIN_FIT;
  c.rhs_ff.neg.run_intercept_duty = DRIVE_RHS_NEG_RUN_INTERCEPT_DUTY;
  c.rhs_ff.neg.run_slope_duty_per_rpm = DRIVE_RHS_NEG_RUN_SLOPE_DUTY_PER_RPM;
  c.rhs_ff.neg.u_break = DRIVE_RHS_NEG_U_BREAK;
  c.rhs_ff.neg.u_move_min = DRIVE_RHS_NEG_U_MOVE_MIN;
  c.rhs_ff.neg.rpm_min_fit = DRIVE_RHS_NEG_RPM_MIN_FIT;

  c.stall_guard.enabled = DRIVE_STALL_GUARD_ENABLED;
  c.stall_guard.duty_threshold = DRIVE_STALL_DUTY_THRESH;
  c.stall_guard.target_rpm_threshold = DRIVE_STALL_TARGET_RPM_THRESH;
  c.stall_guard.min_count_delta = DRIVE_STALL_MIN_COUNT_DELTA;
  c.stall_guard.timeout_ms = DRIVE_STALL_TIMEOUT_MS;
  c.stall_guard.reset_zero_ms = DRIVE_STALL_RESET_ZERO_MS;

  return c;
}

DriveController::Config g_drive_cfg = makeDriveConfig();
DriveController g_drive(g_drive_cfg);

/*=============================================================================
  MECHANISM CONTROLLER CONFIG + INSTANCE
=============================================================================*/

static MechanismController::Config makeMechanismConfig() {
  MechanismController::Config c;

  // RHS mechanism motor + encoder
  c.pin_rhs_dir = PIN_RHS_ARM_DIR;
  c.pin_rhs_pwm = PIN_RHS_ARM_PWM;
  c.pin_enc_rhs_a = PIN_ENC_RHS_ARM_A;
  c.pin_enc_rhs_b = PIN_ENC_RHS_ARM_B;
  c.invert_rhs_motor = MECH_INVERT_RHS_MOTOR;
  c.invert_rhs_encoder = MECH_INVERT_RHS_ENCODER;
  c.counts_per_rev_rhs = MECH_COUNTS_PER_REV_RHS;
  c.rhs_encoder_rpm_filter_alpha = MECH_RHS_ENCODER_RPM_FILTER_ALPHA;

  // LHS mechanism motor + encoder
  c.pin_lhs_dir = PIN_LHS_ARM_DIR;
  c.pin_lhs_pwm = PIN_LHS_ARM_PWM;
  c.pin_enc_lhs_a = PIN_ENC_LHS_ARM_A;
  c.pin_enc_lhs_b = PIN_ENC_LHS_ARM_B;
  c.invert_lhs_motor = MECH_INVERT_LHS_MOTOR;
  c.invert_lhs_encoder = MECH_INVERT_LHS_ENCODER;
  c.counts_per_rev_lhs = MECH_COUNTS_PER_REV_LHS;
  c.lhs_encoder_rpm_filter_alpha = MECH_LHS_ENCODER_RPM_FILTER_ALPHA;

  // Motor actuation + control
  c.pwm_min = MECH_PWM_MIN;
  c.pwm_max = MECH_PWM_MAX;
  c.max_abs_duty = MECH_MAX_ABS_DUTY;
  c.rhs_max_abs_rpm = MECH_RHS_MAX_ABS_RPM;
  c.lhs_max_abs_rpm = MECH_LHS_MAX_ABS_RPM;

  // RHS mechanism cascaded control
  c.rhs_pos_kp = MECH_RHS_POS_KP;
  c.rhs_pos_ki = MECH_RHS_POS_KI;
  c.rhs_pos_kd = MECH_RHS_POS_KD;
  c.rhs_pos_integral_limit = MECH_POS_INTEGRAL_LIMIT_RHS;
  c.rhs_pos_integral_zone_deg = MECH_POS_INTEGRAL_ZONE_DEG_RHS;
  c.rhs_pos_deadband_deg = MECH_POS_DEADBAND_DEG_RHS;
  c.rhs_pos_min_deg = MECH_POS_MIN_DEG_RHS;
  c.rhs_pos_max_deg = MECH_POS_MAX_DEG_RHS;
  c.rhs_speed_kp = MECH_RHS_SPEED_KP;
  c.rhs_speed_ki = MECH_RHS_SPEED_KI;
  c.rhs_speed_kd = MECH_RHS_SPEED_KD;
  c.rhs_speed_integral_limit = MECH_SPEED_INTEGRAL_LIMIT_RHS;
  c.rhs_ff.rpm_zero_deadband = MECH_RHS_RPM_ZERO_DEADBAND;
  c.rhs_ff.rpm_stopped_thresh = MECH_RHS_RPM_STOPPED_THRESH;
  c.rhs_ff.pos.run_intercept_duty = MECH_RHS_POS_RUN_INTERCEPT_DUTY;
  c.rhs_ff.pos.run_slope_duty_per_rpm = MECH_RHS_POS_RUN_SLOPE_DUTY_PER_RPM;
  c.rhs_ff.pos.u_break = MECH_RHS_POS_U_BREAK;
  c.rhs_ff.pos.u_move_min = MECH_RHS_POS_U_MOVE_MIN;
  c.rhs_ff.pos.rpm_min_fit = MECH_RHS_POS_RPM_MIN_FIT;
  c.rhs_ff.neg.run_intercept_duty = MECH_RHS_NEG_RUN_INTERCEPT_DUTY;
  c.rhs_ff.neg.run_slope_duty_per_rpm = MECH_RHS_NEG_RUN_SLOPE_DUTY_PER_RPM;
  c.rhs_ff.neg.u_break = MECH_RHS_NEG_U_BREAK;
  c.rhs_ff.neg.u_move_min = MECH_RHS_NEG_U_MOVE_MIN;
  c.rhs_ff.neg.rpm_min_fit = MECH_RHS_NEG_RPM_MIN_FIT;

  // LHS mechanism cascaded control
  c.lhs_pos_kp = MECH_LHS_POS_KP;
  c.lhs_pos_ki = MECH_LHS_POS_KI;
  c.lhs_pos_kd = MECH_LHS_POS_KD;
  c.lhs_pos_integral_limit = MECH_POS_INTEGRAL_LIMIT_LHS;
  c.lhs_pos_integral_zone_deg = MECH_POS_INTEGRAL_ZONE_DEG_LHS;
  c.lhs_pos_deadband_deg = MECH_POS_DEADBAND_DEG_LHS;
  c.lhs_pos_min_deg = MECH_POS_MIN_DEG_LHS;
  c.lhs_pos_max_deg = MECH_POS_MAX_DEG_LHS;
  c.lhs_speed_kp = MECH_LHS_SPEED_KP;
  c.lhs_speed_ki = MECH_LHS_SPEED_KI;
  c.lhs_speed_kd = MECH_LHS_SPEED_KD;
  c.lhs_speed_integral_limit = MECH_SPEED_INTEGRAL_LIMIT_LHS;
  c.lhs_ff.rpm_zero_deadband = MECH_LHS_RPM_ZERO_DEADBAND;
  c.lhs_ff.rpm_stopped_thresh = MECH_LHS_RPM_STOPPED_THRESH;
  c.lhs_ff.pos.run_intercept_duty = MECH_LHS_POS_RUN_INTERCEPT_DUTY;
  c.lhs_ff.pos.run_slope_duty_per_rpm = MECH_LHS_POS_RUN_SLOPE_DUTY_PER_RPM;
  c.lhs_ff.pos.u_break = MECH_LHS_POS_U_BREAK;
  c.lhs_ff.pos.u_move_min = MECH_LHS_POS_U_MOVE_MIN;
  c.lhs_ff.pos.rpm_min_fit = MECH_LHS_POS_RPM_MIN_FIT;
  c.lhs_ff.neg.run_intercept_duty = MECH_LHS_NEG_RUN_INTERCEPT_DUTY;
  c.lhs_ff.neg.run_slope_duty_per_rpm = MECH_LHS_NEG_RUN_SLOPE_DUTY_PER_RPM;
  c.lhs_ff.neg.u_break = MECH_LHS_NEG_U_BREAK;
  c.lhs_ff.neg.u_move_min = MECH_LHS_NEG_U_MOVE_MIN;
  c.lhs_ff.neg.rpm_min_fit = MECH_LHS_NEG_RPM_MIN_FIT;

  c.rhs_stall_guard.enabled = MECH_STALL_GUARD_ENABLED;
  c.rhs_stall_guard.duty_threshold = MECH_STALL_DUTY_THRESH;
  c.rhs_stall_guard.target_rpm_threshold = MECH_STALL_TARGET_RPM_THRESH;
  c.rhs_stall_guard.min_count_delta = MECH_RHS_STALL_MIN_COUNT_DELTA;
  c.rhs_stall_guard.timeout_ms = MECH_STALL_TIMEOUT_MS;
  c.rhs_stall_guard.reset_zero_ms = MECH_STALL_RESET_ZERO_MS;

  c.lhs_stall_guard.enabled = MECH_STALL_GUARD_ENABLED;
  c.lhs_stall_guard.duty_threshold = MECH_STALL_DUTY_THRESH;
  c.lhs_stall_guard.target_rpm_threshold = MECH_STALL_TARGET_RPM_THRESH;
  c.lhs_stall_guard.min_count_delta = MECH_LHS_STALL_MIN_COUNT_DELTA;
  c.lhs_stall_guard.timeout_ms = MECH_STALL_TIMEOUT_MS;
  c.lhs_stall_guard.reset_zero_ms = MECH_STALL_RESET_ZERO_MS;

  // Servo hardware + behavior
  c.pin_servo_lid = PIN_SERVO_LID;
  c.pin_servo_sweep_a = PIN_SERVO_SWEEP_A;
  c.pin_servo_sweep_b = PIN_SERVO_SWEEP_B;

  c.servo_min_deg = SERVO_MIN_DEG;
  c.servo_max_deg = SERVO_MAX_DEG;
  c.servo_deadband_deg = SERVO_DEADBAND_DEG;

  c.lid_ramp_dps = LID_SERVO_RAMP_DPS;
  c.lid_settle_ms = LID_SERVO_SETTLE_MS;
  c.lid_auto_detach_on_closed = LID_SERVO_AUTO_DETACH_ON_CLOSED;

  c.sweep_ramp_dps = SWEEP_SERVO_RAMP_DPS;
  c.sweep_settle_ms = SWEEP_SERVO_SETTLE_MS;
  c.sweep_auto_detach_on_closed = SWEEP_SERVO_AUTO_DETACH_ON_CLOSED;
  c.sweep_mirror_center_deg = SWEEP_SERVO_MIRROR_CENTER_DEG;

  // Safe/home positions
  c.lid_closed_deg = (float)LID_CLOSED_DEG;
  c.sweep_stow_deg = (float)SWEEP_STOW_DEG;
  c.rhs_home_deg = MECH_RHS_HOME_DEG;
  c.lhs_home_deg = MECH_LHS_HOME_DEG;

  return c;
}

MechanismController::Config g_mech_cfg = makeMechanismConfig();
MechanismController g_mech(g_mech_cfg);

/*=============================================================================
  TASK RATES
=============================================================================*/

// Fast RX parsing
Rate g_comms_rate(RxCOMM_UPDATE_HZ);

// Independent subsystem rates
Rate g_drive_rate(DRIVE_UPDATE_HZ);
Rate g_mech_rate(MECH_UPDATE_HZ);
Rate g_ultrasonic_rate(ULTRASONIC_UPDATE_HZ);

// Telemetry publish rate
Rate g_telemetry_rate(TELEMETRY_UPDATE_HZ);

/*=============================================================================
  COMMAND TRACKING + TIMEOUT FLAGS
=============================================================================*/

// Last applied command sequence (apply each new seq once)
static uint32_t g_last_applied_seq = 0;

// Timeout edge latch
static bool g_in_timeout = false;

/*=============================================================================
  SETUP
=============================================================================*/

void setup() {
  // Serial protocol
  SERIAL_USB.begin(SERIAL_BAUD);
  g_link.begin();

  // Sensors
  g_distance_sensor.begin();

  // Controllers
  g_drive.begin();
  g_mech.begin();
}

/*=============================================================================
  LOOP
=============================================================================*/

void loop() {
  const uint32_t now_ms = millis();

  // 1) RX TASK
  if (g_comms_rate.ready(now_ms)) {
    g_link.RxTick(now_ms);

    if (g_link.hasCommand()) {
      const CommandFrame& cmd = g_link.latestCommand();

      if (cmd.seq != g_last_applied_seq) {
        g_last_applied_seq = cmd.seq;

        // Route host command to each subsystem
        g_drive.setCommand(cmd.drive);
        g_mech.setCommand(cmd.mech, now_ms);
      }
    }
  }

  // 2) TIMEOUT SAFETY
  const bool timed_out = g_link.commandTimedOut(now_ms);

  if (timed_out && !g_in_timeout) {
    g_in_timeout = true;
    g_drive.stop();
    g_mech.stopSafe(now_ms);
  } else if (!timed_out) {
    g_in_timeout = false;
  }

  // 3) CONTROL TASKS
  if (g_drive_rate.ready(now_ms)) {
    g_drive.tick(now_ms);
  }

  if (g_mech_rate.ready(now_ms)) {
    g_mech.tick(now_ms);
  }

  // 4) SENSOR TASKS
  if (g_ultrasonic_rate.ready(now_ms)) {
    g_distance_sensor.tick(now_ms);
  }

  // 5) TX TELEMETRY TASK
  if (g_telemetry_rate.ready(now_ms)) {
    TelemetryFrame t;
    t.arduino_time_ms = now_ms;
    t.ack_seq = g_link.ackSeq();

    // Drive telemetry
    const auto& drive_state = g_drive.getState();
    if (drive_state.valid_feedback) {
      t.wheel.left_rpm  = drive_state.meas_left_rpm;
      t.wheel.right_rpm = drive_state.meas_right_rpm;
    } else {
      t.wheel.left_rpm  = NAN;
      t.wheel.right_rpm = NAN;
    }
    t.wheel.left_duty = drive_state.duty_left;
    t.wheel.right_duty = drive_state.duty_right;
    t.wheel.left_target_rpm = drive_state.target_left_rpm;
    t.wheel.right_target_rpm = drive_state.target_right_rpm;
    t.wheel.left_stall_fault = drive_state.stall_left;
    t.wheel.right_stall_fault = drive_state.stall_right;
    t.wheel.left_stall_dir = drive_state.stall_left_dir;
    t.wheel.right_stall_dir = drive_state.stall_right_dir;

    // Mechanism telemetry
    g_mech.fillTelemetry(t.mech);

    // Ultrasonic telemetry
    const auto& us = g_distance_sensor.getState();
    t.ultrasonic.valid = us.valid;
    t.ultrasonic.distance_in = us.valid ? us.distance_in : NAN;

    // Optional RX debug note
    t.note = g_link.debugNote(now_ms);

    g_link.TxTick(t);
  }
}
