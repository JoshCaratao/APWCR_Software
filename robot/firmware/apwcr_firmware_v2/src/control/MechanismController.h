// src/control/MechanismController.h
#pragma once
#include <Arduino.h>

#include "comms/Messages.h"
#include "actuators/DcMotorActuator.h"
#include "actuators/ServoActuator.h"
#include "sensors/EncoderSensor.h"
#include "control/PID.h"

/*
===============================================================================
  MechanismController.h
===============================================================================

  PURPOSE
  -------
  Unified low-level controller for mechanism subsystem:
    - 2 DC mechanism motors (RHS + LHS)
    - Lid servo
    - Dual sweeper servos (single logical command mirrored in firmware)

  DESIGN
  ------
  - Keeps main.cpp clean by centralizing mechanism command handling.
  - Preserves existing comms schema (MechanismCommand / MechanismState).
  - Motor modes:
      * DUTY    -> open-loop duty command
      * RPM     -> feedforward + closed-loop speed control in output RPM
      * POS_DEG -> cascaded position control:
                   position PID (deg -> target RPM) followed by
                   feedforward + speed PID (RPM -> duty)
  - Sweeper mirroring:
      * Host sends one logical sweeper angle.
      * Servo B is computed from Servo A command:
          sweep_b = 2*mirror_center_deg - sweep_a
===============================================================================
*/

class MechanismController {
public:
  struct DirectionFeedforwardParams {
    // Running-region linear fit:
    //   duty_mag = run_intercept_duty + run_slope_duty_per_rpm * |target_rpm|
    // Fill these from no-stop mechanism characterization.
    float run_intercept_duty = 0.0f;
    float run_slope_duty_per_rpm = 0.0f;

    // Startup / low-speed floors from characterization.
    float u_break = 0.0f;      // from-rest breakaway floor
    float u_move_min = 0.0f;   // sustaining floor once already moving
    float rpm_min_fit = 0.0f;  // lowest RPM where the running fit is trusted
  };

  struct MechFeedforwardParams {
    DirectionFeedforwardParams pos;
    DirectionFeedforwardParams neg;

    // Treat very small target RPM as stop to avoid chatter.
    float rpm_zero_deadband = 0.0f;

    // Below this measured RPM magnitude, the mechanism is treated as stopped.
    float rpm_stopped_thresh = 0.0f;
  };

  /*=============================================================================
    CONFIG
  =============================================================================*/
  struct Config {
    // RHS mechanism motor + encoder
    uint8_t pin_rhs_dir = 0;
    uint8_t pin_rhs_pwm = 0;
    uint8_t pin_enc_rhs_a = 0;
    uint8_t pin_enc_rhs_b = 0;
    bool invert_rhs_motor = false;
    bool invert_rhs_encoder = false;
    float counts_per_rev_rhs = 1.0f;
    float rhs_encoder_rpm_filter_alpha = 1.0f;

    // LHS mechanism motor + encoder
    uint8_t pin_lhs_dir = 0;
    uint8_t pin_lhs_pwm = 0;
    uint8_t pin_enc_lhs_a = 0;
    uint8_t pin_enc_lhs_b = 0;
    bool invert_lhs_motor = false;
    bool invert_lhs_encoder = false;
    float counts_per_rev_lhs = 1.0f;
    float lhs_encoder_rpm_filter_alpha = 1.0f;

    // Motor output limits
    uint8_t pwm_min = 0;
    uint8_t pwm_max = 255;
    float max_abs_duty = 1.0f;
    float rhs_max_abs_rpm = 30.0f;
    float lhs_max_abs_rpm = 30.0f;

    // RHS position control (POS_DEG outer loop)
    float rhs_pos_kp = 0.0f;
    float rhs_pos_ki = 0.0f;
    float rhs_pos_kd = 0.0f;
    float rhs_pos_integral_limit = 1.0f;
    float rhs_pos_integral_zone_deg = 5.0f;
    float rhs_pos_deadband_deg = 1.0f;
    float rhs_pos_min_deg = -180.0f;
    float rhs_pos_max_deg = 180.0f;
    float rhs_speed_kp = 0.0f;
    float rhs_speed_ki = 0.0f;
    float rhs_speed_kd = 0.0f;
    float rhs_speed_integral_limit = 1.0f;
    MechFeedforwardParams rhs_ff;

    // LHS position control (POS_DEG outer loop)
    float lhs_pos_kp = 0.0f;
    float lhs_pos_ki = 0.0f;
    float lhs_pos_kd = 0.0f;
    float lhs_pos_integral_limit = 1.0f;
    float lhs_pos_integral_zone_deg = 5.0f;
    float lhs_pos_deadband_deg = 1.0f;
    float lhs_pos_min_deg = -180.0f;
    float lhs_pos_max_deg = 180.0f;
    float lhs_speed_kp = 0.0f;
    float lhs_speed_ki = 0.0f;
    float lhs_speed_kd = 0.0f;
    float lhs_speed_integral_limit = 1.0f;
    MechFeedforwardParams lhs_ff;

    // Servo hardware
    uint8_t pin_servo_lid = 0;
    uint8_t pin_servo_sweep_a = 0;
    uint8_t pin_servo_sweep_b = 0;

    // Servo behavior
    float servo_min_deg = 0.0f;
    float servo_max_deg = 180.0f;
    float servo_deadband_deg = 2.0f;

    float lid_ramp_dps = 25.0f;
    uint32_t lid_settle_ms = 1000;
    bool lid_auto_detach_on_closed = true;

    float sweep_ramp_dps = 10.0f;
    uint32_t sweep_settle_ms = 1000;
    bool sweep_auto_detach_on_closed = true;

    // Sweeper mirror center (for mapping sweep_b = 2*center - sweep_a)
    float sweep_mirror_center_deg = 75.0f;

    // Safe/home positions
    float lid_closed_deg = 0.0f;
    float sweep_stow_deg = 15.0f;   // logical stow in sweep A command space
    float rhs_home_deg = 0.0f;
    float lhs_home_deg = 0.0f;
  };

  /*=============================================================================
    STATE
  =============================================================================*/
  struct State {
    // Motor command intent
    MechMotorMode rhs_mode = MechMotorMode::UNKNOWN;
    MechMotorMode lhs_mode = MechMotorMode::UNKNOWN;
    float rhs_setpoint = 0.0f;   // DUTY, RPM, or POS_DEG
    float lhs_setpoint = 0.0f;   // DUTY, RPM, or POS_DEG
    float rhs_target_rpm = 0.0f; // resolved speed target after mode logic
    float lhs_target_rpm = 0.0f; // resolved speed target after mode logic

    // Motor feedback
    float rhs_deg = NAN;
    float lhs_deg = NAN;
    float rhs_rpm = NAN;
    float lhs_rpm = NAN;

    // Applied duty
    float rhs_duty = 0.0f;
    float lhs_duty = 0.0f;
    float rhs_ff = 0.0f;
    float lhs_ff = 0.0f;
    float rhs_pid = 0.0f;
    float lhs_pid = 0.0f;

    // Servo telemetry (logical sweeper value = sweep A)
    float lid_deg = NAN;
    float sweep_deg = NAN;

    bool started = false;
    uint32_t last_tick_ms = 0;
  };

  explicit MechanismController(const Config& cfg);

  void begin();
  void setCommand(const MechanismCommand& cmd, uint32_t now_ms);
  void tick(uint32_t now_ms);
  void stopSafe(uint32_t now_ms);

  void fillTelemetry(MechanismState& mech_out) const;
  const State& getState() const { return _state; }

private:
  /*=============================================================================
    HELPERS
  =============================================================================*/
  static float clamp_(float x, float lo, float hi);

  float computeRhsTargetRpm_(float measured_deg, float dt_s);
  float computeLhsTargetRpm_(float measured_deg, float dt_s);
  float computeMechFeedforward_(float target_rpm,
                                float measured_rpm,
                                const MechFeedforwardParams& mech_cfg) const;
  float computeMechDuty_(MechMotorMode mode,
                         float direct_setpoint,
                         float target_rpm,
                         float measured_rpm,
                         float dt_s,
                         const MechFeedforwardParams& mech_cfg,
                         PID& pid,
                         float& ff_out,
                         float& pid_out);

  // Apply one logical sweeper setpoint to mirrored physical servos
  void setSweepLogicalDeg_(float sweep_a_deg, uint32_t now_ms);

  /*=============================================================================
    MEMBERS
  =============================================================================*/
  Config _cfg;
  State _state;

  DcMotorActuator _rhs_motor;
  DcMotorActuator _lhs_motor;

  EncoderSensor _rhs_enc;
  EncoderSensor _lhs_enc;

  PID _rhs_pos_pid;
  PID _lhs_pos_pid;
  PID _rhs_speed_pid;
  PID _lhs_speed_pid;

  ServoActuator _lid_servo;
  ServoActuator _sweep_servo_a;
  ServoActuator _sweep_servo_b;
};
