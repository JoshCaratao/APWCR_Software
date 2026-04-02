// src/control/MechanismController.cpp
#include "control/MechanismController.h"
#include <math.h>

/*
===============================================================================
  MechanismController.cpp
===============================================================================

  CONTROL BEHAVIOR
  ----------------
  Motors:
    - DUTY mode: direct passthrough with clamping
    - POS_DEG mode: side-specific PID(position_deg -> duty)

  Servos:
    - Lid: independent setpoint
    - Sweeper A/B: mirrored setpoint from one logical command

  MIRROR MAPPING
  --------------
    sweep_b = 2*sweep_mirror_center_deg - sweep_a

  SAFETY
  ------
  stopSafe():
    - coasts both mechanism motors
    - commands lid closed
    - commands logical sweeper stow (auto-mirrored to both servos)
===============================================================================
*/

/*=============================================================================
  CONSTRUCTOR
=============================================================================*/

MechanismController::MechanismController(const Config& cfg)
: _cfg(cfg),

  _rhs_motor(cfg.pin_rhs_dir, cfg.pin_rhs_pwm, cfg.invert_rhs_motor, cfg.pwm_min, cfg.pwm_max),
  _lhs_motor(cfg.pin_lhs_dir, cfg.pin_lhs_pwm, cfg.invert_lhs_motor, cfg.pwm_min, cfg.pwm_max),

  _rhs_enc(cfg.pin_enc_rhs_a, cfg.pin_enc_rhs_b, cfg.counts_per_rev_rhs, cfg.invert_rhs_encoder),
  _lhs_enc(cfg.pin_enc_lhs_a, cfg.pin_enc_lhs_b, cfg.counts_per_rev_lhs, cfg.invert_lhs_encoder),

  _rhs_pos_pid(
    cfg.rhs_pos_kp, cfg.rhs_pos_ki, cfg.rhs_pos_kd,
    -cfg.max_abs_duty, +cfg.max_abs_duty,
    -cfg.rhs_pos_integral_limit, +cfg.rhs_pos_integral_limit
  ),
  _lhs_pos_pid(
    cfg.lhs_pos_kp, cfg.lhs_pos_ki, cfg.lhs_pos_kd,
    -cfg.max_abs_duty, +cfg.max_abs_duty,
    -cfg.lhs_pos_integral_limit, +cfg.lhs_pos_integral_limit
  ),

  _lid_servo(
    cfg.pin_servo_lid,
    cfg.servo_min_deg,
    cfg.servo_max_deg,
    cfg.lid_ramp_dps,
    cfg.servo_deadband_deg,
    cfg.lid_settle_ms,
    cfg.lid_auto_detach_on_closed,
    cfg.lid_closed_deg
  ),
  _sweep_servo_a(
    cfg.pin_servo_sweep_a,
    cfg.servo_min_deg,
    cfg.servo_max_deg,
    cfg.sweep_ramp_dps,
    cfg.servo_deadband_deg,
    cfg.sweep_settle_ms,
    cfg.sweep_auto_detach_on_closed,
    cfg.sweep_stow_deg
  ),
  _sweep_servo_b(
    cfg.pin_servo_sweep_b,
    cfg.servo_min_deg,
    cfg.servo_max_deg,
    cfg.sweep_ramp_dps,
    cfg.servo_deadband_deg,
    cfg.sweep_settle_ms,
    cfg.sweep_auto_detach_on_closed,
    cfg.sweep_stow_deg
  )
{
}

/*=============================================================================
  HELPERS
=============================================================================*/

float MechanismController::clamp_(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void MechanismController::setSweepLogicalDeg_(float sweep_a_deg, uint32_t now_ms) {
  const float sweep_b_deg = (2.0f * _cfg.sweep_mirror_center_deg) - sweep_a_deg;
  _sweep_servo_a.setTargetDeg(sweep_a_deg, now_ms);
  _sweep_servo_b.setTargetDeg(sweep_b_deg, now_ms);
}

float MechanismController::computeRhsDuty_(float measured_deg, float dt_s) {
  if (_state.rhs_mode == MechMotorMode::DUTY) {
    return clamp_(_state.rhs_setpoint, -_cfg.max_abs_duty, +_cfg.max_abs_duty);
  }

  if (_state.rhs_mode == MechMotorMode::POS_DEG) {
    const float sp = clamp_(_state.rhs_setpoint, _cfg.rhs_pos_min_deg, _cfg.rhs_pos_max_deg);
    const float err = sp - measured_deg;
    if (fabsf(err) <= _cfg.rhs_pos_deadband_deg) {
      _rhs_pos_pid.reset();
      return 0.0f;
    }

    // Only let the integral term help near the target. This prevents large
    // position errors from winding integral and distorting the move shape.
    const bool integral_enabled = fabsf(err) <= _cfg.rhs_pos_integral_zone_deg;
    if (!integral_enabled) {
      _rhs_pos_pid.clearIntegral();
    }

    return _rhs_pos_pid.update(sp, measured_deg, dt_s, integral_enabled);
  }

  return 0.0f;
}

float MechanismController::computeLhsDuty_(float measured_deg, float dt_s) {
  if (_state.lhs_mode == MechMotorMode::DUTY) {
    return clamp_(_state.lhs_setpoint, -_cfg.max_abs_duty, +_cfg.max_abs_duty);
  }

  if (_state.lhs_mode == MechMotorMode::POS_DEG) {
    const float sp = clamp_(_state.lhs_setpoint, _cfg.lhs_pos_min_deg, _cfg.lhs_pos_max_deg);
    const float err = sp - measured_deg;
    if (fabsf(err) <= _cfg.lhs_pos_deadband_deg) {
      _lhs_pos_pid.reset();
      return 0.0f;
    }

    // Only let the integral term help near the target. This prevents large
    // position errors from winding integral and distorting the move shape.
    const bool integral_enabled = fabsf(err) <= _cfg.lhs_pos_integral_zone_deg;
    if (!integral_enabled) {
      _lhs_pos_pid.clearIntegral();
    }

    return _lhs_pos_pid.update(sp, measured_deg, dt_s, integral_enabled);
  }

  return 0.0f;
}

/*=============================================================================
  PUBLIC API
=============================================================================*/

void MechanismController::begin() {
  _rhs_motor.begin();
  _lhs_motor.begin();

  _rhs_enc.begin();
  _lhs_enc.begin();

  // The RHS arm is expected to boot in its stowed pose.
  // Preload the encoder so startup position already reads as stow angle.
  const int32_t rhs_home_count =
      (int32_t)((_cfg.rhs_home_deg / 360.0f) * _cfg.counts_per_rev_rhs);
  _rhs_enc.reset(rhs_home_count);

  _rhs_pos_pid.reset();
  _lhs_pos_pid.reset();

  _lid_servo.begin(_cfg.lid_closed_deg);

  // Initialize sweeper with logical stow + mirror mapping
  _sweep_servo_a.begin(_cfg.sweep_stow_deg);
  const float sweep_b_init = (2.0f * _cfg.sweep_mirror_center_deg) - _cfg.sweep_stow_deg;
  _sweep_servo_b.begin(sweep_b_init);

  _state = State();
  _state.started = true;
  _state.last_tick_ms = millis();

  // Default to safe home behavior on first tick unless commanded otherwise.
  _state.rhs_mode = MechMotorMode::POS_DEG;
  _state.lhs_mode = MechMotorMode::POS_DEG;
  _state.rhs_setpoint = _cfg.rhs_home_deg;
  _state.lhs_setpoint = _cfg.lhs_home_deg;
}

void MechanismController::setCommand(const MechanismCommand& cmd, uint32_t now_ms) {
  if (cmd.reset_RHS_zero) {
    _rhs_enc.reset(0);
    _rhs_pos_pid.reset();
    _state.rhs_mode = MechMotorMode::DUTY;
    _state.rhs_setpoint = 0.0f;
    _state.rhs_deg = 0.0f;
    _state.rhs_rpm = 0.0f;
    _state.rhs_duty = 0.0f;
    _rhs_motor.setDuty(0.0f);
  }

  if (cmd.reset_LHS_zero) {
    _lhs_enc.reset(0);
    _lhs_pos_pid.reset();
    _state.lhs_mode = MechMotorMode::DUTY;
    _state.lhs_setpoint = 0.0f;
    _state.lhs_deg = 0.0f;
    _state.lhs_rpm = 0.0f;
    _state.lhs_duty = 0.0f;
    _lhs_motor.setDuty(0.0f);
  }

  // RHS motor command
  if (cmd.motor_RHS.present) {
    const MechMotorMode prev_mode = _state.rhs_mode;
    _state.rhs_mode = cmd.motor_RHS.mode;
    _state.rhs_setpoint = cmd.motor_RHS.value;

    // Reset PID when mode changes or when leaving POS_DEG mode
    if (prev_mode != _state.rhs_mode || _state.rhs_mode != MechMotorMode::POS_DEG) {
      _rhs_pos_pid.reset();
    }
  }

  // LHS motor command
  if (cmd.motor_LHS.present) {
    const MechMotorMode prev_mode = _state.lhs_mode;
    _state.lhs_mode = cmd.motor_LHS.mode;
    _state.lhs_setpoint = cmd.motor_LHS.value;

    if (prev_mode != _state.lhs_mode || _state.lhs_mode != MechMotorMode::POS_DEG) {
      _lhs_pos_pid.reset();
    }
  }

  // Lid servo command
  if (cmd.servo_LID_present) {
    _lid_servo.setTargetDeg(cmd.servo_LID_deg, now_ms);
  }

  // Sweeper command (logical value mirrored to both servos)
  if (cmd.servo_SWEEP_present) {
    setSweepLogicalDeg_(cmd.servo_SWEEP_deg, now_ms);
  }
}

void MechanismController::tick(uint32_t now_ms) {
  if (!_state.started) return;

  const uint32_t dt_ms = now_ms - _state.last_tick_ms;
  _state.last_tick_ms = now_ms;

  if (dt_ms == 0) return;
  const float dt_s = (float)dt_ms / 1000.0f;

  // Sample mechanism encoders
  _rhs_enc.sample(now_ms);
  _lhs_enc.sample(now_ms);

  const auto& rhs_enc_state = _rhs_enc.getState();
  const auto& lhs_enc_state = _lhs_enc.getState();

  _state.rhs_deg = rhs_enc_state.degrees;
  _state.lhs_deg = lhs_enc_state.degrees;
  _state.rhs_rpm = rhs_enc_state.rpm;
  _state.lhs_rpm = lhs_enc_state.rpm;

  // Compute and apply motor outputs
  const float rhs_duty = computeRhsDuty_(rhs_enc_state.degrees, dt_s);
  const float lhs_duty = computeLhsDuty_(lhs_enc_state.degrees, dt_s);

  _rhs_motor.setDuty(rhs_duty);
  _lhs_motor.setDuty(lhs_duty);

  _state.rhs_duty = rhs_duty;
  _state.lhs_duty = lhs_duty;

  // Servo updates
  _lid_servo.tick(now_ms);
  _sweep_servo_a.tick(now_ms);
  _sweep_servo_b.tick(now_ms);

  _state.lid_deg = _lid_servo.getState().current_deg;
  _state.sweep_deg = _sweep_servo_a.getState().current_deg;  // logical value
}

void MechanismController::stopSafe(uint32_t now_ms) {
  // Reset motor intent
  _state.rhs_mode = MechMotorMode::POS_DEG;
  _state.lhs_mode = MechMotorMode::POS_DEG;
  _state.rhs_setpoint = _cfg.rhs_home_deg;
  _state.lhs_setpoint = _cfg.lhs_home_deg;

  _rhs_pos_pid.reset();
  _lhs_pos_pid.reset();

  // Immediate motor safe output
  _rhs_motor.coast();
  _lhs_motor.coast();
  _state.rhs_duty = 0.0f;
  _state.lhs_duty = 0.0f;

  // Servo safe targets
  _lid_servo.setTargetDeg(_cfg.lid_closed_deg, now_ms);
  setSweepLogicalDeg_(_cfg.sweep_stow_deg, now_ms);
}

void MechanismController::fillTelemetry(MechanismState& mech_out) const {
  mech_out.servo_LID_deg = _state.lid_deg;
  mech_out.servo_SWEEP_deg = _state.sweep_deg;   // logical sweep angle
  mech_out.motor_RHS_deg = _state.rhs_deg;
  mech_out.motor_LHS_deg = _state.lhs_deg;
  mech_out.motor_RHS_rpm = _state.rhs_rpm;
  mech_out.motor_LHS_rpm = _state.lhs_rpm;
  mech_out.motor_RHS_duty = _state.rhs_duty;
  mech_out.motor_LHS_duty = _state.lhs_duty;
}
