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
    - RPM mode: side-specific feedforward + speed PID (rpm -> duty)
    - POS_DEG mode: side-specific cascaded control
        position PID (deg -> target rpm)
        feedforward + speed PID (rpm -> duty)
    - Feedforward assist:
        uses a running-region linear fit plus explicit startup/low-speed floors.
        if a nonzero RPM target is requested while the mechanism is effectively
        stopped, enforce a minimum breakaway duty before handing regulation
        back to the RPM PID loop.

  Servos:
    - Lid: independent setpoint
    - Sweeper A/B: mirrored setpoint from one logical command

  MIRROR MAPPING
  --------------
    sweep_b = 2*sweep_mirror_center_deg - sweep_a

  SAFETY
  ------
  stopSafe():
    - coasts both mechanism motors and leaves them at their current position
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

  _rhs_enc(cfg.pin_enc_rhs_a, cfg.pin_enc_rhs_b, cfg.counts_per_rev_rhs,
           cfg.invert_rhs_encoder, cfg.rhs_encoder_rpm_filter_alpha),
  _lhs_enc(cfg.pin_enc_lhs_a, cfg.pin_enc_lhs_b, cfg.counts_per_rev_lhs,
           cfg.invert_lhs_encoder, cfg.lhs_encoder_rpm_filter_alpha),

  _rhs_pos_pid(
    cfg.rhs_pos_kp, cfg.rhs_pos_ki, cfg.rhs_pos_kd,
    -cfg.rhs_max_abs_rpm, +cfg.rhs_max_abs_rpm,
    -cfg.rhs_pos_integral_limit, +cfg.rhs_pos_integral_limit
  ),
  _lhs_pos_pid(
    cfg.lhs_pos_kp, cfg.lhs_pos_ki, cfg.lhs_pos_kd,
    -cfg.lhs_max_abs_rpm, +cfg.lhs_max_abs_rpm,
    -cfg.lhs_pos_integral_limit, +cfg.lhs_pos_integral_limit
  ),
  _rhs_speed_pid(
    cfg.rhs_speed_kp, cfg.rhs_speed_ki, cfg.rhs_speed_kd,
    -cfg.max_abs_duty, +cfg.max_abs_duty,
    -cfg.rhs_speed_integral_limit, +cfg.rhs_speed_integral_limit
  ),
  _lhs_speed_pid(
    cfg.lhs_speed_kp, cfg.lhs_speed_ki, cfg.lhs_speed_kd,
    -cfg.max_abs_duty, +cfg.max_abs_duty,
    -cfg.lhs_speed_integral_limit, +cfg.lhs_speed_integral_limit
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

float MechanismController::bucketGroundDeg_(float rhs_deg, float lhs_deg) {
  return rhs_deg + lhs_deg;
}

void MechanismController::setSweepLogicalDeg_(float sweep_a_deg, uint32_t now_ms) {
  const float sweep_b_deg = (2.0f * _cfg.sweep_mirror_center_deg) - sweep_a_deg;
  _sweep_servo_a.setTargetDeg(sweep_a_deg, now_ms);
  _sweep_servo_b.setTargetDeg(sweep_b_deg, now_ms);
}

float MechanismController::computeRhsTargetRpm_(float measured_deg, float dt_s) {
  if (_state.rhs_mode == MechMotorMode::DUTY) {
    _rhs_pos_pid.reset();
    return 0.0f;
  }

  if (_state.rhs_mode == MechMotorMode::RPM) {
    _rhs_pos_pid.reset();
    return clamp_(_state.rhs_setpoint, -_cfg.rhs_max_abs_rpm, +_cfg.rhs_max_abs_rpm);
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

    return clamp_(
      _rhs_pos_pid.update(sp, measured_deg, dt_s, integral_enabled),
      -_cfg.rhs_max_abs_rpm,
      +_cfg.rhs_max_abs_rpm
    );
  }

  return 0.0f;
}

float MechanismController::computeLhsTargetRpm_(float measured_deg, float dt_s) {
  if (_state.lhs_mode == MechMotorMode::DUTY) {
    _lhs_pos_pid.reset();
    return 0.0f;
  }

  if (_state.lhs_mode == MechMotorMode::RPM) {
    _lhs_pos_pid.reset();
    return clamp_(_state.lhs_setpoint, -_cfg.lhs_max_abs_rpm, +_cfg.lhs_max_abs_rpm);
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

    return clamp_(
      _lhs_pos_pid.update(sp, measured_deg, dt_s, integral_enabled),
      -_cfg.lhs_max_abs_rpm,
      +_cfg.lhs_max_abs_rpm
    );
  }

  return 0.0f;
}

float MechanismController::computeMechFeedforward_(
    float target_rpm,
    float measured_rpm,
    const MechFeedforwardParams& mech_cfg) const {
  const float abs_target_rpm = fabsf(target_rpm);
  if (abs_target_rpm <= mech_cfg.rpm_zero_deadband) {
    // Near-zero target: command zero so the mechanism does not chatter.
    return 0.0f;
  }

  const bool positive_target = target_rpm > 0.0f;
  const float sign = positive_target ? 1.0f : -1.0f;
  const DirectionFeedforwardParams& dir_cfg =
      positive_target ? mech_cfg.pos : mech_cfg.neg;

  // Running-region linear fit from mechanism no-stop characterization.
  // Before that data exists, these coefficients can stay zero and the explicit
  // floors below provide the same startup / low-speed behavior as before.
  float run_mag =
      dir_cfg.run_intercept_duty +
      dir_cfg.run_slope_duty_per_rpm * abs_target_rpm;
  run_mag = clamp_(run_mag, 0.0f, _cfg.max_abs_duty);

  const bool mechanism_stopped =
      fabsf(measured_rpm) <= mech_cfg.rpm_stopped_thresh;

  if (mechanism_stopped) {
    // Startup from rest: enforce enough duty to break static friction.
    const float ff_mag = (run_mag > dir_cfg.u_break) ? run_mag : dir_cfg.u_break;
    return sign * clamp_(ff_mag, 0.0f, _cfg.max_abs_duty);
  }

  if (abs_target_rpm <= dir_cfg.rpm_min_fit) {
    // Already moving but target is below the reliable running fit region:
    // keep at least the sustaining floor so duty does not collapse too low.
    const float ff_mag =
        (run_mag > dir_cfg.u_move_min) ? run_mag : dir_cfg.u_move_min;
    return sign * clamp_(ff_mag, 0.0f, _cfg.max_abs_duty);
  }

  // Normal characterized running region: use the linear feedforward directly.
  return sign * run_mag;
}

float MechanismController::computeMechDuty_(MechMotorMode mode,
                                            float direct_setpoint,
                                            float target_rpm,
                                            float measured_rpm,
                                            float dt_s,
                                            const MechFeedforwardParams& mech_cfg,
                                            PID& pid,
                                            float& ff_out,
                                            float& pid_out) {
  if (mode == MechMotorMode::DUTY) {
    // Raw duty mode is reserved for low-level testing.
    pid.reset();
    ff_out = 0.0f;
    pid_out = 0.0f;
    return clamp_(direct_setpoint, -_cfg.max_abs_duty, +_cfg.max_abs_duty);
  }

  if (fabsf(target_rpm) <= mech_cfg.rpm_zero_deadband) {
    // Near-zero target: clear PID memory and command zero.
    pid.reset();
    ff_out = 0.0f;
    pid_out = 0.0f;
    return 0.0f;
  }

  // 1) Open-loop mechanism speed feedforward from characterization.
  ff_out = computeMechFeedforward_(target_rpm, measured_rpm, mech_cfg);

  // 2) Closed-loop RPM correction on top of feedforward.
  pid_out = pid.update(target_rpm, measured_rpm, dt_s);

  // 3) Final duty command.
  return clamp_(ff_out + pid_out, -_cfg.max_abs_duty, +_cfg.max_abs_duty);
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
  _rhs_speed_pid.reset();
  _lhs_speed_pid.reset();
  _rhs_stall_guard.reset();
  _lhs_stall_guard.reset();

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
  _state.rhs_cmd_present = cmd.motor_RHS.present;
  _state.lhs_cmd_present = cmd.motor_LHS.present;
  _state.auto_bucket_ground_hold_requested =
      _cfg.auto_bucket_ground_hold_enabled &&
      cmd.motor_RHS.present &&
      !cmd.motor_LHS.present;

  if (cmd.reset_RHS_zero) {
    _rhs_enc.reset(0);
    _rhs_pos_pid.reset();
    _state.rhs_mode = MechMotorMode::DUTY;
    _state.rhs_setpoint = 0.0f;
    _state.rhs_deg = 0.0f;
    _state.rhs_rpm = 0.0f;
    _state.rhs_target_rpm = 0.0f;
    _state.rhs_duty = 0.0f;
    _state.rhs_ff = 0.0f;
    _state.rhs_pid = 0.0f;
    _rhs_motor.setDuty(0.0f);
    _rhs_speed_pid.reset();
    _rhs_stall_guard.reset();
    _state.rhs_stall = false;
    _state.rhs_stall_dir = 0;
    _state.bucket_ground_hold_active = false;
    _state.bucket_ground_hold_target_deg = NAN;
  }

  if (cmd.reset_LHS_zero) {
    // "Set ground level" should zero the bucket angle relative to the ground,
    // not the raw LHS joint angle. With bucket_ground = rhs + lhs, that means
    // the LHS joint must be offset to -rhs at the instant of calibration.
    const float rhs_deg_for_lhs_zero = isfinite(_state.rhs_deg) ? _state.rhs_deg : 0.0f;
    const float lhs_ground_zero_deg = -rhs_deg_for_lhs_zero;
    const int32_t lhs_ground_zero_count =
        (int32_t)((lhs_ground_zero_deg / 360.0f) * _cfg.counts_per_rev_lhs);
    _lhs_enc.reset(lhs_ground_zero_count);
    _lhs_pos_pid.reset();
    _state.lhs_mode = MechMotorMode::DUTY;
    _state.lhs_setpoint = 0.0f;
    _state.lhs_deg = lhs_ground_zero_deg;
    _state.lhs_rpm = 0.0f;
    _state.lhs_target_rpm = 0.0f;
    _state.lhs_duty = 0.0f;
    _state.lhs_ff = 0.0f;
    _state.lhs_pid = 0.0f;
    _lhs_motor.setDuty(0.0f);
    _lhs_speed_pid.reset();
    _lhs_stall_guard.reset();
    _state.lhs_stall = false;
    _state.lhs_stall_dir = 0;
    _state.bucket_ground_hold_active = false;
    _state.bucket_ground_hold_target_deg = NAN;
    _state.bucket_ground_deg = 0.0f;
  }

  // RHS motor command
  if (cmd.motor_RHS.present) {
    const MechMotorMode prev_mode = _state.rhs_mode;
    _state.rhs_mode = cmd.motor_RHS.mode;
    _state.rhs_setpoint = cmd.motor_RHS.value;

    if (prev_mode != _state.rhs_mode || _state.rhs_mode != MechMotorMode::POS_DEG) {
      _rhs_pos_pid.reset();
    }
    if (prev_mode != _state.rhs_mode || _state.rhs_mode == MechMotorMode::DUTY) {
      _rhs_speed_pid.reset();
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
    if (prev_mode != _state.lhs_mode || _state.lhs_mode == MechMotorMode::DUTY) {
      _lhs_speed_pid.reset();
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
  _state.bucket_ground_deg = bucketGroundDeg_(rhs_enc_state.degrees, lhs_enc_state.degrees);
  _state.rhs_rpm = rhs_enc_state.rpm;
  _state.lhs_rpm = lhs_enc_state.rpm;

  MechMotorMode lhs_effective_mode = _state.lhs_mode;
  float lhs_effective_setpoint = _state.lhs_setpoint;

  if (_state.auto_bucket_ground_hold_requested) {
    if (!_state.bucket_ground_hold_active) {
      _state.bucket_ground_hold_target_deg = _state.bucket_ground_deg;
      _state.bucket_ground_hold_active = true;
    }
    lhs_effective_mode = MechMotorMode::POS_DEG;
    lhs_effective_setpoint = _state.bucket_ground_hold_target_deg - _state.rhs_deg;
  } else {
    _state.bucket_ground_hold_active = false;
    _state.bucket_ground_hold_target_deg = NAN;
  }

  // Compute and apply motor outputs
  const float rhs_target_rpm = computeRhsTargetRpm_(rhs_enc_state.degrees, dt_s);
  const MechMotorMode lhs_mode_saved = _state.lhs_mode;
  const float lhs_setpoint_saved = _state.lhs_setpoint;
  _state.lhs_mode = lhs_effective_mode;
  _state.lhs_setpoint = lhs_effective_setpoint;
  const float lhs_target_rpm = computeLhsTargetRpm_(lhs_enc_state.degrees, dt_s);
  _state.lhs_mode = lhs_mode_saved;
  _state.lhs_setpoint = lhs_setpoint_saved;
  float rhs_duty = computeMechDuty_(
      _state.rhs_mode,
      _state.rhs_setpoint,
      rhs_target_rpm,
      rhs_enc_state.rpm,
      dt_s,
      _cfg.rhs_ff,
      _rhs_speed_pid,
      _state.rhs_ff,
      _state.rhs_pid);

  float lhs_duty = computeMechDuty_(
      lhs_effective_mode,
      lhs_effective_setpoint,
      lhs_target_rpm,
      lhs_enc_state.rpm,
      dt_s,
      _cfg.lhs_ff,
      _lhs_speed_pid,
      _state.lhs_ff,
      _state.lhs_pid);

  const float rhs_guard_target =
      (_state.rhs_mode == MechMotorMode::DUTY)
        ? (rhs_duty * _cfg.rhs_max_abs_rpm)
        : rhs_target_rpm;

  const float lhs_guard_target =
      (_state.lhs_mode == MechMotorMode::DUTY)
        ? (lhs_duty * _cfg.lhs_max_abs_rpm)
        : lhs_target_rpm;

  rhs_duty = _rhs_stall_guard.apply(
      rhs_duty,
      rhs_guard_target,
      rhs_enc_state.count,
      now_ms,
      _cfg.rhs_stall_guard);

  lhs_duty = _lhs_stall_guard.apply(
      lhs_duty,
      lhs_guard_target,
      lhs_enc_state.count,
      now_ms,
      _cfg.lhs_stall_guard);

  if (_rhs_stall_guard.faulted()) {
    _rhs_speed_pid.reset();
    _rhs_pos_pid.reset();
    _state.rhs_ff = 0.0f;
    _state.rhs_pid = 0.0f;
  }

  if (_lhs_stall_guard.faulted()) {
    _lhs_speed_pid.reset();
    _lhs_pos_pid.reset();
    _state.lhs_ff = 0.0f;
    _state.lhs_pid = 0.0f;
  }

  _rhs_motor.setDuty(rhs_duty);
  _lhs_motor.setDuty(lhs_duty);

  _state.rhs_target_rpm = rhs_target_rpm;
  _state.lhs_target_rpm = lhs_target_rpm;
  _state.rhs_duty = rhs_duty;
  _state.lhs_duty = lhs_duty;
  _state.rhs_stall = _rhs_stall_guard.faulted();
  _state.lhs_stall = _lhs_stall_guard.faulted();
  _state.rhs_stall_dir = _rhs_stall_guard.faultDir();
  _state.lhs_stall_dir = _lhs_stall_guard.faultDir();

  // Servo updates
  _lid_servo.tick(now_ms);
  _sweep_servo_a.tick(now_ms);
  _sweep_servo_b.tick(now_ms);

  _state.lid_deg = _lid_servo.getState().current_deg;
  _state.sweep_deg = _sweep_servo_a.getState().current_deg;  // logical value
}

void MechanismController::stopSafe(uint32_t now_ms) {
  // Clear motor intent and leave both mechanisms where they are. On comms
  // loss or host shutdown we want the motors to coast in place rather than
  // automatically driving back to a home/stow angle.
  _state.rhs_mode = MechMotorMode::DUTY;
  _state.lhs_mode = MechMotorMode::DUTY;
  _state.rhs_setpoint = 0.0f;
  _state.lhs_setpoint = 0.0f;

  _rhs_pos_pid.reset();
  _lhs_pos_pid.reset();
  _rhs_speed_pid.reset();
  _lhs_speed_pid.reset();
  _rhs_stall_guard.reset();
  _lhs_stall_guard.reset();

  // Immediate motor safe output
  _rhs_motor.coast();
  _lhs_motor.coast();
  _state.rhs_duty = 0.0f;
  _state.lhs_duty = 0.0f;
  _state.rhs_target_rpm = 0.0f;
  _state.lhs_target_rpm = 0.0f;
  _state.rhs_ff = 0.0f;
  _state.lhs_ff = 0.0f;
  _state.rhs_pid = 0.0f;
  _state.lhs_pid = 0.0f;
  _state.rhs_stall = false;
  _state.lhs_stall = false;
  _state.rhs_stall_dir = 0;
  _state.lhs_stall_dir = 0;
  _state.rhs_cmd_present = false;
  _state.lhs_cmd_present = false;
  _state.auto_bucket_ground_hold_requested = false;
  _state.bucket_ground_hold_active = false;
  _state.bucket_ground_hold_target_deg = NAN;

  // Servo safe targets
  _lid_servo.setTargetDeg(_cfg.lid_closed_deg, now_ms);
  setSweepLogicalDeg_(_cfg.sweep_stow_deg, now_ms);
}

void MechanismController::fillTelemetry(MechanismState& mech_out) const {
  mech_out.servo_LID_deg = _state.lid_deg;
  mech_out.servo_SWEEP_deg = _state.sweep_deg;   // logical sweep angle
  mech_out.motor_RHS_deg = _state.rhs_deg;
  mech_out.motor_LHS_deg = _state.lhs_deg;
  mech_out.bucket_ground_deg = _state.bucket_ground_deg;
  mech_out.motor_RHS_target_rpm = _state.rhs_target_rpm;
  mech_out.motor_LHS_target_rpm = _state.lhs_target_rpm;
  mech_out.motor_RHS_rpm = _state.rhs_rpm;
  mech_out.motor_LHS_rpm = _state.lhs_rpm;
  mech_out.motor_RHS_duty = _state.rhs_duty;
  mech_out.motor_LHS_duty = _state.lhs_duty;
  mech_out.motor_RHS_stall_fault = _state.rhs_stall;
  mech_out.motor_LHS_stall_fault = _state.lhs_stall;
}
