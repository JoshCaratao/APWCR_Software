#include "control/DriveController.h"

#include <math.h>

#include "Params.h"

/*
===============================================================================
  DriveController.cpp
===============================================================================

  Differential-drive equations:
    v_left  = v - omega * (track_width / 2)
    v_right = v + omega * (track_width / 2)

  Units:
    v       = ft/s
    omega   = rad/s

  Compensation:
    Each wheel uses its own characterized feedforward parameters for:
      - positive direction
      - negative direction

    The drive command is formed as:
      duty_cmd = feedforward(target_rpm, measured_rpm) + PID(error_rpm)

    The feedforward intentionally uses:
      - a running-region linear fit
      - a startup breakaway floor from rest
      - a low-speed sustaining floor while already moving
===============================================================================
*/

/*=============================================================================
  CONSTRUCTOR
=============================================================================*/

DriveController::DriveController(const Config& cfg)
: _cfg(cfg),

  _motor_lhs(cfg.pin_lhs_dir, cfg.pin_lhs_pwm, cfg.invert_lhs_motor, PWM_MIN, PWM_MAX),
  _motor_rhs(cfg.pin_rhs_dir, cfg.pin_rhs_pwm, cfg.invert_rhs_motor, PWM_MIN, PWM_MAX),

  _enc_lhs(cfg.pin_enc_lhs_a, cfg.pin_enc_lhs_b, cfg.counts_per_wheel_rev, cfg.invert_lhs_encoder),
  _enc_rhs(cfg.pin_enc_rhs_a, cfg.pin_enc_rhs_b, cfg.counts_per_wheel_rev, cfg.invert_rhs_encoder),

  _pid_lhs(
    cfg.lhs_kp, cfg.lhs_ki, cfg.lhs_kd,
    -1.0f, +1.0f,
    -cfg.integral_limit, +cfg.integral_limit
  ),
  _pid_rhs(
    cfg.rhs_kp, cfg.rhs_ki, cfg.rhs_kd,
    -1.0f, +1.0f,
    -cfg.integral_limit, +cfg.integral_limit
  )
{
}

/*=============================================================================
  HELPERS
=============================================================================*/

float DriveController::clamp_(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void DriveController::computeWheelTargets_() {
  // Convert commanded body motion into left/right wheel linear speeds.
  const float omega_rad_s = _state.cmd_angular_dps * (PI / 180.0f);
  const float half_track = 0.5f * _cfg.track_width_ft;

  float v_left = _state.cmd_linear_ftps - (omega_rad_s * half_track);
  float v_right = _state.cmd_linear_ftps + (omega_rad_s * half_track);

  v_left = clamp_(v_left, -_cfg.max_linear_ftps, _cfg.max_linear_ftps);
  v_right = clamp_(v_right, -_cfg.max_linear_ftps, _cfg.max_linear_ftps);

  _state.target_left_ftps = v_left;
  _state.target_right_ftps = v_right;

  // Convert linear wheel targets into wheel RPM for the feedback loop.
  if (_cfg.wheel_circumference_ft > 0.0f) {
    _state.target_left_rpm = (v_left / _cfg.wheel_circumference_ft) * 60.0f;
    _state.target_right_rpm = (v_right / _cfg.wheel_circumference_ft) * 60.0f;
  } else {
    _state.target_left_rpm = 0.0f;
    _state.target_right_rpm = 0.0f;
  }
}

void DriveController::updateEncoderFeedback_(uint32_t now_ms) {
  _enc_lhs.sample(now_ms);
  _enc_rhs.sample(now_ms);

  const auto& s_l = _enc_lhs.getState();
  const auto& s_r = _enc_rhs.getState();

  _state.meas_left_rpm = s_l.rpm;
  _state.meas_right_rpm = s_r.rpm;

  _state.valid_feedback = s_l.valid_speed && s_r.valid_speed;
}

float DriveController::computeWheelFeedforward_(float target_rpm,
                                                float measured_rpm,
                                                const WheelFeedforwardParams& wheel_cfg) const {
  const float abs_target_rpm = fabsf(target_rpm);
  if (abs_target_rpm < wheel_cfg.rpm_zero_deadband) {
    // Near-zero target: command zero so the wheel does not chatter.
    return 0.0f;
  }

  const bool positive_target = target_rpm > 0.0f;
  const float sign = positive_target ? 1.0f : -1.0f;
  const DirectionFeedforwardParams& dir_cfg =
      positive_target ? wheel_cfg.pos : wheel_cfg.neg;

  // Running-region linear fit from the no-stop data.
  // This fit is only trusted in the characterized moving region, so the
  // intercept may be negative. Clamp to zero before applying explicit floors.
  float run_mag =
      dir_cfg.run_intercept_duty +
      dir_cfg.run_slope_duty_per_rpm * abs_target_rpm;
  run_mag = clamp_(run_mag, 0.0f, 1.0f);

  const bool wheel_stopped = fabsf(measured_rpm) < wheel_cfg.rpm_stopped_thresh;

  if (wheel_stopped) {
    // Startup from rest: enforce a breakaway floor so feedforward does not ask
    // for less than the minimum duty required to begin motion.
    const float ff_mag = (run_mag > dir_cfg.u_break) ? run_mag : dir_cfg.u_break;
    return sign * clamp_(ff_mag, 0.0f, 1.0f);
  }

  if (abs_target_rpm < dir_cfg.rpm_min_fit) {
    // Already moving but requested RPM is below the reliable fit region.
    // Keep at least the sustaining floor so the wheel does not fall out of
    // motion just because the linear fit extrapolates too low.
    const float ff_mag =
        (run_mag > dir_cfg.u_move_min) ? run_mag : dir_cfg.u_move_min;
    return sign * clamp_(ff_mag, 0.0f, 1.0f);
  }

  // Normal characterized running region: use the linear feedforward directly.
  return sign * run_mag;
}

float DriveController::computeWheelDuty_(float target_rpm,
                                         float measured_rpm,
                                         float dt_s,
                                         const WheelFeedforwardParams& wheel_cfg,
                                         PID& pid,
                                         float& ff_out,
                                         float& pid_out) {
  if (fabsf(target_rpm) < wheel_cfg.rpm_zero_deadband) {
    // Near-zero target: clear PID memory and command zero so the wheel
    // does not chatter around stop.
    pid.reset();
    ff_out = 0.0f;
    pid_out = 0.0f;
    return 0.0f;
  }

  // 1) Compute the open-loop duty from the characterized wheel model.
  ff_out = computeWheelFeedforward_(target_rpm, measured_rpm, wheel_cfg);

  // 2) Compute closed-loop RPM error correction on top of feedforward.
  pid_out = pid.update(target_rpm, measured_rpm, dt_s);

  // 3) Sum and clamp into the legal motor duty range.
  return clamp_(ff_out + pid_out, -1.0f, 1.0f);
}

/*=============================================================================
  PUBLIC API
=============================================================================*/

void DriveController::begin() {
  _motor_lhs.begin();
  _motor_rhs.begin();

  _enc_lhs.begin();
  _enc_rhs.begin();

  _pid_lhs.reset();
  _pid_rhs.reset();

  _state = State();
  _state.last_tick_ms = millis();

  _started = true;
}

void DriveController::setCommand(const DriveCommand& cmd) {
  _state.cmd_linear_ftps =
      clamp_(cmd.linear_ftps, -_cfg.max_linear_ftps, _cfg.max_linear_ftps);

  _state.cmd_angular_dps =
      clamp_(cmd.angular_dps, -_cfg.max_angular_dps, _cfg.max_angular_dps);

  computeWheelTargets_();
}

void DriveController::tick(uint32_t now_ms) {
  if (!_started) return;

  const uint32_t dt_ms = now_ms - _state.last_tick_ms;
  _state.last_tick_ms = now_ms;

  if (dt_ms == 0) return;
  const float dt_s = (float)dt_ms / 1000.0f;

  updateEncoderFeedback_(now_ms);

  // Hold motors off until encoder speed becomes valid.
  if (!_state.valid_feedback) {
    _motor_lhs.setDuty(0.0f);
    _motor_rhs.setDuty(0.0f);
    _state.duty_left = 0.0f;
    _state.duty_right = 0.0f;
    _state.ff_left = 0.0f;
    _state.ff_right = 0.0f;
    _state.pid_left = 0.0f;
    _state.pid_right = 0.0f;
    return;
  }

  // Wheel-speed control is done directly in RPM because encoder feedback is
  // naturally measured in RPM and telemetry/debugging are clearer in RPM.
  // Each wheel gets:
  //   1) explicit feedforward from characterization
  //   2) PID error correction on top
  const float duty_l = computeWheelDuty_(
      _state.target_left_rpm,
      _state.meas_left_rpm,
      dt_s,
      _cfg.lhs_ff,
      _pid_lhs,
      _state.ff_left,
      _state.pid_left);

  const float duty_r = computeWheelDuty_(
      _state.target_right_rpm,
      _state.meas_right_rpm,
      dt_s,
      _cfg.rhs_ff,
      _pid_rhs,
      _state.ff_right,
      _state.pid_right);

  _motor_lhs.setDuty(duty_l);
  _motor_rhs.setDuty(duty_r);

  _state.duty_left = duty_l;
  _state.duty_right = duty_r;
}

void DriveController::stop() {
  _state.cmd_linear_ftps = 0.0f;
  _state.cmd_angular_dps = 0.0f;
  _state.target_left_ftps = 0.0f;
  _state.target_right_ftps = 0.0f;
  _state.target_left_rpm = 0.0f;
  _state.target_right_rpm = 0.0f;

  _pid_lhs.reset();
  _pid_rhs.reset();

  _state.ff_left = 0.0f;
  _state.ff_right = 0.0f;
  _state.pid_left = 0.0f;
  _state.pid_right = 0.0f;
  _motor_lhs.coast();
  _motor_rhs.coast();

  _state.duty_left = 0.0f;
  _state.duty_right = 0.0f;
}
