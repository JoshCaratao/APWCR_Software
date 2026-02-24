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
===============================================================================
*/

DriveController::DriveController(const Config& cfg)
: _cfg(cfg),

  _motor_lhs(cfg.pin_lhs_dir, cfg.pin_lhs_pwm, cfg.invert_lhs_motor, PWM_MIN, PWM_MAX),
  _motor_rhs(cfg.pin_rhs_dir, cfg.pin_rhs_pwm, cfg.invert_rhs_motor, PWM_MIN, PWM_MAX),

  _enc_lhs(cfg.pin_enc_lhs_a, cfg.pin_enc_lhs_b, cfg.counts_per_wheel_rev, cfg.invert_lhs_encoder),
  _enc_rhs(cfg.pin_enc_rhs_a, cfg.pin_enc_rhs_b, cfg.counts_per_wheel_rev, cfg.invert_rhs_encoder),

  _pid_lhs(
    cfg.kp, cfg.ki, cfg.kd,
    -1.0f, +1.0f,
    -cfg.integral_limit, +cfg.integral_limit
  ),
  _pid_rhs(
    cfg.kp, cfg.ki, cfg.kd,
    -1.0f, +1.0f,
    -cfg.integral_limit, +cfg.integral_limit
  )
{
}

float DriveController::clamp_(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

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

void DriveController::computeWheelTargets_() {
  const float omega_rad_s = _state.cmd_angular_dps * (PI / 180.0f);
  const float half_track = 0.5f * _cfg.track_width_ft;

  float v_left = _state.cmd_linear_ftps - (omega_rad_s * half_track);
  float v_right = _state.cmd_linear_ftps + (omega_rad_s * half_track);

  v_left = clamp_(v_left, -_cfg.max_linear_ftps, _cfg.max_linear_ftps);
  v_right = clamp_(v_right, -_cfg.max_linear_ftps, _cfg.max_linear_ftps);

  _state.target_left_ftps = v_left;
  _state.target_right_ftps = v_right;
}

void DriveController::updateEncoderFeedback_(uint32_t now_ms) {
  _enc_lhs.sample(now_ms);
  _enc_rhs.sample(now_ms);

  const auto& s_l = _enc_lhs.getState();
  const auto& s_r = _enc_rhs.getState();

  _state.meas_left_rpm = s_l.rpm;
  _state.meas_right_rpm = s_r.rpm;

  _state.meas_left_ftps = s_l.rps * _cfg.wheel_circumference_ft;
  _state.meas_right_ftps = s_r.rps * _cfg.wheel_circumference_ft;

  _state.valid_feedback = s_l.valid_speed && s_r.valid_speed;
}

void DriveController::tick(uint32_t now_ms) {
  if (!_started) return;

  uint32_t dt_ms = now_ms - _state.last_tick_ms;
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
    return;
  }

  // Simple zero-command behavior: coast and clear PID memory.
  const bool near_zero_cmd =
      (fabsf(_state.target_left_ftps) < 0.01f) &&
      (fabsf(_state.target_right_ftps) < 0.01f);

  if (near_zero_cmd) {
    _pid_lhs.reset();
    _pid_rhs.reset();

    _motor_lhs.setDuty(0.0f);
    _motor_rhs.setDuty(0.0f);

    _state.duty_left = 0.0f;
    _state.duty_right = 0.0f;
    return;
  }

  float duty_l = _pid_lhs.update(_state.target_left_ftps, _state.meas_left_ftps, dt_s);
  float duty_r = _pid_rhs.update(_state.target_right_ftps, _state.meas_right_ftps, dt_s);

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

  _pid_lhs.reset();
  _pid_rhs.reset();

  _motor_lhs.coast();
  _motor_rhs.coast();

  _state.duty_left = 0.0f;
  _state.duty_right = 0.0f;
}
