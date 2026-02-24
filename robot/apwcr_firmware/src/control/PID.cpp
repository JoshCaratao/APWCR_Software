#include "control/PID.h"

/*
===============================================================================
  PID.cpp
===============================================================================

  Notes:
  - dt_s must be > 0 for a valid update.
  - First update skips derivative term to avoid startup spike.
===============================================================================
*/

PID::PID(float kp, float ki, float kd,
         float out_min, float out_max,
         float i_min, float i_max)
: _kp(kp),
  _ki(ki),
  _kd(kd),
  _out_min(out_min),
  _out_max(out_max),
  _i_min(i_min),
  _i_max(i_max),
  _integral(0.0f),
  _prev_error(0.0f),
  _has_prev_error(false)
{
  // Guard swapped output bounds
  if (_out_max < _out_min) {
    float t = _out_max;
    _out_max = _out_min;
    _out_min = t;
  }

  // Guard swapped integral bounds
  if (_i_max < _i_min) {
    float t = _i_max;
    _i_max = _i_min;
    _i_min = t;
  }
}

void PID::setGains(float kp, float ki, float kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

void PID::reset() {
  _integral = 0.0f;
  _prev_error = 0.0f;
  _has_prev_error = false;
}

float PID::clamp_(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float PID::update(float setpoint, float measurement, float dt_s) {
  if (dt_s <= 0.0f) {
    return 0.0f;
  }

  float error = setpoint - measurement;

  // Proportional term
  float p = _kp * error;

  // Integral term with state clamp (anti-windup)
  _integral += error * dt_s;
  _integral = clamp_(_integral, _i_min, _i_max);
  float i = _ki * _integral;

  // Derivative term
  float d = 0.0f;
  if (_has_prev_error) {
    float de_dt = (error - _prev_error) / dt_s;
    d = _kd * de_dt;
  }

  _prev_error = error;
  _has_prev_error = true;

  // Total output with clamp
  float out = p + i + d;
  out = clamp_(out, _out_min, _out_max);
  return out;
}
