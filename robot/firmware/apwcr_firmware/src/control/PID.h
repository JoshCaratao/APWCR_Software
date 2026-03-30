#pragma once
#include <Arduino.h>

/*
===============================================================================
  PID.h
===============================================================================

  PURPOSE
  -------
  Simple reusable PID controller for one control loop instance.
  Intended use: one PID object per motor/wheel.

  FEATURES
  --------
  - Full PID (P, I, D)
  - Output clamping
  - Integral clamping (anti-windup)

  TUNING
  ------
  - Disable I by setting KI = 0
  - Disable D by setting KD = 0

  USAGE
  -----
  1) Construct PID with gains and limits
  2) Call reset() when enabling control or changing modes
  3) Call update(setpoint, measurement, dt_s) at fixed loop rate
===============================================================================
*/

class PID {
public:
  PID(float kp, float ki, float kd,
      float out_min, float out_max,
      float i_min, float i_max);

  // Update gains at runtime (useful during tuning)
  void setGains(float kp, float ki, float kd);

  // Clear controller memory (integral + derivative history)
  void reset();

  // Return controller output (clamped to [out_min, out_max])
  float update(float setpoint, float measurement, float dt_s);

private:
  float clamp_(float x, float lo, float hi);

  // Gains
  float _kp;
  float _ki;
  float _kd;

  // Output limits
  float _out_min;
  float _out_max;

  // Integral state limits (state clamp before gain application)
  float _i_min;
  float _i_max;

  // Internal state
  float _integral;
  float _prev_error;
  bool _has_prev_error;
};
