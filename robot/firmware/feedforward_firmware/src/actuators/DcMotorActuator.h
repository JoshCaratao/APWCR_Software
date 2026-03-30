#pragma once
#include <Arduino.h>

/*
===============================================================================
  DcMotorActuator.h
===============================================================================

  Purpose:
  Thin wrapper for a brushed DC motor driver using one direction pin and one
  PWM pin.

  Responsibilities:
  - Configure motor output pins
  - Accept normalized duty commands in [-1, 1]
  - Convert duty into direction + PWM output
  - Expose the last commanded duty/PWM for debugging

  This class does not perform closed-loop control.
===============================================================================
*/

class DcMotorActuator {
public:
  DcMotorActuator(uint8_t pin_dir,
                  uint8_t pin_pwm,
                  bool invert = false,
                  uint8_t pwm_min = 0,
                  uint8_t pwm_max = 255);

  // Configure pins and place the motor in a safe idle state.
  void begin();

  // Apply a normalized command in [-1, 1].
  void setDuty(float duty);

  // Let the motor coast with no drive applied.
  void coast();

  // Actively brake the motor driver outputs.
  void brake();

  // Last commanded values, useful for debug telemetry.
  float dutyCmd() const { return _duty_cmd; }
  int pwmCmd() const { return _pwm_cmd; }

private:
  // Clamp duty into the supported normalized range.
  float clampDuty_(float d) const;

  // Convert |duty| in [0, 1] to a PWM byte.
  uint8_t dutyToPwm_(float abs_duty) const;

  uint8_t _pin_dir;
  uint8_t _pin_pwm;
  bool _invert;

  uint8_t _pwm_min;
  uint8_t _pwm_max;

  float _duty_cmd = 0.0f;
  int _pwm_cmd = 0;
};
