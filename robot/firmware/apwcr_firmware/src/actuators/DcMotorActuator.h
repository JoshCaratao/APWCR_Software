#pragma once
#include <Arduino.h>

/*
===============================================================================
  DcMotorActuator.h
===============================================================================

  Purpose:
  Thin wrapper for a DRV8871-style brushed DC motor driver using two control
  inputs (IN1 + IN2).

  Responsibilities:
  - Configure motor output pins
  - Accept normalized duty commands in [-1, 1]
  - Convert signed duty into DRV8871 IN1/IN2 states
  - Expose the last commanded duty/PWM for debugging

  This class does not perform closed-loop control.
===============================================================================
*/

class DcMotorActuator {
public:
  // pin_in1 / pin_in2 are the two DRV8871 logic inputs.
  DcMotorActuator(uint8_t pin_in1,
                  uint8_t pin_in2,
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

  uint8_t _pin_in1;
  uint8_t _pin_in2;
  bool _invert;

  uint8_t _pwm_min;
  uint8_t _pwm_max;

  float _duty_cmd = 0.0f;
  int _pwm_cmd = 0;
};
