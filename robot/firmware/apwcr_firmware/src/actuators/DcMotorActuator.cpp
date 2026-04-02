#include "actuators/DcMotorActuator.h"
#include <math.h>  // fabsf

/*
===============================================================================
  DcMotorActuator.cpp
===============================================================================

  DRV8871 truth table:
    - IN1=0, IN2=0   -> Coast
    - IN1=1, IN2=1   -> Brake
    - IN1=1, IN2=0   -> One direction
    - IN1=0, IN2=1   -> Opposite direction

  To get symmetric speed control in both directions, PWM must alternate between
  a drive state and coast, not between a drive state and brake.

  This implementation maps:
    duty > 0  -> PWM IN1, hold IN2 LOW
    duty < 0  -> hold IN1 LOW, PWM IN2
    duty = 0  -> coast()
===============================================================================
*/

DcMotorActuator::DcMotorActuator(uint8_t pin_in1,
                                 uint8_t pin_in2,
                                 bool invert,
                                 uint8_t pwm_min,
                                 uint8_t pwm_max)
: _pin_in1(pin_in1),
  _pin_in2(pin_in2),
  _invert(invert),
  _pwm_min(pwm_min),
  _pwm_max(pwm_max)
{
  // Guard against swapped bounds.
  if (_pwm_max < _pwm_min) {
    uint8_t tmp = _pwm_max;
    _pwm_max = _pwm_min;
    _pwm_min = tmp;
  }
}

void DcMotorActuator::begin() {
  pinMode(_pin_in1, OUTPUT);
  pinMode(_pin_in2, OUTPUT);
  coast();
}

float DcMotorActuator::clampDuty_(float d) const {
  if (d > 1.0f) return 1.0f;
  if (d < -1.0f) return -1.0f;
  return d;
}

uint8_t DcMotorActuator::dutyToPwm_(float abs_duty) const {
  // Map normalized magnitude to an 8-bit PWM command.
  if (abs_duty <= 0.0f) return 0;

  const float span = (float)(_pwm_max - _pwm_min);
  int pwm = (int)(_pwm_min + abs_duty * span + 0.5f);

  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  return (uint8_t)pwm;
}

void DcMotorActuator::setDuty(float duty) {
  // Clamp into the normalized command range first.
  duty = clampDuty_(duty);

  // Optional inversion lets higher-level control keep a consistent sign
  // convention even if motor wiring is mirrored side-to-side.
  if (_invert) {
    duty = -duty;
  }

  // Save the signed duty that will actually be applied.
  _duty_cmd = duty;

  if (duty == 0.0f) {
    coast();
    return;
  }

  // Convert command magnitude into PWM magnitude.
  const uint8_t pwm = dutyToPwm_(fabsf(duty));

  if (duty > 0.0f) {
    // Positive duty: PWM IN1 while holding IN2 low.
    analogWrite(_pin_in1, pwm);
    digitalWrite(_pin_in2, LOW);
  } else {
    // Negative duty: PWM IN2 while holding IN1 low.
    digitalWrite(_pin_in1, LOW);
    analogWrite(_pin_in2, pwm);
  }

  _pwm_cmd = (int)pwm;
}

void DcMotorActuator::coast() {
  // Both inputs low -> coast.
  digitalWrite(_pin_in1, LOW);
  digitalWrite(_pin_in2, LOW);

  _duty_cmd = 0.0f;
  _pwm_cmd = 0;
}

void DcMotorActuator::brake() {
  // Both inputs high -> active brake.
  digitalWrite(_pin_in1, HIGH);
  digitalWrite(_pin_in2, HIGH);

  _duty_cmd = 0.0f;
  _pwm_cmd = 255;
}
