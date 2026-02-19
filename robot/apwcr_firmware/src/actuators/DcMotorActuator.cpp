#include "actuators/DcMotorActuator.h"
#include <math.h>  // fabsf

/*
===============================================================================
  DcMotorActuator.cpp
===============================================================================

  DRV8871 truth table (PH/EN style using IN1 + PWM on IN2):
    - IN1=0, IN2=0   -> Coast
    - IN1=1, IN2=1   -> Brake
    - IN1=1, IN2=PWM -> One direction
    - IN1=0, IN2=PWM -> Opposite direction

  This implementation maps:
    duty > 0  -> IN1 HIGH, IN2 PWM
    duty < 0  -> IN1 LOW,  IN2 PWM
    duty = 0  -> coast()
===============================================================================
*/

DcMotorActuator::DcMotorActuator(uint8_t pin_dir,
                                 uint8_t pin_pwm,
                                 bool invert,
                                 uint8_t pwm_min,
                                 uint8_t pwm_max)
: _pin_dir(pin_dir),
  _pin_pwm(pin_pwm),
  _invert(invert),
  _pwm_min(pwm_min),
  _pwm_max(pwm_max)
{
  // Guard against swapped bounds
  if (_pwm_max < _pwm_min) {
    uint8_t tmp = _pwm_max;
    _pwm_max = _pwm_min;
    _pwm_min = tmp;
  }
}

void DcMotorActuator::begin() {
  pinMode(_pin_dir, OUTPUT);
  pinMode(_pin_pwm, OUTPUT);

  // Safe default state at startup
  coast();
}

float DcMotorActuator::clampDuty_(float d) const {
  if (d > 1.0f) return 1.0f;
  if (d < -1.0f) return -1.0f;
  return d;
}

uint8_t DcMotorActuator::dutyToPwm_(float abs_duty) const {
  // abs_duty expected in [0, 1]
  if (abs_duty <= 0.0f) return 0;

  const float span = (float)(_pwm_max - _pwm_min);
  int pwm = (int)(_pwm_min + abs_duty * span + 0.5f);

  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  return (uint8_t)pwm;
}

void DcMotorActuator::setDuty(float duty) {
  duty = clampDuty_(duty);

  // Optional polarity inversion for mirrored drivetrain/mechanism sides
  if (_invert) duty = -duty;

  _duty_cmd = duty;

  // Stop command uses coast by default
  if (duty == 0.0f) {
    coast();
    return;
  }

  const bool forward = (duty > 0.0f);
  digitalWrite(_pin_dir, forward ? HIGH : LOW);

  const uint8_t pwm = dutyToPwm_(fabsf(duty));
  analogWrite(_pin_pwm, pwm);

  _pwm_cmd = (int)pwm;
}

void DcMotorActuator::coast() {
  digitalWrite(_pin_dir, LOW);
  analogWrite(_pin_pwm, 0);

  _duty_cmd = 0.0f;
  _pwm_cmd = 0;
}

void DcMotorActuator::brake() {
  digitalWrite(_pin_dir, HIGH);
  analogWrite(_pin_pwm, 255);

  // Treat as explicit stop mode for debug view
  _duty_cmd = 0.0f;
  _pwm_cmd = 255;
}
