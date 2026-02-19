#pragma once
#include <Arduino.h>

/*
===============================================================================
  DcMotorActuator.h
===============================================================================

  PURPOSE
  -------
  Thin hardware wrapper for one brushed DC motor driven by DRV8871.

  Assumed wiring (matches your current Pins.h comments):
    - IN1 = direction GPIO
    - IN2 = PWM GPIO

  Responsibilities:
    - Initialize motor driver pins
    - Accept normalized duty command in [-1.0, +1.0]
    - Convert duty to direction + PWM output
    - Provide explicit coast() and brake() helpers

  Notes:
    - This class does NOT do closed-loop control.
    - PID / speed control should live in DriveController (or other controller).
===============================================================================
*/

class DcMotorActuator {
public:
  /*
    pin_dir:
      DRV8871 IN1 pin (direction signal)

    pin_pwm:
      DRV8871 IN2 pin (PWM signal)

    invert:
      If true, flips sign of commanded duty
      (useful when motor wiring polarity differs side-to-side)

    pwm_min / pwm_max:
      Output clamp range when duty != 0.
      Typical full range is 0..255 on Arduino Mega.
  */
  DcMotorActuator(uint8_t pin_dir,
                  uint8_t pin_pwm,
                  bool invert = false,
                  uint8_t pwm_min = 0,
                  uint8_t pwm_max = 255);

  // Configure GPIO and force safe stopped state (coast).
  void begin();

  /*
    Set normalized duty command.

    duty:
      -1.0 = full reverse
       0.0 = stop (coast)
      +1.0 = full forward
  */
  void setDuty(float duty);

  // Explicit stop modes (DRV8871 behavior)
  void coast();   // IN1=LOW, IN2=LOW
  void brake();   // IN1=HIGH, IN2=HIGH

  // Optional runtime polarity update.
  void setInverted(bool invert) { _invert = invert; }

  // Debug/introspection
  float dutyCmd() const { return _duty_cmd; }
  int pwmCmd() const { return _pwm_cmd; }

private:
  float clampDuty_(float d) const;
  uint8_t dutyToPwm_(float abs_duty) const;

  uint8_t _pin_dir;
  uint8_t _pin_pwm;

  bool _invert;

  uint8_t _pwm_min;
  uint8_t _pwm_max;

  // Last command values (for telemetry/debug)
  float _duty_cmd = 0.0f;
  int _pwm_cmd = 0;
};
