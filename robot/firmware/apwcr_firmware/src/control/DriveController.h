#pragma once
#include <Arduino.h>

#include "comms/Messages.h"
#include "actuators/DcMotorActuator.h"
#include "sensors/EncoderSensor.h"
#include "control/PID.h"

/*
===============================================================================
  DriveController.h
===============================================================================

  PURPOSE
  -------
  Closed-loop differential drive controller.

  Responsibilities:
    - Accept high-level drive command (linear ft/s, angular deg/s)
    - Convert to left/right wheel speed targets
    - Convert wheel targets to RPM
    - Read encoder RPM feedback
    - Run PID per drive wheel in RPM
    - Apply side/direction compensation to account for real hardware asymmetry
    - Command drive motors

  NOTES
  -----
  - Hardware mapping and tuning are passed in through Config.
  - This controller manages 2 drive motors (LHS + RHS).
===============================================================================
*/

class DriveController {
public:
  struct Config {
    // Motor pins
    uint8_t pin_lhs_dir = 0;
    uint8_t pin_lhs_pwm = 0;
    uint8_t pin_rhs_dir = 0;
    uint8_t pin_rhs_pwm = 0;

    // Encoder pins
    uint8_t pin_enc_lhs_a = 0;
    uint8_t pin_enc_lhs_b = 0;
    uint8_t pin_enc_rhs_a = 0;
    uint8_t pin_enc_rhs_b = 0;

    // Direction inversions
    bool invert_lhs_motor = false;
    bool invert_rhs_motor = false;
    bool invert_lhs_encoder = false;
    bool invert_rhs_encoder = false;

    // Geometry / conversion
    float track_width_ft = 1.0f;
    float wheel_circumference_ft = 1.0f;
    float counts_per_wheel_rev = 1.0f;

    // Limits
    float max_linear_ftps = 1.0f;
    float max_angular_dps = 90.0f;

    // PID tuning
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integral_limit = 1.0f;

    // Output compensation
    // These correct real side/direction asymmetry after PID output is computed.
    float lhs_fwd_scale = 1.1f;
    float lhs_rev_scale = 1.0f;
    float rhs_fwd_scale = 1.1f;
    float rhs_rev_scale = 1.0f;

    float lhs_fwd_ff = 0.0f;
    float lhs_rev_ff = 0.0f;
    float rhs_fwd_ff = 0.0f;
    float rhs_rev_ff = 0.0f;
  };

  struct State {
    float cmd_linear_ftps = 0.0f;
    float cmd_angular_dps = 0.0f;

    float target_left_ftps = 0.0f;
    float target_right_ftps = 0.0f;
    float target_left_rpm = 0.0f;
    float target_right_rpm = 0.0f;

    float meas_left_rpm = 0.0f;
    float meas_right_rpm = 0.0f;

    float duty_left = 0.0f;
    float duty_right = 0.0f;

    bool valid_feedback = false;
    uint32_t last_tick_ms = 0;
  };

  explicit DriveController(const Config& cfg);

  void begin();
  void setCommand(const DriveCommand& cmd);
  void tick(uint32_t now_ms);
  void stop();

  const State& getState() const { return _state; }

private:
  float clamp_(float x, float lo, float hi);
  void computeWheelTargets_();
  void updateEncoderFeedback_(uint32_t now_ms);

  // Apply per-side, per-direction scale/feedforward compensation.
  float applyCompensation_(float duty, bool is_lhs);

  Config _cfg;
  State _state;
  bool _started = false;

  DcMotorActuator _motor_lhs;
  DcMotorActuator _motor_rhs;

  EncoderSensor _enc_lhs;
  EncoderSensor _enc_rhs;

  PID _pid_lhs;
  PID _pid_rhs;
};
