#pragma once
#include <Arduino.h>

#include "comms/Messages.h"
#include "actuators/DcMotorActuator.h"
#include "sensors/EncoderSensor.h"
#include "control/PID.h"
#include "control/StallGuard.h"

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
    - Compute per-wheel feedforward from characterized drive data
    - Run PID per drive wheel in RPM and add it on top of feedforward
    - Command drive motors

  NOTES
  -----
  - Hardware mapping and tuning are passed in through Config.
  - This controller manages 2 drive motors (LHS + RHS).
===============================================================================
*/

class DriveController {
public:
  struct DirectionFeedforwardParams {
    // Running-region linear fit:
    //   duty_mag = run_intercept_duty + run_slope_duty_per_rpm * |target_rpm|
    // The intercept may be negative because the fit is only trusted in the
    // measured running region, not down near zero speed.
    float run_intercept_duty = 0.0f;
    float run_slope_duty_per_rpm = 0.0f;

    // Startup / low-speed floors identified from characterization.
    float u_break = 0.0f;      // from-rest breakaway floor
    float u_move_min = 0.0f;   // minimum sustaining floor once already moving
    float rpm_min_fit = 0.0f;  // lowest RPM where the running fit is trusted
  };

  struct WheelFeedforwardParams {
    DirectionFeedforwardParams pos;
    DirectionFeedforwardParams neg;

    // Treat very small target RPM as zero command to avoid chatter.
    float rpm_zero_deadband = 0.0f;

    // Below this measured RPM magnitude, the wheel is treated as stopped.
    float rpm_stopped_thresh = 0.0f;
  };

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
    float lhs_encoder_rpm_filter_alpha = 1.0f;
    float rhs_encoder_rpm_filter_alpha = 1.0f;

    // Geometry / conversion
    float track_width_ft = 1.0f;
    float wheel_circumference_ft = 1.0f;
    float counts_per_wheel_rev = 1.0f;

    // Limits
    float max_linear_ftps = 1.0f;
    float max_angular_dps = 90.0f;

    // PID tuning (per wheel)
    float lhs_kp = 0.0f;
    float lhs_ki = 0.0f;
    float lhs_kd = 0.0f;
    float rhs_kp = 0.0f;
    float rhs_ki = 0.0f;
    float rhs_kd = 0.0f;
    float integral_limit = 1.0f;

    // Per-wheel feedforward characterization.
    WheelFeedforwardParams lhs_ff;
    WheelFeedforwardParams rhs_ff;

    // Shared stall behavior, with separate guard state per drive motor.
    StallGuardConfig stall_guard;
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
    float ff_left = 0.0f;
    float ff_right = 0.0f;
    float pid_left = 0.0f;
    float pid_right = 0.0f;
    bool stall_left = false;
    bool stall_right = false;
    int8_t stall_left_dir = 0;
    int8_t stall_right_dir = 0;

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
  static float clamp_(float x, float lo, float hi);
  void computeWheelTargets_();
  void updateEncoderFeedback_(uint32_t now_ms);
  float computeWheelFeedforward_(float target_rpm,
                                 float measured_rpm,
                                 const WheelFeedforwardParams& wheel_cfg) const;
  float computeWheelDuty_(float target_rpm,
                          float measured_rpm,
                          float dt_s,
                          const WheelFeedforwardParams& wheel_cfg,
                          PID& pid,
                          float& ff_out,
                          float& pid_out);

  Config _cfg;
  State _state;
  bool _started = false;

  DcMotorActuator _motor_lhs;
  DcMotorActuator _motor_rhs;

  EncoderSensor _enc_lhs;
  EncoderSensor _enc_rhs;

  PID _pid_lhs;
  PID _pid_rhs;
  StallGuard _stall_lhs;
  StallGuard _stall_rhs;
};
