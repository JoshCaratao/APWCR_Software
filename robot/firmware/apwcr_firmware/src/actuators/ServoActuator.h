// src/actuators/ServoActuator.h
#pragma once

#include <Arduino.h>
#include <Servo.h>

/*
  ServoActuator

  Purpose:
  - Accept a degree setpoint
  - Smoothly ramp the servo toward the target (non-blocking)
  - Optionally auto-detach after reaching the CLOSED setpoint and settling
    (useful when gravity keeps the lid shut)

  Usage pattern:
  - Call setTargetDeg(...) only when a new target is desired (new command/state)
  - Call tick(now_ms) at a fixed rate (for example 50 Hz) to perform ramping
*/

class ServoActuator {
public:
  struct State {
    float target_deg = 90.0f;
    float current_deg = 90.0f;
    bool  is_attached = false;

    bool  at_target = true;
    uint32_t last_update_ms = 0;

    // For settle timing
    uint32_t at_target_since_ms = 0;
  };

  /*
    pin            : servo signal pin
    min_deg/max_deg: clamp limits
    ramp_dps       : degrees per second (0 disables ramping, jumps to target)
  */
  ServoActuator(uint8_t pin, float min_deg, float max_deg, float ramp_dps, float deadband_deg, uint32_t settle_ms, bool auto_detach_on_closed, float closed_deg);


  // Attach and initialize to initial_deg (clamped). Records timestamps.
  void begin(float initial_deg);

  // Attach (if detached) and immediately output current_deg.
  void attach(uint32_t now_ms);

  // Detach (stop PWM pulses). Servo will not hold torque.
  void detach();

  bool isAttached() const { return _state.is_attached; }

  // Set a new desired target (clamped). Does not block.
  // If ramp_dps <= 0, snaps immediately (writes once).
  void setTargetDeg(float deg, uint32_t now_ms);

  // Optional helpers
  void setRampDps(float ramp_dps);

  // Auto-detach logic (commonly enable for lid when gravity holds closed)
  // closed_deg: the "closed" setpoint in degrees
  void setAutoDetachOnClosed(bool enable, float closed_deg);

  // deadband_deg: how close is "at target"
  // settle_ms   : how long it must remain at target before detaching
  void setSettleParams(float deadband_deg, uint32_t settle_ms);

  // Call periodically at a fixed rate (external Rate)
  void tick(uint32_t now_ms);

  const State& getState() const { return _state; }

private:
  float clampDeg_(float deg) const;
  int roundDeg_(float deg) const;

  void updateAtTargetFlags_(uint32_t now_ms);

  Servo _servo;
  uint8_t _pin;

  float _min_deg;
  float _max_deg;

  float _ramp_dps;

  float _deadband_deg;
  uint32_t _settle_ms;
  bool _auto_detach_on_closed;
  float _closed_deg;


  State _state;
};
