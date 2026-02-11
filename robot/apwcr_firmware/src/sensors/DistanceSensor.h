#pragma once

#include <Arduino.h>

// Forward declaration: we only store a pointer in the header
class UltraSonicDistanceSensor;

class DistanceSensor {
public:
  struct State {
    float distance_in = 0.0f;       // last distance in inches
    bool  valid = false;            // last reading valid?
    uint32_t last_update_ms = 0;    // millis() at last measurement
    float distance_cm = -1.0f;      // last raw cm (optional, -1 if invalid)
  };

  // max_distance_cm must match how you want the library to behave.
  // max_timeout_us caps blocking time inside pulseIn (0 means "no extra cap").
  // period_ms rate limits measurements (recommended 50-100 ms).
  DistanceSensor(uint8_t trig_pin,
                 uint8_t echo_pin,
                 uint16_t max_distance_cm = 400,
                 uint32_t max_timeout_us = 30000,
                 uint32_t period_ms = 60,
                 float min_valid_in = 0.8f,
                 float max_valid_in = 160.0f);

  ~DistanceSensor();

  void begin();                       // consistency hook (library sets pinMode in ctor)
  void tick(uint32_t now_ms);         // uses default temp inside Martinsos (19.307C)
  void tick(uint32_t now_ms, float temp_c);

  const State& getState() const { return _state; }

  uint32_t ageMs(uint32_t now_ms) const { return now_ms - _state.last_update_ms; }

private:
  void measure_(uint32_t now_ms, const float* temp_c);

  static float cmToIn_(float cm) { return cm * 0.3937007874f; }

  uint32_t _period_ms = 60;
  uint32_t _next_ms = 0;

  float _min_valid_in = 0.8f;
  float _max_valid_in = 160.0f;

  State _state;

  UltraSonicDistanceSensor* _sonar = nullptr;
};
