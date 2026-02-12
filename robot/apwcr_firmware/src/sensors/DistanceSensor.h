#pragma once

#include <Arduino.h>

// Forward declaration: we only store a pointer to the Martinsos class
class UltraSonicDistanceSensor;

class DistanceSensor {
public:
  struct State {
    float distance_in = 0.0f;       // last distance in inches
    bool  valid = false;            // last reading valid?
    uint32_t last_update_ms = 0;    // millis() when last measurement happened
    float distance_cm = -1.0f;      // last raw cm value (-1 if invalid)
  };

  /*
    Constructor

    trig_pin / echo_pin:
      Pins connected to the HC-SR04.

    max_distance_cm:
      Maximum measurable distance (library will not measure beyond this).

    max_timeout_us:
      Hard timeout for pulseIn() to prevent long blocking.

    min_valid_in / max_valid_in:
      Simple sanity bounds for accepting a reading.
  */
  DistanceSensor(uint8_t trig_pin,
                 uint8_t echo_pin,
                 uint16_t max_distance_cm = 400,
                 uint32_t max_timeout_us = 30000,
                 float min_valid_in = 0.8f,
                 float max_valid_in = 160.0f);

  ~DistanceSensor();

  void begin();                      // kept for consistency with other modules
  void tick(uint32_t now_ms);        // measure using default temperature
  void tick(uint32_t now_ms, float temp_c);  // measure using provided temperature

  const State& getState() const { return _state; }

  uint32_t ageMs(uint32_t now_ms) const {
    return now_ms - _state.last_update_ms;
  }

private:
  void measure_(uint32_t now_ms, const float* temp_c);

  State _state;

  float _min_valid_in = 0.8f;
  float _max_valid_in = 160.0f;

  UltraSonicDistanceSensor* _sonar = nullptr;
};
