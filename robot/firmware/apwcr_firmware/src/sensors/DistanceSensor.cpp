#include "sensors/DistanceSensor.h"
#include <HCSR04.h>  // Martinsos library

/*
  DistanceSensor.cpp

  This is a simple wrapper around the Martinsos HC-SR04 library.

  Responsibilities:
  - Take a measurement when tick() is called
  - Convert cm to inches
  - Validate the reading
  - Store the latest state for other code to read

  Rate limiting is handled in main.cpp using your Rate class.
*/

DistanceSensor::DistanceSensor(uint8_t trig_pin,
                               uint8_t echo_pin,
                               uint16_t max_distance_cm,
                               uint32_t max_timeout_us,
                               float min_valid_in,
                               float max_valid_in)
{
  _min_valid_in = min_valid_in;
  _max_valid_in = max_valid_in;

  // Martinsos constructor sets pinMode() internally
  _sonar = new UltraSonicDistanceSensor(
      trig_pin,
      echo_pin,
      max_distance_cm,
      max_timeout_us
  );
}

DistanceSensor::~DistanceSensor() {
  if (_sonar) {
    delete _sonar;
    _sonar = nullptr;
  }
}

void DistanceSensor::begin() {
  // Nothing required here, but kept for consistency
}

void DistanceSensor::tick(uint32_t now_ms) {
  measure_(now_ms, nullptr);
}

void DistanceSensor::tick(uint32_t now_ms, float temp_c) {
  measure_(now_ms, &temp_c);
}

void DistanceSensor::measure_(uint32_t now_ms, const float* temp_c) {

  // Safety: ensure sonar exists
  if (!_sonar) {
    _state.valid = false;
    _state.distance_cm = -1.0f;
    _state.last_update_ms = now_ms;
    return;
  }

  float cm;

  if (temp_c != nullptr) {
    cm = _sonar->measureDistanceCm(*temp_c);
  } else {
    cm = _sonar->measureDistanceCm();
  }

  _state.last_update_ms = now_ms;
  _state.distance_cm = cm;

  // Library returns -1.0 when invalid
  if (cm <= 0.0f) {
    _state.valid = false;
    return;
  }

  // Convert cm to inches
  float inches = cm * 0.3937007874f;

  if (inches < _min_valid_in || inches > _max_valid_in) {
    _state.valid = false;
    return;
  }

  _state.distance_in = inches;
  _state.valid = true;
}
