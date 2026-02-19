#include "sensors/EncoderSensor.h"

/*
===============================================================================
  EncoderSensor.cpp
===============================================================================

  The Encoder library gives raw signed counts.
  This wrapper converts those counts into position and speed units.
===============================================================================
*/

EncoderSensor::EncoderSensor(uint8_t pin_a,
                             uint8_t pin_b,
                             float counts_per_output_rev,
                             bool invert_direction)
: _enc(pin_a, pin_b)
{
  if (counts_per_output_rev > 0.0f) {
    _counts_per_output_rev = counts_per_output_rev;
  } else {
    _counts_per_output_rev = 1.0f;
  }

  _invert_direction = invert_direction;
  _last_sample_count = 0;
}

void EncoderSensor::begin() {
  _enc.write(0);

  _state = State();
  _state.last_sample_ms = millis();

  _last_sample_count = 0;
}

int32_t EncoderSensor::applySign_(int32_t raw_count) const {
  if (_invert_direction) {
    return -raw_count;
  }
  return raw_count;
}

int32_t EncoderSensor::undoSign_(int32_t signed_count) const {
  if (_invert_direction) {
    return -signed_count;
  }
  return signed_count;
}

int32_t EncoderSensor::getCount(){
  int32_t raw = (int32_t)_enc.read();
  return applySign_(raw);
}

void EncoderSensor::reset(int32_t new_count) {
  int32_t raw_target = undoSign_(new_count);
  _enc.write((long)raw_target);

  _state = State();
  _state.count = new_count;
  _state.revolutions = (float)new_count / _counts_per_output_rev;
  _state.degrees = _state.revolutions * 360.0f;
  _state.last_sample_ms = millis();

  _last_sample_count = new_count;
}

void EncoderSensor::sample(uint32_t now_ms) {
  int32_t count_now = getCount();
  int32_t dc = count_now - _last_sample_count;
  uint32_t dt_ms = now_ms - _state.last_sample_ms;

  _state.count = count_now;
  _state.delta_counts = dc;

  _state.revolutions = (float)count_now / _counts_per_output_rev;
  _state.degrees = _state.revolutions * 360.0f;

  _state.last_sample_ms = now_ms;
  _last_sample_count = count_now;

  if (dt_ms == 0) {
    _state.valid_speed = false;
    return;
  }

  float dt_s = (float)dt_ms / 1000.0f;
  float d_rev = (float)dc / _counts_per_output_rev;

  _state.rps = d_rev / dt_s;
  _state.rpm = _state.rps * 60.0f;
  _state.dps = _state.rps * 360.0f;
  _state.valid_speed = true;
}
