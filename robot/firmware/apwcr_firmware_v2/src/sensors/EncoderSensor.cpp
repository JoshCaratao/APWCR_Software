#include "sensors/EncoderSensor.h"

/*
===============================================================================
  EncoderSensor.cpp
===============================================================================

  The underlying Encoder library provides raw signed counts only.
  This wrapper converts those counts into output-side position and speed units.
===============================================================================
*/

EncoderSensor::EncoderSensor(uint8_t pin_a,
                             uint8_t pin_b,
                             float counts_per_output_rev,
                             bool invert_direction,
                             float rpm_filter_alpha)
: _enc(pin_a, pin_b)
{
  if (counts_per_output_rev > 0.0f) {
    _counts_per_output_rev = counts_per_output_rev;
  } else {
    _counts_per_output_rev = 1.0f;
  }

  _invert_direction = invert_direction;
  _rpm_filter_alpha = clampAlpha_(rpm_filter_alpha);
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

float EncoderSensor::clampAlpha_(float alpha) const {
  if (alpha < 0.0f) return 0.0f;
  if (alpha > 1.0f) return 1.0f;
  return alpha;
}

int32_t EncoderSensor::getCount() {
  const int32_t raw = (int32_t)_enc.read();
  return applySign_(raw);
}

void EncoderSensor::reset(int32_t new_count) {
  const int32_t raw_target = undoSign_(new_count);
  _enc.write((long)raw_target);

  _state = State();
  _state.count = new_count;
  _state.revolutions = (float)new_count / _counts_per_output_rev;
  _state.degrees = _state.revolutions * 360.0f;
  _state.last_sample_ms = millis();

  _last_sample_count = new_count;
}

void EncoderSensor::sample(uint32_t now_ms) {
  const int32_t count_now = getCount();
  const int32_t dc = count_now - _last_sample_count;
  const uint32_t dt_ms = now_ms - _state.last_sample_ms;

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

  const float dt_s = (float)dt_ms / 1000.0f;
  const float d_rev = (float)dc / _counts_per_output_rev;
  const float rps_instant = d_rev / dt_s;
  const float rpm_instant = rps_instant * 60.0f;

  if (!_state.valid_speed || _rpm_filter_alpha >= 1.0f) {
    _state.rps = rps_instant;
    _state.rpm = rpm_instant;
  } else {
    const float a = _rpm_filter_alpha;
    _state.rps = a * rps_instant + (1.0f - a) * _state.rps;
    _state.rpm = a * rpm_instant + (1.0f - a) * _state.rpm;
  }

  _state.dps = _state.rps * 360.0f;
  _state.valid_speed = true;
}
