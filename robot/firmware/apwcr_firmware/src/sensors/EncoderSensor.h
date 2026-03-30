#pragma once
#include <Arduino.h>

#include <Encoder.h>

/*
===============================================================================
  EncoderSensor.h
===============================================================================

  Purpose:
  Thin wrapper around the Paul Stoffregen Encoder library that converts raw
  counts into output-side position and speed units.

  Responsibilities:
  - Read signed encoder counts
  - Apply a configured direction convention
  - Convert counts into revolutions, degrees, and speed
  - Track the latest sampled encoder state

  Exposes:
  - signed count
  - delta counts since the last sample
  - revolutions
  - degrees
  - RPS, RPM, and DPS
  - valid_speed flag
  - last sample timestamp

  This class does not estimate motion between samples. It only updates state
  when sample(now_ms) is called.
===============================================================================
*/

class EncoderSensor {
public:
  struct State {
    int32_t count = 0;
    int32_t delta_counts = 0;

    float revolutions = 0.0f;
    float degrees = 0.0f;

    float rps = 0.0f;
    float rpm = 0.0f;
    float dps = 0.0f;

    bool valid_speed = false;
    uint32_t last_sample_ms = 0;
  };

  EncoderSensor(uint8_t pin_a,
                uint8_t pin_b,
                float counts_per_output_rev,
                bool invert_direction = false);

  // Reset the hardware/software state to zero.
  void begin();

  // Read the current signed count in the configured direction convention.
  int32_t getCount();

  // Reset the encoder count and internal state to a chosen value.
  void reset(int32_t new_count = 0);

  // Sample the encoder and update position/speed state.
  void sample(uint32_t now_ms);

  // Latest computed encoder state.
  const State& getState() const { return _state; }

private:
  // Apply the configured sign convention to a raw encoder count.
  int32_t applySign_(int32_t raw_count) const;

  // Convert a signed logical count back to the raw encoder sign convention.
  int32_t undoSign_(int32_t signed_count) const;

  Encoder _enc;
  State _state;

  float _counts_per_output_rev = 1.0f;
  bool _invert_direction = false;

  int32_t _last_sample_count = 0;
};
