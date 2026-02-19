#pragma once
#include <Arduino.h>
#include <Encoder.h>  // Paul Stoffregen Encoder library

/*
===============================================================================
  EncoderSensor.h
===============================================================================

  PURPOSE
  -------
  Wrapper around the Encoder library that provides:
    - signed count
    - sampled delta counts
    - position in revolutions/degrees
    - speed in rps/rpm/dps

  USAGE
  -----
  - Call begin() once in setup()
  - Call sample(now_ms) at a fixed rate
  - Read getState() for position and speed

  IMPORTANT
  ---------
  counts_per_output_rev should represent the output you care about:
    - Drive wheel control -> counts per WHEEL revolution
    - Mechanism position   -> counts per JOINT/OUTPUT revolution
===============================================================================
*/

class EncoderSensor {
public:
  struct State {
    int32_t count = 0;             // signed accumulated counts
    int32_t delta_counts = 0;      // counts since last sample

    float revolutions = 0.0f;      // output revolutions
    float degrees = 0.0f;          // output angle in degrees

    float rps = 0.0f;              // revolutions per second
    float rpm = 0.0f;              // revolutions per minute
    float dps = 0.0f;              // degrees per second

    uint32_t last_sample_ms = 0;   // timestamp of last sample
    bool valid_speed = false;      // false until first valid dt > 0 sample
  };

  /*
    pin_a / pin_b:
      Quadrature encoder channels A and B

    counts_per_output_rev:
      Total counts for one output revolution (after gearing)

    invert_direction:
      Set true if forward physical motion reads as negative count
  */
  EncoderSensor(uint8_t pin_a,
                uint8_t pin_b,
                float counts_per_output_rev,
                bool invert_direction = false);

  void begin();
  void sample(uint32_t now_ms);

  int32_t getCount();
  void reset(int32_t new_count = 0);

  const State& getState() const { return _state; }

private:
  int32_t applySign_(int32_t raw_count) const;
  int32_t undoSign_(int32_t signed_count) const;

  Encoder _enc;

  float _counts_per_output_rev;
  bool _invert_direction;

  int32_t _last_sample_count;
  State _state;
};
