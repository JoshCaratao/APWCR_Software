#pragma once
#include <Arduino.h>

/*
===============================================================================
  StallGuard.h
===============================================================================

  PURPOSE
  -------
  Small reusable safety helper for DC motors with encoder feedback.

  It watches for the pattern:
    - meaningful nonzero motor duty
    - meaningful nonzero target RPM
    - too little encoder count movement for too long

  If that persists, it latches a stall fault and returns zero duty. A zero
  command held for reset_zero_ms clears the fault. Reversing direction is allowed
  so the robot can back away from a jam or hard stop.
===============================================================================
*/

struct StallGuardConfig {
  bool enabled = false;
  float duty_threshold = 0.25f;
  float target_rpm_threshold = 0.1f;
  int32_t min_count_delta = 5;
  uint32_t timeout_ms = 800;
  uint32_t reset_zero_ms = 300;
};

class StallGuard {
public:
  float apply(float duty,
              float target_rpm,
              int32_t encoder_count,
              uint32_t now_ms,
              const StallGuardConfig& cfg);

  void reset();

  bool faulted() const { return _faulted; }
  int8_t faultDir() const { return _fault_dir; }

private:
  static int8_t sign_(float x);

  bool _faulted = false;
  int8_t _fault_dir = 0;

  bool _watching = false;
  int8_t _watch_dir = 0;
  uint32_t _watch_start_ms = 0;
  int32_t _watch_start_count = 0;

  bool _zero_seen = false;
  uint32_t _zero_start_ms = 0;
};
