#pragma once

#include <Arduino.h>

class Rate {
public:
  // hz = how many times per second you want to run
  explicit Rate(uint16_t hz = 1) { setHz(hz); }

  void setHz(uint16_t hz) {
    if (hz == 0) hz = 1;
    _period_ms = (uint32_t)(1000UL / hz);
    if (_period_ms == 0) _period_ms = 1;
  }

  void setPeriodMs(uint32_t period_ms) {
    _period_ms = (period_ms == 0) ? 1 : period_ms;
  }

  // Returns true when it's time to run. If true, it schedules the next tick.
  bool ready(uint32_t now_ms) {
    if (!_initialized) {
      _next_ms = now_ms;      // run immediately on first call
      _initialized = true;
    }

    // Safe with millis() rollover because of signed subtraction trick
    if ((int32_t)(now_ms - _next_ms) >= 0) {
      _next_ms = now_ms + _period_ms;
      return true;
    }
    return false;
  }

  uint32_t periodMs() const { return _period_ms; }
  uint32_t nextMs() const { return _next_ms; }

private:
  uint32_t _period_ms = 1000;
  uint32_t _next_ms = 0;
  bool _initialized = false;
};
